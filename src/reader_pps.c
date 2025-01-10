/**
 * \file reader_sifs.c
 * \copyright This file is part of the Open-ISO7816-Stack project and is distributed under the MIT license. See LICENSE file in the root directory. 
 * This file is the higher level of the abstraction layer (HAL). The code contained in this file is almost not hardware dependent.
 * TODO: The goal in the next versions is to move the last (very few) pieces of hardware dependent code from this file to reader_sifs_basis.c or reader_sifs_comm_settings.c or reader_periph.c.
 */

#include "reader_pps.h"
#include "reader_hal.h"

#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
#include "feature_config.h"
#include "ifx_debug.h"
#include "ifx_hal_psoc6.h"
#endif

//============================================================================
// local definitions
//============================================================================
#define SIZEOF_PPS 4

typedef struct {
    uint8_t byteTA1;
    uint8_t cmdPPS[SIZEOF_PPS];
    float uartBaudRateMultiple;
} PPS_Info_t;


//============================================================================
// local data
//============================================================================
// Ref: ISO/IEC FDIS 7816-3:2006(E) section 9
//
//  Table 7
//  whether TA1 is 0x1X or 0x9X, the max ISO7816 CLK freq is 5MHz (i.e. max pwmClkFreq)
//
//  Min CLK freq is 1MHz

static PPS_Info_t ppsInfo_T0[] = {
    {0x11, { 0xFF, 0x10, 0x11, 0xFE }, 1      }, // Fi=372, Di=1  => DF=372/1 = 372
    {0x18, { 0xFF, 0x10, 0x18, 0xF7 }, 12     }, // Fi=372, Di=12 => DF=372/12= 31 (Note: 372/31 = 12)
    {0x95, { 0xFF, 0x10, 0x95, 0x7A }, 11.625 }, // Fi=512, Di=16 => DF=512/16= 32 (Note: 372/32 = 11.625)
    {0x96, { 0xFF, 0x10, 0x96, 0x79 }, 23.25  }, // Fi=512, Di=32 => DF=512/32= 16 (Note: 372/16 = 23.25)
    {0x97, { 0xFF, 0x10, 0x97, 0x78 }, 46.5   }, // Fi=512, Di=64 => DF=512/64= 8  (Note: 372/8  = 46.5)
};

static PPS_Info_t ppsInfo_T1[] = {
    {0x11, { 0xFF, 0x11, 0x11, 0xFF }, 1,      }, // Fi=372, Di=1  => DF=372/1 = 372
    {0x18, { 0xFF, 0x11, 0x18, 0xF6 }, 12,     }, // Fi=372, Di=12 => DF=372/12= 31 (Note: 372/31 = 12)
    {0x95, { 0xFF, 0x11, 0x95, 0x7B }, 11.625, }, // Fi=512, Di=16 => DF=512/16= 32 (Note: 372/32 = 11.625)
    {0x96, { 0xFF, 0x11, 0x96, 0x78 }, 23.25,  }, // Fi=512, Di=32 => DF=512/32= 16 (Note: 372/16 = 23.25)
    {0x97, { 0xFF, 0x11, 0x97, 0x79 }, 46.5,   }, // Fi=512, Di=64 => DF=512/64= 8  (Note: 372/8  = 46.5)
};

/**
 * \fn READER_Status Reader_PPS_Exchange(READER_ATR_Atr *atr, READER_HAL_CommSettings *pSettings)
 * \return The function returns an execution code of type READER_Status that indicates if the function behaved as expected or not.
 * \param *atr Pointer to a valid READER_ATR_Atr structure, which has the TA1 byte
 * \param *pSettings is a pointer on a READER_HAL_CommSettings data structure that should already be containing the low level communications settings for the hardware abstraction layer.*
 * This function is used to exchange PPS values with the ICC as defined in the ISO7816-3 section 9.
 */ 
READER_Status Reader_PPS_Exchange(READER_ATR_Atr *atr, READER_HAL_CommSettings *pSettings)
{
	READER_Status retVal = READER_OK;

	DEBUG_PRINT(("%s [%d]\n", __FUNCTION__, __LINE__));
	DEBUG_PRINT(("TA1=0x%02X, Fi=%lu, Di=%lu, fMax=%lu\n", atr->TA1, atr->Fi, atr->Di, atr->fMax));

	uint8_t byteTA1 = atr->TA1;
    int i;
    int found = 0;  // default entry to use if no match is found
    PPS_Info_t *p_ppsInfo = NULL;

	// impose a limit on the baud rate if the SCB is unable to support
	if (byteTA1 > MAX_BYTE_TA1) {
		byteTA1 = MAX_BYTE_TA1;
	}

	if (READER_ATR_IsT1(atr) == READER_OK) {
		// T=1
        for (i = 0; i < sizeof(ppsInfo_T1)/sizeof(ppsInfo_T1[0]); i++) {
            if (ppsInfo_T1[i].byteTA1 == byteTA1) {
                found = i;
                break;
            }
        }
        p_ppsInfo = &ppsInfo_T1[found];
	}
	else {
		// T=0
        for (i = 0; i < sizeof(ppsInfo_T0)/sizeof(ppsInfo_T0[0]); i++) {
            if (ppsInfo_T0[i].byteTA1 == byteTA1) {
                found = i;
                break;
            }
        }
        p_ppsInfo = &ppsInfo_T0[found];
	}
    ReturnAssert(p_ppsInfo != NULL, READER_ERR);

    uint32_t timeout = READER_ATR_DEFAULT_TIMEOUT;
    READER_HAL_Protocol protocol = READER_HAL_PROTOCOL_T0;
    uint8_t rxBuffer[SIZEOF_PPS];
    uint32_t rxLength = sizeof(rxBuffer);

	retVal = READER_HAL_SendCharFrame(	pSettings,
										protocol,
										p_ppsInfo->cmdPPS,
										sizeof(p_ppsInfo->cmdPPS),
										timeout);
	if(retVal != READER_OK) return retVal;

	retVal = READER_HAL_RcvCharFrameCount(	pSettings,
											protocol,
											rxBuffer,
											rxLength,
											&rxLength,
											timeout);
	if(retVal != READER_OK) return retVal;

    if (rxLength != sizeof(p_ppsInfo->cmdPPS)) {
        DEBUG_PRINT(("%s [%d]: Error in PPS response length = %d\n",
                     __FUNCTION__, __LINE__, rxLength));
        if (rxLength > 0) {
            print_bytes("", rxBuffer, rxLength);
        }
        return READER_ERR;
    }

    // save the modified values into pSettings, so they can be retrieved by
    // READER_T0_CONTEXT_Init() or READER_T1_CONTEXT_Init()
    retVal = READER_HAL_SetUartBaudRateMultiplier(pSettings, p_ppsInfo->uartBaudRateMultiple);
	if(retVal != READER_OK) return retVal;

    retVal = READER_HAL_SetFi(pSettings, READER_ATR_ComputeFi(byteTA1));
	if(retVal != READER_OK) return retVal;

	retVal = READER_HAL_SetDi(pSettings, READER_ATR_ComputeDi(byteTA1));
	if(retVal != READER_OK) return retVal;

	return retVal;
}
