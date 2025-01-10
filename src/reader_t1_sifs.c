/**
 * \file reader_sifs.c
 * \copyright This file is part of the Open-ISO7816-Stack project and is distributed under the MIT license. See LICENSE file in the root directory. 
 * This file is the higher level of the abstraction layer (HAL). The code contained in this file is almost not hardware dependent.
 * TODO: The goal in the next versions is to move the last (very few) pieces of hardware dependent code from this file to reader_sifs_basis.c or reader_sifs_comm_settings.c or reader_periph.c.
 */

#include "reader_t1_sifs.h"
#include "reader_t1_control.h"

#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
#include "feature_config.h"
#include "ifx_debug.h"
#endif

/**
 * \fn READER_Status READER_T1_SIFS_PrintCurrent(READER_T1_ContextHandler *pContext, char* pLabel)
 * \return The function returns an execution code of type READER_Status that indicates if the function behaved as expected or not.
 * \param *pContext is a pointer to a valid READER_T1_ContextHandler data structure.
 * \param *pLabel Pointer to a char string, which will be printed as text
 * This function is used to print the current IFSC and IFSD values of the T1 context
 */
void READER_T1_SIFS_PrintCurrent(READER_T1_ContextHandler *pContext, char* pLabel)
{
	READER_Status retVal;
	uint32_t currentIfsc = 0;
	uint32_t currentIfsd = 0;

	retVal = READER_T1_CONTEXT_GetCurrentIFSC(pContext, &currentIfsc);
	if (retVal == READER_OK) {
		retVal = READER_T1_CONTEXT_GetCurrentIFSD(pContext, &currentIfsd);

		if (retVal == READER_OK) {
			DEBUG_PRINT(("%s: currentIfsc=%lu, currentIfsd=%lu\n", pLabel, currentIfsc, currentIfsd));
		}
	}
}

/**
 * \fn READER_Status READER_T1_SIFS_Modify(READER_T1_ContextHandler *pContext, READER_ATR_Atr *atr)
 * \return The function returns an execution code of type READER_Status that indicates if the function behaved as expected or not.
 * \param *pContext is a pointer on a READER_T1_ContextHandler data structure to be initialized by these function.
 * \param *atr Pointer to a valid READER_ATR_Atr structure, which has the IFSC value of the ICC
 * This function is used to modify the IFSC and IFSD values with the ICC as defined in the ISO7816-3 section 11.4 and 11.6.
 */ 
READER_Status READER_T1_SIFS_Modify(READER_T1_ContextHandler *pContext, READER_ATR_Atr *atr)
{
	READER_Status retVal = READER_ERR;

	DEBUG_PRINT(("%s [%d]\n", __FUNCTION__, __LINE__));

	// if card is capable of a bigger Information Field Size (Card), use it
	uint8_t atrIfsc = 0;
	retVal = READER_ATR_GetT1IFSC(atr, &atrIfsc);

	if (retVal == READER_OK) {
		retVal = READER_T1_CONTEXT_SetCurrentIFSC(pContext, (uint32_t)atrIfsc);

		if (retVal == READER_OK) {
			// tell the card we are changing the Information Field Size (Device)
			// to have the same value as IFSC
			uint8_t newIfsd = atrIfsc;
			retVal = READER_T1_CONTROL_SendIfsdRequest(pContext, newIfsd);

			if (retVal != READER_OK) {
				DEBUG_PRINT(("READER_T1_CONTROL_SendIfsdRequest Failed\n"));
			}
		}
		else {
			DEBUG_PRINT(("READER_T1_CONTEXT_SetCurrentIFSC Failed\n"));
		}
	}
	else {
		DEBUG_PRINT(("READER_ATR_GetT1IFSC Failed\n"));
	}

	return retVal;
}
