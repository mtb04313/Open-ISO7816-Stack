/**
 * \file reader_hal_comm_settings.h
 * \copyright This file is part of the Open-ISO7816-Stack project and is distributed under the MIT license. See LICENSE file in the root directory. 
 */



#ifndef __READER_HAL_COMM_SETTINGS_H__
#define __READER_HAL_COMM_SETTINGS_H__



#include "stdint.h"
#include "reader.h"
#include "reader_utils.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define READER_HAL_DEFAULT_FI         (uint32_t)(372)
#define READER_HAL_DEFAULT_DI         (uint32_t)(1)

#define READER_HAL_DEFAULT_GT         (uint32_t)(12)
#define READER_HAL_DEFAULT_UART_BAUD_RATE_MULTIPLIER	1

#ifdef TARGET_STM32F411
    #define READER_HAL_DEFAULT_FREQ       (uint32_t)(4200000)
	#define READER_HAL_STM32_SYSCLK       (uint32_t)(100000000)
#elif TARGET_STM32F407
    #define READER_HAL_DEFAULT_FREQ       (uint32_t)(4200000)
	#define READER_HAL_STM32_SYSCLK       (uint32_t)(168000000)
#elif defined CY_TARGET_BOARD
    // PSoC Edge and PSoC6
    #define READER_HAL_DEFAULT_FREQ       (uint32_t)(5000000)   // 5MHz: this is actually set in Device Configurator and both values must match
#else
	#ifndef TEST
	#error No target is defined. Impossible to go through compilation. Please define a target by setting a constant in the style TARGET_STM32F411 or TARGET_STM32F407. See documentation for the list of supported targets.
	#endif
#endif

#define READER_HAL_STM32_APB1_PRESC   (uint32_t)(4)
#define READER_HAL_STM32_AHB_PRESC    (uint32_t)(1)



/**
 * \struct READER_HAL_CommSettings
 * This structure contains all the low level communication settings that are needed by the abstraction layer to work propelly.
 * These parameters are defined in ISO7816-3 section 7.1.
 * Warning : the values in this structure should not by modified by other means that the intended accessors.
 */
typedef struct READER_HAL_CommSettings READER_HAL_CommSettings;
struct READER_HAL_CommSettings{
	uint32_t f;     /*!< f is the frequency currently used on the shared CLK line between the reader device and the card. */
	uint32_t Fi;    /*!< Fi is the clock rate conversion integer as defined in ISO7816-3 section 7.1. */
	uint32_t Di;    /*!< Di is the baud rate adjustement integer as defined in ISO7816-3 section 7.1. */
	uint32_t GT;    /*!< GT is the Guard Time between two characters as defined in ISO7816-3 section 7.2. */
	float uartBaudRateMultiplier;	/*!< uartBaudRateMultiplier is the multiplier to adjust UART baud rate for PSoC devices */
};


READER_Status READER_HAL_SetUartBaudRateMultiplier(READER_HAL_CommSettings *pSettings, float uartBaudRateMultiplier);
READER_Status READER_HAL_SetFreq(READER_HAL_CommSettings *pSettings, uint32_t newFreq);
READER_Status READER_HAL_SetEtu(READER_HAL_CommSettings *pSettings, uint32_t Fi, uint32_t Di);
READER_Status READER_HAL_SetGT(READER_HAL_CommSettings *pSettings, uint32_t newGT);
READER_Status READER_HAL_SetFi(READER_HAL_CommSettings *pSettings, uint32_t Fi);
READER_Status READER_HAL_SetDi(READER_HAL_CommSettings *pSettings, uint32_t Di);
READER_Status READER_HAL_SetRedundancyType(READER_HAL_CommSettings *pSettings, uint32_t rType);
uint32_t READER_HAL_GetGT(READER_HAL_CommSettings *pSettings);
uint32_t READER_HAL_GetGTMili(READER_HAL_CommSettings *pSettings);
uint32_t READER_HAL_GetFreq(READER_HAL_CommSettings *pSettings);
uint32_t READER_HAL_GetFi(READER_HAL_CommSettings *pSettings);
uint32_t READER_HAL_GetDi(READER_HAL_CommSettings *pSettings);
float READER_HAL_GetUartBaudRateMultiplier(READER_HAL_CommSettings *pSettings);
uint32_t READER_HAL_ComputePrescFromFreq(uint32_t freq);

#ifdef __cplusplus
}
#endif

#endif
