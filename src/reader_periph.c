/**
 * \file reader_periph.c
 * \copyright This file is part of the Open-ISO7816-Stack project and is distributed under the MIT license. See LICENSE file in the root directory. 
 * This file provides code for initializing all the hardware peripherals being in charge of providing signals to the smartcard.
 * It includes I/O, CLK, RST, VCC.
 */

#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
#include "feature_config.h"
#include "ifx_hal.h"
#include "ifx_hal_psoc6.h"
#include "ifx_debug.h"

#if (FEATURE_PWM_HAL == ENABLE_FEATURE) || (FEATURE_GPIO_HAL == ENABLE_FEATURE)
#include "cyhal.h"
#endif
#include "cybsp.h"

#else
// STM32
#include "stm32f4xx_hal.h"
#endif

#include "reader_periph.h"
#include "reader_utils.h"
#include "reader.h"
#include "reader_hal_comm_settings.h"

#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
static bool s_vccGpioInitialized = false;
static bool s_resetGpioInitialized = false;
static bool s_pwmGpioInitialized = false;
static bool s_uartInitialized = false;

#if (FEATURE_PWM_HAL == ENABLE_FEATURE)
/* PWM object */
static cyhal_pwm_t s_pwmClkControl;
#endif

#else
// STM32
extern SMARTCARD_HandleTypeDef smartcardHandleStruct;
#endif

/**
 * \fn READER_PERIPH_InitIOLine(void)
 * \return READER_Status execution code. READER_OK indicates successful execution. Any other value is an error.
 * Initializes the GPIO peripheral in charge of the I/O transmission line.
 */
READER_Status READER_PERIPH_InitIOLine(void){
#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
	DEBUG_PRINT(("%s [%d]\n", __FUNCTION__, __LINE__));

	/* Configure UART to operate */
	cy_en_scb_uart_status_t status;
	status = Cy_SCB_UART_Init(ISO_UART_HW, &ISO_UART_config, NULL); //&s_uartContext);
	if (status != CY_SCB_UART_SUCCESS) {
		s_uartInitialized = false;
		ReturnAssert(s_uartInitialized, false);
	}

	s_uartInitialized = true;

#else
// STM32
	GPIO_InitTypeDef gpioInitStruct;
	
	gpioInitStruct.Pin = READER_PERIPH_IO_PIN;
	gpioInitStruct.Mode = GPIO_MODE_AF_OD;              /* Voir en.DM00105879 section 30.3.11  - TX Open-Drain */
	gpioInitStruct.Pull = GPIO_PULLUP;
	gpioInitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	gpioInitStruct.Alternate = GPIO_AF7_USART2;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	HAL_GPIO_Init(READER_PERIPH_IO_PORT, &gpioInitStruct);
#endif

	return READER_OK;
}


/**
 * \fn READER_PERIPH_InitClkLine(void)
 * \return READER_Status execution code. READER_OK indicates successful execution. Any other value is an error.
 * Initializes the GPIO peripheral in charge of providing the CLK signal to the smartcard.
 */
READER_Status READER_PERIPH_InitClkLine(void){
#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
	DEBUG_PRINT(("%s [%d]\n", __FUNCTION__, __LINE__));

#if (FEATURE_PWM_HAL == ENABLE_FEATURE)
	cy_rslt_t result = ~CY_RSLT_SUCCESS;
	/* Initialize the PWM using HAL APIs */
	result = cyhal_pwm_init(&s_pwmClkControl, ISO7816_CLK, NULL);
	if(CY_RSLT_SUCCESS != result) {
		s_pwmGpioInitialized = false;
		ReturnAssert(s_pwmGpioInitialized, false);
	}
	s_pwmGpioInitialized = true;

	/* Set the PWM output frequency and duty cycle */
	result = cyhal_pwm_set_duty_cycle(&s_pwmClkControl, PWM_DUTY_CYCLE, HAL_GetDefaultPwmClkfreq());
	ReturnAssert(result == CY_RSLT_SUCCESS, false);

#else
	/* Initialize the PWM using PDL APIs */
	cy_en_tcpwm_status_t tcpwm_status;
	tcpwm_status = Cy_TCPWM_PWM_Init(ISO_TCPWM_HW, ISO_TCPWM_NUM, &ISO_TCPWM_config);
	if (tcpwm_status != CY_TCPWM_SUCCESS) {
		s_pwmGpioInitialized = false;
		ReturnAssert(s_pwmGpioInitialized, false);
	}
	s_pwmGpioInitialized = true;

	// PWM output frequency and duty cycle are
	// configured using Device Configurator

#endif

#else
// STM32
	GPIO_InitTypeDef gpioInitStruct;
	
	gpioInitStruct.Pin = READER_PERIPH_CLK_PIN;
	gpioInitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
	gpioInitStruct.Pull = GPIO_NOPULL;
	//gpioInitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	//gpioInitStruct.Alternate = GPIO_AF7_USART2;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	HAL_GPIO_Init(READER_PERIPH_CLK_PORT, &gpioInitStruct);
#endif
	return READER_OK;
}


/**
 * \fn READER_PERIPH_InitRstLine(void)
 * \return READER_Status execution code. READER_OK indicates successful execution. Any other value is an error.
 * Initializes the GPIO peripheral in charge of providing the RST signal to teh smartcard.
 */
READER_Status READER_PERIPH_InitRstLine(void){
#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
	DEBUG_PRINT(("%s [%d]\n", __FUNCTION__, __LINE__));

#if (FEATURE_GPIO_HAL == ENABLE_FEATURE)
	cy_rslt_t result = ~CY_RSLT_SUCCESS;
	result = cyhal_gpio_init( 	ISO7816_RESET,
								CYHAL_GPIO_DIR_OUTPUT,
								CYHAL_GPIO_DRIVE_STRONG,
								ISO7816_RESET_ON);

	if (result != CY_RSLT_SUCCESS) {
		s_resetGpioInitialized = false;
		DEBUG_PRINT(("cyhal_gpio_init failed -- Have you unchecked ISO7816_RESET pin in Device Configurator?\n"));
		ReturnAssert(s_resetGpioInitialized, false);
	}
#endif
	s_resetGpioInitialized = true;

#else
// STM32
	GPIO_InitTypeDef gpioInitStruct;
	
	gpioInitStruct.Pin = READER_PERIPH_RST_PIN;
	gpioInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	gpioInitStruct.Pull = GPIO_NOPULL;
	gpioInitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	HAL_GPIO_Init(READER_PERIPH_RST_PORT, &gpioInitStruct);
#endif
	return READER_OK;
}


/**
 * \fn READER_PERIPH_InitPwrLine(void)
 * \return READER_Status execution code. READER_OK indicates successful execution. Any other value is an error.
 * Initializes the GPIO peripheral in charge of the power supply of the smartcard.
 */
READER_Status READER_PERIPH_InitPwrLine(void){
#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
	cy_rslt_t result = ~CY_RSLT_SUCCESS;

	DEBUG_PRINT(("%s [%d]\n", __FUNCTION__, __LINE__));

#ifdef ISO7816_VCC
#if (FEATURE_GPIO_HAL == ENABLE_FEATURE)
	result = cyhal_gpio_init( 	ISO7816_VCC,
								CYHAL_GPIO_DIR_OUTPUT,
								CYHAL_GPIO_DRIVE_STRONG,
								ISO7816_VCC_OFF);

	if (result != CY_RSLT_SUCCESS) {
		DEBUG_PRINT(("cyhal_gpio_init failed -- Have you unchecked ISO7816_VCC pin in Device Configurator?\n"));
		ReturnAssert((result == CY_RSLT_SUCCESS), false);
	}
#else
	result = CY_RSLT_SUCCESS;
#endif
#endif

	s_vccGpioInitialized = (result == CY_RSLT_SUCCESS);

#else
// STM32
	GPIO_InitTypeDef gpioInitStruct;
	
	gpioInitStruct.Pin = READER_PERIPH_PWR_PIN;
	gpioInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	gpioInitStruct.Pull = GPIO_NOPULL;
	gpioInitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	HAL_GPIO_Init(READER_PERIPH_PWR_PORT, &gpioInitStruct);
#endif
	return READER_OK;
}


/**
 * \fn READER_PERIPH_Init(void)
 * \return READER_Status execution code. READER_OK indicates successful execution. Any other value is an error.
 * Initializes all the peripherals (I/O, CLK, RST, VCC).
 */
READER_Status READER_PERIPH_Init(void){
	DEBUG_PRINT(("%s [%d]\n", __FUNCTION__, __LINE__));

	if(READER_PERIPH_InitIOLine() != READER_OK)  return READER_ERR;
	if(READER_PERIPH_InitClkLine() != READER_OK) return READER_ERR;
	if(READER_PERIPH_InitRstLine() != READER_OK) return READER_ERR;
	if(READER_PERIPH_InitPwrLine() != READER_OK) return READER_ERR;
	
#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
	// do nothing

#else
// STM32
	smartcardHandleStruct.Instance = USART2;
	smartcardHandleStruct.Init.BaudRate = READER_UTILS_ComputeBaudRate(READER_HAL_DEFAULT_FREQ, READER_HAL_DEFAULT_FI, READER_HAL_DEFAULT_DI); 
	smartcardHandleStruct.Init.WordLength = SMARTCARD_WORDLENGTH_9B;
	smartcardHandleStruct.Init.StopBits = SMARTCARD_STOPBITS_1_5;
	smartcardHandleStruct.Init.Parity = SMARTCARD_PARITY_EVEN;
	smartcardHandleStruct.Init.Mode = SMARTCARD_MODE_TX_RX;
	smartcardHandleStruct.Init.CLKPolarity = SMARTCARD_POLARITY_LOW;
	smartcardHandleStruct.Init.CLKPhase = SMARTCARD_PHASE_1EDGE;
	smartcardHandleStruct.Init.CLKLastBit = SMARTCARD_LASTBIT_ENABLE;
	smartcardHandleStruct.Init.GuardTime = 0; //READER_HAL_DEFAULT_GT;
	smartcardHandleStruct.Init.NACKState = SMARTCARD_NACK_ENABLE;
	
	smartcardHandleStruct.Init.Prescaler = READER_HAL_ComputePrescFromFreq(READER_HAL_DEFAULT_FREQ);
	
	
	__HAL_RCC_USART2_CLK_ENABLE();
	if(HAL_SMARTCARD_Init(&smartcardHandleStruct) != HAL_OK) return READER_ERR;
#endif

	return READER_OK;
}


void READER_PERIPH_ErrHandler(void){
#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
	// do nothing

#else
// STM32
	GPIO_InitTypeDef gpioInitStruct;
	
	gpioInitStruct.Pin = GPIO_PIN_14;
	gpioInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	gpioInitStruct.Pull = GPIO_NOPULL;
	gpioInitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	
	__HAL_RCC_GPIOD_CLK_ENABLE();
	HAL_GPIO_Init(GPIOD, &gpioInitStruct);

	
	while(1){
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		HAL_Delay(100);
	}
#endif
}
