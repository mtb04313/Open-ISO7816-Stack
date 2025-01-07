/**
 * \file reader_hal_basis.c
 * \copyright This file is part of the Open-ISO7816-Stack project and is distributed under the MIT license. See LICENSE file in the root directory. 
 * This file provides primitives for interacting with the hardware in order to manipulate ISO7816-3 signals (I/O line, clock, reset, power ...).
 * These primitives are mainly used in the reader_hal.c file to order to construct higher (abstraction) level primitives.
 * All the hardware dependent functions should be located in this file, reader_hal_comm_settings.c and reader_periph.c.
 */


#include "reader_hal_basis.h"
#include "reader_periph.h"

#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
#include "feature_config.h"
#include "ifx_hal.h"
#include "ifx_hal_psoc6.h"
#include "ifx_debug.h"
#include "ifx_time.h"

#if (FEATURE_PWM_HAL == ENABLE_FEATURE) || (FEATURE_GPIO_HAL == ENABLE_FEATURE)
#include "cyhal.h"
#endif
#include "cybsp.h"

#else
// STM32
#include "stm32f4xx_hal.h"

SMARTCARD_HandleTypeDef smartcardHandleStruct;
#endif


/**
 * \fn READER_HAL_InitHardware(void)
 * \return READER_Status execution code. READER_OK is nominal execution.
 * This function initializes the hardware components which are necessary to execute the code and interrect with the smartcard. Mainly clock trees and GPIO configuration...
 * It contains define flags to be adjusted depending on the stm32 target being in use. It is recommend to set those flag directly from the Makfile by following the instructions in the README file.
 */
READER_Status READER_HAL_InitHardware(void){	
#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
	DEBUG_PRINT(("%s [%d]\n", __FUNCTION__, __LINE__));
	// Do nothing

#else
// STM32
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	
	
	/* Initialisation de la HAL */
	if(HAL_Init() != HAL_OK) return READER_ERR;
	
	/* Configuration du reseau d'horloges du stm32 */
	__HAL_RCC_PWR_CLK_ENABLE();
	
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
#ifdef TARGET_STM32F411
	RCC_OscInitStruct.PLL.PLLN = 400;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
#elif TARGET_STM32F407
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
#else
	#ifndef TEST
	#error No target is defined. Impossible to go through compilation. Please define a target by setting a constant in the style TARGET_STM32F411 or TARGET_STM32F407. See documentation for the list of supported targets.
	#endif
#endif
	RCC_OscInitStruct.PLL.PLLQ = 7;	
	
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) return READER_ERR;


	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; 
	
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);   // Attention pas de valeur de retour
#endif
	
	/* Initialisation de tous les peripheriques materiels                                      */
	if(READER_PERIPH_Init() != READER_OK) return READER_ERR;
	
	/* Configuration de tous les etats initiaux des E/S                                        */
	if(READER_HAL_SetIOLine(READER_HAL_STATE_OFF) != READER_OK)       return READER_ERR;
	if(READER_HAL_SetPwrLine(READER_HAL_STATE_OFF) != READER_OK)      return READER_ERR;
	if(READER_HAL_SetRstLine(READER_HAL_STATE_OFF) != READER_OK)      return READER_ERR;
	if(READER_HAL_SetClkLine(READER_HAL_STATE_OFF) != READER_OK)      return READER_ERR;
	
	
	return READER_OK;
}


/**
 * \fn READER_HAL_RcvChar(READER_HAL_CommSettings *pSettings, READER_HAL_Protocol protocol, uint8_t *character, uint32_t timeout)
 * \return READER_Status execution code. READER_OK means successful reception of a character. READER_TIMEOUT means timeout. Any other value indicates an error.
 * \param *pSettings is a pointer on a READER_HAL_CommSettings data structure. It has to be previously initialized and has to contain correct comminication parameters.
 * \param protocol is a READER_HAL_Protocol value indicating wether this function is called from a T=0 or a T=1 context (it has some slight influence over the character transmission).
 * \param *character is a pointer to an uint8_t variable where the received byte is going to be copied in.
 * \param timeout is the timeout value to be applied for the reception of the character.
 * This function is used to receive a single byte/character from the I/O transmission line.
 */
READER_Status READER_HAL_RcvChar(READER_HAL_CommSettings *pSettings, READER_HAL_Protocol protocol, uint8_t *character, uint32_t timeout){
	uint32_t newTimeout, currentGTMilli, tickstart;
	uint8_t dummy;
	
	
	/* Verification elementaire des parametres ...  */
	if(timeout==0){
		return READER_BAD_ARG;
	}
	
#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
	(void)dummy;    // suppressed warning; what's dummy used for?

	tickstart = READER_HAL_GetTick();
	currentGTMilli = READER_HAL_GetGTMili(pSettings);
	newTimeout = timeout + currentGTMilli;

	do {
		if (Cy_SCB_UART_GetNumInRxFifo(ISO_UART_HW) > 0) {
			*character = Cy_SCB_UART_Get(ISO_UART_HW);
			return READER_OK;
		}
	} while ((READER_HAL_GetTick() - tickstart) < newTimeout);

	return READER_TIMEOUT;

#else
// STM32
	/* Si on a un overrun ici, c'est que des carateres parasites sont arrives avant que l'on commence a recevoir. On les elimine ...  */
	if(USART2->SR & USART_SR_ORE){
		dummy = USART2->DR;
	}
	
	/* Reception d'un caractere */
	/* On suppose ici que le bloc USART2 a deja ete configure en mode smartcard et qu'il est active et correctement initailise avec les bon parametres de communication */
	tickstart = READER_HAL_GetTick();
	
	/* On active le bloc USART */
	USART2->CR1 |= USART_CR1_UE;
	
	/* On bloque l'envoi pendant la duree de la reception.  (Communication half duplex, RX et TX internally connected). Explication en.DM00031020 section 30.3.10 */
	USART2->CR1 &= ~USART_CR1_TE;
	
	/* On active la partie reception de l'USART */
	USART2->CR1 |= USART_CR1_RE;
	//USART2->SR &= ~USART_SR_RXNE;
	
	/* Pour prendre en compte le GT, on ajoute le GTMilli au timeout ...  */
	currentGTMilli = READER_HAL_GetGTMili(pSettings);
	newTimeout = timeout + currentGTMilli;
	
	while(!(USART2->SR & USART_SR_RXNE) && !(READER_HAL_GetTick()-tickstart >= newTimeout)){
			
	}
	
	/* Quand on sort de la boucle d'attente, on verifie si on est sorti a cause d'un timeout */
	if(READER_HAL_GetTick()-tickstart >= newTimeout){
		USART2->CR1 &= ~USART_CR1_RE;
		return READER_TIMEOUT;
	}
	
	/* TODO: Check for parity error, overrun error, frame error ...  */
	
	
	/* On recupere la donnee recue dans le Data Register */
	*character = USART2->DR;
	
	/* Une fois la reception termine, on reactive l'envoi. */
	//USART2->CR1 |= USART_CR1_TE;
	
	/* On desactive la partie reception de l'USART. Cela evite d'avoir des Overrun si on recoit des caracteres inattendus */
	USART2->CR1 &= ~USART_CR1_RE;
#endif
	
	return READER_OK;
}


/**
 * \fn READER_HAL_SendChar(READER_HAL_CommSettings *pSettings, READER_HAL_Protocol protocol, uint8_t character, uint32_t timeout)
 * \return READER_Status execution code. READER_OK means successful transmission of the character. READER_TIMEOUT means timeout. Any other value indicates an error.
 * \param *pSettings is a pointer on a READER_HAL_CommSettings data structure. It has to be previously initialized and has to contain correct comminication parameters.
 * \param protocol is a READER_HAL_Protocol value indicating wether this function is called from a T=0 or a T=1 context (it has some slight influence over the character transmission).
 * \param character is the character to be sent.
 * \param timeout is the timeout value to be applied for the transmission of the character.
 * This function is used to transmit a single byte/character on the I/O transmission line.
 */
READER_Status READER_HAL_SendChar(READER_HAL_CommSettings *pSettings, READER_HAL_Protocol protocol, uint8_t character, uint32_t timeout){
	uint32_t tickstart;
	uint32_t guardTime;
	
	
	if(timeout==0){
		return READER_BAD_ARG;
	}
	
#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
    // REVISIT: wait for guard time to elapse before sending
	(void)tickstart;
	(void)guardTime;

	do {
		if (Cy_SCB_UART_Put(ISO_UART_HW, character) == 1) {
			while (!Cy_SCB_IsTxComplete(ISO_UART_HW));
			break;
		}
	} while (true);

	// read the echo
	uint8_t dataEcho = 0x00;

	if (READER_HAL_RcvChar(pSettings, protocol, &dataEcho, 1000) != READER_OK) {
		DEBUG_PRINT(("%s [%d] %s\n", __FUNCTION__, __LINE__, "READER_HAL_RcvChar failed"));
		return READER_TIMEOUT;
	}

	if (dataEcho == character) {
		return READER_OK;
	} else {
		DEBUG_PRINT(("%s [%d] dataEcho=0x%02x, bt=0x%02x, %s\n",
					__FUNCTION__, __LINE__, dataEcho, character, "dataEcho != bt"));
	}

	return READER_TIMEOUT;

#else
// STM32
	/* On active le bloc USART */
	USART2->CR1 |= USART_CR1_UE;
	
	/* On desactive la reception le temps de l'envoi. (Communication half duplex, RX et TX internally connected). Explication en.DM00031020 section 30.3.10 */
	USART2->CR1 &= ~USART_CR1_RE;
	
	/* On suppose ici que le bloc USART2 a deja ete configure en mode smartcard et qu'il est active et correctement initailise avec les bon parametres de communication */
	USART2->CR1 |= USART_CR1_TE;
	
	/* On set le GT register. Attention pour le STM32 le GT defini dans la datasheet ne correspond pas exactement a celui defini dans la norme ISO7816-S section 7.2 */
	/* Dans la norme ISO il s'agit du nombre d'etu entre les starts bits de deux caracteres consecutifs                                                              */
	/* Dans le STM32 il s'agit du delai (en nombre d'etu) a ajouter apres le stop bit du premier caractere avant d'envoyer le start bit du second                    */
	guardTime = READER_HAL_GetGT(pSettings);
	MODIFY_REG(USART2->GTPR, USART_GTPR_GT, ((guardTime-READER_HAL_DEFAULT_GT)<<8U));
	
	tickstart = READER_HAL_GetTick();
	
	/* On attend que le buffer d'envoi soit empty. On verifie aussi qu'on depasse pas timeout. */
	while(!(USART2->SR & USART_SR_TXE) && ((READER_HAL_GetTick()-tickstart < timeout))){
		
	}
	
	/* On attend le flag Transmit Complete (TC). Cela permet de respecter le Guard Time (GT) entre les caracteres                    */
	/* En effet, le bloc materiel USART du STM32 est concu de sorte a set TC lorsque la transmission est termine ET le GT est ecoule */
	/* Dans une implementation sans bloc materiel USART de ce type il faudrait faire une verification manuelle de delai              */
	while(!(USART2->SR & USART_SR_TC)){
		
	}
	
	if((READER_HAL_GetTick()-tickstart) >= timeout){
		return READER_TIMEOUT;
	}
	
		
	/* On place le caractere dans le Data Register */
	USART2->DR = character;
	
	/* On reactive la reception une fois l'envoi termine */
	/* ... En fait on ne peut pas le reactiver ici : Il faudrait en realite le faire uniquement lorsque TC (transmit complete). Or, ici on a fait le choix de quitter la fonction avant TC. */
	//USART2->CR1 |= USART_CR1_RE;
#endif
	
	return READER_OK;
}


/**
 * \fn READER_Status READER_HAL_SetPwrLine(READER_HAL_State state)
 * \brief Cette fonction permet de changer l'état de la broche PWR.
 * \return Valeur de type READER_Status. READER_OK si l'exécution s'est correctement déroulée. Toute autre valeur suggère une erreur.
 * \param state paramètre de type READER_HAL_State. Peut prendre les valeurs READER_HAL_STATE_ON et READER_HAL_STATE_OFF. Indique l'état à imposer à la broche.
 */
READER_Status READER_HAL_SetPwrLine(READER_HAL_State state){
#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
	DEBUG_PRINT(("%s [%d]: %s\n", __FUNCTION__, __LINE__, (state == READER_HAL_STATE_ON)? "ON" : "OFF"));

	if(state == READER_HAL_STATE_ON){
#ifdef ISO7816_VCC
#if (FEATURE_GPIO_HAL == ENABLE_FEATURE)
		cyhal_gpio_write(ISO7816_VCC, ISO7816_VCC_ON);
#else
		Cy_GPIO_Write(ISO7816_VCC_PORT, ISO7816_VCC_PIN, ISO7816_VCC_ON);
#endif
#endif
	}
	else if(state == READER_HAL_STATE_OFF){
#ifdef ISO7816_VCC
#if (FEATURE_GPIO_HAL == ENABLE_FEATURE)
		cyhal_gpio_write(ISO7816_VCC, ISO7816_VCC_OFF);
#else
		Cy_GPIO_Write(ISO7816_VCC_PORT, ISO7816_VCC_PIN, ISO7816_VCC_OFF);
#endif
#endif
	}
	else{
		return READER_ERR;
	}

#else
// STM32
	if(state == READER_HAL_STATE_ON){
		HAL_GPIO_WritePin(GPIOA, READER_PERIPH_PWR_PIN, GPIO_PIN_SET);
	}
	else if(state == READER_HAL_STATE_OFF){
		HAL_GPIO_WritePin(GPIOA, READER_PERIPH_PWR_PIN, GPIO_PIN_RESET);
	}
	else{
		return READER_ERR;
	}
#endif
	
	return READER_OK;
}



/**
 * \fn READER_Status READER_HAL_SetRstLine(READER_HAL_State state)
 * \brief Cette fonction permet de changer l'état de la broche RST.
 * \return Valeur de type READER_Status. READER_OK si l'exécution s'est correctement déroulée. Toute autre valeur suggère une erreur.
 * \param state paramètre de type READER_HAL_State. Peut prendre les valeurs READER_HAL_STATE_ON et READER_HAL_STATE_OFF. Indique l'état à imposer à la broche.
 */
READER_Status READER_HAL_SetRstLine(READER_HAL_State state){
#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
	DEBUG_PRINT(("%s [%d]: %s\n", __FUNCTION__, __LINE__, (state == READER_HAL_STATE_ON)? "ON" : "OFF"));

	if(state == READER_HAL_STATE_ON){
#if (FEATURE_GPIO_HAL == ENABLE_FEATURE)
		cyhal_gpio_write(ISO7816_RESET, ISO7816_RESET_ON);
#else
		Cy_GPIO_Write(ISO7816_RESET_PORT, ISO7816_RESET_PIN, ISO7816_RESET_ON);
#endif
	}
	else if(state == READER_HAL_STATE_OFF){
#if (FEATURE_GPIO_HAL == ENABLE_FEATURE)
		cyhal_gpio_write(ISO7816_RESET, ISO7816_RESET_OFF);
#else
		Cy_GPIO_Write(ISO7816_RESET_PORT, ISO7816_RESET_PIN, ISO7816_RESET_OFF);
#endif
	}
	else{
		return READER_ERR;
	}

#else
// STM32
	if(state == READER_HAL_STATE_ON){
		HAL_GPIO_WritePin(READER_PERIPH_RST_PORT, READER_PERIPH_RST_PIN, GPIO_PIN_SET);
	}
	else if(state == READER_HAL_STATE_OFF){
		HAL_GPIO_WritePin(READER_PERIPH_RST_PORT, READER_PERIPH_RST_PIN, GPIO_PIN_RESET);
	}
	else{
		return READER_ERR;
	}
#endif
	return READER_OK;
}


/**
 * \fn READER_Status READER_HAL_SetIOLine(READER_HAL_State state)
 * \brief Cette fonction permet de changer l'état de la broche IO.
 * \return Valeur de type READER_Status. READER_OK si l'exécution s'est correctement déroulée. Toute autre valeur suggère une erreur.
 * \param state paramètre de type READER_HAL_State. Peut prendre les valeurs READER_HAL_STATE_ON et READER_HAL_STATE_OFF. Indique l'état à imposer à la broche.
 */
READER_Status READER_HAL_SetIOLine(READER_HAL_State state){
	/* On veut forcer l'etat de la ligne IO, donc on deconnecte le GPIO du bloc USART. PB: Quand est ce que on rend la main au bloc usart ? */
	/* Peut etre possible de pull down la ligne meme lorsque l'UART est dessus */
	
#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
	DEBUG_PRINT(("%s [%d]: %s\n", __FUNCTION__, __LINE__, (state == READER_HAL_STATE_ON)? "ON" : "OFF"));

	if(state == READER_HAL_STATE_ON){
		/* Enable UART to operate */
		Cy_SCB_UART_Enable(ISO_UART_HW);

		/* Clear UART buffer */
		Cy_SCB_UART_ClearRxFifo(ISO_UART_HW);
	}
	else if(state == READER_HAL_STATE_OFF){
		/* Stop UART */
		Cy_SCB_UART_Disable(ISO_UART_HW, NULL); //&s_uartContext);
	}
	else{
		return READER_ERR;
	}

#else
// STM32
	if(state == READER_HAL_STATE_ON){
		/*  ... Chgt alternate fct, chgt etat ... mais apres la main par l'uart est perdue */
		HAL_GPIO_WritePin(READER_PERIPH_IO_PORT, READER_PERIPH_IO_PIN, GPIO_PIN_SET);
	}
	else if(state == READER_HAL_STATE_OFF){
		HAL_GPIO_WritePin(READER_PERIPH_IO_PORT, READER_PERIPH_IO_PIN, GPIO_PIN_RESET);
	}
	else{
		return READER_ERR;
	}
#endif
	return READER_OK;
}


/**
 * \fn READER_Status READER_HAL_SetClkLine(READER_HAL_State state)
 * \brief Cette fonction permet de changer l'état de la broche CLK. En l'occurence elle permet d'activer ou non la génération de l'horloge.
 * \return Valeur de type READER_Status. READER_OK si l'exécution s'est correctement déroulée. Toute autre valeur suggère une erreur.
 * \param state paramètre de type READER_HAL_State. Peut prendre les valeurs READER_HAL_STATE_ON et READER_HAL_STATE_OFF. Indique l'état à imposer à la broche.
 */
READER_Status READER_HAL_SetClkLine(READER_HAL_State state){
#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
	DEBUG_PRINT(("%s [%d]: %s\n", __FUNCTION__, __LINE__, (state == READER_HAL_STATE_ON)? "ON" : "OFF"));

	if(state == READER_HAL_STATE_ON){
#if (FEATURE_PWM_HAL == ENABLE_FEATURE)
		cy_rslt_t result = ~CY_RSLT_SUCCESS;
		/* Start the PWM using HAL APIs */
		result = cyhal_pwm_start(&s_pwmClkControl);
		ReturnAssert(result == CY_RSLT_SUCCESS, false);

#else
		/* Start the PWM using PDL APIs */
		Cy_TCPWM_Enable_Single(ISO_TCPWM_HW, ISO_TCPWM_NUM);
		Cy_TCPWM_TriggerStart_Single(ISO_TCPWM_HW, ISO_TCPWM_NUM);
#endif
	}
	else if(state == READER_HAL_STATE_OFF){
#if (FEATURE_PWM_HAL == ENABLE_FEATURE)
		/* Stop the PWM using HAL APIs */
		//if (s_pwmGpioInitialized) {
			result = cyhal_pwm_stop(&s_pwmClkControl);
			ReturnAssert(result == CY_RSLT_SUCCESS, false);
		//}
#else
		/* Stop the PWM using PDL APIs */
		//if (s_pwmGpioInitialized) {
			Cy_TCPWM_Disable_Single(ISO_TCPWM_HW, ISO_TCPWM_NUM);
		//}
#endif
	}
	else{
		return READER_ERR;
	}

#else
// STM32
	GPIO_InitTypeDef gpioInitStruct;
	
	if(state == READER_HAL_STATE_ON){
		/* On connecte le GPIO sur la clock du block UART */
		gpioInitStruct.Pin = READER_PERIPH_CLK_PIN;
		gpioInitStruct.Mode = GPIO_MODE_AF_PP; 
		gpioInitStruct.Pull = GPIO_PULLUP;
		//gpioInitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
		gpioInitStruct.Alternate = GPIO_AF7_USART2;
		
		__HAL_RCC_GPIOA_CLK_ENABLE();
		HAL_GPIO_Init(READER_PERIPH_CLK_PORT, &gpioInitStruct);
	}
	else if(state == READER_HAL_STATE_OFF){
		/* On connecte le GPIO sur la clock du block UART */
		gpioInitStruct.Pin = READER_PERIPH_CLK_PIN;
		gpioInitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
		gpioInitStruct.Pull = GPIO_NOPULL;
		//gpioInitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
		//gpioInitStruct.Alternate = GPIO_AF7_USART2;
		
		__HAL_RCC_GPIOA_CLK_ENABLE();
		HAL_GPIO_Init(READER_PERIPH_CLK_PORT, &gpioInitStruct);
	}
	else{
		return READER_ERR;
	}

#endif
	return READER_OK;
}


void READER_HAL_Delay(uint32_t tMili){
#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
	msleep(tMili);
#else
// STM32
	HAL_Delay(tMili);
#endif
}

uint32_t READER_HAL_GetTick(void){
#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
	return current_time();

#else
// STM32
	return (uint32_t)HAL_GetTick();

#endif
}


/**
 * \fn READER_Status READER_HAL_WaitSendComplete(READER_HAL_CommSettings *pSettings)
 * \brief Waits until a byte send process is over.
 * \param *pSettings is a pointer a #READER_HAL_CommSettings strcut containing the HAL low level communication parameters.
 * \return This function returns an #HAL_Status execution code.
 */
READER_Status READER_HAL_WaitUntilSendComplete(READER_HAL_CommSettings *pSettings){
#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
	while (!Cy_SCB_IsTxComplete(ISO_UART_HW));
#else
// STM32
	while(!(USART2->SR & USART_SR_TC));
#endif

	return READER_OK;
}
