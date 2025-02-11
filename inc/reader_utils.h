/**
 * \file reader_utils.h
 * \copyright This file is part of the Open-ISO7816-Stack project and is distributed under the MIT license. See LICENSE file in the root directory. 
 */


#ifndef __READER_UTILS_H__
#define __READER_UTILS_H__

#include "reader.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define MIN(a, b)  ((a<b)? a:b)
#define MAX(a, b)  ((a>b)? a:b)


uint32_t READER_UTILS_ComputeBaudRate(uint32_t freq, uint32_t Fi, uint32_t Di);
uint32_t READER_UTILS_GetCardFreq(uint32_t SYSCLK, uint32_t AHB, uint32_t APB1, uint32_t value_USART_GTPR_PSC);
uint32_t READER_UTILS_ComputeNewBaudRate(uint32_t oldBaudRate, uint32_t oldFreq, uint32_t newFreq);                /* Calcul du nouveau baudrate de sorte a pouvoir faire un chagement de frequence sans chager l'ETU => il faut modifier le baudrate  */
uint32_t READER_UTILS_ComputeTimeoutMiliSec(uint32_t baudRate, uint32_t WT);
uint32_t READER_UTILS_ComputeWT1(uint32_t f, uint32_t Fi, uint32_t WI);
uint32_t READER_UTILS_ComputeWT2(uint32_t baudRate, uint32_t Di, uint32_t WI);
uint32_t READER_UTILS_ComputeBestFreq(uint32_t maxFreq);
uint32_t READER_UTILS_ComputeBWTEtu(uint32_t BWI, uint32_t f);
uint32_t READER_UTILS_ComputeBWTMili(uint32_t BWTEtu, uint32_t F, uint32_t D, uint32_t f);
uint32_t READER_UTILS_ComputeEtuMili(uint32_t F, uint32_t D, uint32_t f);
float READER_UTILS_ComputeEtuMiliFloat(uint32_t F, uint32_t D, uint32_t f);
uint32_t READER_UTILS_Pow(uint32_t a, uint32_t b);

#ifdef __cplusplus
}
#endif

#endif
