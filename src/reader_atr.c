/**
 * \file reader_atr.c
 * \copyright This file is part of the Open-ISO7816-Stack project and is distributed under the MIT license. See LICENSE file in the root directory. 
 * This file contains functions for dealing with ATR (Answer-To-Reset).
 */

#include "reader_atr.h"
#include "reader_hal.h"

#ifdef CY_TARGET_BOARD
// PSoC Edge and PSoC6
#include "feature_config.h"
#include <stdbool.h>
#include "ifx_debug.h"

static const bool s_verboseLogging = true;
#endif

/* Voir table 7 section 8.3 ISO7816_3 */
static uint16_t globalFiConvTable[0x10] = {372, 372, 558, 744, 1116, 1488, 1860, 372, 372, 512, 768, 1024, 1536, 2048, 372, 372};
static uint8_t  globalDiConvTable[0x10] = {1, 1, 2, 4, 8, 16, 32, 64, 12, 20, 1, 1, 1, 1, 1, 1};
static uint32_t globalFMaxConvTable[0x10] = {4000000, 5000000, 6000000, 8000000, 12000000, 16000000, 20000000, 0xFFFFFFFF, 0xFFFFFFFF, 5000000, 7500000, 10000000, 15000000, 20000000, 0xFFFFFFFF, 0xFFFFFFFF};

READER_Status READER_ATR_Receive(READER_ATR_Atr *atr, READER_HAL_CommSettings *pSettings){
	READER_Status retVal;
	uint32_t j, i = 1;
	uint8_t TS, T0, TA, TB, TC, TD;
	uint8_t Y, T = 0;
	uint8_t checkByte;
	uint8_t rcvdBytes[READER_ATR_MAX_SIZE];                     /* l'ATR fait au max 32 octets, voir ISO7816-3 section 8.2.1. */
	uint32_t rcvdCount = 0;
	uint8_t byte;
	
	
	/* Initialisation des certains elements de la structure ATR */
	retVal = READER_ATR_InitStruct(atr);
	if(retVal != READER_OK) return retVal;
		
	
	/* Recuperation de TS */
	retVal = READER_HAL_RcvChar(pSettings, READER_HAL_PROTOCOL_ANY, &TS, READER_ATR_DEFAULT_TIMEOUT);
	if (s_verboseLogging) {
		DEBUG_PRINT(("TS=%02X\n", TS));
    }
	if(retVal != READER_OK) return retVal;

	retVal = READER_ATR_CheckTS(TS);
	if(retVal != READER_OK) return retVal;
	atr->encodingConv = READER_ATR_GetEncoding(TS);
	
	/* Recuperation de T0 */
	retVal = READER_HAL_RcvChar(pSettings, READER_HAL_PROTOCOL_ANY, &T0, READER_ATR_DEFAULT_TIMEOUT);
	if (s_verboseLogging) {
		DEBUG_PRINT(("T0=%02X\n", T0));
	}
	if(retVal != READER_OK) return retVal;
	atr->K = READER_ATR_ComputeK(T0);
	retVal = READER_ATR_AddRcvdByte(T0, rcvdBytes, &rcvdCount);
	if(retVal != READER_OK) return retVal;
	
	
	Y = READER_ATR_ComputeY(T0);
	
	/* Recupertion de tous les Interfaces Bytes */
	while(READER_ATR_IsInterfacesBytesToRead(Y)){
		if(READER_ATR_IsTAToRead(Y)){
			retVal = READER_HAL_RcvChar(pSettings, READER_HAL_PROTOCOL_ANY, &TA, READER_ATR_DEFAULT_TIMEOUT);
			if (s_verboseLogging) {
				DEBUG_PRINT(("TA%lu=%02X\n", i, TA));
			}
			if (retVal != READER_OK) return READER_ERR;
			retVal = READER_ATR_ProcessTA(atr, TA, i, T);
            if (retVal != READER_OK) return READER_ERR;
			retVal = READER_ATR_AddRcvdByte(TA, rcvdBytes, &rcvdCount);
            if (retVal != READER_OK) return READER_ERR;
		}
		if(READER_ATR_IsTBToRead(Y)){
			retVal = READER_HAL_RcvChar(pSettings, READER_HAL_PROTOCOL_ANY, &TB, READER_ATR_DEFAULT_TIMEOUT);
			if (s_verboseLogging) {
				DEBUG_PRINT(("TB%lu=%02X\n", i, TB));
			}
            if (retVal != READER_OK) return READER_ERR;
			retVal = READER_ATR_ProcessTB(atr, TB, i, T);
            if (retVal != READER_OK) return READER_ERR;
			retVal = READER_ATR_AddRcvdByte(TB, rcvdBytes, &rcvdCount);
            if (retVal != READER_OK) return READER_ERR;
		}
		if(READER_ATR_IsTCToRead(Y)){
			retVal = READER_HAL_RcvChar(pSettings, READER_HAL_PROTOCOL_ANY, &TC, READER_ATR_DEFAULT_TIMEOUT);
			if (s_verboseLogging) {
				DEBUG_PRINT(("TC%lu=%02X\n", i, TC));
			}
            if (retVal != READER_OK) return READER_ERR;
			retVal = READER_ATR_ProcessTC(atr, TC, i, T);
            if (retVal != READER_OK) return READER_ERR;
			retVal = READER_ATR_AddRcvdByte(TC, rcvdBytes, &rcvdCount);
            if (retVal != READER_OK) return READER_ERR;
		}
		if(READER_ATR_IsTDToRead(Y)){
			retVal = READER_HAL_RcvChar(pSettings, READER_HAL_PROTOCOL_ANY, &TD, READER_ATR_DEFAULT_TIMEOUT);
            if (retVal != READER_OK) return READER_ERR;
			Y = READER_ATR_ComputeY(TD);
			T = READER_ATR_ComputeT(TD);
			if (s_verboseLogging) {
				if (i == 1) {
					DEBUG_PRINT(("TD%lu=%02X <-- T=%u\n", i, TD, T));
				}
				else {
					DEBUG_PRINT(("TD%lu=%02X\n", i, TD));
				}
			}
			READER_ATR_ProcessT(atr, T);
			retVal = READER_ATR_AddRcvdByte(TD, rcvdBytes, &rcvdCount);
            if (retVal != READER_OK) return READER_ERR;
		}
		else{
			Y = 0x00;
		}
		i++;
	}
	
	/* Recuperation de tous les Historical Bytes */
	for(j=0; j<atr->K; j++){
		retVal = READER_HAL_RcvChar(pSettings, READER_HAL_PROTOCOL_ANY, &byte, READER_ATR_DEFAULT_TIMEOUT);
		if(retVal != READER_OK) return retVal;
		
		atr->histBytes[j] = byte;
		
		retVal = READER_ATR_AddRcvdByte(byte, rcvdBytes, &rcvdCount);
		if(retVal != READER_OK) return retVal;
	}
	
	if (s_verboseLogging) {
	    DEBUG_PRINT(("HistoricalBytes: "));
	    for (j=0; j<atr->K; j++) {
	        DEBUG_PRINT(("%02X ", atr->histBytes[j]));
	    }
	    DEBUG_PRINT(("\n"));
	}

	/* Recuperation du Check Byte */
	/* La presence du check byte n'est pas systematique, voir ISO7816-3 section 8.2.5. */
	if(!(READER_ATR_IsT0(atr) && !READER_ATR_IsT15(atr))){
		retVal = READER_HAL_RcvChar(pSettings, READER_HAL_PROTOCOL_ANY, &checkByte, READER_ATR_DEFAULT_TIMEOUT);
		if (s_verboseLogging) {
			DEBUG_PRINT(("TCK=%02X\n", checkByte));
		}
		if(retVal != READER_OK) return retVal;	
		
		/* Verification des caracteres recus avec le TCK */
		retVal = READER_ATR_CheckTCK(rcvdBytes, rcvdCount, checkByte);
		if(retVal != READER_OK) return retVal;
	}
	
	return READER_OK;
}


/**
 * \fn READER_Status READER_ATR_ApplySettings(READER_ATR_Atr *atr)
 * \brief Cette fonction permet d'appliquer les parametres de communication indiqués dans l'ATR.
 * \return Retourne uen code d'erreur de stype READER_Status. Retourne READER_OK si l'exécution s'est correctement déroulée. Pour toute autre valeur il s'agit d'une erreur.
 * \param *atr Pointeur sur une structure de type READER_ATR_Atr;
 */
 READER_Status READER_ATR_ApplySettings(READER_ATR_Atr *atr){
	 return READER_OK;
 }


READER_Status READER_ATR_InitStruct(READER_ATR_Atr *atr){
	uint32_t j;
	
	atr->Fi = READER_ATR_VALUE_NOT_INDICATED;
	atr->Di = READER_ATR_VALUE_NOT_INDICATED;
	atr->fMax = READER_ATR_VALUE_NOT_INDICATED;
	atr->N = 0;
	
	atr->T0Protocol.TABytesCount = 0;
	atr->T0Protocol.TBBytesCount = 0;
	atr->T0Protocol.TCBytesCount = 0;
	atr->T1Protocol.TABytesCount = 0;
	atr->T1Protocol.TBBytesCount = 0;
	atr->T1Protocol.TCBytesCount = 0;
	
	atr->isT1Indicated = READER_ATR_NOT_INDICATED;
	atr->isT0Indicated = READER_ATR_NOT_INDICATED;
	atr->isT15Indicated = READER_ATR_NOT_INDICATED;
	
	atr->clockStopIndicator      = READER_ATR_CLOCKSTOP_NOTINDICATED;
	atr->classIndicator          = READER_ATR_CLASS_NOTINDICATED;
	atr->useOfSPU                = READER_ATR_SPU_NOTINDICATED;
	atr->K                       = 0;
	
	
	for(j=0; j<READER_ATR_MAX_SPECIFIC_BYTES; j++){
		atr->T0Protocol.TABytes[j] = 0x00;
	}
	for(j=0; j<READER_ATR_MAX_SPECIFIC_BYTES; j++){
		atr->T0Protocol.TBBytes[j] = 0x00;
	}
	for(j=0; j<READER_ATR_MAX_SPECIFIC_BYTES; j++){
		atr->T0Protocol.TCBytes[j] = 0x00;
	}
	
	
	for(j=0; j<READER_ATR_MAX_SPECIFIC_BYTES; j++){
		atr->T1Protocol.TABytes[j] = 0x00;
	}
	for(j=0; j<READER_ATR_MAX_SPECIFIC_BYTES; j++){
		atr->T1Protocol.TBBytes[j] = 0x00;
	}
	for(j=0; j<READER_ATR_MAX_SPECIFIC_BYTES; j++){
		atr->T1Protocol.TCBytes[j] = 0x00;
	}
	
	for(j=0; j<READER_ATR_MAX_HIST_BYTES; j++){
		atr->histBytes[j] = 0x00;
	}
	
	atr->TA1 = READER_ATR_VALUE_NOT_INDICATED;
	atr->T0Protocol_WI   = READER_ATR_VALUE_INVALID;
	atr->T1Protocol_IFSC = READER_ATR_VALUE_INVALID;
	atr->T1Protocol_BWI  = READER_ATR_VALUE_INVALID;
	atr->T1Protocol_CWI  = READER_ATR_VALUE_INVALID;
	atr->T1Protocol_RedundancyType = READER_ATR_VALUE_INVALID;

	return READER_OK;
}

READER_Status READER_ATR_IsInterfacesBytesToRead(uint8_t Y){
	if(Y != 0){
		return READER_OK;
	}
	else{
		return READER_NO;
	}
}

READER_Status READER_ATR_IsTAToRead(uint8_t Y){
	if(0x0001 & Y){
		return READER_OK;
	}
	else{
		return READER_NO;
	}
}

READER_Status READER_ATR_IsTBToRead(uint8_t Y){
	if(0x0002 & Y){
		return READER_OK;
	}
	else{
		return READER_NO;
	}
}

READER_Status READER_ATR_IsTCToRead(uint8_t Y){
	if(0x0004 & Y){
		return READER_OK;
	}
	else{
		return READER_NO;
	}
}

READER_Status READER_ATR_IsTDToRead(uint8_t Y){
	if(0x0008 & Y){
		return READER_OK;
	}
	else{
		return READER_NO;
	}
}

uint8_t READER_ATR_ComputeY(uint8_t TD){
	return (TD >> 4) & 0x0F;
}

uint8_t READER_ATR_ComputeT(uint8_t TD){
	return TD & 0x0F;
}

uint8_t READER_ATR_ComputeK(uint8_t T0){
	return T0 & 0x0F;
}


uint32_t READER_ATR_ComputeFi(uint8_t TA1){
	uint8_t k;
	
	k = (TA1 >> 4) & 0x0F;
	return (uint32_t)(globalFiConvTable[k]);
}

uint32_t READER_ATR_ComputeFMax(uint8_t TA1){
	uint8_t k;
	
	k = (TA1 >> 4) & 0x0F;
	return (uint32_t)(globalFMaxConvTable[k]);
}

uint32_t READER_ATR_ComputeDi(uint8_t TA1){
	uint8_t k;
	
	k = TA1 & 0x0F;
	return (uint32_t)(globalDiConvTable[k]);
}

READER_ATR_ClockStopIndicator READER_ATR_GetClockStopIndic(uint8_t TA15){
	return (TA15 >> 6) & 0x03;
}

READER_ATR_ClassIndicator READER_ATR_GetClassIndic(uint8_t TA15){
	return  TA15 & 0x40;
}

READER_ATR_UseOfSPU READER_ATR_GetUseSPU(uint8_t TB15){
	if(TB15 == 0x00){
		return READER_ATR_SPU_NOTUSED;
	}
	else if((TB15 & 0x80) == 0){
		return READER_ATR_SPU_PROPRIETARY;
	}
	else{
		return READER_ATR_SPU_STANDARD;
	}
}

void READER_ATR_ErrHandler(void){
	while(1){
		
	}
}

READER_Status READER_ATR_CheckTS(uint8_t TS){
	if(TS == READER_ATR_ENCODING_DIRECT){
		return READER_OK;
	}
	else if(TS == READER_ATR_ENCODING_REVERSE){
		return READER_OK;
	}
	else{
		return READER_ERR;
	}
}

READER_ATR_EncodingConv READER_ATR_GetEncoding(uint8_t TS){
	if(TS == READER_ATR_ENCODING_DIRECT){
		return READER_ATR_ENCODING_DIRECT;
	}
	else{
		return READER_ATR_ENCODING_REVERSE;
	}
}

READER_Status READER_ATR_IsT0(READER_ATR_Atr *atr){
	if(atr->isT0Indicated == READER_ATR_INDICATED){
		return READER_OK;
	}
	else{
		return READER_NO;
	}
}

READER_Status READER_ATR_IsT1(READER_ATR_Atr *atr){
	if(atr->isT1Indicated == READER_ATR_INDICATED){
		return READER_OK;
	}
	else{
		return READER_NO;
	}
}

READER_Status READER_ATR_IsT15(READER_ATR_Atr *atr){
	if(atr->isT15Indicated == READER_ATR_INDICATED){
		return READER_OK;
	}
	else{
		return READER_NO;
	}
}

/**
 * \fn READER_Status READER_ATR_ProcessT(READER_ATR_Atr *atr, uint8_t T)
 * \brief Cette fonction sert a mettre à jour la structure ATR en indiquant les protocoles (T=0 ou T=1) qui sont pris en charge par la carte.
 * \return La fonction retourne READER_OK si la fonction d'est exécutée correctement. Une autre valeur dans le cas contraire.
 * \param *atr est un pointeur vers une structure ATR. Elle sera mise à jour avec les informations du paramètre T, selon la prise en charge ou non du protocole.
 * \param T est le champs "T" des historical bytes TD. Voir ISO7816-3 section 8.2.3.
 */
READER_Status READER_ATR_ProcessT(READER_ATR_Atr *atr, uint8_t T){
	if(T==0){
		atr->isT0Indicated = READER_ATR_INDICATED;
	}
	
	if(T==1){
		atr->isT1Indicated = READER_ATR_INDICATED;
	}
	
	if(T==15){
		atr->isT15Indicated = READER_ATR_INDICATED;
	}
	
	return READER_OK;
}

READER_Status READER_ATR_ProcessTA(READER_ATR_Atr *atr, uint8_t TA, uint32_t i, uint8_t T){	
	if(i == 1){          /* Global interface Byte */
		atr->Fi = READER_ATR_ComputeFi(TA);
		atr->Di = READER_ATR_ComputeDi(TA);
		atr->fMax = READER_ATR_ComputeFMax(TA);
		atr->TA1 = TA;
	}
	else if(T == 15){    /* Global Interface Byte */
		atr->clockStopIndicator = READER_ATR_GetClockStopIndic(TA);
		atr->classIndicator = READER_ATR_GetClassIndic(TA);
	}
	else if(T == 0){
		atr->T0Protocol.TABytes[atr->T0Protocol.TABytesCount] = TA;
		atr->T0Protocol.TABytesCount++;
	}
	else if(T == 1){
		atr->T1Protocol.TABytes[atr->T1Protocol.TABytesCount] = TA;
		atr->T1Protocol.TABytesCount++;

		if(i == 3){ // T=1 Protocol: TA3 is IFSC byte
			atr->T1Protocol_IFSC = TA;
		}
	}
	else{
		return READER_ERR;
	}
	
	return READER_OK;
}

READER_Status READER_ATR_ProcessTB(READER_ATR_Atr *atr, uint8_t TB, uint32_t i, uint8_t T){
	if((i == 1) || (i == 2)){
		return READER_OK;        /* TB1 et TB2 sont deprecated voir section 8.3  ISO7816_3. Ils doivent etre ignores */
	}
	else if(T == 15){	/* Global Interface Byte */
		atr->useOfSPU = READER_ATR_GetUseSPU(TB);
	}
	else if(T == 0){
		atr->T0Protocol.TBBytes[atr->T0Protocol.TBBytesCount] = TB;
		atr->T0Protocol.TBBytesCount++;
	}
	else if(T == 1){
		atr->T1Protocol.TBBytes[atr->T1Protocol.TBBytesCount] = TB;
		atr->T1Protocol.TBBytesCount++;

		if(i == 3){ // T=1 Protocol: TB3 is 'BWI and CWI' byte
			atr->T1Protocol_BWI = (TB & 0xF0) >> 4;	// upper nibble
			atr->T1Protocol_CWI = (TB & 0x0F);		// lower nibble
		}
	}
	else{
		return READER_ERR;
	}
	
	return READER_OK;
}

READER_Status READER_ATR_ProcessTC(READER_ATR_Atr *atr, uint8_t TC, uint32_t i, uint8_t T){
	if(i == 1){
		atr->N = TC;  /* Extra guardtime */
	}
	else if(T == 0){
		atr->T0Protocol.TCBytes[atr->T0Protocol.TCBytesCount] = TC;
		atr->T0Protocol.TCBytesCount++;

		if(i == 2){ // T=0 Protocol: TC2 is 'Waiting Time Integer' byte
			atr->T0Protocol_WI = TC;
		}
	}
	else if(T == 1){
		atr->T1Protocol.TCBytes[atr->T1Protocol.TCBytesCount] = TC;
		atr->T1Protocol.TCBytesCount++;

		if(i == 3){ // T=1 Protocol: TC3 is 'Redundancy Type' byte
			atr->T1Protocol_RedundancyType = TC;
		}
	}
	else{
		return READER_ERR;
	}
	
	return READER_OK;
}

/**
 * \fn READER_Status READER_ATR_CheckTCK(uint8_t *rcvdBytesList, uint32_t rcvdBytesCount, uint8_t TCK)
 * \brief Cette fonction permet d'effectuer le test d'intégrité des données reçues dans l'ATR. Il s'agit du test décrit dans l'ISO7816-3 section 8.2.5.
 * \param *rcvdBytesList est un pointeur sur un buffer qui contient tous les caractères qui ont été reçus dans l'ATR (à partir de T0).
 * \param rcvdBytesCount indique le nombre de caractères qui ont été reçus dans l'ATR.
 * \param TCK est le check byte reçu en fin d'ATR.
 * \return Valeur de retour de type READER_Status. READER_OK indique le bon déroulement de la fonction. Toute autre valeur indique une erreur.
 */
READER_Status READER_ATR_CheckTCK(uint8_t *rcvdBytesList, uint32_t rcvdBytesCount, uint8_t TCK){
	uint32_t i;
	uint8_t totalXor;
	
	totalXor = TCK;
	
	for(i=0; i<rcvdBytesCount; i++){
		totalXor = rcvdBytesList[i] ^ totalXor;
	}
	
	/* Voir ISO7816-3 section 8.2.5 pour explication du test */
	if(totalXor == 0x00){
		return READER_OK;
	}
	else{
		return READER_ERR;
	}
}

/**
 * \fn READER_Status READER_ATR_AddRcvdByte(uint8_t byte, uint8_t *byteList, uint32_t *byteCount)
 * \brief Cette fonction est utile en interne de la fonction READER_ATR_Receive(). Elle sert a tenir à jour une liste de tous les octets recus pendant l'ATR. Cette liste d'octets est utile pour le calcul du TCK.
 *        La fonction prend en paramètre l'octet reçu, l'ajoute à la liste des octets reçus et incrémente un compteur.
 * \param byte est l'octet reçu, que 'on veut ajouter à la liste de tous les octets reçus.
 * \param *byteList est un pointeur sur la liste des octets recus pendant l'ATR. On veut ajouter l'octet à cette liste.
 * \param *byteCount est un pointeur sur le compteur qui compte le nombre d'octets reçus durant l'ATR. Cette fonction mets à jour ce compteur, elle l'incrémente de 1.
 * \return Valeur de retour de type READER_Status. READER_OK indique le bon déroulement de la fonction. Toute autre valeur indique une erreur.
 */
READER_Status READER_ATR_AddRcvdByte(uint8_t byte, uint8_t *byteList, uint32_t *byteCount){
	if(*byteCount >= READER_ATR_MAX_SIZE){
		return READER_ERR;
	}
	
	byteList[*byteCount] = byte;
	(*byteCount)++;
	
	return READER_OK;
}

READER_Status READER_ATR_GetT0WI(READER_ATR_Atr *atr, uint8_t *wi){
	READER_Status retVal = READER_ERR;

	if (atr->T0Protocol_WI != READER_ATR_VALUE_INVALID) {
		*wi = atr->T0Protocol_WI;
		retVal = READER_OK;
	}
	return retVal;
}

READER_Status READER_ATR_GetT1BWI(READER_ATR_Atr *atr, uint8_t *bwi){
	READER_Status retVal = READER_ERR;

	if (atr->T1Protocol_BWI != READER_ATR_VALUE_INVALID) {
		*bwi = atr->T1Protocol_BWI;
		retVal = READER_OK;
	}
	return retVal;
}

READER_Status READER_ATR_GetT1CWI(READER_ATR_Atr *atr, uint8_t *cwi){
	READER_Status retVal = READER_ERR;

	if (atr->T1Protocol_CWI != READER_ATR_VALUE_INVALID) {
		*cwi = atr->T1Protocol_CWI;
		retVal = READER_OK;
	}
	return retVal;
}

READER_Status READER_ATR_GetT1RedundancyType(READER_ATR_Atr *atr, uint8_t *type){
	READER_Status retVal = READER_ERR;

	if (atr->T1Protocol_RedundancyType != READER_ATR_VALUE_INVALID) {
		*type = atr->T1Protocol_RedundancyType;
		retVal = READER_OK;
	}
	return retVal;
}

READER_Status READER_ATR_GetT1IFSC(READER_ATR_Atr *atr, uint8_t *ifsc){
	READER_Status retVal = READER_ERR;

	if (atr->T1Protocol_IFSC != READER_ATR_VALUE_INVALID) {
		*ifsc = atr->T1Protocol_IFSC;
		retVal = READER_OK;
	}
	return retVal;
}
