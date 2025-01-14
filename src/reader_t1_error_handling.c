/**
 * \file reader_t1_error_handling.c
 * \copyright This file is part of the Open-ISO7816-Stack project and is distributed under the MIT license. See LICENSE file in the root directory. 
 * The code here defines the behaviour of the state machine when some error happens.
 */


#include "reader_t1_error_handling.h"
#include "reader_hal.h"
#include "reader_hal_comm_settings.h"



READER_Status READER_T1_ERR_IncRepeatCounter(READER_T1_ContextHandler *pContext){
	if(pContext->repeatCounter < READER_T1_MAX_REAPEAT){
		pContext->repeatCounter = pContext->repeatCounter + 1;
	}
	else{
		return READER_ERR;
	}
	
	return READER_OK;
}


READER_Status READER_T1_ERR_ClearRepeatCounter(READER_T1_ContextHandler *pContext){
	pContext->repeatCounter = 0;
	
	return READER_OK;
}


READER_Status READER_T1_ERR_CheckRepeatCounter(READER_T1_ContextHandler *pContext){
	if(pContext->repeatCounter < READER_T1_MAX_REAPEAT){
		return READER_OK;
	}
	else{
		return READER_NO;
	}
}


READER_Status READER_T1_ERR_IncResynchCounter(READER_T1_ContextHandler *pContext){
	if(pContext->resynchCounter < READER_T1_MAX_RESYNCH){
		pContext->resynchCounter = pContext->resynchCounter + 1;
	}
	else{
		return READER_ERR;
	}
	
	return READER_OK;
}


READER_Status READER_T1_ERR_ClearResynchCounter(READER_T1_ContextHandler *pContext){
	pContext->resynchCounter = 0;
	
	return READER_OK;
}



READER_Status READER_T1_ERR_CheckResynchCounter(READER_T1_ContextHandler *pContext){
	if(pContext->resynchCounter < READER_T1_MAX_RESYNCH){
		return READER_OK;
	}
	else{
		return READER_NO;
	}
}


READER_Status READER_T1_ERR_IsItFirstReception(READER_T1_ContextHandler *pContext){
	uint32_t deviceCompleteSeqNum, cardCompleteSeqNum;
	READER_Status retVal;
	
	/* On recupere les numeros de sequence cote carte et cote lecteur */
	retVal = READER_T1_CONTEXT_GetCardCompleteSeqNum(pContext, &cardCompleteSeqNum);
	if(retVal != READER_OK) return retVal;
	
	retVal = READER_T1_CONTEXT_GetDeviceCompleteSeqNum(pContext, &deviceCompleteSeqNum);
	if(retVal != READER_OK) return retVal;
	
	/* On regarde si c'est la premiere reception */
	//if(((cardCompleteSeqNum == 0) || (cardCompleteSeqNum == 1)) && ((deviceCompleteSeqNum == 0) || (deviceCompleteSeqNum == 1))){
	if(cardCompleteSeqNum == 0){
		return READER_OK;
	}
	else{
		return READER_NO;
	}
}


/* Juste un wraper pour READER_T1_FORGE_ErrorBlock */
READER_Status READER_T1_ERR_ForgeErrorBlock(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlockDest, uint32_t integrityFlag){
	READER_Status retVal;
	
	
	retVal = READER_T1_FORGE_ErrorBlock(pContext, pBlockDest, integrityFlag);
	if(retVal != READER_OK) return retVal;
	
	return READER_OK;
}


READER_Status READER_T1_ERR_DealWithError(READER_T1_ContextHandler *pContext, uint32_t integrityFlag){
	READER_Status retVal;
	READER_T1_Block errorBlock;
	

	/* On verifie qu'on a pas depasse le compteur de redemande d'informations */
	retVal = READER_T1_ERR_CheckRepeatCounter(pContext);
	if(retVal == READER_OK){
		/* On forge un Block d'erreur */
		retVal = READER_T1_ERR_ForgeErrorBlock(pContext, &errorBlock, integrityFlag);
		if(retVal != READER_OK) return retVal;
		
		/* On stack le Block d'erreur dans le Buffer */
		retVal = READER_T1_ERR_StackErrorBlock(pContext, &errorBlock);
		if(retVal != READER_OK) return retVal;
		
		/* On incremente le compteur de redemande d'informations */
		retVal = READER_T1_ERR_IncRepeatCounter(pContext);
		if(retVal != READER_OK) return retVal;
	}
	else if(retVal == READER_NO){

		/* On verifie le compteur de Resynchro */
		retVal = READER_T1_ERR_CheckResynchCounter(pContext);
		if(retVal == READER_OK){
			/* On fait une demande de Resynchro */
			retVal = READER_T1_ERR_DoResynch(pContext);
			if(retVal != READER_OK) return retVal;
		}
		else if(retVal == READER_NO){
			/* On procede a une reinitialisation de la carte */
			retVal = READER_T1_ERR_DoReset(pContext);
			if(retVal != READER_OK) return retVal;
		}
		else{
			return retVal;
		}
	}
	else{
		return retVal;
	}
	
	return READER_OK;
}


/* On fait des verifications et on envoie de requette de resynchro, on mets a jour les flags ...  */
/* Attention, on retourne READER_NO si on a pas pu faire de Resynch (compteur max atteint), c'est le contexte exterieur a la fonction qui doit effectuer le DoReset() */
READER_Status READER_T1_ERR_PrepareResynchRequ(READER_T1_ContextHandler *pContext){
	READER_Status retVal;
	READER_T1_Block block;
	
	
	/* On verifie le compteur de Resynch                                                    */
	retVal = READER_T1_ERR_CheckResynchCounter(pContext);
	if(retVal != READER_OK) return retVal;
	
	/* On forge un Block de requette de Resynchro                                           */
	retVal = READER_T1_FORGE_SBlock(pContext, &block, READER_T1_STYPE_RESYNCH_REQU);
	if(retVal != READER_OK) return retVal;
	
	/* On Stack le S-Block ainsi forge dans le Buffer d'envoi                               */
	retVal = READER_T1_BUFFER_Stack(pContext, &block);
	if(retVal != READER_OK) return retVal;
	
	/* On positionne les flags qui indiquent qu'on attend un S-Block en retour ...          */
	retVal = READER_T1_CONTEXT_SetSBlockExpectedResponse(pContext, READER_T1_STYPE_RESYNCH_RESP);	
	if(retVal != READER_OK) return retVal;
		
	/* On incremente le compteur de demandes de Resynchro                                   */
	retVal = READER_T1_ERR_IncResynchCounter(pContext);
	if(retVal != READER_OK) return retVal;
	
	return READER_OK;
}


/* On applique la resynchro sur le contexte de communcation                                    */
READER_Status READER_T1_ERR_DoResynch(READER_T1_ContextHandler *pContext){
	READER_T1_Block *pLastBlock;
	READER_T1_Block tmpBlock;
	READER_T1_BlockType bType = READER_T1_BLOCK_ERR;	// fixed uninitialized variable warning
	READER_Status retVal, retVal2;
	uint8_t rawDataBuff[READER_T1_BUFFER_MAXBYTES];
	uint32_t saveResynchCounter;
	uint32_t nbExtractedBytes;
	
	
	/* On mets de cote la valeur du compteur de resynchro ...  */
	retVal = READER_T1_CONTEXT_GetResynchCounter(pContext, &saveResynchCounter);
	if(retVal != READER_OK) return retVal;
	
	/* On recupere un pointeur sur le dernier Block envoye                                               */
	retVal2 = READER_T1_CONTEXT_GetLastSent(pContext, &pLastBlock);
	if((retVal2 != READER_OK) && (retVal2 != READER_DOESNT_EXIST)) return retVal;
	
	/* Si le dernier Block envoye existe ...  */
	if(retVal2 != READER_DOESNT_EXIST){
		bType = READER_T1_GetBlockType(pLastBlock);
		
		if(bType == READER_T1_IBLOCK){
			retVal = READER_T1_BUFFER_Stack(pContext, pLastBlock);
			if(retVal != READER_OK) return retVal;
		}
		else if((bType == READER_T1_RBLOCK) || (bType == READER_T1_SBLOCK)){
			retVal = READER_T1_CopyBlock(&tmpBlock, pLastBlock);
			if(retVal != READER_OK) return retVal;
		}
		else{
			return READER_ERR;
		}
	}
	
	retVal = READER_T1_BUFFER_ExtractRawDataFromBuffer(pContext, rawDataBuff, READER_T1_BUFFER_MAXBYTES, &nbExtractedBytes);
	if(retVal != READER_OK) return retVal;
	
	
	/* La spec n'indique pas precisement ce qu'il faut reinitialiser lors du RESYNCH.             */
	retVal = READER_T1_CONTEXT_InitContextSettings(pContext);
	if(retVal != READER_OK) return retVal;
	
	retVal = READER_T1_CONTEXT_InitSeqNums(pContext);
	if(retVal != READER_OK) return retVal;
	
	/* On re-remplit le BUFFER de Blocks avec les bons niuveaux nureros de sequence suite au reinit ...  */
	retVal = READER_T1_FORGE_SliceDataAndFillBuffer(pContext, rawDataBuff, nbExtractedBytes);
	if(retVal != READER_OK) return retVal;
	
	
	
	if((retVal != READER_DOESNT_EXIST) && (bType != READER_T1_IBLOCK)){
		retVal = READER_T1_BUFFER_Stack(pContext, &tmpBlock);
		if(retVal != READER_OK) return retVal;
	}
	
	
	/* On remets le compteur de resynch dans le contexte de communication ...  */
	retVal = READER_T1_CONTEXT_SetResynchCounter(pContext, saveResynchCounter);
	if(retVal != READER_OK) return retVal;
 	
	
	return READER_OK;
}


READER_Status READER_T1_ERR_DoReset(READER_T1_ContextHandler *pContext){
	READER_HAL_CommSettings *pSettings;
	READER_Status retVal;
	
	
	/* On recupere un pointeur sur les parametes actuels de communication bas niveau ...  */
	retVal = READER_T1_CONTEXT_GetHalCommSettingsPtr(pContext, &pSettings);
	if(retVal != READER_OK) return retVal;
	
	
	retVal = READER_T1_CONTEXT_Init(pContext, pSettings);
	if(retVal != READER_OK) return retVal;
	
	retVal = READER_HAL_InitWithDefaults(pSettings);
	if(retVal != READER_OK) return retVal;
	
	retVal = READER_HAL_DoColdReset();
	if(retVal != READER_OK) return retVal;
	
	return READER_OK;
}


READER_Status READER_T1_ERR_StackErrorBlock(READER_T1_ContextHandler *pContext, READER_T1_Block *pErrorBlock){
	READER_Status retVal;
	
	
	retVal = READER_T1_BUFFER_Stack(pContext, pErrorBlock);
	if(retVal != READER_OK) return retVal;

	return READER_OK;
}
