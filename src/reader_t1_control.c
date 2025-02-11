/**
 * \file reader_t1_control.c
 * \copyright This file is part of the Open-ISO7816-Stack project and is distributed under the MIT license. See LICENSE file in the root directory. 
 * Functions defining the behaviour of the protocol state machine when sending/receiving a T=1 Block.
 * Most important functions here are READER_T1_CONTROL_SendBlock() and READER_T1_CONTROL_RcvBlock().
 */


#include "reader_t1_control.h"


/* Permet de faire des verifications elementaires sur un R-Block ...       */
/* Retourne  READER_NO si e n'est pas un R-Block valide. READER_OK sinon. */
READER_Status READER_T1_CONTROL_CheckRBlockIsValid(READER_T1_Block *pBlock){
	READER_T1_BlockType bType;
	
	
	/* On verifie que le Block passe en parametre existe ... */
	if(pBlock == NULL){
		return READER_NO;
	}
	
	/* On verifie qu'il est bien du type R-Block ...         */
	bType = READER_T1_GetBlockType(pBlock);
	if(bType != READER_T1_RBLOCK){
		return READER_NO;
	}
	
	return READER_OK;
}


/* Retourne READER_NO si le numero de sequence c'est pas Valide. Retourne READER_ERR en cas d'erreur interne. */
READER_Status READER_T1_CONTROL_IsSeqNumValid(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock){
	READER_Status retVal;
	READER_T1_BlockType bType;
	READER_T1_SeqNumber tmpSeqNum;
	uint32_t expectedSeqNum, blockSeqNum;
	
	
	/* On fait des verifications preliminaires ...                                      */
	if(pBlock == NULL){
		return READER_ERR;
	}
	
	bType = READER_T1_GetBlockType(pBlock);	
	if(bType != READER_T1_IBLOCK){
		return READER_ERR;
	}
	
	/* On recupere le numero de sequence de l'on attend en provenance de la carte ...   */
	retVal = READER_T1_CONTEXT_GetCardExpectedSeqNum(pContext, &expectedSeqNum);
	if(retVal != READER_OK) return retVal;
	
	/* On recupere le numero de sequence du Block que l'on veut verifier ...            */
	tmpSeqNum = READER_T1_GetBlockSeqNumber(pBlock);
	if(tmpSeqNum == READER_T1_SEQNUM_ONE){
		blockSeqNum = 1;
	}
	else if(tmpSeqNum == READER_T1_SEQNUM_ZERO){
		blockSeqNum = 0;
	}
	else{
		return READER_ERR;
	}
	
	/* On compare les numeros de sequence ...                                           */
	if(expectedSeqNum == blockSeqNum){
		return READER_OK;
	}
	else{
		return READER_NO;
	}
}


/* Retourne READER_OK si OK, READER_NO si c'est pas un ACK et READER_ERR en cas d'erreur               */
READER_Status READER_T1_CONTROL_IsRBlockACK(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock){
	READER_Status retVal;
	READER_T1_ChainingStatus chainingStatus;
	READER_T1_ExpSeqNumber blockSeqNum;
	READER_T1_ACKType ACKType;
	uint32_t tmpNextIBlockSeqNum, tmpBlockSeqNum;
	
	/* On procede de la maniere suivante (non detaille dans la spec) :                                             */
	/* Du cote du Device, un R-Block recu est considere comme un ACK si et seulement si (1) et (2) :               */
	/*      (1) Le Device est en train de chainer le dernier I-Block  (M-Bit du dernier == 1)                      */
	/*      (2) Le numero de sequence contenu dans le R-Block correspond au I-Block suivant que l'on veut envoyer  */
	/*      (3) Le R-Block n'indique pas une erreur voir ISO7816-3 section 11.3.2.2                                */
	/*      (?) Le I-Block suivant existe                                                                          */
	
	
	/* On verifie qu'on a bien un R-Block                                                                         */
	retVal = READER_T1_CONTROL_CheckRBlockIsValid(pBlock);
	if(retVal != READER_OK) return retVal;
	
	/* On verifie que cote Device on est bien en train d'essayer de chainer le dernier I-Block qu'on a envoye     */
	retVal = READER_T1_CONTEXT_DeviceIsChainingLastBlock(pContext, &chainingStatus);
	if(retVal != READER_OK) return retVal;
	
	if(chainingStatus == READER_T1_CHAINING_NO){
		return READER_NO;
	}
	
	/* On regarde le numero de sequence contenu dans le R-Block                                                   */
	blockSeqNum = READER_T1_GetExpectedBlockSeqNumber(pBlock);
	if(blockSeqNum == READER_T1_EXPSEQNUM_ZERO){
		tmpBlockSeqNum = 0;
	}
	else if(blockSeqNum == READER_T1_EXPSEQNUM_ONE){
		tmpBlockSeqNum = 1;
	}
	else{
		return READER_ERR;
	}
	
	/* On regarde le numero de sequence du prochain I-Block que l'on doit envoyer (ie : negation du dernier I-Block envoye) */
	retVal = READER_T1_CONTEXT_GetLastIBlockSentSeqNum(pContext, &tmpNextIBlockSeqNum);
	if(retVal == READER_DOESNT_EXIST) return READER_NO;            /* Si le dernier I-Block envoye par le device n'existe pas pas, alors un R-Block recu depuis la carte ne peut pas etre un ACK */
	if(retVal != READER_OK) return retVal;
	
	if(tmpNextIBlockSeqNum == 0){
		tmpNextIBlockSeqNum = 1;
	}
	else if(tmpNextIBlockSeqNum == 1){
		tmpNextIBlockSeqNum = 0;
	}
	else{
		return READER_ERR;
	}
	
	
	/* On verifie que le numero de sequence contenu dans le R-Block correspond au numero de sequence du prochain I-Block que l'on veut envoyer/chainer */
	if(tmpBlockSeqNum != tmpNextIBlockSeqNum){
		return READER_NO;
	}
	
	/* On verifie que le R-Block n'indique pas d'erreur (NACK) ...  */
	ACKType = READER_T1_GetBlockACKType(pBlock);
	if(ACKType != READER_T1_ACKTYPE_ACK){
		return READER_NO;
	}
	
	return READER_OK;
}


/* Retourne READER_OK si ce R-Block indique une erreur. READER_NO si il n'indique pas d'erreur. Autre valeur si la fonction a rencontre une erreur. */
READER_Status READER_T1_CONTROL_IsRBlockError(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock){
	READER_Status retVal;
	READER_T1_ACKType ackType;
	
	
	retVal = READER_T1_CONTROL_CheckRBlockIsValid(pBlock);
	if(retVal != READER_OK) return retVal;
	
	ackType = READER_T1_GetBlockACKType(pBlock);
	if(ackType == READER_T1_ACKTYPE_ACK){
		return READER_NO;
	}
	else if(ackType == READER_T1_ACKTYPE_NACK){
		return READER_OK;
	}
	else if(ackType == READER_T1_ACKTYPE_NACK_CRCLRC){
		return READER_OK;
	}
	else{
		return READER_ERR;
	}
}


/* Retourne READER_OK si ce I-Block indique une erreur. READER_NO si il n'indique pas d'erreur. Autre valeur si la fonction a rencontre une erreur. */
READER_Status READER_T1_CONTROL_IsIBlockACK(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock){
	READER_Status retVal;
	uint32_t seqNum1, seqNum2;
	READER_T1_SeqNumber seqNum;
	
	
	/* On procede de la maniere suivante (non detaille spec) :                                */
	/* Un I-Block recu est considere comme un ACK cote Device si et seulement si :            */
	/*      1) Le dernier Block envoye par le Device existe et est un I-Block                 */
	/*      2) Le M-Bit de ce Block est 0 (le device ne faisait pas/plus de chainage)         */
	/*      3) Le numero de sequence du Block recu est correct                                */
	
	/* Voir ISO7816-3 section 11.6.2.1 ...  */
	/* On verifie que l'on a bien un I-Block en entree ...  */
	retVal = READER_T1_CheckIBlock(pBlock);
	if(retVal != READER_OK) return retVal;
	
	/* On verifie que le dernier I-Block envoye existe (si il n'existe pas avoir ce block recu n'aquitte rien dutout) ...  */
	retVal = READER_T1_CONTEXT_LastIBlockSentExists(pContext);
	if((retVal != READER_OK) && (retVal != READER_DOESNT_EXIST)){
		return retVal;
	}
	if(retVal == READER_DOESNT_EXIST){
		return READER_NO;
	}
	
	/* On recupere le numero de sequence du dernier I-Block recu ...  */
	retVal = READER_T1_CONTEXT_GetLastIBlockRcvdSeqSum(pContext, &seqNum1);
	if((retVal != READER_OK) && (retVal != READER_DOESNT_EXIST)){
		return retVal;
	}
	if(retVal == READER_DOESNT_EXIST){
		return READER_OK;
	}
	
	/* On recupere le numero de sequence du I-Block que l'on est train de recevoir ...  */
	seqNum = READER_T1_GetBlockSeqNumber(pBlock);
	if(seqNum == READER_T1_SEQNUM_ZERO){
		seqNum2 = 0;
	}
	else if(seqNum == READER_T1_SEQNUM_ONE){
		seqNum = 1;
	}
	else{
		return READER_ERR;
	}
	
	/* On compare les numeros de sequence. Voir ISO7816-3 section 11.6.2.1 ...  */
	if(seqNum1 != seqNum2){
		return READER_OK;
	}
	else{
		return READER_NO;
	}
}


/**
 * \fn READER_T1_CONTROL_SendBlock(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlockToSend)
 * \param *pContext est un pointeur sur une structure de type READER_T1_ContextHandler. Cette structure contient le contexte de communication.
 * \param *pBlockToSend est un pointeur sur uen structure de type READER_T1_Block. Il s'agit du Block à envoyer.
 * \return La fonction retourne un code d'erreur de type READER_Status.
 * Cette fonction envoie un Block en tenant compte du contexte de communication. Elle mets également à jour le contexte de communication selon le type de Block envoyé.
 */
READER_Status READER_T1_CONTROL_SendBlock(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlockToSend){
	READER_Status retVal;
	READER_T1_BlockType bType;
	READER_HAL_CommSettings *pSettings;
	uint32_t currentCWT, currentBGT;
	uint32_t tickLastBlock, extraStartDelay, tick;
		
	
	/* On recupere un pointeur sur les parametres de communication bas niveau ...  */
	retVal = READER_T1_CONTEXT_GetHalCommSettingsPtr(pContext, &pSettings);
	if(retVal != READER_OK) return retVal;
	
	/* Avant d'envoyer le Block, on calcule le delai a appliquer pour respecter le BGT entre les Blocks */
	retVal = READER_T1_CONTEXT_GetCurrentBGTMilli(pContext, &currentBGT);
	if(retVal != READER_OK) return retVal;
	
	retVal = READER_T1_CONTEXT_GetTickLastBlock(pContext, &tickLastBlock);
	if((retVal != READER_OK) && (retVal != READER_DOESNT_EXIST)) return retVal;
	
	tick = READER_HAL_GetTick();
	if(retVal == READER_DOESNT_EXIST){
		extraStartDelay = 0;
	}
	else if((tickLastBlock + currentBGT) < tick){
		extraStartDelay = 0;
	}
	else{
		extraStartDelay = tickLastBlock + currentBGT - tick;
	}

	/* On envoie le Block                                                       */
	retVal = READER_T1_CONTEXT_GetCurrentCWTMilli(pContext, &currentCWT);
	if(retVal != READER_OK) return retVal;
	
	retVal = READER_T1_SendBlock(pBlockToSend, currentCWT, extraStartDelay, &tickLastBlock, pSettings);     /* On recupere au passage le tick (date) du leading edge du dernier caractere envoye dans ce Block */
	if(retVal != READER_OK) return retVal;
	
	/* On mets a jour la date (tick) du leading edge du dernier caractere du Block envoye ... (Voir ISO7816 section 11.4.3) */
	retVal = READER_T1_CONTEXT_SetTickLastBlock(pContext, tickLastBlock);
	if(retVal != READER_OK) return retVal;
	
	/* On set le dernier Block envoye                                */
	retVal = READER_T1_CONTEXT_SetLastSent(pContext, pBlockToSend);
	if(retVal != READER_OK) return retVal;
	
	/* On recupere le type du Block que l'on vient d'envoyer et selon son type, on mets a jour le contexte de communication .... */
	bType = READER_T1_GetBlockType(pBlockToSend);
	
	if(bType == READER_T1_IBLOCK){
		retVal = READER_T1_CONTROL_IBlockSentUpdateContext(pContext, pBlockToSend);
		if(retVal != READER_OK) return retVal;		
	}
	else if(bType == READER_T1_RBLOCK){
		retVal = READER_T1_CONTROL_RBlockSentUpdateContext(pContext, pBlockToSend);
		if(retVal != READER_OK) return retVal;	
	}
	else if(bType == READER_T1_SBLOCK){
		retVal = READER_T1_CONTROL_SBlockSentUpdateContext(pContext, pBlockToSend);
		if(retVal != READER_OK) return retVal;	
	}
	else{
		return READER_ERR;
	}
	
	/* On remets a zero les compteurs d'erreurs                      */
	retVal = READER_T1_ERR_ClearRepeatCounter(pContext);
	if(retVal != READER_OK) return retVal;
	
	retVal = READER_T1_ERR_ClearResynchCounter(pContext);
	if(retVal != READER_OK) return retVal;
	
	
	return READER_OK;
}


/**
 * \fn READER_Status READER_T1_CONTROL_IBlockSentUpdateContext(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock)
 * \param *pContext est un pointeur sur une structure de type READER_T1_ContextHandler. La structure pointée contient tout le contexte de communication courant.
 * \param *pBlock est un pointeur sur une structure de type READER_T1_Block. Il doit pointeur sur le Block qui vient d'être envoyé (a priori un I-Block).
 * \return La fonction retourne un code d'erreur de type READER_Status. READER_OK indique que le contexte a correctement été mis à jour.
 * Cette fonction mets à jour le contexte de communication à la suite de l'envoi d'un I-Block.
 * Principalement, il s'agit de mettre a jour : le dernier Block envoyé, les flags de chainage et les numéros de séquence.
 */
READER_Status READER_T1_CONTROL_IBlockSentUpdateContext(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock){
	READER_Status retVal;
	READER_T1_MBit mBit;
	
	
	/* On verifie que c'est bien un I-Block ...  */
	retVal = READER_T1_CheckIBlock(pBlock);
	if(retVal != READER_OK) return retVal;
	
	/* On Set le dernier I-Block envoye                                                */
	retVal = READER_T1_CONTEXT_SetLastIBlockSent(pContext, pBlock);
	if(retVal != READER_OK) return retVal;
	
	/* On met a jour le flag de chainage du dernier Block et le flag de situation de chainage                            */
	mBit = READER_T1_GetBlockMBit(pBlock);
	
	if(mBit == READER_T1_MBIT_ONE){
		retVal = READER_T1_CONTEXT_SetDeviceChainingLastBlockFlag(pContext, READER_T1_CHAINING_YES);
		if(retVal != READER_OK) return retVal;
		
		retVal = READER_T1_CONTEXT_SetDeviceChainingSituationFlag(pContext, READER_T1_CHAINING_YES);
		if(retVal != READER_OK) return retVal;
	}
	else if(mBit == READER_T1_MBIT_ZERO){
		retVal = READER_T1_CONTEXT_SetDeviceChainingLastBlockFlag(pContext, READER_T1_CHAINING_NO);
		if(retVal != READER_OK) return retVal;
	}
	else{
		return READER_ERR;
	}	
	
	/* On mets 	a jour le numero de sequence du Device               */
	retVal = READER_T1_CONTEXT_IncDeviceCompleteSeqNum(pContext);
	if(retVal != READER_OK) return retVal;
	
	
	return READER_OK;
}


READER_Status READER_T1_CONTROL_RBlockSentUpdateContext(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock){
	return READER_OK;
}


/**
 * \fn READER_Status READER_T1_CONTROL_SBlockSentUpdateContext(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock)
 * \param *pContext est un pointeur sur une structure de type READER_T1_ContextHandler. Cette structure stocke le contexte courant de communication.
 * \param *pBlock est un pointeur sur un structure de type READER_T1_Block. Il s'agit d'un pointeur sur le Block qui vient d'être envoyé (à priori un S-Block).
 * \return La fonction retourne un code d'erreur de type READER_Status. READER_Ok indique le bon déroulement.
 * Cette fonction permet de mettre à jour le contexte de communication à la suite de l'envoi d'un Block de type S-Block.
 */
READER_Status READER_T1_CONTROL_SBlockSentUpdateContext(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock){
	READER_Status retVal;
	
	
	/* On fait des verifications elementaires sur le Block ...  */
	retVal = READER_T1_CheckSBlock(pBlock);
	if(retVal != READER_OK) return retVal;
		
	/* On regarde si le S-Block envoye est un S-Block Request ou un S-Block Response ...  */
	retVal = READER_T1_IsSBlockRequest(pBlock);
	if(retVal == READER_OK){
		retVal = READER_T1_CONTROL_SBlockRequestSentUpdateContext(pContext, pBlock);
		if(retVal != READER_OK) return retVal;
	}
	else if(retVal == READER_NO){
		retVal = READER_T1_CONTROL_SBlockResponseSentUpdateContext(pContext, pBlock);
		if(retVal != READER_OK) return retVal;
	}
	else{
		return READER_ERR;
	}
	
	
	return READER_OK;
}


/**
 * \fn READER_Status READER_T1_CONTROL_SBlockRequestSentUpdateContext(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock)
 * \param *pContext est un pointeur sur une structure de type READER_T1_ContextHandler. Cette structure stocke le contexte courant de communication.
 * \param *pBlock est un pointeur sur un structure de type READER_T1_Block. Il s'agit d'un pointeur sur le Block qui vient d'être envoyé (à priori un S-Block REQUEST).
 * \return La fonction retourne un code d'erreur de type READER_Status. READER_Ok indique le bon déroulement.
 * Cette fonction permet de mettre à jour le contexte de communication à la suite de l'envoi d'un Block de type S-Block REQUEST.
 */
READER_Status READER_T1_CONTROL_SBlockRequestSentUpdateContext(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock){
	READER_Status retVal;
	READER_T1_SBlockType SBlockType, expectedSBlockType;
	uint8_t expectedINF = 0;	// fixed uninitialized variable warning
	
	
	/* On fait des verifications elementaires sur le Block ...  */
	retVal = READER_T1_CheckSBlock(pBlock);
	if(retVal != READER_OK) return retVal;
	
	retVal = READER_T1_IsSBlockRequest(pBlock);
	if((retVal != READER_OK) && (retVal != READER_NO)) return retVal;
	
	if(retVal == READER_NO){
		return READER_ERR;
	}
	
	/* On recupere le type du S-Block que l'on vient d'envoyer et on en deduit le type de S-Block Response que l'on attend ...  */
	SBlockType = READER_T1_GetBlockSType(pBlock);
	
	if(SBlockType == READER_T1_STYPE_ABORT_REQU){
		expectedSBlockType = READER_T1_STYPE_ABORT_RESP;
	}
	else if(SBlockType == READER_T1_STYPE_IFS_REQU){
		expectedSBlockType = READER_T1_STYPE_IFS_RESP;
		expectedINF = READER_T1_GetBlockSPayload(pBlock);
	}
	else if(SBlockType == READER_T1_STYPE_RESYNCH_REQU){
		expectedSBlockType = READER_T1_STYPE_RESYNCH_RESP;
	}
	else if(SBlockType == READER_T1_STYPE_WTX_REQU){
		expectedSBlockType = READER_T1_STYPE_WTX_RESP;
		expectedINF = READER_T1_GetBlockSPayload(pBlock);
	}
	else{
		return READER_ERR;
	}
	
	/* On mets a jour le Flag qui indique que l'on attend desormais une S-Block Response et son type ...  */
	retVal = READER_T1_CONTEXT_SetSBlockExpectedResponse(pContext, expectedSBlockType);
	if(retVal != READER_OK) return retVal;
	
	/* On mets a jour la valeur du champs INF qu'on attend en reponse pour l'acquittement de ce S-Block */
	pContext->SBlockExpectedINF = expectedINF;
	
	
	return READER_OK;
}


/**
 * \fn READER_Status READER_T1_CONTROL_SBlockResponseSentUpdateContext(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock)
 * \param *pContext est un pointeur sur une structure de type READER_T1_ContextHandler. Cette structure stocke le contexte courant de communication.
 * \param *pBlock est un pointeur sur un structure de type READER_T1_Block. Il s'agit d'un pointeur sur le Block qui vient d'être envoyé (à priori un S-Block RESPONSE).
 * \return La fonction retourne un code d'erreur de type READER_Status. READER_Ok indique le bon déroulement.
 * Cette fonction permet de mettre à jour le contexte de communication à la suite de l'envoi d'un Block de type S-Block REPONSE.
 */
READER_Status READER_T1_CONTROL_SBlockResponseSentUpdateContext(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock){
	return READER_OK;
}


/**
 * \fn READER_Status READER_T1_CONTROL_RcvBlock(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock) 
 * \return La fonction retourne un code d'erreur de type READER_Status. READER_OK indique le bon déroulement.
 * \param \param *pContext est un pointeur sur une structure de type READER_T1_ContextHandler. Cette structure stocke le contexte courant de communication.
 * \param *pBlock est un pointeur sur un structure de type READER_T1_Block.
 * Cette fonction permet de recevoir un Block en tenant compte du contexte de communication actuel. A la réception du Block, la fonction applique les règles du protocole T=1 selon ce qui a été recu. Elle met aussi a jour le contexte de communication.
 */
READER_Status READER_T1_CONTROL_RcvBlock(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock){
	READER_Status retVal;
	READER_T1_BlockType bType;
	READER_T1_RedundancyType rType;
	READER_T1_FlagStatus SBlockExpected;
	READER_HAL_CommSettings *pCommSettings;
	uint32_t currentCWT, currentBWT;
	uint32_t extraTimeout;
	uint32_t tickLastBlock, tick;
	uint32_t blockLEN;
	
	
	/* On recupere les valeurs de timeout a appliquer ... */
	retVal = READER_T1_CONTEXT_GetCurrentCWTMilli(pContext, &currentCWT);
	if(retVal != READER_OK) return retVal;
	
	retVal = READER_T1_CONTEXT_GetCurrentBWTMilli(pContext, &currentBWT);
	if(retVal != READER_OK) return retVal;
	
	/* On calcule le extraTimout pour le premier caractere du Block que l'on va recevoir. Sert a garantir le BWT  */
	retVal = READER_T1_CONTEXT_GetTickLastBlock(pContext, &tickLastBlock);
	if((retVal != READER_OK) && (retVal != READER_DOESNT_EXIST)) return retVal;
	
	
	tick = READER_HAL_GetTick();
	if(retVal == READER_DOESNT_EXIST){
		extraTimeout = currentBWT;
	}
	else if((tickLastBlock + currentBWT) < tick){
		extraTimeout = 0;
	}
	else{
		extraTimeout = tickLastBlock + currentBWT - tick;
	}
	
	/* On receptionne le Block ... */
	retVal = READER_T1_CONTEXT_GetCurrentRedundancyType(pContext, &rType);            /* On recupere le type de code corrcteur actuellement utilise dans le contexte de communication */             
	if(retVal != READER_OK) return retVal;
	
	retVal = READER_T1_CONTEXT_GetHalCommSettingsPtr(pContext, &pCommSettings);       /* On recuepre les paramettres de comm bas niveau actuellement utilises dans le contexte de communication ...  */
	if(retVal != READER_OK) return retVal;
	
	retVal = READER_T1_RcvBlock(pBlock, rType, currentCWT, extraTimeout, &tickLastBlock, pCommSettings);    /* La fonction de reception de Block fait remonter la date du leading Edge du Block recu ...  */
	if((retVal != READER_OK) && (retVal != READER_TIMEOUT)) return retVal;
	
	/* On regarde si on a eu un timeout     */
	if(retVal == READER_TIMEOUT){
		retVal = READER_T1_ERR_DealWithError(pContext, 0);
		if(retVal != READER_OK) return retVal;
		return READER_OK;
	}
	
	/* On verifie que le Block recu ne depase pas le MAX IFSD que l'on peut accepter cote device ... */
	blockLEN = READER_T1_GetBlockLEN(pBlock);
	if(blockLEN > READER_T1_MAX_IFSD_ACCEPTED){
		retVal = READER_T1_ERR_DoResynch(pContext);
		if(retVal != READER_OK) return retVal;
		return READER_OK;
	}
	
	/* On mets a jour le tick du leading edge du dernier Block recu ...  */
	retVal = READER_T1_CONTEXT_SetTickLastBlock(pContext, tickLastBlock);
	if(retVal != READER_OK) return retVal;
	
	/* On regarde si le Block est corrompu ... */
	retVal = READER_T1_CheckBlockIntegrity(pBlock, rType);
	if((retVal != READER_OK) && (retVal != READER_INTEGRITY)) return retVal;		
	
	if(retVal == READER_INTEGRITY){
		retVal = READER_T1_ERR_DealWithError(pContext, 1);
		if(retVal != READER_OK) return retVal;
		
		return READER_OK;
	}
	
	/* On mets a jour l'existance du dernier Block recu ...  */
	retVal = READER_T1_CONTEXT_SetLastRcvd(pContext, pBlock);
	if(retVal != READER_OK) return retVal;
	
	/* On regarde si on attendait un S-Block ... */
	retVal = READER_T1_CONTEXT_IsSBlockResponseExpectedNow(pContext, &SBlockExpected);
	if(retVal != READER_OK) return retVal;
	
	/* On regarde le type du Block recu ... */
	bType = READER_T1_GetBlockType(pBlock);
	if(bType == READER_T1_IBLOCK){
		if(SBlockExpected == READER_T1_FLAGSTATUS_SET){
			retVal = READER_T1_CONTROL_SBlockResponseNotReceived(pContext);
			if(retVal != READER_OK) return retVal;
			
			return READER_OK;
		}
		
		retVal = READER_T1_CONTROL_ApplyIBlockRcvd(pContext, pBlock);
		if(retVal != READER_OK) return retVal;
	}
	else if(bType == READER_T1_RBLOCK){
		if(SBlockExpected == READER_T1_FLAGSTATUS_SET){
			retVal = READER_T1_CONTROL_SBlockResponseNotReceived(pContext);
			if(retVal != READER_OK) return retVal;
			
			return READER_OK;
		}
		
		retVal = READER_T1_CONTROL_ApplyRBlockRcvd(pContext, pBlock);
		if(retVal != READER_OK) return retVal;
	}
	else if(bType == READER_T1_SBLOCK){		
		retVal = READER_T1_CONTROL_ApplySBlockRcvd(pContext, pBlock);
		if(retVal != READER_OK) return retVal;
	}
	else{
		return READER_ERR;
	}
	
	return READER_OK;
}


READER_Status READER_T1_CONTROL_ApplyIBlockRcvd(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock){
	READER_Status retVal;
	READER_T1_ACKStatus ACKStatus;
	READER_T1_ChainingStatus chainingStatus;
	READER_T1_ChainingStatus globalChainingStatus, lastBlockChainingStatus;
	READER_T1_MBit mBit;
	READER_T1_Block tmpBlock;


	/* On verifie si c'est pertinant de recevoir un I-Block maintenant ... */
	

	/* On verifie le numero de sequence */
	retVal = READER_T1_CONTROL_IsSeqNumValid(pContext, pBlock);
	if((retVal != READER_OK) && (retVal != READER_NO)) return retVal;
	
	if(retVal == READER_NO){
		retVal = READER_T1_ERR_DealWithError(pContext, 0);     /* On Stack le bon error Block dans le Buffer d'envoi et on s'arrete la ... */
		if(retVal != READER_OK) return retVal;
		
		return READER_OK;
	}
	
	/* On regarde si ce I-Block est un ACK ...                                                                                      */
	retVal = READER_T1_CONTROL_IsIBlockACK(pContext, pBlock);
	if((retVal != READER_OK) && (retVal != READER_NO)) return retVal;
	
	/* Si ce I-Block est un ACK, on mets a jour la situation de l'ACK dans le contexte (Flag)                                       */
	if(retVal == READER_OK){
		retVal = READER_T1_CONTEXT_SetACKStatus(pContext, READER_T1_ACK);
		if(retVal != READER_OK) return retVal;
	}
	
	/* Si ce I-Block n'est pas un ACK ...                                                                                           */
	else if(retVal == READER_NO){
		/* On regarde si on a deja precedement recu un ACK */
		retVal = READER_T1_CONTEXT_GetACKStatus(pContext, &ACKStatus);
		if(retVal != READER_OK) return retVal;
		
		/* On regarde si on est en situation de chainage   */
		retVal = READER_T1_CONTEXT_CardIsChaining(pContext, &chainingStatus);
		if(retVal != READER_OK) return retVal;
		
		/* On gere les cas problematiques ...                                                                                        */
		/* Si on a pas deja recu d'ACK auparavent et que ce I-Block n'est pas lui meme un ACK ...                                    */
		if(ACKStatus == READER_T1_NOACK){
			retVal = READER_T1_ERR_DoResynch(pContext);
			if(retVal != READER_OK) return retVal;
		}
		
		/* Si ce Block ne constitue pas un ACK, qu'on a deja recu un ACK auparavent, mais qu'on n'est pas en situation de chainage   */
		if((ACKStatus == READER_T1_ACK) && (chainingStatus == READER_T1_CHAINING_NO)){
			retVal = READER_T1_ERR_DoResynch(pContext);
			if(retVal != READER_OK) return retVal;
		}
	}
	else{
		return READER_ERR;
	}
	
	/* On incremente le numero de sequence du cote carte ...  */
	retVal = READER_T1_CONTEXT_IncCardCompleteSeqNum(pContext);
	if(retVal != READER_OK) return retVal;
	
	
	/* On extrait les donnees contenues dans le I-Block                                                        */
	retVal = READER_T1_RCPTBUFF_ExtractDataFromIBlock(pContext, pBlock);
	if(retVal != READER_OK) return retVal;
	
	
	/* On regarde le M-Bit du Block en provenance de la carte et on mets a jour les flags ...            */
	mBit = READER_T1_GetBlockMBit(pBlock);
	if(mBit == READER_T1_MBIT_ZERO){
		/* On mets a jour la situation de chainage                                                             */
		retVal = READER_T1_CONTEXT_SetCardChainingLastBlockFlag(pContext, READER_T1_CHAINING_NO);
		if(retVal != READER_OK) return retVal;
	}
	else if(mBit == READER_T1_MBIT_ONE){
		/* On mets a jour la situation de chainage                                                             */
		retVal = READER_T1_CONTEXT_SetCardChainingLastBlockFlag(pContext, READER_T1_CHAINING_YES);
		if(retVal != READER_OK) return retVal;
		
		retVal = READER_T1_CONTEXT_SetCardChainingSituationFlag(pContext, READER_T1_CHAINING_YES);
		if(retVal != READER_OK) return retVal;
	}
	else{
		return READER_ERR;
	}
	
	
	/* Si on est en situation globale de chainage ...                                                         */
	retVal = READER_T1_CONTEXT_CardIsChainingLastBlock(pContext, &lastBlockChainingStatus);
	if(retVal != READER_OK) return retVal;
	
	retVal = READER_T1_CONTEXT_CardIsChaining(pContext, &globalChainingStatus);
	if(retVal != READER_OK) return retVal;
	
	
	if(globalChainingStatus == READER_T1_CHAINING_YES){
		if(lastBlockChainingStatus == READER_T1_CHAINING_YES){
			/* On prepare un R-Block de chainage */
			retVal = READER_T1_FORGE_ChainingRBlockForCard(pContext, &tmpBlock);
			if(retVal != READER_OK) return retVal;
			
			retVal = READER_T1_BUFFER_Stack(pContext, &tmpBlock);
			if(retVal != READER_OK) return retVal;
		}
		else if(lastBlockChainingStatus == READER_T1_CHAINING_NO){			
			retVal = READER_T1_CONTEXT_SetCardChainingSituationFlag(pContext, READER_T1_CHAINING_NO);
			if(retVal != READER_OK) return retVal;
		}
		else{
			return READER_ERR;
		}
	}
	
	return READER_OK;
}


READER_Status READER_T1_CONTROL_ApplyRBlockRcvd(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock){
	READER_Status retVal;
	READER_T1_Block *pLastBlock;
	READER_T1_SeqNumber tmpSeqNumRBlock;
	uint32_t seqNumRBlock, seqNumIBlock;
	
	
	/* On verifie si c'est pertinant de recevoir un R-Block maintenant  */
	
	
	/* Ce R-Block est il un ACK d'un precedent Block qu'on a envoye ?  */
	retVal = READER_T1_CONTROL_IsRBlockACK(pContext, pBlock);
	if((retVal != READER_OK) && (retVal != READER_NO)) return retVal;
	
	/* Si ce R-Block est un ACK ...                                        */
	if(retVal == READER_OK){
		retVal = READER_T1_CONTEXT_SetACKStatus(pContext, READER_T1_ACK);
		if(retVal != READER_OK) return retVal;
	}
	/* Si ce R-Block n'est pas un ACK ...                                  */
	else{
		/* On regarde le numero de sequence du R-Block et du dernier I-Block qu'on a envoye ...  */
		tmpSeqNumRBlock = READER_T1_GetExpectedBlockSeqNumber(pBlock);
		if(tmpSeqNumRBlock == READER_T1_SEQNUM_ONE){
			seqNumRBlock = 1;
		}
		else if(tmpSeqNumRBlock == READER_T1_SEQNUM_ZERO){
			seqNumRBlock = 0;
		}
		else{
			return READER_ERR;
		}
		
		retVal = READER_T1_CONTEXT_GetLastIBlockSentSeqNum(pContext, &seqNumIBlock);
		if(retVal != READER_OK) return retVal;
		
		/* On compare les numeros de sequence du R-Block et du dernier I-Block envoye ...   */
		if(seqNumRBlock == seqNumIBlock){
			/* On recupere et on prepare le dernier I-Block qu'on a envoye */
			retVal = READER_T1_CONTEXT_GetLastIBlockSent(pContext, &pLastBlock);
			if(retVal != READER_OK) return retVal;
			
			retVal = READER_T1_BUFFER_Stack(pContext, pLastBlock);
			if(retVal != READER_OK) return retVal;
		}
		else{
			/* On recupere et on prepare le dernier Block qu'on a envoye   */
			retVal = READER_T1_CONTEXT_GetLastSent(pContext, &pLastBlock);
			if(retVal != READER_OK) return retVal;
			
			retVal = READER_T1_BUFFER_Stack(pContext, pLastBlock);
			if(retVal != READER_OK) return retVal;
		}
	}
	
	return READER_OK;
}


READER_Status READER_T1_CONTROL_ApplySBlockRcvd(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock){
	READER_Status retVal;
	
	
	/* On fait des verifications elementaires sur le Block ...  */
	retVal = READER_T1_CheckSBlock(pBlock);
	if(retVal != READER_OK) return retVal;
		
	/* On regarde si le S-Block recu est un S-Block Request ou un S-Block Response ...  */
	retVal = READER_T1_IsSBlockRequest(pBlock);
	if(retVal == READER_OK){
		retVal = READER_T1_CONTROL_ApplySBlockRequestRcvd(pContext, pBlock);
		if(retVal != READER_OK) return retVal;
	}
	else if(retVal == READER_NO){
		retVal = READER_T1_CONTROL_ApplySBlockResponseRcvd(pContext, pBlock);
		if(retVal != READER_OK) return retVal;
	}
	else{
		return READER_ERR;
	}
	
	
	return READER_OK;
}


READER_Status READER_T1_CONTROL_ApplySBlockRequestRcvd(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock){
	READER_Status retVal;
	READER_T1_FlagStatus flag;
	READER_T1_SBlockType rcvdSBlockType;
	READER_T1_Block responseBlock;
	uint8_t blockINF;
	
	
	/* Verifications elementaires sur le suppose S-Block ...  */
	retVal = READER_T1_CheckSBlock(pBlock);
	if(retVal != READER_OK) return retVal;
	
	/* On a recu un Request, mais on verifie si S-Block Resp n'etait pas attendu  */
	retVal = READER_T1_CONTEXT_IsSBlockResponseExpectedNow(pContext, &flag);
	if(retVal != READER_OK) return retVal;
	
	if(flag == READER_T1_FLAGSTATUS_SET){
		/* On a recu une S-Block Request alors que on etait en train d'attendre un S-Bloc Response ...  */
		retVal = READER_T1_CONTROL_SBlockResponseNotReceived(pContext);
		if(retVal != READER_OK) return retVal;
	}
	else if(flag == READER_T1_FLAGSTATUS_RESET){
		/* Si on a recu une S-Block Request et que tout est bon ... On applique la demande ...  */
		rcvdSBlockType = READER_T1_GetBlockSType(pBlock);
		
		if(rcvdSBlockType == READER_T1_STYPE_ABORT_REQU){
			retVal = READER_T1_CONTROL_ApplySBlockAbort(pContext, pBlock);
			if(retVal != READER_OK) return retVal;
			
			retVal = READER_T1_FORGE_SBlockAbortResponse(pContext, &responseBlock);
			if(retVal != READER_OK) return retVal;
		}
		else if(rcvdSBlockType == READER_T1_STYPE_IFS_REQU){
			retVal = READER_T1_CONTROL_ApplySBlockIfsc(pContext, pBlock);
			if((retVal != READER_OK) && (retVal != READER_BAD_VALUE)) return retVal;
			
			/* Si on accepte pas la valeur suggeree par la carte, on ne fait rien (ie : on ne fait pas d'acquittement via S-Block Response) */
			if(retVal != READER_BAD_VALUE){
				blockINF = READER_T1_GetBlockSPayload(pBlock);
			
				retVal = READER_T1_FORGE_SBlockIfsResponse(pContext, &responseBlock, blockINF);
				if(retVal != READER_OK) return retVal;
			}			
		}
		else if(rcvdSBlockType == READER_T1_STYPE_RESYNCH_REQU){
			/* Selon ISO7816-3 section 11.6.3.2, rule 6, seul le Device peut initier une RESYNCH Request ...  */
			/* Donc ici, on fait le choix d'ignorer le Block Recu ...                                         */
			return READER_OK;
		}
		else if(rcvdSBlockType == READER_T1_STYPE_WTX_REQU){
			retVal = READER_T1_CONTROL_ApplySBlockWtx(pContext, pBlock);
			if(retVal != READER_OK) return retVal;
			
			blockINF = READER_T1_GetBlockSPayload(pBlock);
			
			retVal = READER_T1_FORGE_SBlockWtxResponse(pContext, &responseBlock, blockINF);
			if(retVal != READER_OK) return retVal;
		}
		else{
			return READER_ERR;
		}
		
		/* On prepare le S-Block Response pour l'envoi ...  */
		retVal = READER_T1_BUFFER_Stack(pContext, &responseBlock);
		if(retVal != READER_OK) return retVal;
		
	}
	else{
		return READER_ERR;
	}
	
	
	return READER_OK;
}


READER_Status READER_T1_CONTROL_ApplySBlockResponseRcvd(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock){
	READER_Status retVal;
	READER_T1_FlagStatus flag;
	READER_T1_SBlockType rcvdSBlockType;
	
	
	/* Verifications elementaires sur le suppose S-Block recu ...  */
	retVal = READER_T1_CheckSBlock(pBlock);
	if(retVal != READER_OK) return retVal;
	
	/* On verifie q'une S-Block Response etait bien attendue ...        */
	/* Si aucune S-Response attendue alors on ignore le Block recu ...  */
	retVal = READER_T1_CONTEXT_IsSBlockResponseExpectedNow(pContext, &flag);
	if(retVal != READER_OK) return retVal;
	
	if(flag == READER_T1_FLAGSTATUS_RESET){
		/* On ne fait rien, on ignore ce S-Block ...  */
		return READER_OK;
	}
	
	if(flag != READER_T1_FLAGSTATUS_SET){
		return READER_ERR;
	}
	
	/* On verifie que la S-Block Response recu correspond a celle qui etait attendue ...  */
	/* Si on a pas recu la bonne reponse alors que une reponse erait attendue, alors il faut appliquer la bonne regle (ISO7816-3 section 11.6.3.2 rue 7.3) ...  */
	retVal = READER_T1_CONTROL_CheckIfThisSBlockResponseIsCorrect(pContext, pBlock);
	if((retVal != READER_OK) && (retVal != READER_NO)) return retVal;
	
	if(retVal == READER_NO){
		retVal = READER_T1_CONTROL_SBlockResponseNotReceived(pContext);
		if(retVal != READER_OK) return retVal;
	}
	
	/* On RAZ le compteur de S-Block Requests faites a la carte pour obtenir la S-Block Response ...  */
	retVal = READER_T1_CONTEXT_ClearSBlockRequestsCounter(pContext);
	if(retVal != READER_OK) return retVal;
	
	/* On RAZ le Flag isSBlockExpected ...  */
	retVal = READER_T1_CONTEXT_SetNoSBlockExpectedResponse(pContext);
	if(retVal != READER_OK) return retVal;
	
	
	
	/* On applique la demande du S-Block ... */
	rcvdSBlockType = READER_T1_GetBlockSType(pBlock);
	if(rcvdSBlockType == READER_T1_STYPE_ABORT_RESP){
		retVal = READER_T1_CONTROL_ApplySBlockAbort(pContext, pBlock);
		if(retVal != READER_OK) return retVal;
	}
	else if(rcvdSBlockType == READER_T1_STYPE_IFS_RESP){
		retVal = READER_T1_CONTROL_ApplySBlockIfsd(pContext, pBlock);
		if(retVal != READER_OK) return retVal;		
	}
	else if(rcvdSBlockType == READER_T1_STYPE_RESYNCH_RESP){
		retVal = READER_T1_CONTROL_ApplySBlockResynch(pContext, pBlock);
		if(retVal != READER_OK) return retVal;
	}
	else if(rcvdSBlockType == READER_T1_STYPE_WTX_RESP){
		retVal = READER_T1_CONTROL_ApplySBlockWtx(pContext, pBlock);
		if(retVal != READER_OK) return retVal;
	}
	else{
		return READER_ERR;
	}
	
	
	return READER_OK;
}


READER_Status READER_T1_CONTROL_ApplySBlockResynch(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock){
	READER_Status retVal;
	READER_T1_SBlockType SBlockType;
	
	
	/* On fait des verifications sur le Block ...  */
	retVal = READER_T1_CheckSBlock(pBlock);
	if(retVal != READER_OK) return retVal;
	
	SBlockType = READER_T1_GetBlockSType(pBlock);
	if((SBlockType != READER_T1_STYPE_RESYNCH_REQU) && (SBlockType != READER_T1_STYPE_RESYNCH_RESP)){
		return READER_ERR;
	}
	
	/* On verifie le compteur de demandes de Resynch successives (ISO7816-3 section 11.K6.3.2, rule 6.4) ...  */
	retVal = READER_T1_ERR_CheckResynchCounter(pContext);
	if((retVal != READER_OK) && (retVal != READER_NO)) return retVal;
	
	/* Si on a atteint le nombre max de resyncho consecutives */
	if(retVal == READER_NO){
		retVal = READER_T1_ERR_DoReset(pContext);
		if(retVal != READER_OK) return retVal;
	}
	
	/* Si tout va bien, on procede a la resynchro ...  */
	retVal = READER_T1_ERR_DoResynch(pContext);
	if(retVal != READER_OK) return retVal;
	
	/* On incremente le compteur de demandes de Resynchro ...  */
	retVal = READER_T1_ERR_IncResynchCounter(pContext);
	if(retVal != READER_OK) return retVal;
	
	
	return READER_OK;
}


/**
 * \fn READER_Status READER_T1_CONTROL_ApplySBlockIfsc(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock)
 * \param *pContext est un pointeur sur une structure de type READER_T1_ContextHandler. La structure pointée stocke le contexte actuel de communication.
 * \param *pBlock est un pointeur sur une structure de type READER_T1_Block. Il s'agit du S-Block IFS Request recu en provenance de la CARTE.
 * \return La fonction retourne un code d'erreur de type READER_Status. READER_OK indique le bon déroulement de la fonction. READER_BAD_VALUE indique que le IFSC demandé par la CARTE n'est pas supporté par le DEVICE.
 * Cette fonction sert à appliquer au contexte une requette IFS en provenance de la CARTE.
 */
READER_Status READER_T1_CONTROL_ApplySBlockIfsc(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock){
	READER_Status retVal;
	READER_T1_SBlockType SBlockType;
	uint32_t ifsc;
	
	
	/* On fait des verifications sur le Block ...  */
	retVal = READER_T1_CheckSBlock(pBlock);
	if(retVal != READER_OK) return retVal;
	
	SBlockType = READER_T1_GetBlockSType(pBlock);
	if((SBlockType != READER_T1_STYPE_IFS_REQU) && (SBlockType != READER_T1_STYPE_IFS_RESP)){
		return READER_ERR;
	}
	
	/* On recupere la valeur de IFSC contenue dans le S-Block ...  */
	ifsc = READER_T1_GetBlockSPayload(pBlock);
	
	/* On applique cette valeur au contexte de communication ...  */
	retVal = READER_T1_CONTEXT_SetCurrentIFSC(pContext, ifsc);
	if((retVal != READER_OK) && (retVal != READER_BAD_VALUE)) return retVal;
	
	if(retVal == READER_BAD_VALUE){
		return READER_BAD_VALUE;
	}
	
	
	return READER_OK;
}


READER_Status READER_T1_CONTROL_ApplySBlockIfsd(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock){
	READER_Status retVal;
	READER_T1_SBlockType SBlockType;
	uint32_t ifsd;
	
	
	/* On fait des verifications sur le Block ...  */
	retVal = READER_T1_CheckSBlock(pBlock);
	if(retVal != READER_OK) return retVal;
	
	SBlockType = READER_T1_GetBlockSType(pBlock);
	if((SBlockType != READER_T1_STYPE_IFS_REQU) && (SBlockType != READER_T1_STYPE_IFS_RESP)){
		return READER_ERR;
	}
	
	/* On recupere la valeur de IFSD contenue dans le S-Block ...  */
	ifsd = READER_T1_GetBlockSPayload(pBlock);
	
	/* On applique cette valeur au contexte de communication ...  */
	retVal = READER_T1_CONTEXT_SetCurrentIFSD(pContext, ifsd);
	if(retVal == READER_BAD_VALUE) return READER_BAD_VALUE;
	if(retVal != READER_OK) return retVal;
	
	
	return READER_OK;
}


READER_Status READER_T1_CONTROL_ApplySBlockAbort(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock){	
	return READER_OK;
}


READER_Status READER_T1_CONTROL_ApplySBlockWtx(READER_T1_ContextHandler *pContext, READER_T1_Block *pBlock){
	READER_Status retVal;
	READER_T1_SBlockType SBlockType;
	uint32_t multiplier;
	
	
	/* On fait des verifications sur le Block ...  */
	retVal = READER_T1_CheckSBlock(pBlock);
	if(retVal != READER_OK) return retVal;
	
	SBlockType = READER_T1_GetBlockSType(pBlock);
	if((SBlockType != READER_T1_STYPE_WTX_REQU) && (SBlockType != READER_T1_STYPE_WTX_RESP)){
		return READER_ERR;
	}
	
	/* On recupere les donnes multiplieur de BWT dans le S-Block ...  */
	multiplier = READER_T1_GetBlockSPayload(pBlock);
	
	/* On applique au contexte le multipieur recupere dans le S-Block ...  */
	retVal = READER_T1_CONTEXT_ExtendCurrentBWT(pContext, multiplier);
	if(retVal != READER_OK) return retVal;
	
	return READER_OK;
}


READER_Status READER_T1_CONTROL_SendAbortRequest(READER_T1_ContextHandler *pContext){
	READER_Status retVal;
	READER_T1_Block tmpBlock;
	
	
	/* On forge le S-Block correspondant ...  */
	retVal = READER_T1_FORGE_SBlockAbortRequest(pContext, &tmpBlock);
	if(retVal != READER_OK) return retVal;
	
	/* On stack le nouveau S-Block dans le Buffer d'envoi ...  */
	retVal = READER_T1_BUFFER_Stack(pContext, &tmpBlock);
	if(retVal != READER_OK) return retVal;
	
	
	return READER_OK;
}


READER_Status READER_T1_CONTROL_SendResynchRequest(READER_T1_ContextHandler *pContext){
	READER_Status retVal;
	READER_T1_Block tmpBlock;
	
	
	/* On forge le S-Block correspondant ...  */
	retVal = READER_T1_FORGE_SBlockResynchRequest(pContext, &tmpBlock);
	if(retVal != READER_OK) return retVal;
	
	/* On stack le nouveau S-Block dans le Buffer d'envoi ...  */
	retVal = READER_T1_BUFFER_Stack(pContext, &tmpBlock);
	if(retVal != READER_OK) return retVal;
	
	
	return READER_OK;
}


READER_Status READER_T1_CONTROL_SendIfsdRequest(READER_T1_ContextHandler *pContext, uint8_t newIfsd){
	READER_Status retVal;
	READER_T1_Block tmpBlock;
	
	
	/* On forge le S-Block correspondant ...  */
	retVal = READER_T1_FORGE_SBlockIfsRequest(pContext, &tmpBlock, newIfsd);
	if(retVal != READER_OK) return retVal;
	
	/* On stack ce Block dans le Buffer d'envoi ...  */
	retVal = READER_T1_BUFFER_Stack(pContext, &tmpBlock);
	if(retVal != READER_OK) return retVal;
	
	
	return READER_OK;
}


READER_Status READER_T1_CONTROL_SendWtxRequest(READER_T1_ContextHandler *pContext, uint8_t wtMultiplier){
	READER_Status retVal;
	READER_T1_Block tmpBlock;
	
	
	/* On forge le S-Block correspondant ...  */
	retVal = READER_T1_FORGE_SBlockWtxRequest(pContext, &tmpBlock, wtMultiplier);
	if(retVal != READER_OK) return retVal;
	
	/* On stack ce Block dans le Buffer d'envoi ...  */
	retVal = READER_T1_BUFFER_Stack(pContext, &tmpBlock);
	if(retVal != READER_OK) return retVal;
	
	
	return READER_OK;
}


/* Retourne READER_TOO_MUCH_TRIES si on a pas pu reenvoyer la S-Block Requests parceque on a depasse la nombre max de tentatives ...  */
READER_Status READER_T1_CONTROL_ResendRequest(READER_T1_ContextHandler *pContext){
	READER_Status retVal;
	READER_T1_Block *pLastBlockSent;
	READER_T1_BlockType bType;
	
	
	/* On verifie le compteur d'envoi de S-Block Requests successives ... */
	retVal = READER_T1_CONTEXT_CheckSBlockRequestsCounter(pContext);
	if((retVal != READER_OK) && (retVal != READER_NO)) return retVal;
	
	/* Si on a depasse le compteur de S-Block Requests ...  */
	if(retVal == READER_NO){
		return READER_TOO_MUCH_TRIES;
	}
	
	/* On recupere le dernier Block envoye (doit correspondre au S-Block Request) */
	retVal = READER_T1_CONTEXT_GetLastSent(pContext, &pLastBlockSent);
	if(retVal != READER_OK) return retVal;
	
	bType = READER_T1_GetBlockType(pLastBlockSent);
	if(bType != READER_T1_SBLOCK){
		return READER_ERR;
	}
	
	retVal = READER_T1_IsSBlockRequest(pLastBlockSent);
	if((retVal != READER_OK) && (retVal != READER_NO)) return retVal;
	
	if(retVal == READER_NO){
		return READER_ERR;
	}
	
	/* On replace ce Block dans le Buffer d'envoi ...  */
	retVal = READER_T1_BUFFER_Stack(pContext, pLastBlockSent);
	if(retVal != READER_OK) return retVal;
	
	
	/* On incremente le compteur de S-Block Requests ...  */
	retVal = READER_T1_CONTEXT_IncSBlockRequestsCounter(pContext);
	if(retVal != READER_OK) return retVal;
	
	
	return READER_OK;
}


READER_Status READER_T1_CONTROL_SBlockResponseNotReceived(READER_T1_ContextHandler *pContext){
	READER_Status retVal;
	
	
	retVal = READER_T1_CONTROL_ResendRequest(pContext);
	if((retVal != READER_OK) && (retVal != READER_TOO_MUCH_TRIES)) return retVal;
	
	if(retVal == READER_TOO_MUCH_TRIES){
		retVal = READER_T1_ERR_DoReset(pContext);
		if(retVal != READER_OK) return retVal;
	}
	
	return READER_OK;
}


/**
 * \fn READER_Status READER_T1_CONTROL_CheckExpectedINF(READER_T1_ContextHandler *pContext, READER_T1_Block *pRcvdSBlockResp)
 * \param *pContext est un pointeur sur une structure de type READER_T1_ContextHandler. La structure pointée stocke le contexte actuel de communication.
 * \param *pRcvdSBlockResp est un pointeur sur uns structure de type READER_T1_Block. Il doit pointer sur le S-Block Response à vérifier.
 * \return La fonction retourne un code d'erreur de type READER_Status. READER_OK indique que le champs INF contenu dans le S-Block Response correspond bien à celui qui est attendu. READER_NO indique que le champs INF de ce Block ne correspond pas. Toute autre valeur indique une erreur interne à la fonction.
 * Lors de la reception d'un S-Block Response, cette fonction permet de vérifier que le champs INF contenu dans la réponse est bien celui qui est attendu par le contexte de communication (voir ISO7816-3 section 11.6.2.3, Rules 3 et 4).
 */
READER_Status READER_T1_CONTROL_CheckExpectedINF(READER_T1_ContextHandler *pContext, READER_T1_Block *pRcvdSBlockResp){
	READER_Status retVal;
	uint8_t expectedINF, inBlockINF;
	uint8_t blockLEN;
	
	
	/* On recupere dans le contexte la valeur de INF qui est attendu pour le S-Block Response ...  */
	retVal = READER_T1_CONTEXT_GetSBlockExpectedINF(pContext, &expectedINF);
	if(retVal != READER_OK) return retVal;
	
	/* On fait des verifications sur le S-Block Response que l'on manipule ...  */
	retVal = READER_T1_CheckSBlock(pRcvdSBlockResp);
	if(retVal != READER_OK) return retVal;
	
	retVal = READER_T1_IsSBlockResponse(pRcvdSBlockResp);
	if(retVal != READER_OK) return retVal;
	
	blockLEN = READER_T1_GetBlockLEN(pRcvdSBlockResp);
	if(blockLEN != 1){
		return READER_NO;
	}
	
	/* On recupere la valeur INF qui se trouve dans le S-Block Response recu ...  */
	inBlockINF = READER_T1_GetBlockSPayload(pRcvdSBlockResp);
	
	/* On compare les valeurs ...  */
	if(inBlockINF != expectedINF){
		return READER_NO;
	}
	
	
	return READER_OK;
}


/* Retourne READER_NO si n'est pas expected ...  */
/* Quand on lance la fonction, on considere que un S-Block etait effectivement attendu. On veut savoir si il correspond exactement a ce que on attendait ... */
READER_Status READER_T1_CONTROL_CheckIfThisSBlockResponseIsCorrect(READER_T1_ContextHandler *pContext, READER_T1_Block *pRcvdSBlockResp){
	READER_Status retVal;
	READER_T1_SBlockType SBlockTypeRcvd, SBlockTypeExpected;
	READER_T1_FlagStatus isSBlockExpected;
	
	
	/* On fait des verifications elementaires sur le S-Block Response que l'on manipule ...  */
	retVal = READER_T1_CheckSBlock(pRcvdSBlockResp);
	if(retVal != READER_OK) return retVal;
	
	retVal = READER_T1_IsSBlockResponse(pRcvdSBlockResp);
	if(retVal != READER_OK) return retVal;
	
	
	/* On verifie qu'un S-Block Response etait bien attendue (normalement si on entre dans cette fonction c'est bien le cas, mais on verifie quand meme) ...  */
	retVal = READER_T1_CONTEXT_IsSBlockResponseExpectedNow(pContext, &isSBlockExpected);
	if(retVal != READER_OK) return retVal;
	
	if(isSBlockExpected == READER_T1_FLAGSTATUS_RESET){
		return READER_NO;
	}
	else if(isSBlockExpected != READER_T1_FLAGSTATUS_SET){
		return READER_ERR;
	}
	
	/* On verifie que c'est ce type de S-Block Response qui est attendu ...  */
	SBlockTypeRcvd = READER_T1_GetBlockSType(pRcvdSBlockResp);
	
	retVal = READER_T1_CONTEXT_GetSBlockExpectedResponseType(pContext, &SBlockTypeExpected);
	if(retVal != READER_OK) return retVal;
	
	if(SBlockTypeRcvd != SBlockTypeExpected){
		return READER_NO;
	}
	
	/* Si c'est un S-Block sense contenir un champs INF, on le verifie ... */
	if((SBlockTypeRcvd == READER_T1_STYPE_IFS_RESP) || (SBlockTypeRcvd == READER_T1_STYPE_WTX_RESP)){
		retVal = READER_T1_CONTROL_CheckExpectedINF(pContext, pRcvdSBlockResp);
		if((retVal != READER_OK) && (retVal != READER_NO)) return retVal;
		
		if(retVal == READER_NO){
			return READER_NO;
		}
	}

	
	return READER_OK;
}
