/**
 * \file reader_t0_apdu.h
 * \copyright This file is part of the Open-ISO7816-Stack project and is distributed under the MIT license. See LICENSE file in the root directory. 
 * 
 * In this file we define all the prototype of the functions used to manipulate APDUs with the T=0 protocol.
 */



#ifndef __READER_T0_APDU_H__
#define __READER_T0_APDU_H__


#include "reader.h"
#include "reader_apdu.h"
#include "reader_tpdu.h"
#include "reader_atr.h"
#include "reader_t0_context_handler.h"

#ifdef __cplusplus
extern "C"
{
#endif

READER_Status READER_T0_APDU_Init(READER_T0_ContextHandler *pContext, READER_HAL_CommSettings *pSettings);
READER_Status READER_T0_APDU_Execute(READER_T0_ContextHandler *pContext, READER_APDU_Command *pApduCmd, READER_APDU_Response *pApduResp);


READER_Status READER_T0_APDU_ExecuteCase1(READER_T0_ContextHandler *pContext, READER_APDU_Command *pApduCmd, READER_APDU_Response *pApduResp);
READER_Status READER_T0_APDU_ExecuteCase2S(READER_T0_ContextHandler *pContext, READER_APDU_Command *pApduCmd, READER_APDU_Response *pApduResp);
READER_Status READER_T0_APDU_ExecuteCase2E(READER_T0_ContextHandler *pContext, READER_APDU_Command *pApduCmd, READER_APDU_Response *pApduResp);
READER_Status READER_T0_APDU_ExecuteCase3S(READER_T0_ContextHandler *pContext, READER_APDU_Command *pApduCmd, READER_APDU_Response *pApduResp);
READER_Status READER_T0_APDU_ExecuteCase3E(READER_T0_ContextHandler *pContext, READER_APDU_Command *pApduCmd, READER_APDU_Response *pApduResp);
READER_Status READER_T0_APDU_ExecuteCase4S(READER_T0_ContextHandler *pContext, READER_APDU_Command *pApduCmd, READER_APDU_Response *pApduResp);
READER_Status READER_T0_APDU_ExecuteCase4E(READER_T0_ContextHandler *pContext, READER_APDU_Command *pApduCmd, READER_APDU_Response *pApduResp);

READER_Status READER_T0_APDU_RcvSW(READER_T0_ContextHandler *pContext, uint16_t *SW);
READER_Status READER_T0_APDU_RcvResponse(READER_T0_ContextHandler *pContext, uint8_t *buffer, uint32_t Ne, uint16_t *SW);


READER_Status READER_T0_APDU_MapTpduRespToApdu(READER_T0_ContextHandler *pContext, READER_TPDU_Response *pTpduResp, READER_APDU_Response *pApduResp);

#ifdef __cplusplus
}
#endif

#endif
