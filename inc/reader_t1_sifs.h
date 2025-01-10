/**
 * \file reader_sifs.h
 * \copyright This file is part of the Open-ISO7816-Stack project and is distributed under the MIT license. See LICENSE file in the root directory. 
 */


#ifndef __READER_T1_SIFS_H__
#define __READER_T1_SIFS_H__

#include "reader_atr.h"
#include "reader_t1_context_handler.h"

#ifdef __cplusplus
extern "C"
{
#endif

void READER_T1_SIFS_PrintCurrent(READER_T1_ContextHandler *pContext, char* label);
READER_Status READER_T1_SIFS_Modify(READER_T1_ContextHandler *pContext, READER_ATR_Atr *atr);

#ifdef __cplusplus
}
#endif

#endif
