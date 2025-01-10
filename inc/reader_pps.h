/**
 * \file reader_sifs.h
 * \copyright This file is part of the Open-ISO7816-Stack project and is distributed under the MIT license. See LICENSE file in the root directory. 
 */


#ifndef __READER_PPS_H__
#define __READER_PPS_H__

#include "reader_atr.h"

#ifdef __cplusplus
extern "C"
{
#endif

READER_Status Reader_PPS_Exchange(READER_ATR_Atr *atr, READER_HAL_CommSettings *pSettings);

#ifdef __cplusplus
}
#endif

#endif
