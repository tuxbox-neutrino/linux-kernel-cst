/*
 * Copyright (C) 2014, Entropic Communications. All Rights Reserved
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef TMHW_I2C_CFGLOCAL_H
#define TMHW_I2C_CFGLOCAL_H

#ifdef LINUX_BUILD
#include <HwAPI/tmNxTypes.h>
#else
#include <tmNxTypes.h>
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#define I2C_CFG_DMA_BLOCK_LENGTH 1024

//-----------------------------------------------------------------------------
// FUNCTION   : tmhwI2c_CfgGet
//
// DESCRIPTION: Returns pointer to the tmhwI2c configuration structure array
//
// Note       : This function will lie in the internal interface of tmhwI2c
//              and should not be visible to clients of tmhwI2c
//-----------------------------------------------------------------------------
ptmhwI2c_Cfg_t tmhwI2c_CfgGet(void);

//-----------------------------------------------------------------------------
// FUNCTION:    tmhwI2c_CfgGetState
//
// DESCRIPTION: Returns pointer to the tmhwI2c configuration state structure array
//
// Note       : This function will lie in the internal interface of tmhwI2c
//              and should not be visible to clients of tmhwI2c
//-----------------------------------------------------------------------------
ptmhwI2c_CfgState_t tmhwI2c_CfgGetState(void);

//-----------------------------------------------------------------------------
// FUNCTION:    tmhwI2c_CfgGetNumUnits
//
// DESCRIPTION: Returns total number of units available in I2C hardware
//
// Note       : This function will lie in the internal interface of tmhwI2c
//              and should not be visible to clients of tmhwI2c
//-----------------------------------------------------------------------------
UInt32  tmhwI2c_CfgGetNumUnits(void);


#ifdef __cplusplus
}
#endif

#endif // TMHW_I2C_CFGLOCAL_H



