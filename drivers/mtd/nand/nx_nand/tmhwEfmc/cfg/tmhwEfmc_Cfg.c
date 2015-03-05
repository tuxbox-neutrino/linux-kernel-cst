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


#include "tmNxTypes.h"
#include "tmhwEfmc_Cfg.h"

/*
 * Types and defines:
 */
/* Base addresses of the peripheral */
#ifndef TMHW_EFMC_PHY_MMIO_ADDRESS0
#define TMHW_EFMC_PHY_MMIO_ADDRESS0      (0x8001A000)
#define TMHW_EFMC_AHB_MEM_ADDRESS0 (0xC0000000)
#endif

/*-----------------------------------------------------------------------------
* Global data
*-----------------------------------------------------------------------------*/
//const tmhwEfmc_Cfg_t gktmhwEfmc_Config [NUMBER_OF_MODULES] =
tmhwEfmc_Cfg_t gktmhwEfmc_Config [NUMBER_OF_MODULES] =
{
    { TMHW_EFMC_PHY_MMIO_ADDRESS0, (pUInt8) TMHW_EFMC_AHB_MEM_ADDRESS0}
};

/*-----------------------------------------------------------------------------
* Exported functions
*-----------------------------------------------------------------------------*/



