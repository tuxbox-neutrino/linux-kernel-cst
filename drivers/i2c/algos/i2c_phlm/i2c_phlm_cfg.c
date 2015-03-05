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


#include <linux/i2c.h>
#include <linux/irq.h>
#include "i2c_phlm_cfg.h"
#include "tmhwI2c.h"

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

/* ***********************************************
 *  The device register bases
 *  *********************************************** */

#if defined (CONFIG_ARCH_APOLLO)
unsigned long i2c_phlm_cfg_intpins[TMHW_I2C_UNIT_MAX] =
{ 55, 56, 57 ,149};
#elif (defined (CONFIG_ARCH_KRONOS) || defined (CONFIG_ARCH_KROME) || defined (CONFIG_ARCH_KORE3))
unsigned long i2c_phlm_cfg_intpins[TMHW_I2C_UNIT_MAX] =
{ 55, 56, 57};
#elif defined (CONFIG_ARCH_PULSAR)
unsigned long i2c_phlm_cfg_intpins[TMHW_I2C_UNIT_MAX] =
{ 55, 56};
#endif

struct i2c_adapter i2c_phlm_cfg_adapters[TMHW_I2C_UNIT_MAX];

void i2c_phlm_cfg_init(unsigned int num_units)
{
	return;
}

#ifdef __cplusplus
}
#endif /* __cplusplus */





