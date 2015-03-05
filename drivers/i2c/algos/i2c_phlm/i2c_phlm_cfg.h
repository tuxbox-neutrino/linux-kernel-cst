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


#ifndef I2CPHCFG_H
#define I2CPHCFG_H

#ifdef __cplusplus
extern "C"
{
#endif

#define I2C_PHCFG_DEBUG

#define I2C_PHCFG_TIMEOUT         500 /* min 500millisecs for master transfer timeout*/
#define I2C_PHCFG_SLV_TIMEOUT     500 /* min 500millisecs for slave transfer timeout*/
#define I2C_PHCFG_NUM_RETIRES     3

/* Interrupt lines on the board for i2c units 1,2...*/
extern unsigned long i2c_phlm_cfg_intpins[];
extern struct i2c_adapter i2c_phlm_cfg_adapters[];
void i2c_phlm_cfg_init(unsigned int num_units);


#ifdef __cplusplus
}
#endif


#endif
