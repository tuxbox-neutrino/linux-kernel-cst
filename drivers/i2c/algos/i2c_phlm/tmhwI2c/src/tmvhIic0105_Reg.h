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


#ifndef TMHWIIC0105_VHIP_H
#define TMHWIIC0105_VHIP_H

#ifdef __cplusplus
extern "C" {  /* Assume C declarations for C++ */
#endif

/* ---------------------------------------------------------------------------
 *  Register offsets from base address
 * ---------------------------------------------------------------------------
 */

#define TMVH_IIC0105_I2CCONTROL_OFFSET          0x0000
#define TMVH_IIC0105_I2CDAT_OFFSET              0x0004
#define TMVH_IIC0105_I2CSTATUS_OFFSET           0x0008
#define TMVH_IIC0105_I2C_ADDR_OFFSET            0x000C
#define TMVH_IIC0105_I2C_STOP_OFFSET            0x0010
#define TMVH_IIC0105_I2C_PD_OFFSET              0x0014
#define TMVH_IIC0105_I2C_SETPINS_OFFSET         0x0018
#define TMVH_IIC0105_I2C_OBS_PINS_OFFSET        0x001C
#define TMVH_IIC0105_INT_STATUS_OFFSET          0x0FE0
#define TMVH_IIC0105_INT_ENABLE_OFFSET          0x0FE4
#define TMVH_IIC0105_INT_CLEAR_OFFSET           0x0FE8
#define TMVH_IIC0105_INT_SET_OFFSET             0x0FEC
#define TMVH_IIC0105_I2C_POWERDOWN_OFFSET       0x0FF4
#define TMVH_IIC0105_MODULE_ID_OFFSET           0x0FFC



/* ---------------------------------------------------------------------------
 *  Control register
 * ---------------------------------------------------------------------------
 */

#define TMVH_IIC0105_I2CCONTROL_AA_POS                 7
#define TMVH_IIC0105_I2CCONTROL_AA_MSK                 0x00000080u

#define TMVH_IIC0105_I2CCONTROL_EN_I2C_POS             6
#define TMVH_IIC0105_I2CCONTROL_EN_I2C_MSK             0x00000040u

#define TMVH_IIC0105_I2CCONTROL_START_POS              5
#define TMVH_IIC0105_I2CCONTROL_START_MSK              0x00000020u

#define TMVH_IIC0105_I2CCONTROL_SETSTOP_POS            4
#define TMVH_IIC0105_I2CCONTROL_SETSTOP_MSK            0x00000010

#define TMVH_IIC0105_I2CCONTROL_CR2_POS                0
#define TMVH_IIC0105_I2CCONTROL_CR2_MSK                0x00000007u



/* ---------------------------------------------------------------------------
 *  Data register
 * ---------------------------------------------------------------------------
 */

#define TMVH_IIC0105_I2CDAT_DATA_POS                   0
#define TMVH_IIC0105_I2CDAT_DATA_MSK                   0x000000FFu



/* ---------------------------------------------------------------------------
 *  Status register
 * ---------------------------------------------------------------------------
 */

/* NOTE: 
 * The datasheet specifies that bits 7:3 contain the status code. 
 * And bits 2:0 are reserved. However, status code is 8bit long. 
 * Hence the status code mask will be bit positions 7:3, but no
 * right shifting must be done. Hence the last 3 bits will always 
 * be zeros.(PR PB5#54)
 */
#define TMVH_IIC0105_I2CSTATUS_STATUS_CODE_POS         0
#define TMVH_IIC0105_I2CSTATUS_STATUS_CODE_MSK         0x000000F8



/* ---------------------------------------------------------------------------
 *  Slave address register
 * ---------------------------------------------------------------------------
 */

#define TMVH_IIC0105_I2C_ADDR_SLAVE_ADDR_POS           1
#define TMVH_IIC0105_I2C_ADDR_SLAVE_ADDR_MSK           0x000000FEu

#define TMVH_IIC0105_I2C_ADDR_WATCH_GC_POS             0
#define TMVH_IIC0105_I2C_ADDR_WATCH_GC_MSK             0x00000001u



/* ---------------------------------------------------------------------------
 *  I2C STOP register
 * ---------------------------------------------------------------------------
 */

#define TMVH_IIC0105_I2C_STOP_STO_POS                  0
#define TMVH_IIC0105_I2C_STOP_STO_MSK                  0x00000001


/* ---------------------------------------------------------------------------
 *  I2C PD register
 * ---------------------------------------------------------------------------
 */

#define TMVH_IIC0105_I2C_PD_RESET_CLKDOMAIN_POS        2
#define TMVH_IIC0105_I2C_PD_RESET_CLKDOMAIN_MSK        0x00000004



/* ---------------------------------------------------------------------------
 *  I2C Bus set register
 * ---------------------------------------------------------------------------
 */

#define TMVH_IIC0105_I2C_SETPINS_SETSCL_LOW_POS        1
#define TMVH_IIC0105_I2C_SETPINS_SETSCL_LOW_MSK        0x00000002

#define TMVH_IIC0105_I2C_SETPINS_SETSDA_LOW_POS        0
#define TMVH_IIC0105_I2C_SETPINS_SETSDA_LOW_MSK        0x00000001



/* ---------------------------------------------------------------------------
 *  I2C Bus observation register
 * ---------------------------------------------------------------------------
 */

#define TMVH_IIC0105_I2C_OBS_PINS_OBS_SCL_POS          1
#define TMVH_IIC0105_I2C_OBS_PINS_OBS_SCL_MSK          0x00000002

#define TMVH_IIC0105_I2C_OBS_PINS_OBS_SDA_POS          0
#define TMVH_IIC0105_I2C_OBS_PINS_OBS_SDA_MSK          0x00000001



/* ---------------------------------------------------------------------------
 *  Interrupt Status register
 * ---------------------------------------------------------------------------
 */

#define TMVH_IIC0105_INT_STATUS_STA_POS                0
#define TMVH_IIC0105_INT_STATUS_STA_MSK                0x00000001



/* ---------------------------------------------------------------------------
 *  Interrupt Enable register
 * ---------------------------------------------------------------------------
 */

#define TMVH_IIC0105_INT_ENABLE_EN_POS                 0
#define TMVH_IIC0105_INT_ENABLE_EN_MSK                 0x00000001u


/* ---------------------------------------------------------------------------
 *  Interrupt Clear register
 * ---------------------------------------------------------------------------
 */

#define TMVH_IIC0105_INT_CLEAR_CLR_POS                 0
#define TMVH_IIC0105_INT_CLEAR_CLR_MSK                 0x00000001


/* ---------------------------------------------------------------------------
 *  Interrupt Set register
 * ---------------------------------------------------------------------------
 */

#define TMVH_IIC0105_INT_SET_SET0_POS                  0
#define TMVH_IIC0105_INT_SET_SET0_MSK                  0x00000001


/* ---------------------------------------------------------------------------
 *  Power Setting register
 * ---------------------------------------------------------------------------
 */

#define TMVH_IIC0105_I2C_POWERDOWN_PWR_DOWN_POS        31
#define TMVH_IIC0105_I2C_POWERDOWN_PWR_DOWN_MSK        0x80000000u


/* ---------------------------------------------------------------------------
 *  Module ID register
 * ---------------------------------------------------------------------------
 */

#define TMVH_IIC0105_MODULE_ID_MODID_POS               16
#define TMVH_IIC0105_MODULE_ID_MODID_MSK               0xFFFF0000

#define TMVH_IIC0105_MODULE_ID_MAJ_REV_POS             12
#define TMVH_IIC0105_MODULE_ID_MAJ_REV_MSK             0x0000F000

#define TMVH_IIC0105_MODULE_ID_MIN_REV_POS             8
#define TMVH_IIC0105_MODULE_ID_MIN_REV_MSK             0x00000F00

#define TMVH_IIC0105_MODULE_ID_APERTURE_POS            0
#define TMVH_IIC0105_MODULE_ID_APERTURE_MSK            0x000000FF



#ifdef __cplusplus
}
#endif

#endif /*TMHWIIC0105_VHIP_H*/
