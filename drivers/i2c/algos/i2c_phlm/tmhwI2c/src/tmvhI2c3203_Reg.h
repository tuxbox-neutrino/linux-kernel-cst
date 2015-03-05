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


#ifndef TMHWI2C3203_VHIP_H
#define TMHWI2C3203_VHIP_H

#ifdef __cplusplus
extern "C" {  /* Assume C declarations for C++ */
#endif

/* ---------------------------------------------------------------------------
 *  Register offsets from base address
 * ---------------------------------------------------------------------------
 */

#define TMVH_I2C3203_I2CCON_OFFSET              0x0000
#define TMVH_I2C3203_I2CSTAT_OFFSET             0x0004
#define TMVH_I2C3203_I2CDAT_OFFSET              0x0008
#define TMVH_I2C3203_I2CSLA_OFFSET              0x000C
#define TMVH_I2C3203_HSBIR_OFFSET               0x0010
#define TMVH_I2C3203_FSBIR_OFFSET               0x0014
#define TMVH_I2C3203_INTROG_OFFSET              0x0018
#define TMVH_I2C3203_DMA_ADDR_OFFSET            0x0020
#define TMVH_I2C3203_DMA_LEN_OFFSET             0x0024
#define TMVH_I2C3203_DMA_COUNTER_OFFSET         0x0028
#define TMVH_I2C3203_DMA_CONTROL_OFFSET         0x002C
#define TMVH_I2C3203_DMA_STATUS_OFFSET          0x0030
#define TMVH_I2C3203_INT_STATUS_OFFSET          0x0FE0
#define TMVH_I2C3203_INT_ENABLE_OFFSET          0x0FE4
#define TMVH_I2C3203_INT_CLEAR_OFFSET           0x0FE8
#define TMVH_I2C3203_INT_SET_OFFSET             0x0FEC
#define TMVH_I2C3203_POWERDOWN_OFFSET           0x0FF4
#define TMVH_I2C3203_MODULE_ID_OFFSET           0x0FFC

#define TMVH_I2C3203_SDA_HOLD                   0x0001c


/* ---------------------------------------------------------------------------
 *  Control register
 * ---------------------------------------------------------------------------
 */

#define TMVH_I2C3203_I2CCON_EN_I2C_POS                 6
#define TMVH_I2C3203_I2CCON_EN_I2C_MSK                 0x00000040

#define TMVH_I2C3203_I2CCON_START_POS                  5
#define TMVH_I2C3203_I2CCON_START_MSK                  0x00000020

#define TMVH_I2C3203_I2CCON_SETSTOP_POS                4
#define TMVH_I2C3203_I2CCON_SETSTOP_MSK                0x00000010

#define TMVH_I2C3203_I2CCON_AA_POS                     2
#define TMVH_I2C3203_I2CCON_AA_MSK                     0x00000004u


/* ---------------------------------------------------------------------------
 *  Status register
 * ---------------------------------------------------------------------------
 */

#define TMVH_I2C3203_I2CSTAT_STATUS_CODE_POS           0
#define TMVH_I2C3203_I2CSTAT_STATUS_CODE_MSK           0x000000FF


/* ---------------------------------------------------------------------------
 *  Data register
 * ---------------------------------------------------------------------------
 */

#define TMVH_I2C3203_I2CDAT_DATA_POS                   0
#define TMVH_I2C3203_I2CDAT_DATA_MSK                   0x000000FFu


/* ---------------------------------------------------------------------------
 *  Slave address register
 * ---------------------------------------------------------------------------
 */

#define TMVH_I2C3203_I2CSLA_SLAVE_ADDR_POS             1
#define TMVH_I2C3203_I2CSLA_SLAVE_ADDR_MSK             0x000000FEu

#define TMVH_I2C3203_I2CSLA_WATCH_GC_POS               0
#define TMVH_I2C3203_I2CSLA_WATCH_GC_MSK               0x00000001u


/* ---------------------------------------------------------------------------
 *  High speed mode bit rate register
 * ---------------------------------------------------------------------------
 */

#define TMVH_I2C3203_HSBIR_BITRATE_POS                 0
#define TMVH_I2C3203_HSBIR_BITRATE_MSK                 0x0000001F


/* ---------------------------------------------------------------------------
 *  Fast/Standard mode bit rate register
 * ---------------------------------------------------------------------------
 */

#define TMVH_I2C3203_FSBIR_MODE_POS                    7
#define TMVH_I2C3203_FSBIR_MODE_MSK                    0x00000080u

#define I2C_SS_MODE                                    0
#define I2C_FS_MODE                                    1

#define TMVH_I2C3203_FSBIR_BITRATE_POS                 0
#define TMVH_I2C3203_FSBIR_BITRATE_MSK                 0x0000007Fu


/* ---------------------------------------------------------------------------
 *  Interrogate register
 * ---------------------------------------------------------------------------
 */

#define TMVH_I2C3203_INTROG_INTRO_MST_POS              7
#define TMVH_I2C3203_INTROG_INTRO_MST_MSK              0x00000080

#define TMVH_I2C3203_INTROG_INTRO_TRX_POS              6
#define TMVH_I2C3203_INTROG_INTRO_TRX_MSK              0x00000040

#define TMVH_I2C3203_INTROG_INTRO_HS_POS               5
#define TMVH_I2C3203_INTROG_INTRO_HS_MSK               0x00000020

#define TMVH_I2C3203_INTROG_INTRO_STO_POS              4
#define TMVH_I2C3203_INTROG_INTRO_STO_MSK              0x00000010

#define TMVH_I2C3203_INTROG_INTRO_SI_POS               3
#define TMVH_I2C3203_INTROG_INTRO_SI_MSK               0x00000008

#define TMVH_I2C3203_INTROG_INTRO_BB_POS               2
#define TMVH_I2C3203_INTROG_INTRO_BB_MSK               0x00000004

#define TMVH_I2C3203_INTROG_INTRO_SEL_POS              1
#define TMVH_I2C3203_INTROG_INTRO_SEL_MSK              0x00000002

#define TMVH_I2C3203_INTROG_INTRO_ACK_POS              0
#define TMVH_I2C3203_INTROG_INTRO_ACK_MSK              0x00000001



/* ---------------------------------------------------------------------------
 *  DMA Length register
 * ---------------------------------------------------------------------------
 */

#define TMVH_I2C3203_DMA_LEN_SIZE_POS                  0
#define TMVH_I2C3203_DMA_LEN_SIZE_MSK                  0x0000FFFF


/* ---------------------------------------------------------------------------
 *  DMA Counter register
 * ---------------------------------------------------------------------------
 */

#define TMVH_I2C3203_DMA_COUNTER_LEN_POS               0
#define TMVH_I2C3203_DMA_COUNTER_LEN_MSK               0x0000FFFF


/* ---------------------------------------------------------------------------
 *  DMA Control register
 * ---------------------------------------------------------------------------
 */

#define TMVH_I2C3203_DMA_CONTROL_SLAVE_TX_POS          3
#define TMVH_I2C3203_DMA_CONTROL_SLAVE_TX_MSK          0x00000008

#define TMVH_I2C3203_DMA_CONTROL_SLAVE_RX_POS          2
#define TMVH_I2C3203_DMA_CONTROL_SLAVE_RX_MSK          0x00000004

#define TMVH_I2C3203_DMA_CONTROL_MASTER_RX_POS         1
#define TMVH_I2C3203_DMA_CONTROL_MASTER_RX_MSK         0x00000002

#define TMVH_I2C3203_DMA_CONTROL_MASTER_TX_POS         0
#define TMVH_I2C3203_DMA_CONTROL_MASTER_TX_MSK         0x00000001



/* ---------------------------------------------------------------------------
 *  DMA Status register
 * ---------------------------------------------------------------------------
 */

#define TMVH_I2C3203_DMA_STATUS_DMACOUNT_ZERO_POS      1
#define TMVH_I2C3203_DMA_STATUS_DMACOUNT_ZERO_MSK      0x00000002


/* ---------------------------------------------------------------------------
 *  Interrupt Status register
 * ---------------------------------------------------------------------------
 */

#define TMVH_I2C3203_INT_STATUS_CODE_POS               24
#define TMVH_I2C3203_INT_STATUS_CODE_MSK               0xFF000000u

#define TMVH_I2C3203_INT_STATUS_STA0_POS               0
#define TMVH_I2C3203_INT_STATUS_STA0_MSK               0x00000001


/* ---------------------------------------------------------------------------
 *  Interrupt Enable register
 * ---------------------------------------------------------------------------
 */

#define TMVH_I2C3203_INT_ENABLE_EN_MODE_POS            0
#define TMVH_I2C3203_INT_ENABLE_EN_MODE_MSK            0x00000001u


/* ---------------------------------------------------------------------------
 *  Interrupt Clear register
 * ---------------------------------------------------------------------------
 */

#define TMVH_I2C3203_INT_CLEAR_INTCLR_POS              0
#define TMVH_I2C3203_INT_CLEAR_INTCLR_MSK              0x00000001


/* ---------------------------------------------------------------------------
 *  Interrupt Set register
 * ---------------------------------------------------------------------------
 */

#define TMVH_I2C3203_INT_SET_SET0_POS                  0
#define TMVH_I2C3203_INT_SET_SET0_MSK                  0x00000001


/* ---------------------------------------------------------------------------
 *  Power Setting register
 * ---------------------------------------------------------------------------
 */

#define TMVH_I2C3203_POWERDOWN_PWR_DOWN_POS            31
#define TMVH_I2C3203_POWERDOWN_PWR_DOWN_MSK            0x80000000u


/* ---------------------------------------------------------------------------
 *  Module ID register
 * ---------------------------------------------------------------------------
 */

#define TMVH_I2C3203_MODULE_ID_MODID_POS               16
#define TMVH_I2C3203_MODULE_ID_MODID_MSK               0xFFFF0000

#define TMVH_I2C3203_MODULE_ID_MAJ_REV_POS             12
#define TMVH_I2C3203_MODULE_ID_MAJ_REV_MSK             0x0000F000

#define TMVH_I2C3203_MODULE_ID_MIN_REV_POS             8
#define TMVH_I2C3203_MODULE_ID_MIN_REV_MSK             0x00000F00

#define TMVH_I2C3203_MODULE_ID_APERTURE_POS            0
#define TMVH_I2C3203_MODULE_ID_APERTURE_MSK            0x000000FF


#ifdef __cplusplus
}
#endif


#endif /*TMHWI2C3203_VHIP_H*/
