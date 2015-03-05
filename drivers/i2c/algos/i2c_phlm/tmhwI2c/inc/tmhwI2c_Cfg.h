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



#ifndef TMHW_I2C_CFG_H
#define TMHW_I2C_CFG_H

/*----------------------------------------------------------------------------
 * Standard include files:
 *----------------------------------------------------------------------------
 */
#ifdef LINUX_BUILD
#include <HwAPI/tmNxTypes.h>
#else
#include <tmNxTypes.h>
#endif


/*----------------------------------------------------------------------------
 * Project include files:
 *----------------------------------------------------------------------------
 */
#include "tmhwI2c.h"


#ifdef __cplusplus
extern "C"
{
#endif
/*----------------------------------------------------------------------------
 * Types and defines:
 *----------------------------------------------------------------------------
 */
#define I2C_IP3203_HWMODULE_ID          0X00003203
#define IIC_IP0105_HWMODULE_ID          0X00000105

/*----------------------------------------------------------------------------
 *  The device register bases for Linux
 *----------------------------------------------------------------------------
 */

#ifdef LINUX_BUILD

#if defined CONFIG_ARCH_APOLLO
#define TMHW_I2C_CFG_UNIT_COUNT         4           /* Number of I2C hardware unit */
#define TMHW_I2C_CFG_REG_START          0xE0600000  /* I2C base register offset in system */

#elif (defined (CONFIG_ARCH_KRONOS) || defined (CONFIG_ARCH_KROME) || defined (CONFIG_ARCH_KORE3))
#define TMHW_I2C_CFG_UNIT_COUNT         3           /* Number of I2C hardware unit */
#define TMHW_I2C_CFG_REG_START          0xE0600000  /* I2C base register offset in system */

#elif defined (CONFIG_ARCH_PULSAR)
#define TMHW_I2C_CFG_UNIT_COUNT         2           /* Number of I2C hardware unit */
#define TMHW_I2C_CFG_REG_START          0xE0600000  /* I2C base register offset in system */

#else

#define TMHW_I2C_CFG_UNIT_COUNT         4           /* Number of I2C hardware unit */
#define TMHW_I2C_CFG_REG_START          0x1BE00000  /* I2C base register offset in system */

#endif

extern UInt32 tmhwI2c_CfgPhyAddr[TMHW_I2C_CFG_UNIT_COUNT];
#endif

/* Standard configuration structures */

typedef struct tmhwI2c_Cfg
{
	UInt32 baseAddress;
	UInt32 moduleID;
}tmhwI2c_Cfg_t, *ptmhwI2c_Cfg_t;


/* tmhwI2c configuration state structure. At this stage of development, the contents of this structure
 * have not been finalised.
 */
typedef struct tmhwI2c_CfgState
{
	UInt32 intNum;
}tmhwI2c_CfgState_t, *ptmhwI2c_CfgState_t;


/* Pre-existing Configuration structure. TO DO: Might be merged with standard configuration
 * "state" strucure
 */
typedef struct _tmhwI2cCfg_t
{
	UInt32      clockFreqKHz;
	Bool        bHS;
	UInt8       hsMasterCode;
	UInt32      maxFSSpeedKhz;
	UInt32      maxHsSpeedKhz;      /* preferably 400 <= maxHsSpeedKhz <= 3400 */
	Bool        bDma;
	UInt32      timeout;            /* Timeout in ms for the data transction.
					 * This timeout should be grater than time required to
					 * transmit largest possible message.
					 * 0 means Infinite wait */
} tmhwI2cCfg_t, *ptmhwI2cCfg_t;

/*----------------------------------------------------------------------------
 * Exported references:
 *----------------------------------------------------------------------------
 */
extern tmUnitSelect_t      gI2cIP3203UnitCount;
extern tmUnitSelect_t      gI2cIP0105UnitCount;


/*----------------------------------------------------------------------------
 * Exported functions:
 *----------------------------------------------------------------------------
 */

/*----------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cCfgInit:
 *
 * DESCRIPTION: Run time intialisation for unit i2cUnit
 *
 * RETURN:      -
 *
 * NOTES:       None
 *----------------------------------------------------------------------------
 */
Void tmhwI2cCfgInit(tmUnitSelect_t i2cUnit, ptmhwI2cCfg_t gpI2cCfg);

#ifdef __cplusplus
}
#endif  /* __cplusplus */


#endif /* TMHW_I2C_CFG_H */

