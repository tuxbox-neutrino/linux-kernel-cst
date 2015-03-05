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

#if !defined(SDE4_BUILD)
#include <HwAPI/tmSysCfg.h>
#endif

#include <HwAPI/tmNxTypes.h>
#include <asm/io.h>

#include <tmhwI2c.h>
#include <tmhwI2c_Cfg.h>

#if defined CONFIG_ARCH_APOLLO
/* Apollo */
#undef TMHW_I2C_UNIT_MAX
#define TMHW_I2C_UNIT_MAX 4

#ifndef TMHW_I2C_PCFKHZ
#define TMHW_I2C_PCFKHZ {24000, 24000, 24000, 24000}
#endif
#ifndef TMHW_I2C_HS
#define TMHW_I2C_HS {False, False, False, False}
#endif
#ifndef TMHW_I2C_HSMCODE
#define TMHW_I2C_HSMCODE {0x08,0x09, 0x0A, 0x0B} /*FIXME::CHECK If this is indeed 0x0B*/
#endif
#ifndef TMHW_I2C_MAXFSSPEEDKHZ
#define TMHW_I2C_MAXFSSPEEDKHZ {TMHW_I2C_MAX_FS_SPEED, TMHW_I2C_MAX_FS_SPEED, TMHW_I2C_MAX_FS_SPEED, TMHW_I2C_MAX_FS_SPEED}
#endif

#ifndef TMHW_I2C_MAXHSSPEEDKHZ
#define TMHW_I2C_MAXHSSPEEDKHZ {TMHW_I2C_MAX_HS_SPEED, TMHW_I2C_MAX_HS_SPEED, TMHW_I2C_MAX_HS_SPEED, TMHW_I2C_MAX_HS_SPEED}
#endif
#ifndef TMHW_I2C_DMA
#define TMHW_I2C_DMA { True, True, True, False}
#endif

#elif (defined (CONFIG_ARCH_KRONOS) || defined (CONFIG_ARCH_KROME) || defined (CONFIG_ARCH_KORE3))

/* Kronos/Krome/Kore3 */
#undef TMHW_I2C_UNIT_MAX
#define TMHW_I2C_UNIT_MAX 3

#ifndef TMHW_I2C_PCFKHZ
#define TMHW_I2C_PCFKHZ {24000, 24000, 24000}
#endif
#ifndef TMHW_I2C_HS
#define TMHW_I2C_HS {False, False, False}
#endif
#ifndef TMHW_I2C_HSMCODE
#define TMHW_I2C_HSMCODE {0x08,0x09, 0x0A} 
#endif
#ifndef TMHW_I2C_MAXFSSPEEDKHZ
#define TMHW_I2C_MAXFSSPEEDKHZ {TMHW_I2C_MAX_FS_SPEED, TMHW_I2C_MAX_FS_SPEED, TMHW_I2C_MAX_FS_SPEED}
#endif

#ifndef TMHW_I2C_MAXHSSPEEDKHZ
#define TMHW_I2C_MAXHSSPEEDKHZ {TMHW_I2C_MAX_HS_SPEED, TMHW_I2C_MAX_HS_SPEED, TMHW_I2C_MAX_HS_SPEED}
#endif
#ifndef TMHW_I2C_DMA
#define TMHW_I2C_DMA { True, True, True}
#endif

#elif defined (CONFIG_ARCH_PULSAR)

/* Pulsar */
#undef TMHW_I2C_UNIT_MAX
#define TMHW_I2C_UNIT_MAX 2

#ifndef TMHW_I2C_PCFKHZ
#define TMHW_I2C_PCFKHZ {24000, 24000}
#endif
#ifndef TMHW_I2C_HS
#define TMHW_I2C_HS {False, False}
#endif
#ifndef TMHW_I2C_HSMCODE
#define TMHW_I2C_HSMCODE {0x08,0x09} 
#endif
#ifndef TMHW_I2C_MAXFSSPEEDKHZ
#define TMHW_I2C_MAXFSSPEEDKHZ {TMHW_I2C_MAX_FS_SPEED, TMHW_I2C_MAX_FS_SPEED}
#endif

#ifndef TMHW_I2C_MAXHSSPEEDKHZ
#define TMHW_I2C_MAXHSSPEEDKHZ {TMHW_I2C_MAX_HS_SPEED, TMHW_I2C_MAX_HS_SPEED}
#endif
#ifndef TMHW_I2C_DMA
#define TMHW_I2C_DMA { True, True}
#endif

#else
#error "Arch not supported"
#endif 


/* The peripheral clock frequency among others determines how the speed
 *  registers must be set to obtain a certain i2c-bus speed.
 */
static const UInt32 i2cPeripheralClockFrequencyKHz[TMHW_I2C_UNIT_MAX] = TMHW_I2C_PCFKHZ;

/* All units have high speed mode disabled */
static const Bool i2cHS[TMHW_I2C_UNIT_MAX] =TMHW_I2C_HS ;

/* the master code must be unique over the i2c-bus where the particular
 * master is connected to.
 */
static const UInt8 i2cHsMasterCode[TMHW_I2C_UNIT_MAX] = TMHW_I2C_HSMCODE;

static const UInt32 i2cMaxFSSpeedKhz[TMHW_I2C_UNIT_MAX] = TMHW_I2C_MAXFSSPEEDKHZ ;

static const UInt32 i2cMaxHsSpeedKhz[TMHW_I2C_UNIT_MAX] = TMHW_I2C_MAXHSSPEEDKHZ;

static const Bool i2cDma[TMHW_I2C_UNIT_MAX] = TMHW_I2C_DMA;

/* Timeout for the data transmitt */
#ifndef TMHW_I2C_TIMEOUT
#define TMHW_I2C_TIMEOUT  1000
#endif


/*----------------------------------------------------------------------------
 * Global data:
 *----------------------------------------------------------------------------
 */

#define I2C_MEM_SIZE        (4*1024)

extern tmUnitSelect_t       gI2cIP3203UnitCount;
extern tmUnitSelect_t       gI2cIP0105UnitCount;

#define  TMHW_I2C_CFG_NUM_UNITS     TMHW_I2C_UNIT_MAX

/* tmhwI2c configuration structure array. This array will have size equal to */
/* number of units available in I2c hardware. */
static tmhwI2c_Cfg_t  gI2c_Cfg[TMHW_I2C_CFG_NUM_UNITS];
static Bool gbI2c_CfgInit = False;  /* Flag to know if configuration been initialized */

static UInt32 tmhwI2c_Cfg_numunits = ~0L;


#if defined CONFIG_ARCH_APOLLO
UInt32 tmhwI2c_CfgPhyAddr[TMHW_I2C_CFG_UNIT_COUNT] =
{
	TMHW_I2C_CFG_REG_START + 0x0C5000, // I2c unit 0
	TMHW_I2C_CFG_REG_START + 0x0C6000, // I2c unit 1
	TMHW_I2C_CFG_REG_START + 0x0C7000, // I2c unit 2
	TMHW_I2C_CFG_REG_START + 0x0C8000, // I2c unit 3
};

#elif (defined (CONFIG_ARCH_KRONOS) || defined (CONFIG_ARCH_KROME) ||  defined (CONFIG_ARCH_KORE3))
UInt32 tmhwI2c_CfgPhyAddr[TMHW_I2C_CFG_UNIT_COUNT] =
{
	TMHW_I2C_CFG_REG_START + 0x08D000, // I2c unit 0
	TMHW_I2C_CFG_REG_START + 0x08E000, // I2c unit 1
	TMHW_I2C_CFG_REG_START + 0x08F000, // I2c unit 2
};
#elif defined (CONFIG_ARCH_PULSAR)
UInt32 tmhwI2c_CfgPhyAddr[TMHW_I2C_CFG_UNIT_COUNT] =
{
	TMHW_I2C_CFG_REG_START + 0x08E000, // I2c unit 0
	TMHW_I2C_CFG_REG_START + 0x08F000, // I2c unit 1
};

#endif


/*----------------------------------------------------------------------------
 * Internal Prototypes:
 *----------------------------------------------------------------------------
 */
void  i2c_CfgGetModulueInfo(void);

/*----------------------------------------------------------------------------
 * tmhwI2c configuration structure array. This array will have size equal to
 * number of units available in I2c hardware.
 *-----------------------------------------------------------------------------
 * FUNCTION:    i2c_CfgGetModulueInfo
 *
 * DESCRIPTION: Retrieves base addresses of the I2c units using ioremap function.
 *
 * RETURN:      None
 *
 * NOTES:
 *
 *-----------------------------------------------------------------------------
 */
void  i2c_CfgGetModulueInfo(void)
{
	tmUnitSelect_t          unit = 0;


	/* pnx8550 */
	if( gbI2c_CfgInit == False)
	{


		for(unit=0; unit < TMHW_I2C_UNIT_MAX; unit++ )
		{
			gI2c_Cfg[unit].baseAddress = (volatile UInt32)ioremap(tmhwI2c_CfgPhyAddr[unit], I2C_MEM_SIZE );
			gI2c_Cfg[unit].moduleID    = I2C_IP3203_HWMODULE_ID;
		}
		gI2cIP3203UnitCount = 2;

		gI2cIP0105UnitCount = 0;

		tmhwI2c_Cfg_numunits = TMHW_I2C_UNIT_MAX;

		gbI2c_CfgInit = True;
	}


}


/* tmhwI2c configuration state structure array. This array will have size equal to
 * number of units available in I2c hardware. At this stage of the development
 * process the contents and purpose of state structure has not been decided yet.
 */
static tmhwI2c_CfgState_t  gI2c_CfgState[TMHW_I2C_CFG_NUM_UNITS] =
{
	{
		0     /* Unit number 0 */
	}
};


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
 * NOTES:       -
 *----------------------------------------------------------------------------
 */
Void tmhwI2cCfgInit(tmUnitSelect_t i2cUnit, ptmhwI2cCfg_t gpI2cCfg)
{
	/* define the peripheral clock frequency */
	gpI2cCfg->clockFreqKHz  = i2cPeripheralClockFrequencyKHz[i2cUnit];

	/* define whether HS of the indicated unit is available and enabled */
	gpI2cCfg->bHS           = i2cHS[i2cUnit];

	/* define the master code of the indicated unit */
	gpI2cCfg->hsMasterCode  = i2cHsMasterCode[i2cUnit];

	/* define the maximumn i2c communication speed when in FS (max 400) */
	gpI2cCfg->maxFSSpeedKhz = i2cMaxFSSpeedKhz[i2cUnit];

	/* define the maximumn i2c communication speed when in HS (max 3400) */
	gpI2cCfg->maxHsSpeedKhz = i2cMaxHsSpeedKhz[i2cUnit];

	gpI2cCfg->bDma          = i2cDma[ i2cUnit ];

	/* Timeout for the transmitt */
	gpI2cCfg->timeout = TMHW_I2C_TIMEOUT;
}/* tmhwI2cCfgInit */



/*-----------------------------------------------------------------------------
 * FUNCTION:    tmhwI2c_CfgGet
 *
 * DESCRIPTION: Initializes the tmhwI2c configuration structure array and returns
 *              it's pointer
 *-----------------------------------------------------------------------------
 */
ptmhwI2c_Cfg_t  tmhwI2c_CfgGet(void)
{

	i2c_CfgGetModulueInfo();

	return gI2c_Cfg;
}


/*----------------------------------------------------------------------------
 * FUNCTION:    tmhwI2c_CfgGetState
 *
 * DESCRIPTION: Returns pointer to the tmhwI2c configuration state structure array
 *
 *----------------------------------------------------------------------------
 */
ptmhwI2c_CfgState_t  tmhwI2c_CfgGetState(void)
{
	return gI2c_CfgState;
}


/*----------------------------------------------------------------------------
 * FUNCTION:    tmhwI2c_CfgGetNumUnits
 *
 * DESCRIPTION: Returns total number of units available in I2c hardware.
 *
 * NOTE       : IMPORTANT: tmhwI2c_CfgGet() must have been run atleast once
 *              before running tmhwI2c_CfgGetNumUnits(). If not default value -1
 *              is returned.
 *----------------------------------------------------------------------------
 */
UInt32  tmhwI2c_CfgGetNumUnits(void)
{
	return (tmhwI2c_Cfg_numunits);
}


