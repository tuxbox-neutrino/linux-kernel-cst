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


/* ---------------------------------------------------------------------------
 * Standard include files:
 * ---------------------------------------------------------------------------
 */
#ifndef LINUX_BUILD

#include <tmNxTypes.h>
#include <tmNxModId.h>
#include <tmFlags.h>

#else

#include <HwAPI/tmNxTypes.h>
#include <HwAPI/tmNxModId.h>
#include <HwAPI/tmFlags.h>
#include <mach/soc.h>

#endif

#ifndef TMHWI2C_BSL_INDEPENDANT
#include <tmbslCore.h>
#endif


/* ---------------------------------------------------------------------------
 * Project include files:
 * ---------------------------------------------------------------------------
 */
#include "tmvhI2c3203_Reg.h"
#include "tmvhIic0105_Reg.h"
#include <tmhwI2c_Cfg.h>
#include <tmhwI2c.h>
#include <tmhwI2c_LocalCfg.h>

#include <linux/delay.h>

#ifdef LINUX_BUILD

#include <linux/i2c.h>
#include <linux/module.h>
#include <asm/io.h>
#include <asm/delay.h>

#else

#include <tmhwI2cIP3203.h>

#endif

#ifdef I2C_PRINTK
#include <asm/io.h>
#endif

/* ---------------------------------------------------------------------------
 * Types and defines:
 * ---------------------------------------------------------------------------
 */

/* version */
#define TMFL_I2C_INTERFACE_VERSION      1
#define TMFL_I2C_COMPATIBILITY_EXP      1
#define TMFL_I2C_MAJOR_VERSION_EXP      1

/* TIMEOUT should be wait period of 100ms i.e  100KHz
 * Not cosidering DMA full bandwidth.
 * with MIPS it is 325mips. Approximately 10 instructions per timeout loop
 */
//#define HWI2C_RESET_TIMEOUT 0x32DCD5

#ifdef LINUX_BUILD

/* i2cWaitForInt will have a delay(udelay) in the loop of 100us and execute
   the 10 instructions(which read i2c event) and
   the loop for 1000 times. This will give 100ms in theory.
   In practice, this could be little more than
   100ms( considering Kernel swapping, scheduling )
   but certainly not more than 200ms.*/

#define HWI2C_RESET_TIMEOUT 1000

#else
/* For 1ms delay, on 333 Mips
 * i.e (333 * 1000,000) * 1ms = 333000 instructions
 * 10 instructions per loop : gives 33300 loops = 0x8214
 */
#define HWI2C_RESET_TIMEOUT 0x8214
#endif

#if (TMFL_I2C_INTERFACE_VERSION != TMHW_I2C_INTERFACE_VERSION)
#error Wrong header file version is included
#endif /* TMFL_I2C_INTERFACE_VERSION */

#if (TMFL_I2C_COMPATIBILITY_EXP != TMHW_I2C_COMPATIBILITY_NR)
#error ERROR: Invalid I2C HWAPI compatibility number!
#endif /* TMFL_I2C_COMPATIBILITY_EXP */

#if (TMFL_I2C_MAJOR_VERSION_EXP > TMHW_I2C_MAJOR_VERSION_NR)
#error ERROR: Invalid I2C HWAPI major number!
#endif /* TMFL_I2C_MAJOR_VERSION_EXP */

/* error: */
#define I2C_BUFFER_EMPTY                  (TMHW_ERR_I2C_COMP+0x000a)
#define I2C_LAST_BYTE                     (TMHW_ERR_I2C_COMP+0x000b)
#define I2C_DMA_NOT_SUCCESSFUL            (TMHW_ERR_I2C_COMP+0x000c)


/* initialization value for variables */
#define I2C_DATA_INVALID             -1


/* Register Read Write Macro's  */
#ifdef LINUX_BUILD
#ifdef CONFIG_I2C_DEBUG
// log all read and writes to registers in local buffer,
// so we dump it on the console in case of errors
#define LOG_SIZE 64
static UInt32 G_logI2c[LOG_SIZE];
static UInt32 G_logI2cIndex = 0;

#define I2C_READ(address,result) do {				\
	(result) = *(volatile UInt32 *)(address);		\
	if ( (result) == 0xDEADABBA ) {				\
		printk( KERN_EMERG "I2C READ -- 0xDEADABBA line=%d a=0x%08x\n", __LINE__, (unsigned int)address); \
		(result) = *(volatile UInt32 *)(address);	\
	}							\
	G_logI2c[G_logI2cIndex++]= (0x0000ffff & ((unsigned int) (address))); \
	G_logI2c[G_logI2cIndex++]= (result);			\
	G_logI2cIndex %= LOG_SIZE;				\
} while(0)

#define I2C_WRITE(address,value) do {				\
	*(volatile UInt32 *)(address) = (value);		\
	{							\
		G_logI2c[G_logI2cIndex++] = (0x80000000 | (0x0000ffff & (unsigned int) (address))); \
		G_logI2c[G_logI2cIndex++] = (value);		\
		G_logI2cIndex %= LOG_SIZE;			\
	}							\
} while( 0 )
#else

#define I2C_READ(address,result) (result) = *(volatile UInt32 *)(address)
#define I2C_WRITE(address,value) *(volatile UInt32 *)(address) = (value)
#endif

#ifdef NOT_NOW_PLEASE_NO_NO

#define I2C_READ(address,result) do {				\
	(result) = *(volatile UInt32 *)(address);		\
	if( 0 )							\
	printk( KERN_EMERG "I2C_READ a=0x%08x, r=0x%08x\n", (unsigned int)address, (unsigned int)result); \
	if ((result)==0xDEADABBA) {				\
		printk( KERN_EMERG "I2C READ -- 0xDEADABBA line=%d a=0x%08x\n", __LINE__, (unsigned int)address); \
		(result) = *(volatile UInt32 *)(address);	\
	}							\
} while (0)

#define I2C_WRITE(address,value) do {				\
	*(volatile UInt32 *)(address) = (value);		\
	if( 0 ) printk("I2C_WRITE a=0x%08x, v=0x%08x\n",(unsigned int)address, (unsigned int)value); \
} while( 0 )
#endif /*CONFIG_I2C_DEBUG*/
#else
#define I2C_READ(address,result) (result) = *(volatile UInt32 *)(address)
#define I2C_WRITE(address,value) *(volatile UInt32 *)(address) = (value)
#endif /* LINUX_BUILD*/

/* #define I2C_DMA_WORD_ALINED */

typedef enum
{
	I2cIP0105 = 0,
	I2cIP3203
} I2cModuleType_t;

typedef struct _I2cModuleDesc
{
	I2cModuleType_t  I2cType;
	UInt32           pRegBase;
} I2cModuleDesc_t;

typedef struct _I2cRegisterMmFunc
{
	/* memory management functions for DMA use */
	ptmhwI2cVirtToPhys_t        gpVirtToPhys;
	ptmhwI2cCacheFlush_t        gpCacheFlush;
	ptmhwI2cCacheInvalidate_t   gpCacheInvalidate;
	/* intermediate DMA buffer */
	pUInt8                      bufferPtr;
#ifdef LINUX_BUILD
	UInt32                      phyAddr;
#endif
	UInt32                      length;
} I2cRegisterMmFunc_t, *pI2cRegisterMmFunc_t;

typedef enum
{
	tmhwI2cControlNone          = 0,
	tmhwI2cAcknowledge          = 1 << TMVH_I2C3203_I2CCON_AA_POS,
	tmhwI2cStopCond             = 1 << TMVH_I2C3203_I2CCON_SETSTOP_POS,
	tmhwI2cStartCond            = 1 << TMVH_I2C3203_I2CCON_START_POS,
	tmhwI2cEnable               = 1 << TMVH_I2C3203_I2CCON_EN_I2C_POS
} tmhwI2cControl_t, *ptmhwI2cControl_t;

typedef enum
{
	tmhwI2cDmaNone              = 0,
	tmhwI2cDmaMasterTransmit    = 1 << TMVH_I2C3203_DMA_CONTROL_MASTER_TX_POS,
	tmhwI2cDmaMasterReceive     = 1 << TMVH_I2C3203_DMA_CONTROL_MASTER_RX_POS,
	tmhwI2cDmaSlaveReceive      = 1 << TMVH_I2C3203_DMA_CONTROL_SLAVE_RX_POS,
	tmhwI2cDmaSlaveTransmit     = 1 << TMVH_I2C3203_DMA_CONTROL_SLAVE_TX_POS
} tmhwI2cDmaControl_t, *ptmhwI2cDmaControl_t;


/* i2c device states */
typedef enum
{
	tmhwI2cEvent0x00            = 0x00,     /* Bus Error */
	tmhwI2cEvent0x08            = 0x08,     /* A START Condition has been transmitted */
	tmhwI2cEvent0x10            = 0x10,     /* A Repeated START condition has been transmitted */
	tmhwI2cEvent0x18            = 0x18,     /* SLA.W has been transmitted, ACK has been received */
	tmhwI2cEvent0x20            = 0x20,     /* SLA.W or master code has been transmitted: NOT ACK has been received */
	tmhwI2cEvent0x28            = 0x28,     /* Data byte in I2CDAT has been transmitted, ACK received */
	tmhwI2cEvent0x30            = 0x30,     /* Data byte in I2CDAT has been transmitted, NOT ACK received */
	tmhwI2cEvent0x38            = 0x38,     /* Arbitration lost in SLA, R/W or in Data byte or in NOT ACK bit or in master code */
	tmhwI2cEvent0x40            = 0x40,     /* SLA.R has been transmitted, ACK received */
	tmhwI2cEvent0x48            = 0x48,     /* SLA.R or Master code has been transmitted, NOT ACK received */
	tmhwI2cEvent0x50            = 0x50,     /* Data byte has been received, ACK has been returned */
	tmhwI2cEvent0x58            = 0x58,     /* Data byte has been recieved; NOT ACK has been returned */
	tmhwI2cEvent0x60            = 0x60,     /* Own SLA.W has been rceived, ACK has been returned */
	tmhwI2cEvent0x68            = 0x68,     /* Arbitration lost in SLA.R/W as master and own SLA.W has received, ACK returned */
	tmhwI2cEvent0x70            = 0x70,     /* Generall call address (0x00) has been received, ACK returned */
	tmhwI2cEvent0x78            = 0x78,     /* Arbitration lost in SLA.R/W or in Master code ... ACK returned*/
	tmhwI2cEvent0x80            = 0x80,     /* Previously addressed with own SLA.W; Data byte has been received ACK returned */
	tmhwI2cEvent0x88            = 0x88,     /* Previously addressed with own SLA.W; Data byte received, NOT ACK returned */
	tmhwI2cEvent0x90            = 0x90,     /* Previously addressed with generall call; Data byte recieved; ACK returned */
	tmhwI2cEvent0x98            = 0x98,     /* Previously addressed with generall call; Data byte recieved; NOT ACK returned */
	tmhwI2cEvent0xa0            = 0xa0,     /* A STOP condition or repeated START condition has been received ... */
	tmhwI2cEvent0xa8            = 0xa8,     /* Own SLA.R is been received; ACK returned */
	tmhwI2cEvent0xb0            = 0xb0,     /* Arbitration lost in SLA.R/W as master and own SLA.R received, ACK sent */
	tmhwI2cEvent0xb8            = 0xb8,     /* Data byte in I2CDAT has been transmitted; ACK sent */
	tmhwI2cEvent0xc0            = 0xc0,     /* Data bytes in I2CDAT transmitted, NOT ACK is received */
	tmhwI2cEvent0xc8            = 0xc8,     /* Last data byte in I2CDAT has sent ACK received ... */
	tmhwI2cEvent0xf8            = 0xf8      /* No Information available, SI flag is '0' */
} tmhwI2cEvent_t, *ptmhwI2cEvent_t;


/* number of I2c units found */
static tmUnitSelect_t   gI2cUnitCount       = I2C_DATA_INVALID;
/* which one do we need? three vars hold redundancy ! */
tmUnitSelect_t      gI2cIP3203UnitCount = I2C_DATA_INVALID;
tmUnitSelect_t      gI2cIP0105UnitCount = I2C_DATA_INVALID;

/* The I2cDriverInit is implemented in a way that the IP_3203 units
 * always preceed the IP_0105 units, with possibly gI2cIP3203UnitCount=0
 */
/* Array of Module Descriptors */
static volatile I2cModuleDesc_t gI2cModule[TMHW_I2C_UNIT_MAX];

/* Devider values for the serial clock rate of the 0105 */
static const UInt32 gDivFact0105[] = {60, 80, 120, 160, 240, 320, 480, 960};

/* configuration data */
static tmhwI2cCfg_t gI2cCfg[TMHW_I2C_UNIT_MAX];

#ifndef TMHWI2C_BSL_INDEPENDANT
static UInt32 gTimeTicks20us; /* for STOP HW issue on 0105 */
#endif

/* ---------------------------------------------------------------------------
 * Global (protected) data for ISR layer:
 * ---------------------------------------------------------------------------
 */
static I2cRegisterMmFunc_t  gI2cRegisterMmFunc[TMHW_I2C_UNIT_MAX];
static tmhwI2cData_t        gI2cData[TMHW_I2C_UNIT_MAX];
static Bool                 gI2cDma[TMHW_I2C_UNIT_MAX];    /* = {False} */
static Bool                 gI2cByteWise[TMHW_I2C_UNIT_MAX]; /* = {False} */


/* ---------------------------------------------------------------------------
 * Internal Prototypes
 * ---------------------------------------------------------------------------
 */
#ifdef LINUX_BUILD
/* tmhwI2cIP3203SetDMA() is declared in tmhwI2cIP3203.h. However, this file is
 * not included for Linux build. Hence declare the function here, for Linux build
 */
Void
tmhwI2cIP3203SetDMA (tmUnitSelect_t  i2cUnit);

#ifdef CONFIG_I2C_DEBUG
/* Meant for debugging purposes */
void I2c_Error_Trigger( void );
static const char* RegToName( unsigned long offset);
static void DumpI2c( void );
#endif

#endif

/* ---------------------------------------------------------------------------
 * Internal Prototypes (protected) for Driver initialization:
 * ---------------------------------------------------------------------------
 */
static Void i2cDriverInit( Void );

/* ---------------------------------------------------------------------------
 * Internal Prototypes for Register abstraction:
 * ---------------------------------------------------------------------------
 */
/* speed */
static Void
i2cConvertSpeedToReg (
		tmUnitSelect_t          i2cUnit,
		UInt32                  fsSpeed,
		UInt32                  hsSpeed,
		pUInt32                 pFSBIR,
		pUInt32                 pHSBIR
		);

static Void
i2cConvertSpeedToActual (
		tmUnitSelect_t          i2cUnit,
		UInt32                  FSBIR,
		UInt32                  HSBIR,
		pUInt32                 pFsSpeed,
		pUInt32                 pHsSpeed
		);

static Void
i2cIP0105SetSpeed (
		tmUnitSelect_t          i2cUnit
		);

/* control */
#ifdef LINUX_BUILD
Void
#else
static Void
#endif
i2cControl (
		tmUnitSelect_t          i2cUnit,
		tmhwI2cControl_t        control
	   );

static Void
i2cSetDmaControl (
		tmUnitSelect_t          i2cUnit,
		tmhwI2cDmaControl_t     dmaControl
		);

static Void
i2cGetStatus (
		tmUnitSelect_t          i2cUnit,
		ptmhwI2cEvent_t         pI2cEvent
	     );

/* ---------------------------------------------------------------------------
 * Internal Prototypes for ISR layer:
 * ---------------------------------------------------------------------------
 * remark:
 * In a multi processor environment, functions in the ISR layer may not be
 * used outside the scope of the user process where the ISR runs!!! This is
 * secured.
 * ---------------------------------------------------------------------------
 */
/* DMA */
static Void
i2cPrepareDma (
		tmUnitSelect_t          i2cUnit,
		tmhwI2cDmaControl_t     dmaControl
	      );

static tmErrorCode_t
i2cCheckDma (
		tmUnitSelect_t          i2cUnit,
		tmhwI2cRequest_t        request
	    );

/* Byte transfer */
static Void
i2cWriteDat (
		tmUnitSelect_t          i2cUnit,
		UInt8                   dat
	    );

static Void
i2cTransmitByte (
		tmUnitSelect_t          i2cUnit
		);

static Void
i2cReceiveByte (
		tmUnitSelect_t          i2cUnit
	       );

/* Block transfer */
static tmErrorCode_t
i2cTransceiveData (
		tmUnitSelect_t          i2cUnit,
		tmhwI2cDmaControl_t     dmaControl
		);

/* Communication aids */
static Void
i2cSendSlaveAddress (
		tmUnitSelect_t          i2cUnit
		);

static Bool
i2cIncludeLastByte (
		tmUnitSelect_t          i2cUnit,
		tmhwI2cDmaControl_t     dmaControl
		);

static Void
i2cWaitForInt(
		tmUnitSelect_t              i2cUnit
	     );

static Void
i2cWaitForSTO(
		tmUnitSelect_t              i2cUnit
	     );


#if( defined(LINUX_BUILD) && defined(I2C_PRINTK) )

Void debugSMU (Void)
{
	UInt32 i2cUnit;
	tmhwI2cEvent_t  eventLocal;
	UInt32 reg_value;

	for ( i2cUnit = 0; i2cUnit < 2; i2cUnit++)
	{
		printk("unit num %d\n",i2cUnit);

		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_I2CSLA_OFFSET, reg_value);
		printk( "slave address register %x \n",reg_value);

		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_INT_ENABLE_OFFSET, reg_value);
		printk("int enable %x\n",reg_value);

		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_I2CCON_OFFSET, reg_value);
		printk("control register %x\n",reg_value);

		i2cGetStatus(i2cUnit, &eventLocal);
		printk("status = %x\n",eventLocal);
	}
} /* debugSMU */

#endif


/* ---------------------------------------------------------------------------
 * Exported functions register abstraction layer:
 * ---------------------------------------------------------------------------
 */

/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cRegisterMmFunc
 *
 * DESCRIPTION: Register functions for memory management (only required when
 *              DMA is enabled).
 *
 * RETURN:      TM_OK
 *
 * PRE:         None
 *
 * NOTES:       non re-entrant.
 * ---------------------------------------------------------------------------
 */
tmErrorCode_t
tmhwI2cRegisterMmFunc (
		tmUnitSelect_t          i2cUnit,
		ptmhwI2cMmFunc_t        pFuncStruct
		)
{

#ifndef LINUX_BUILD
	if (gI2cUnitCount == I2C_DATA_INVALID)
	{
		i2cDriverInit();
	}
#endif
	gI2cRegisterMmFunc[i2cUnit].gpVirtToPhys       = pFuncStruct->pI2cVirtToPhys;
	gI2cRegisterMmFunc[i2cUnit].gpCacheFlush       = pFuncStruct->pI2cCacheFlush;
	gI2cRegisterMmFunc[i2cUnit].gpCacheInvalidate  = pFuncStruct->pI2cCacheInvalidate;
	gI2cRegisterMmFunc[i2cUnit].bufferPtr = pFuncStruct->pI2cDynamicMemPtr;
#ifdef LINUX_BUILD
	gI2cRegisterMmFunc[i2cUnit].phyAddr   = pFuncStruct->phyAddr;
#endif
	gI2cRegisterMmFunc[i2cUnit].length = pFuncStruct->i2cDynamicMemLength;
	if(gI2cRegisterMmFunc[i2cUnit].length > I2C_CFG_DMA_BLOCK_LENGTH)
	{
		gI2cRegisterMmFunc[i2cUnit].length = I2C_CFG_DMA_BLOCK_LENGTH;
	}
	tmhwI2cIP3203SetDMA(i2cUnit);

	return TM_OK;
} /* tmhwI2cRegisterMmFunc */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cGetSWVersion
 *
 * DESCRIPTION: Get the I2c HWAPI interface compatibility number, major
 *              version and minor version.
 *
 * RETURN:      TM_OK
 *
 * PRE:         None
 *
 * NOTES:       re-entrant.
 * ---------------------------------------------------------------------------
 */
tmErrorCode_t
tmhwI2cGetSWVersion (
		ptmSWVersion_t          pI2cVersion
		)
{
	/* provide the requested information */
	pI2cVersion->compatibilityNr    = TMHW_I2C_COMPATIBILITY_NR;
	pI2cVersion->majorVersionNr     = TMHW_I2C_MAJOR_VERSION_NR;
	pI2cVersion->minorVersionNr     = TMHW_I2C_MINOR_VERSION_NR;

	return TM_OK;
} /* tmhwI2cGetSWVersion */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cGetCapabilities:
 *
 * DESCRIPTION: This function returns the I2c HWAPI capabilities.
 *
 * RETURN:      TMHW_ERR_I2C_BAD_UNIT_ID or TM_OK
 *
 * PRE:         None
 *
 * NOTES:       non re-entrant.
 * ---------------------------------------------------------------------------
 */
tmErrorCode_t
tmhwI2cGetCapabilities (
		tmUnitSelect_t      i2cUnit,
		ptmhwI2cCapabilities_t  pI2cCaps
		)
{
	UInt32 FSSpeed, HsSpeed;
	tmErrorCode_t   status = TM_OK;
	if (gI2cUnitCount == I2C_DATA_INVALID)
	{
		i2cDriverInit();
	}

	if (i2cUnit >= gI2cUnitCount)
	{
		status = TMHW_ERR_I2C_BAD_UNIT_ID;
	}
	else
	{
		pI2cCaps->i2cUnitCount = (UInt32) gI2cUnitCount;

		if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
		{
			pI2cCaps->i2cModuleID = I2C_IP3203_HWMODULE_ID;

			FSSpeed = TMHW_I2C_MAX_FS_SPEED;
			HsSpeed = TMHW_I2C_MAX_HS_SPEED;
			tmhwI2cConvertSpeed ( i2cUnit, &FSSpeed, &HsSpeed );

			pI2cCaps->i2cMaxSpeedKHZ    = FSSpeed;
			pI2cCaps->i2cMaxHsSpeedKHZ  = HsSpeed;
		}
		else
		{
			pI2cCaps->i2cModuleID = IIC_IP0105_HWMODULE_ID;

			FSSpeed = TMHW_I2C_MAX_FS_SPEED;
			HsSpeed = 0;
			tmhwI2cConvertSpeed ( i2cUnit, &FSSpeed, &HsSpeed );

			pI2cCaps->i2cMaxSpeedKHZ    = FSSpeed;
			pI2cCaps->i2cMaxHsSpeedKHZ  = HsSpeed;
		}
	}
	return  status;
} /* tmhwI2cGetCapabilities */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cGetBlockId:
 *
 * DESCRIPTION: This function returns the specified I2c HW Module Block ID
 *
 * RETURN:      TMHW_ERR_I2C_BAD_UNIT_ID, TM_OK
 *
 * PRE:         None
 *
 * NOTES:       non re-entrant.
 * ---------------------------------------------------------------------------
 */
tmErrorCode_t
tmhwI2cGetBlockId(
		tmUnitSelect_t  i2cUnit,
		pUInt32     pBlockId
		)
{
	tmErrorCode_t   status = TM_OK;

	if (gI2cUnitCount == I2C_DATA_INVALID)
	{
		i2cDriverInit();
	}

	if (i2cUnit >= gI2cUnitCount)
	{
		status = TMHW_ERR_I2C_BAD_UNIT_ID;
	}
	else
	{
		if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
		{
			I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_MODULE_ID_OFFSET, *pBlockId);
		}
		else
		{
			I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_MODULE_ID_OFFSET, *pBlockId);
		}
	}
	return status;
} /* tmhwI2cGetBlockId */

#ifdef LINUX_BUILD
#ifdef CONFIG_I2C_TM_DOWNLOAD
//
// The early TM program downloads the Channel decoder
// using the I2c IP block.
//
// We want to ensure that this one is finished before
// proceeding here with claiming the I2c bus.
//
#define DSP_TM32_INI_BASE   0x1BF40000
#define DSP_TM32_PC         0x004C
#define DSP_TM32_START      0x0048
#define DSP_WINDOW_SIZE     0x0100
#define MAX_DELAY_MSEC      100
static unsigned long earlytm_addr;

static void Await_Early_Download( void );
static void TriggerPin( int pinNr, int state);

static int __init earlytm_setup(char *str)
{
	earlytm_addr = simple_strtoul(str,NULL,0);
	printk(KERN_INFO "earlytm_addr = %x\n",earlytm_addr);
	return 1;
}

__setup("earlytm=", earlytm_setup);

void TriggerPin( int pinNr, int state)
{
	volatile unsigned long* p1 = (volatile unsigned long*) 0xbbf4c000;
	volatile unsigned long* p2 = (volatile unsigned long*) 0xbbf4c010;

	if( pinNr == 0 )
	{
		if( state )
		{
			*p1 = 0x00000002;
			*p2 = 0x00010001;
		}
		else
		{
			*p1 = 0x00000002;
			*p2 = 0x00010000;
		}
	}
	else if ( pinNr == 3 )
	{
		if( state )
		{
			*p1 = 0x00000080;
			*p2 = 0x00080008;
		}
		else
		{
			*p1 = 0x00000080;
			*p2 = 0x00080000;
		}
	}
	else
	{
		printk( "unsupported pin %d\n", pinNr );
	}
}

static void Await_Early_Download( void )
{
	static int initDone = 0;

	if( initDone )
	{
		// nothing to be done
	}
	else
	{
		unsigned int pTM;
		initDone = 1;

#ifdef CONFIG_I2C_DEBUG
		TriggerPin( 3, 0);
#endif

		pTM = (unsigned int) ioremap( DSP_TM32_INI_BASE, DSP_WINDOW_SIZE );

		if( pTM )
		{
			volatile unsigned int * pTM_PC       = (volatile unsigned int* ) (pTM + DSP_TM32_PC    );
			volatile unsigned int * pTM_START    = (volatile unsigned int* ) (pTM + DSP_TM32_START );
			unsigned int            prev_PC      = 0;   // previous value for PC
			unsigned int            cur_PC       = 0;   // current  value of PC
			int                     identicalCnt = 0;   // number of consecutive identical PC
			int                     iteration    = 0;   // total iteration count, just for debugging
			unsigned long           end_time;
			unsigned long cur_TM_START;

			cur_TM_START = *pTM_START;

			// early TM start address a kernel bootcmdline option
			if( cur_TM_START == earlytm_addr )
			{
#ifdef CONFIG_I2C_DEBUG
				printk(KERN_EMERG "checking early TM status\n");
				printk(KERN_EMERG "earlytm_addr = %x\n",earlytm_addr);
#endif

				end_time = jiffies + (MAX_DELAY_MSEC * HZ / 1000 );
				do {
					iteration++;
					cur_PC = *pTM_PC;
					if( cur_PC == prev_PC )
					{
						identicalCnt++;
					}
					else
					{
						identicalCnt = 0;
					}
					prev_PC = cur_PC;
					schedule();
				} while (  ( identicalCnt < 5 )
						&& ( time_before( jiffies, end_time) ) );

				if( identicalCnt < 5 )
				{
					printk(KERN_EMERG "ERROR: early TM wait loop gives up after %d msec (and %d iterations)\n", MAX_DELAY_MSEC, iteration );
					printk(KERN_EMERG "IdenticalCnt= %d, cur_PC= 0x%08x, prev_PC= 0x%08x\n", identicalCnt, cur_PC, prev_PC );
				}
				else
				{
#ifdef CONFIG_I2C_DEBUG
					printk(KERN_EMERG "early TM wait ok after %lu msec (and %d iterations)\n", jiffies + (MAX_DELAY_MSEC * HZ / 1000) - end_time, iteration );
#endif
				}
			}
			else
			{
#ifdef CONFIG_I2C_DEBUG
				printk(KERN_EMERG "skipp early TM status check (looks like hotboot ?)\n" );
				printk(KERN_EMERG "TM_START = 0x%08x\n", *pTM_START );
#endif
			}
		}
		else
		{
			printk(KERN_EMERG "ERROR: Could not map TM registers into memory, so can't check early TM download stuff\n" );
		}

#ifdef CONFIG_I2C_DEBUG
		TriggerPin( 3, 1);
#endif
	}
}

#endif /*CONFIG_I2C_TM_DOWNLOAD*/
#endif /* LINUX_BUILD*/

/* -------------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cInit:
 *
 * DESCRIPTION: Initializes the indicated I2c device. The device will be
 *              initialized with the following defaults:
 *              - SS speed = TMHW_I2C_MAX_SS_SPEED,
 *              - HS speed = 0
 *              - enable I2c module interface (EN is set),
 *              - disable and clear I2c interrupt.
 *
 *              The I2c MMIO base address is obtained from configuration files
 *              and used by the subsequent I2c functions to access I2c MMIO
 *              registers.
 *
 * RETURN:      TMHW_ERR_I2C_BAD_UNIT_ID or TM_OK
 *
 * PRE:         state = idle
 *
 * POST         state = initialized
 *
 * NOTES:       re-entrant.
 * ---------------------------------------------------------------------------
 */
tmErrorCode_t
tmhwI2cInit (
		tmUnitSelect_t   i2cUnit
	    )
{
	tmErrorCode_t   retVal = TM_OK;

#ifdef LINUX_BUILD
#ifdef CONFIG_I2C_TM_DOWNLOAD
	/* Ensuring TM has finished using the I2C bus*/
	Await_Early_Download();
#endif
#endif

	if (gI2cUnitCount == I2C_DATA_INVALID)
	{
		i2cDriverInit();
	}
	if (i2cUnit >= gI2cUnitCount)
	{
		retVal = TMHW_ERR_I2C_BAD_UNIT_ID;
	}
	else
	{
		/* Make sure I2c is not in the power down mode */
		retVal = tmhwI2cSetPowerState(i2cUnit, tmPowerOn);

		/* Reset I2c module */
		i2cControl(i2cUnit, tmhwI2cControlNone);
		i2cSetDmaControl(i2cUnit, tmhwI2cDmaNone);
		retVal = tmhwI2cIntClear(i2cUnit);

		/* disable slave addresses */
		tmhwI2cSetSlaveAddr(i2cUnit, 0);
		tmhwI2cDisableGeneralCall(i2cUnit);

		/* Set the default speed to 100KHz (maximum Standard Speed mode)
		 * Disable High Speed
		 * Device IP3203 requires that the speed must be set before using Slave mode.
		 * this requirement now is fulfilled then.
		 */
		if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
		{
#if 0 /* TODO */
			/* program SDA_HOLD register */
			/* Multiply by 2 if the PCFClk is 24Mhz to get appr 14 value in
			   the SDA HOLD register*/
			UInt32 multiplyFactor = 48000/gI2cCfg[i2cUnit].clockFreqKHz;
			I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_SDA_HOLD,
					(gI2cCfg[i2cUnit].clockFreqKHz/10000) * 3 * (multiplyFactor));
			tmhwI2cSetSpeed(i2cUnit, TMHW_I2C_MAX_SS_SPEED, TMHW_I2C_MAX_SS_SPEED);
#endif     
			I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_SDA_HOLD,0x2D);
			I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_FSBIR_OFFSET, 0x3E);
			I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_HSBIR_OFFSET, 0x0);
		}
		else
		{
			i2cIP0105SetSpeed(i2cUnit);
		}

		/* Enable I2c module */
		retVal = tmhwI2cIntEnable(i2cUnit);
		i2cControl(i2cUnit,tmhwI2cEnable);
	}
	return retVal;
} /* tmhwI2cInit */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cDeinit:
 *
 * DESCRIPTION: Deinitialize the specified I2c device to known state and disable
 *              the interrupt. Al activities are aborted.
 *
 * RETURN:      TM_OK
 *
 * PRE:         state != idle
 *
 * POST         state = idle
 *
 * NOTES:       re-entrant.
 * ---------------------------------------------------------------------------
 */
tmErrorCode_t
tmhwI2cDeinit (
		tmUnitSelect_t       i2cUnit
	      )
{
	tmErrorCode_t   retVal;
	/* Disable I2c module and make sure Ack and Start are off.
	 * When the module is in DMA mode, all registers are blocked.
	 * So first be sure that DMA is disabled.
	 */
	i2cSetDmaControl(i2cUnit, tmhwI2cDmaNone);

	/* Disable I2c module */
	i2cControl(i2cUnit, tmhwI2cControlNone);
	retVal = tmhwI2cIntDisable(i2cUnit);
	retVal = tmhwI2cIntClear(i2cUnit);

	/* Turn off the power for I2c unit */
	retVal = tmhwI2cSetPowerState(i2cUnit, tmPowerOff);

	return retVal;
} /* tmhwI2cDeinit */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cGetPowerState:
 *
 * DESCRIPTION: Get the current device power state.
 *
 * RETURN:      TM_OK
 *
 * PRE:
 *
 * NOTES:       non re-entrant.
 * ---------------------------------------------------------------------------
 */
tmErrorCode_t
tmhwI2cGetPowerState (
		tmUnitSelect_t      i2cUnit,
		ptmPowerState_t     pPowerState
		)
{
	UInt32 reg_value = 0;
	tmErrorCode_t   status = TM_OK;

	if (gI2cUnitCount == I2C_DATA_INVALID)
	{
		i2cDriverInit();
	}

	if (i2cUnit >= gI2cUnitCount)
	{
		status = TMHW_ERR_I2C_BAD_UNIT_ID;
	}
	else if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_POWERDOWN_OFFSET, reg_value);
		if(reg_value & TMVH_I2C3203_POWERDOWN_PWR_DOWN_MSK)
		{
			*pPowerState = tmPowerOff;
		}
		else
		{
			*pPowerState = tmPowerOn;
		}
	} else
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2C_POWERDOWN_OFFSET, reg_value);
		if(reg_value & TMVH_IIC0105_I2C_POWERDOWN_PWR_DOWN_MSK)
		{
			*pPowerState = tmPowerOff;
		}
		else
		{
			*pPowerState = tmPowerOn;
		}
	}
	return status;
} /* tmhwI2cGetPowerState */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cSetPowerState:
 *
 * DESCRIPTION: Set the device current power state.
 *
 * RETURN:      TM_OK
 *
 * PRE:         state = initialized
 *
 * NOTES:       non re-entrant.
 * ---------------------------------------------------------------------------
 */
tmErrorCode_t
tmhwI2cSetPowerState (
		tmUnitSelect_t          i2cUnit,
		tmPowerState_t          powerState
		)
{
	UInt32 powerdown = 0;
	UInt32 reg_value = 0;

	if (powerState == tmPowerOff)
	{
		powerdown = 1;
	}
	else
	{
		powerdown = 0;
	}

	if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_POWERDOWN_OFFSET, reg_value);
		reg_value &= ~TMVH_I2C3203_POWERDOWN_PWR_DOWN_MSK;
		reg_value |= ((powerdown << TMVH_I2C3203_POWERDOWN_PWR_DOWN_POS) & TMVH_I2C3203_POWERDOWN_PWR_DOWN_MSK);
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_POWERDOWN_OFFSET, reg_value);
	}
	else
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2C_POWERDOWN_OFFSET, reg_value);
		reg_value &= ~TMVH_IIC0105_I2C_POWERDOWN_PWR_DOWN_MSK;
		reg_value |= ((powerdown << TMVH_IIC0105_I2C_POWERDOWN_PWR_DOWN_POS) & TMVH_IIC0105_I2C_POWERDOWN_PWR_DOWN_MSK);
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2C_POWERDOWN_OFFSET, reg_value);
	}
	return TM_OK;
} /* tmhwI2cSetPowerState */

/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cSetPowerState:
 *
 * DESCRIPTION: Set the device current power state.
 *
 * RETURN:      TM_OK
 *
 * PRE:         state = initialized
 *
 * NOTES:       non re-entrant.
 * ---------------------------------------------------------------------------
 */
tmErrorCode_t
tmhwI2cSetIntEnable (
		tmUnitSelect_t          i2cUnit,
		bool                    enable
		)
{
	if (enable)
	{
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_INT_ENABLE_OFFSET, 0x1);
	}
	else
	{
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_INT_ENABLE_OFFSET, 0);
	}

	return TM_OK;
} /* tmhwI2cSetIntEnable */

/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cEnableGeneralCall:
 *
 * DESCRIPTION: monitor the I2C-bus for the general call. An interrupt
 *              is generated if the general call address is detected.
 *              General call address is the 7-bit slave address = 0 and R/W
 *              bit = 0.
 *
 * RETURN:      -
 *
 * PRE:         idle
 *
 * NOTES:       non re-entrant.
 * ---------------------------------------------------------------------------
 */
Void
tmhwI2cEnableGeneralCall (
		tmUnitSelect_t          i2cUnit
		)
{
	UInt32 reg_value = 0;

	if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_I2CSLA_OFFSET, reg_value);
		reg_value |= TMVH_I2C3203_I2CSLA_WATCH_GC_MSK; /* Set the bit field */
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_I2CSLA_OFFSET, reg_value);
	}
	else
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2C_ADDR_OFFSET, reg_value);
		reg_value |= TMVH_IIC0105_I2C_ADDR_WATCH_GC_MSK; /* Set the bit field */
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2C_ADDR_OFFSET, reg_value);
	}
} /* tmhwI2cEnableGeneralCall */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cDisableGeneralCall:
 *
 * DESCRIPTION: Ignore the general call address on the I2c bus.
 *
 * RETURN:      -
 *
 * PRE:         state != idle
 *
 * NOTES:       non re-entrant.
 * ---------------------------------------------------------------------------
 */
Void
tmhwI2cDisableGeneralCall (
		tmUnitSelect_t          i2cUnit
		)
{
	UInt32 reg_value;

	if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_I2CSLA_OFFSET, reg_value);
		reg_value &= ~TMVH_I2C3203_I2CSLA_WATCH_GC_MSK; /* Reset(clear) the bit field */
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_I2CSLA_OFFSET, reg_value);
	}
	else
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2C_ADDR_OFFSET, reg_value);
		reg_value &= ~TMVH_IIC0105_I2C_ADDR_WATCH_GC_MSK; /* Reset(clear) the bit field */
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2C_ADDR_OFFSET, reg_value);
	}
} /* tmhwI2cDisableGeneralCall */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cSetSlaveAddr:
 *
 * DESCRIPTION: When the slave monitors the I2C-bus,
 *              it answeres the here specified slave address
 *
 * RETURN:      -
 *
 * PRE:         state !=idle
 *
 * NOTES:       non re-entrant.
 * ---------------------------------------------------------------------------
 */
Void
tmhwI2cSetSlaveAddr (
		tmUnitSelect_t          i2cUnit,        /*  I:  I2c Unit ID number */
		UInt8                   moduleSlaveAddr /*  I:  7-bit I2c slave address while the I2c
							    module is in the slave mode. */
		)
{
	UInt32 reg_value = 0;

	if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_I2CSLA_OFFSET, reg_value);
		reg_value &= ~TMVH_I2C3203_I2CSLA_SLAVE_ADDR_MSK;
		reg_value |= ((moduleSlaveAddr << TMVH_I2C3203_I2CSLA_SLAVE_ADDR_POS) & TMVH_I2C3203_I2CSLA_SLAVE_ADDR_MSK);
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_I2CSLA_OFFSET, reg_value);
	}
	else
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2C_ADDR_OFFSET, reg_value);
		reg_value &= ~TMVH_IIC0105_I2C_ADDR_SLAVE_ADDR_MSK;
		reg_value |= ((moduleSlaveAddr << TMVH_IIC0105_I2C_ADDR_SLAVE_ADDR_POS) & TMVH_IIC0105_I2C_ADDR_SLAVE_ADDR_MSK);
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2C_ADDR_OFFSET, reg_value);
	}

} /* tmhwI2cSetSlaveAddr */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cSetSpeed:
 *
 * DESCRIPTION: Determines the I2C-bus speed when the device controls the bus
 *              (master mode)
 *
 * RETURN:      -
 *
 * PRE:         state = initialized
 *
 * NOTES:       non re-entrant.
 * ---------------------------------------------------------------------------
 */
Void
tmhwI2cSetSpeed (
		tmUnitSelect_t          i2cUnit,
		UInt32                  fsSpeed,
		UInt32                  hsSpeed
		)
{
	UInt32 fsbir, hsbir;

	i2cConvertSpeedToReg(i2cUnit, fsSpeed, hsSpeed, &fsbir, &hsbir);
	/* conversion intelligence is in i2cConvertSpeed , unit is also passed */

	if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
	{
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_FSBIR_OFFSET, fsbir);
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_HSBIR_OFFSET, hsbir);
	}
	else
	{
		/* HwApi 0105 says to stop the module first!
		 * Speed for 0105 modules is done at Initialisation only, according
		 * to the speed set in tmhwI2cCfg.c
		 */
	}
} /* tmhwI2cSetSpeed */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cGetSpeed:
 *
 * DESCRIPTION: Gets the speed that the I2C-bus will have when the indicated
 *              device controls the bus (master mode)
 *
 * RETURN:      -
 *
 * PRE:         state!= idle
 *
 * NOTES:       non re-entrant.
 * ---------------------------------------------------------------------------
 */
Void
tmhwI2cGetSpeed (
		tmUnitSelect_t          i2cUnit,
		pUInt32                 pFsSpeed,
		pUInt32                 pHsSpeed        /* 0: if no HS */
		)
{
	UInt32 fsbir, hsbir;
	if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_FSBIR_OFFSET, fsbir);
		fsbir = ((fsbir & TMVH_I2C3203_FSBIR_BITRATE_MSK) >> TMVH_I2C3203_FSBIR_BITRATE_POS);

		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_HSBIR_OFFSET, hsbir);
		hsbir = ((hsbir & TMVH_I2C3203_HSBIR_BITRATE_MSK) >> TMVH_I2C3203_HSBIR_BITRATE_POS);
	}
	else
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2CCONTROL_OFFSET, fsbir);
		fsbir = ((fsbir & TMVH_IIC0105_I2CCONTROL_CR2_MSK) >> TMVH_IIC0105_I2CCONTROL_CR2_POS);
		hsbir = 0;
	}
	/* intelligence goes to i2cConvertSpeedToActual */
	i2cConvertSpeedToActual(i2cUnit, fsbir, hsbir, pFsSpeed, pHsSpeed);

} /* tmhwI2cGetSpeed */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cConvertSpeed:
 *
 * DESCRIPTION: This function derives the actual speed at which the specific
 *              master will communicate when it controls the I2C-bus from the
 *              intended speed
 *
 * RETURN:      -
 *
 * PRE:         -
 *
 * NOTES:       non Re-entrant.
 * ---------------------------------------------------------------------------
 */
Void
tmhwI2cConvertSpeed (
		tmUnitSelect_t          i2cUnit,
		pUInt32                 pFsSpeed,
		pUInt32                 pHsSpeed
		)
{
	UInt32 fsbir, hsbir;

	i2cConvertSpeedToReg(i2cUnit, *pFsSpeed, *pHsSpeed, &fsbir, &hsbir);
	i2cConvertSpeedToActual(i2cUnit, fsbir, hsbir, pFsSpeed, pHsSpeed);
} /* tmhwI2cConvertSpeed */

/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cGetBusStatus:
 *
 * DESCRIPTION: This function indicates if the bus is free or busy by qeurying
 *              the INTROG register in IP3203 and the I2C BUS OBSERVATION
 *              register in the IP0105. This API shall be used to ensure a
 *              safe close of the driver.
 * RETURN:      -
 *
 * PRE:         -
 *
 * NOTES:       Re-entrant.
 * ---------------------------------------------------------------------------
 */
Void tmhwI2cGetBusStatus(
		tmUnitSelect_t          i2cUnit,
		ptmhwI2cStatusType_t    pStatusType
		)
{
	UInt32 status;
	if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_INTROG_OFFSET, status);
		if ( (status & TMVH_I2C3203_INTROG_INTRO_BB_MSK)
				== TMVH_I2C3203_INTROG_INTRO_BB_MSK )
		{
			*pStatusType = tmhwI2cBusBusy;
		}
		else
		{
			*pStatusType = tmhwI2cBusFree;
		}
	}
	else
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2C_OBS_PINS_OFFSET, status);
		if ( ((status & TMVH_IIC0105_I2C_OBS_PINS_OBS_SCL_MSK)
					== TMVH_IIC0105_I2C_OBS_PINS_OBS_SCL_MSK) &&
				((status & TMVH_IIC0105_I2C_OBS_PINS_OBS_SDA_MSK)
				 == TMVH_IIC0105_I2C_OBS_PINS_OBS_SDA_MSK) )
		{ /* bus is fre if both lines are released*/
			*pStatusType = tmhwI2cBusFree;
		}
		else
		{
			*pStatusType = tmhwI2cBusBusy;
		}
	}

}

/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cIntGetStatus:
 *
 * DESCRIPTION: Get the current HwAPI state
 *
 * RETURN:      -
 *
 * PRE:         state != idle
 *
 * NOTES:       non Re-entrant.
 * ---------------------------------------------------------------------------
 */
Void
tmhwI2cIntGetStatus (
		tmUnitSelect_t        i2cUnit,
		ptmhwI2cStatusType_t    pStatusType
		)
{
	tmhwI2cEvent_t  eventLocal;

	i2cGetStatus(i2cUnit, &eventLocal);
	if (eventLocal == tmhwI2cEvent0xf8)          /* no interrupt, state = 0xf8 */
	{
		*pStatusType = tmhwI2cNoInterrupt;
	}
	else if (eventLocal == tmhwI2cEvent0x00)     /* bus error */
	{
		*pStatusType = tmhwI2cBusError;
	}
	else if (eventLocal < tmhwI2cEvent0x60)      /* master mode */
	{
		*pStatusType = tmhwI2cMaster;
	}
	else                                    /* slave mode */
	{
		*pStatusType = tmhwI2cSlave;
	}

} /* tmhwI2cIntGetStatus */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cIntEnable:
 *
 * DESCRIPTION: Enables the interrupt of the specified I2C device.
 *
 * RETURN:      TM_OK
 *
 * PRE:         state != idle
 *
 * NOTES:       Re-entrant.
 * ---------------------------------------------------------------------------
 */
tmErrorCode_t
tmhwI2cIntEnable (
		tmUnitSelect_t          i2cUnit
		)
{
	if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
	{
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_INT_ENABLE_OFFSET, TMVH_I2C3203_INT_ENABLE_EN_MODE_MSK);
	}
	else
	{
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_INT_ENABLE_OFFSET, TMVH_IIC0105_INT_ENABLE_EN_MSK);
	}

	return TM_OK;
} /* tmhwI2cIntEnable */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cIntDisable:
 *
 * DESCRIPTION: Disables the interrupt for the specified I2C device.
 *
 * RETURN:      -
 *
 * PRE:         state != idle
 *
 * NOTES:       Re-entrant.
 * ---------------------------------------------------------------------------
 */
tmErrorCode_t
tmhwI2cIntDisable (
		tmUnitSelect_t  i2cUnit
		)
{
	UInt32 reg_value = 0;

	if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
	{
		reg_value &= ~TMVH_I2C3203_INT_ENABLE_EN_MODE_MSK; /* Reset the bit field */
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_INT_ENABLE_OFFSET, reg_value);
	}
	else
	{
		reg_value &= ~TMVH_IIC0105_INT_ENABLE_EN_MSK; /* Reset the bit field */
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_INT_ENABLE_OFFSET, reg_value);
	}

	return TM_OK;
} /* tmhwI2cIntDisable */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cIntClear:
 *
 * DESCRIPTION: Clears the interrupt for the specific I2c device.
 *
 * RETURN:      TM_OK
 *
 * PRE:         state != idle
 *
 * NOTES:       Re-entrant.
 * ---------------------------------------------------------------------------
 */
tmErrorCode_t
tmhwI2cIntClear (
		tmUnitSelect_t          i2cUnit
		)
{
	UInt32 reg_value = 0;

	if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
	{
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_INT_CLEAR_OFFSET, TMVH_I2C3203_INT_CLEAR_INTCLR_MSK);
		/* wait for peripheral to be sure that the interrupt has been cleared: Do dummy read */
		//I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_INT_CLEAR_OFFSET, reg_value);
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_INT_STATUS_OFFSET, reg_value);
	}
	else
	{
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_INT_CLEAR_OFFSET, TMVH_IIC0105_INT_CLEAR_CLR_MSK);
		/* wait for peripheral to be sure that the interrupt has been cleared: Do dummy read */
		//I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_INT_CLEAR_OFFSET, reg_value);
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_INT_STATUS_OFFSET, reg_value);
	}

	(Void)reg_value;    /* To avoid compile time remark variable not used */

	return TM_OK;
} /* tmhwI2cIntClear */



/* ---------------------------------------------------------------------------
 * External functions for ISR layer:
 * ---------------------------------------------------------------------------
 */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cSetData:
 *
 * DESCRIPTION: Fills the HwAPI data buffer
 *
 * RETURN:      -
 *
 * PRE:         state != idle & (module is monitorring | waiting for request)
 *              tmhwI2cEvent has returned TMHW_ERR_I2C_REQUEST while interrupt
 *              is not yet cleared,
 *              or module is not communicating
 *
 * POST:        gI2cIP3203Data.counter = gI2cIP3203Data.length
 *
 * NOTES:       Not re-entrant.
 *
 *              gI2cIP3203Data[i2cUnit].attach -> tmhwI2cRestart is for master
 *                                                  only
 *              gI2cIP3203Data[i2cUnit].slaveAddress    -> for master only
 *              gI2cIP3203Data[i2cUnit].direction       -> for master only
 *
 *              In a multi processor environment this function
 *              must not be used outside the scope of the user process where
 *              the ISR runs!!!
 * ---------------------------------------------------------------------------
 */
Void
tmhwI2cSetData (
		tmUnitSelect_t      i2cUnit,
		ptmhwI2cData_t      pData
	       )
{
	gI2cData[i2cUnit].pAddress      = pData->pAddress;
	gI2cData[i2cUnit].length        = pData->length;
	gI2cData[i2cUnit].counter       = pData->length;
	gI2cData[i2cUnit].attach        = pData->attach;        /* tmhwI2cRestart is for master only */
	gI2cData[i2cUnit].slaveAddress  = pData->slaveAddress;  /* for master only */
	gI2cData[i2cUnit].direction     = pData->direction;     /* for master only */
	gI2cData[i2cUnit].bHS           = pData->bHS;           /* for master only */
	if (gI2cCfg[i2cUnit].bHS == False)
	{
		gI2cData[i2cUnit].bHS = False;
	}

} /* tmhwI2cSetData */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cGetData:
 *
 * DESCRIPTION: Gets the content of the HwAPI data buffer
 *
 * RETURN:      -
 *
 * PRE:         state != idle,
 *
 * NOTES:       Not re-entrant.
 *
 *              In a multi processor environment this function
 *              must not be used outside the scope of the user process where
 *              the ISR runs!!!
 * ---------------------------------------------------------------------------
 */
Void
tmhwI2cGetData (
		tmUnitSelect_t          i2cUnit,
		ptmhwI2cData_t          pdata
	       )
{
	pdata->pAddress         = gI2cData[i2cUnit].pAddress;
	pdata->length           = gI2cData[i2cUnit].length;
	pdata->counter          = gI2cData[i2cUnit].counter;
	pdata->attach           = gI2cData[i2cUnit].attach;
	pdata->slaveAddress     = TMHW_I2C_DATA_DUMMY;                          /* in slave mode, no meaning */
	pdata->direction        = tmhwI2cNoMeaning;                             /* in slave mode, no meaning */
	pdata->bHS              = TMHW_I2C_DATA_DUMMY;                          /* in slave: mode no meaning */
} /* tmhwI2cGetData */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cStartMaster:
 *
 * DESCRIPTION: Puts the module into Master mode if it is monitorring the bus.
 *              (the slave is not addressed) Otherwise, no action
 *
 * RETURN:      -
 *
 * PRE:         state = initialized
 *
 * POST:        not controlling master mode
 *
 * NOTES:       the module can be in master state and in slave state
 *              simultaneously
 *
 *              When the master is aborted for some reason, the caller is
 *              responsible to restart the request. The content of
 *              tmhwI2cData[i2cUnit] indicates the transceived data.
 *
 *              Arguments direction is for compatibility
 *              reasons only and are not needed here.
 *
 *              Re-entrant against shared variables.
 *              not Re-entrant against multi tasks.
 *----------------------------------------------------------------------------
 */
Void
tmhwI2cStartMaster (
		tmUnitSelect_t          i2cUnit,
		UInt8                   slaveAddress,   /* I2c slave address of slave being */
		tmhwI2cDirection_t      direction       /* i2c communication direction */
		)
{
	(Void) direction;                /* parameter not used, hide warning */

	i2cControl( i2cUnit,
			(tmhwI2cControl_t) (tmhwI2cEnable | tmhwI2cStartCond |
				(slaveAddress ? tmhwI2cAcknowledge : tmhwI2cControlNone)) );

} /* tmhwI2cStartMaster */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cStartSlave:
 *
 * DESCRIPTION: starts the module being in slave mode.
 *
 * RETURN:      -
 *
 * PRE:         state != idle
 *
 * POST:        not addressed slave mode
 *
 * NOTES:       the module can be in master state and in slave state
 *              simultaneously
 *
 *              Re-entrant against shared variables.
 *              not Re-entrant against multi tasks.
 * ---------------------------------------------------------------------------
 */
Void
tmhwI2cStartSlave (
		tmUnitSelect_t          i2cUnit
		)
{
	UInt32 reg_value = 0;

	if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_I2CCON_OFFSET, reg_value);
		/* Set the bits correctly to ensure SDA_CTRL, SCL_CTRL, INT_CLEAR bits are set to 0
		   This solves the illegal STOP ticket 82910 PE9#681
		   & Ticket 95737 SETSTO shall be masked for slave.
		   Otherwise it will result in Slave pulling the line low and aborting the
		   transfer*/
		/* patch to ticket 82910
pre1: read != 0xDEADABBA
read update write with redundant Enable bit and bit 0,1,3 forced to 0
pre2: read = 0xDEADABBA
read value is incorrect,
enable forced to 1
scl, sda and int_clr bits forced to 0
written value does not stall device and will be
overwritten in ISR before INT_CLEAR
pre 3: Write is unsuccessful
written value does not have influence and will be
rewritten in ISR mode before INT_CLEAR
		 */
		reg_value = ((reg_value | ( TMVH_I2C3203_I2CCON_AA_MSK
						|  TMVH_I2C3203_I2CCON_EN_I2C_MSK)) & 0xF4);   /* Set the bit field */
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_I2CCON_OFFSET, reg_value);
	}
	else
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2CCONTROL_OFFSET, reg_value);
		reg_value |= TMVH_IIC0105_I2CCONTROL_AA_MSK; /* Set the bit field */
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2CCONTROL_OFFSET, reg_value);
	}
} /* tmhwI2cStartSlave */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cStopSlave:
 *
 * DESCRIPTION: stops the module being in slave mode
 *
 * RETURN:      -
 *
 * PRE:         state != monitorring
 *
 * NOTES:       the module can be in master state and in slave state
 *              simultaneously
 *
 *              Re-entrant against shared variables.
 *              not Re-entrant against multi tasks.
 * ---------------------------------------------------------------------------
 */
Void
tmhwI2cStopSlave (
		tmUnitSelect_t  i2cUnit
		)
{
	UInt32 reg_value = 0;

	if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_I2CCON_OFFSET, reg_value);
		/* Set/Reset the bits correctly to ensure SDA_CTRL, SCL_CTRL, INT_CLEAR bits are set to 0
		   This solves the illegal STOP ticket 82910 PE9#681
		   & Ticket 95737 SETSTO shall be masked for slave.
		   Otherwise it will result in Slave pulling the line low and aborting the
		   transfer*/
		reg_value = ((reg_value & ~TMVH_I2C3203_I2CCON_AA_MSK) & 0xFC) | TMVH_I2C3203_I2CCON_EN_I2C_MSK;      /* Reset the bit field */

		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_I2CCON_OFFSET, reg_value);
	}
	else
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2CCONTROL_OFFSET, reg_value);
		reg_value &= ~TMVH_IIC0105_I2CCONTROL_AA_MSK; /* Reset the bit field */
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2CCONTROL_OFFSET, reg_value);
	}
} /* tmhwI2cStopSlave */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cEvent
 *
 * DESCRIPTION: handles an I2c event
 *
 * RETURN:      TM_OK
 *              TMHW_ERR_I2C_EVENT_MST_REQ
 *              TMHW_ERR_I2C_EVENT_MST_LOST
 *              TMHW_ERR_I2C_EVENT_MST_ABORT
 *              TMHW_ERR_I2C_EVENT_MST_TRX_REQ
 *              TMHW_ERR_I2C_EVENT_MST_TRX_DONE
 *              TMHW_ERR_I2C_EVENT_MST_TRX_ABORT
 *              TMHW_ERR_I2C_EVENT_MST_REC_REQ
 *              TMHW_ERR_I2C_EVENT_MST_REC_DONE
 *              TMHW_ERR_I2C_EVENT_MST_REC_ABORT
 *              TMHW_ERR_I2C_EVENT_SLV_REC_REQ_MST_LOST
 *              TMHW_ERR_I2C_EVENT_SLV_REC_REQ
 *              TMHW_ERR_I2C_EVENT_SLV_GC_REQ_MST_LOST
 *              TMHW_ERR_I2C_EVENT_SLV_GC_REQ
 *              TMHW_ERR_I2C_EVENT_SLV_REC_DONE
 *              TMHW_ERR_I2C_EVENT_SLV_REC_ABORT
 *              TMHW_ERR_I2C_EVENT_SLV_TRX_REQ
 *              TMHW_ERR_I2C_EVENT_SLV_TRX_REQ_MST_LOST
 *              TMHW_ERR_I2C_EVENT_SLV_TRX_DONE
 *              TMHW_ERR_I2C_EVENT_SLV_TRX_ABORT
 *              TMHW_ERR_I2C_EVENT_BUS_ERROR
 *
 * PRE:         (state = controlling master | state = addressed slave)
 *              I2c Device is in interrupt status. Then its status
 *              is ->not<- tmhwI2cEvent0xf8
 *              The driver is protected against improper use of this
 *              latter pre-condition.
 *
 * POST:        returns TM_OK
 *              - the I2c device interrupt has been cleared
 *              returns TMHW_ERR_I2C_INVALID_REQUEST
 *              - wrong argument
 *              othersise
 *              - a higher level event has been occured
 *              when return var = TMHW_ERR_I2C_EVENT_*REQ, a second call to
 *              this function is required with request = tmhwI2cEventProceed.
 *              in between these function calls, data must be set available
 *              and/or retreived.
 *
 * NOTES:       Not re-entrant.
 * ---------------------------------------------------------------------------
 */
tmErrorCode_t tmhwI2cEvent (
		tmUnitSelect_t  i2cUnit,
		tmhwI2cRequest_t    request,
		Bool            bSlaveMonitor
		)
{
	tmErrorCode_t               status = TM_OK, stat;
	static tmErrorCode_t        nextStatus[TMHW_I2C_UNIT_MAX];
	static tmhwI2cAttach_t      attachReminder[TMHW_I2C_UNIT_MAX];
	static tmhwI2cDirection_t   gI2cDirection[TMHW_I2C_UNIT_MAX]; /* reminder for slave */
	static tmhwI2cEvent_t       event;
	tmhwI2cControl_t            slaveMonitor;
	static UInt32               i2cLastState[TMHW_I2C_UNIT_MAX];
	static UInt32               i2cRsCount;

	i2cGetStatus(i2cUnit, &event);
	if (event == tmhwI2cEvent0xf8)
	{/* nothing to handle */
		return TMHW_ERR_I2C_BUSY;
	}
	if (bSlaveMonitor == True)
	{
		slaveMonitor = tmhwI2cAcknowledge;
	}
	else
	{
		slaveMonitor = tmhwI2cControlNone;
	}

	/* check whether to adapt the data due to incomplete DMA transfer */
#ifdef I2C_PRINTK
	if (request == tmhwI2cEventInterrupt)
	{
		printk("Event : %x\n", event);
	}
#endif /* I2C_PRINTK */
	(Void)i2cCheckDma(i2cUnit, request);
	switch (event)
	{
		case (tmhwI2cEvent0x00):        /* Bus Error */
			switch (request)
		{
			case tmhwI2cEventInterrupt:
				if ((gI2cData[i2cUnit].direction == tmhwI2cTransmit) &&
						(gI2cCfg[i2cUnit].bDma == False))
				{/* last byte has not arrived at receiver. correct it. */
					(gI2cData[i2cUnit].counter)++;
				}
				/* asynchronous caller event */
				status = TMHW_ERR_I2C_EVENT_BUS_ERROR;
				/* STOP bit shall not be set.. Ticket # 96591, PB5#434*/
				i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|slaveMonitor));
				break;

			default :
				break;
		}
			break;

		case (tmhwI2cEvent0x08):        /* A START Condition has been transmitted */
			switch (request)
		{
			case tmhwI2cEventInterrupt:
				/* synchronous caller event */
				i2cRsCount=0;
				return TMHW_ERR_I2C_EVENT_MST_REQ;

			case tmhwI2cEventProceed:
				if (gI2cData[i2cUnit].bHS == True)
				{/* send HS master code */
					i2cWriteDat(i2cUnit, gI2cCfg[i2cUnit].hsMasterCode);
				}
				else
				{/* send slave-r/w */
					i2cSendSlaveAddress(i2cUnit);
					/* includes i2cPrepareDma */
					/* but in read mode, DMA will not start */
				}
				i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|slaveMonitor));
				break;

			default:
				break;
		}
			break;

		case (tmhwI2cEvent0x10):        /* A Repeated START condition has been transmitted */
			switch(request)
		{
			case tmhwI2cEventInterrupt:
				/* the request has already been passed. HS is not an issue here. */
				i2cSendSlaveAddress(i2cUnit);
				i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|slaveMonitor));
				i2cRsCount++;
				break;

			default:
				break;
		}
			break;

		case (tmhwI2cEvent0x18):        /* SLA.W has been transmitted, ACK has been received */
			switch(request)
		{
			case tmhwI2cEventInterrupt:
				i2cRsCount=0;
				if (i2cTransceiveData(i2cUnit, tmhwI2cDmaMasterTransmit)
						== I2C_BUFFER_EMPTY)
				{
					if (gI2cData[i2cUnit].length == 0)
					{
						status = TMHW_ERR_I2C_EVENT_MST_TRX_DONE;
					}
					else
					{
						status = TMHW_ERR_I2C_EVENT_MST_TRX_ABORT;
					}
					i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|tmhwI2cStopCond|slaveMonitor));
				}
				else
				{
					i2cControl(i2cUnit, (tmhwI2cControl_t)tmhwI2cEnable);
				}
				break;

			default:
				break;
		}
			break;

		case(tmhwI2cEvent0x20):     /* SLA.W or master code has been transmitted: NOT ACK has been received */
			switch(request)
		{
			case tmhwI2cEventInterrupt:
				if ((gI2cData[i2cUnit].bHS == True) && (gI2cData[i2cUnit].counter > 0))
				{/* HS */
					gI2cData[i2cUnit].bHS = False;
					i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable | tmhwI2cStartCond | slaveMonitor));
				}
				/*
				 * PB5#682 S/w Workaround for unexpected NACK. Ensure exactly 1 restart from Master
				 * to compensate for slow Slave side interrupt  servicing at 0xa0 state
				 * required for read mode, not sure if also required here for write mode.
				 * still to find out
				 * it does not harm though
				 */
				else if(i2cRsCount == 1 && (gI2cData[i2cUnit].counter > 0))
				{/* received unexpected Nack after reStart; reStart again (one retry max)*/
					status = TM_OK;
					i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|tmhwI2cStartCond|slaveMonitor));
				}
				else
				{/* no HS */
					/* asynchronous caller event */
					status = TMHW_ERR_I2C_EVENT_MST_ABORT;
					i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|tmhwI2cStopCond|slaveMonitor));
				}
				break;

			default:
				break;
		}
			break;

		case (tmhwI2cEvent0x28):        /* Data byte in I2CDAT has been transmitted, ACK received */
			switch(request)
		{
			case tmhwI2cEventInterrupt:
				if (i2cTransceiveData(i2cUnit, tmhwI2cDmaMasterTransmit) ==
						I2C_BUFFER_EMPTY)
				{/* buffer empty */
					if (gI2cData[i2cUnit].length == 0)
					{/* empty buffer, stop communication */
						/* asynchronous caller event */
						status = TMHW_ERR_I2C_EVENT_MST_TRX_ABORT;
						i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|tmhwI2cStopCond|slaveMonitor));
					}
					if (gI2cData[i2cUnit].attach == tmhwI2cChain)
					{/* counter = 0 */
						/* synchronous caller event */
						attachReminder[i2cUnit] = tmhwI2cChain;
						return TMHW_ERR_I2C_EVENT_MST_TRX_REQ;
					}
					else if (gI2cData[i2cUnit].attach == tmhwI2cRestart)
					{/* counter = 0 */
						/* synchronous caller event */
						attachReminder[i2cUnit] = tmhwI2cRestart;
						return TMHW_ERR_I2C_EVENT_MST_REQ;
					}
					else
					{/* tmhwI2cStopCond, counter = 0 */
						/* asynchronous caller event */
						status = TMHW_ERR_I2C_EVENT_MST_TRX_DONE;
						i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|tmhwI2cStopCond|slaveMonitor));
					}
				}
				else
				{
					i2cControl(i2cUnit, (tmhwI2cControl_t)tmhwI2cEnable);
				}
				break;

			case tmhwI2cEventProceed:
				/* tmhwI2cChain | tmhwI2cRestart, buffer counter def>0 */
				if (attachReminder[i2cUnit] == tmhwI2cChain)
				{	printk("Got Event Proceed\n");
					if (i2cTransceiveData(i2cUnit, tmhwI2cDmaMasterTransmit) ==
							I2C_BUFFER_EMPTY)
					{
						return TM_OK;
					}
					else
					{
						i2cControl(i2cUnit, (tmhwI2cControl_t)tmhwI2cEnable);
					}
				}
				else
				{/* tmhwI2cRestart */
					i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable | tmhwI2cStartCond|slaveMonitor));
				}
				break;

			default:
				break;
		}
			break;

		case (tmhwI2cEvent0x30):        /* Data byte in I2CDAT has been transmitted, NOT ACK received */
			switch(request)
		{
			case tmhwI2cEventInterrupt:
				/* Master application informed about transfer abort irrespective of
				   whether the following request is Restart or a Chain.*/
				status = TMHW_ERR_I2C_EVENT_MST_TRX_ABORT;
				i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|tmhwI2cStopCond|slaveMonitor));
				break;

			default:
				break;
		}
			break;

		case (tmhwI2cEvent0x38):        /* Arbitration lost in SLA, R/W or in Data byte or in NOT ACK bit or in master code */
			switch (request)
		{
			case tmhwI2cEventInterrupt:

				/* asynchronous caller event */
				status = TMHW_ERR_I2C_EVENT_MST_LOST;
				i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|slaveMonitor));
				/* the start condition will be given by the caller. */
				break;

			default:
				break;
		}
			break;

		case (tmhwI2cEvent0x40):    /* SLA.R has been transmitted, ACK received */
			switch (request)
		{
			case tmhwI2cEventInterrupt:
				/* synchronous caller event */
				i2cRsCount=0;
				nextStatus[i2cUnit] = TM_OK;
				gI2cByteWise[i2cUnit] = False;
				stat = i2cTransceiveData(i2cUnit, tmhwI2cDmaMasterReceive);
				if ( stat == I2C_BUFFER_EMPTY )
				{
					if (gI2cData[i2cUnit].length == 0)
					{
						nextStatus[i2cUnit] = TMHW_ERR_I2C_EVENT_MST_REC_DONE;
					}
					else
					{
						nextStatus[i2cUnit] = TMHW_ERR_I2C_EVENT_MST_REC_ABORT;
					}
					i2cControl(i2cUnit, (tmhwI2cControl_t)tmhwI2cEnable);
				}
				else if ( stat == I2C_LAST_BYTE )
				{
					i2cSetDmaControl(i2cUnit, tmhwI2cDmaNone);
					i2cControl(i2cUnit, (tmhwI2cControl_t)tmhwI2cEnable);
				}
				else
				{
					i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable | tmhwI2cAcknowledge));
				}
				break;

			default:
				break;
		}
			break;

		case (tmhwI2cEvent0x48):    /* SLA.R or Master code has been transmitted, NOT ACK received */
			switch (request)
		{
			case tmhwI2cEventInterrupt:
				if ((gI2cData[i2cUnit].bHS == True) && (gI2cData[i2cUnit].counter > 0))
				{/* HS */
					gI2cData[i2cUnit].bHS = False;
					i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|tmhwI2cStartCond|slaveMonitor));
				}
				/* PB5#682 S/w Workaround for unexpected NACK. Ensure exactly 1 restart from Master
				 * to compensate for slow Slave side interrupt  servicing at 0xa0 state
				 This probably is redundant.This is inserted for safety. It does not harm.*/
				else if(i2cRsCount == 1 && (gI2cData[i2cUnit].counter > 0))
				{   /* received unexpected Nack after reStart; reStart again (one retry max)*/
					status = TM_OK;
					i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|tmhwI2cStartCond|slaveMonitor));
				}
				else
				{/* no HS */
					/* asynchronous caller event */
					status = TMHW_ERR_I2C_EVENT_MST_ABORT;
					i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|tmhwI2cStopCond|slaveMonitor));
				}
				break;

			default:
				break;
		}
			break;

		case (tmhwI2cEvent0x50):    /* Data byte has been received, ACK has been returned */
			switch (request)
		{
			case tmhwI2cEventInterrupt:
				stat = i2cTransceiveData(i2cUnit, tmhwI2cDmaMasterReceive);
				if ( stat == I2C_BUFFER_EMPTY )
				{/* gI2cIP3203Data[i2cUnit].attach == tmhwI2cChain */
					/* synchronous caller event */
					return TMHW_ERR_I2C_EVENT_MST_REC_REQ;
				}
				else if ( stat == I2C_LAST_BYTE )
				{/* (gI2cData[i2cUnit].attach == tmhwI2cRestart |
				    gI2cData[i2cUnit].attach == tmhwI2cStop)
				    tmhwI2cRestart | tmhwI2cStop, counter = 1, send NACK, */
					i2cSetDmaControl(i2cUnit, tmhwI2cDmaNone);
					i2cControl(i2cUnit, (tmhwI2cControl_t)tmhwI2cEnable);
				}
				else
				{
					i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|tmhwI2cAcknowledge));
				}
				break;

			case tmhwI2cEventProceed:
				/* tmhwI2cChain, counter def>0 */
				stat = i2cTransceiveData(i2cUnit, tmhwI2cDmaMasterReceive);
				if (stat == I2C_BUFFER_EMPTY)
				{
					if (gI2cData[i2cUnit].length == 0)
					{
						nextStatus[i2cUnit] = TMHW_ERR_I2C_EVENT_MST_REC_DONE;
					}
					else
					{
						nextStatus[i2cUnit] = TMHW_ERR_I2C_EVENT_MST_REC_ABORT;
					}
					i2cControl(i2cUnit, (tmhwI2cControl_t)tmhwI2cEnable);
				}
				else if (stat == I2C_LAST_BYTE)
				{
					i2cControl(i2cUnit, (tmhwI2cControl_t)tmhwI2cEnable);
				}
				else
				{
					i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable | tmhwI2cAcknowledge));
				}
				break;

			default:
				break;
		}
			break;

		case (tmhwI2cEvent0x58):    /* Data byte has been recieved; NOT ACK has been returned */
			switch (request)
		{
			case tmhwI2cEventInterrupt:
				if (nextStatus[i2cUnit] == TM_OK)
				{/* counter == 0 */
					/* read last byte in case of byte by byte transfer */
					status = i2cTransceiveData(i2cUnit, tmhwI2cDmaMasterReceive);
					/* i2cTransceiveData returned I2C_LAST_BYTE */
					if (gI2cData[i2cUnit].attach == tmhwI2cRestart)
					{
						/* synchronous caller event */
						return TMHW_ERR_I2C_EVENT_MST_REQ;
					}
					else
					{/* tmhwI2cStop */
						/* asynchronous caller event */
						status = TMHW_ERR_I2C_EVENT_MST_REC_DONE;
						i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|tmhwI2cStopCond|slaveMonitor));
					}
				}
				else
				{
					status = nextStatus[i2cUnit];
					i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|tmhwI2cStopCond|slaveMonitor));
				}
				break;

			case tmhwI2cEventProceed:
				/* restart */
				i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable | tmhwI2cStartCond|slaveMonitor));
				break;

			default:
				break;
		}
			break;

		case (tmhwI2cEvent0x60):    /* Own SLA.W has been rceived, ACK has been returned */
		case (tmhwI2cEvent0x68):    /* Arbitration lost in SLA.R/W as master and own SLA.W has received, ACK returned */
		case (tmhwI2cEvent0x70):    /* Generall call address (0x00) has been received, ACK returned */
		case (tmhwI2cEvent0x78):    /* Arbitration lost in SLA.R/W or in Master code ... ACK returned*/
			gI2cDirection[i2cUnit] = tmhwI2cReceive;
			switch (request)
			{
				case tmhwI2cEventInterrupt:
					/* PR eh04#2776.Missing 0xc0 state.*/
					if ( i2cLastState[i2cUnit] == 0xa8 )
					{
						/* Invoke Slave Transmit Done */
						i2cSetDmaControl(i2cUnit, tmhwI2cDmaNone);
						if (gI2cData[i2cUnit].counter == 0)
						{
							status = TMHW_ERR_I2C_EVENT_SLV_TRX_DONE;
						}
						else
						{
							status = TMHW_ERR_I2C_EVENT_SLV_TRX_ABORT;
						}
						i2cLastState [i2cUnit] = 0x60;
						return status;
					}
					i2cLastState [i2cUnit] = 0x60;

					nextStatus[i2cUnit] = TM_OK;
					/* synchronous caller event */
					if ((event == tmhwI2cEvent0x60) || (event == tmhwI2cEvent0x68))
					{
						if (event == tmhwI2cEvent0x68)
						{
							return TMHW_ERR_I2C_EVENT_SLV_REC_REQ_MST_LOST;
						}
						else
						{
							return TMHW_ERR_I2C_EVENT_SLV_REC_REQ;
						}
					}
					else
					{/* event == tmhwI2cEvent0x70 || event == tmhwI2cEvent0x78 */
						if (event == tmhwI2cEvent0x78)
						{
							return TMHW_ERR_I2C_EVENT_SLV_GC_REQ_MST_LOST;
						}
						else
						{
							return TMHW_ERR_I2C_EVENT_SLV_GC_REQ;
						}
					}
					/* break; */   /* not reachable ! */

				case tmhwI2cEventProceed:
					stat = i2cTransceiveData(i2cUnit, tmhwI2cDmaSlaveReceive);
					if ( stat == I2C_BUFFER_EMPTY )
					{
						if (gI2cData[i2cUnit].length == 0)
						{
							nextStatus[i2cUnit] = TMHW_ERR_I2C_EVENT_SLV_REC_DONE;
						}
						else
						{
							nextStatus[i2cUnit] = TMHW_ERR_I2C_EVENT_SLV_REC_ABORT;
						}
						i2cControl(i2cUnit, (tmhwI2cControl_t)tmhwI2cEnable);
					}
					else
					{
						i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable | tmhwI2cAcknowledge));
					}
					break;

				default:
					break;
			}
			break;

		case (tmhwI2cEvent0x80):    /* Previously addressed with own SLA.W; Data byte has been received ACK returned */
		case (tmhwI2cEvent0x90):    /* Previously addressed with generall call; Data byte recieved; ACK returned */
			switch (request)
		{
			case tmhwI2cEventInterrupt:
				/* patch on hw ticket 68117 : Missing 0x60 state*/
				if (i2cLastState [i2cUnit] == 0xa0)
				{
					if (event == tmhwI2cEvent0x80)
					{
						return TMHW_ERR_I2C_EVENT_SLV_REC_REQ;
					}
					else
					{/*event = tmhwI2cEvent0x90*/
						return TMHW_ERR_I2C_EVENT_SLV_GC_REQ;
					}
				}

				if ( i2cTransceiveData(i2cUnit, tmhwI2cDmaSlaveReceive) ==
						I2C_BUFFER_EMPTY )
				{/* buffer empty */
					if (gI2cData[i2cUnit].attach == tmhwI2cChain)
					{/* counter = 0 */
						/* synchronous caller event */
						return TMHW_ERR_I2C_EVENT_SLV_REC_REQ;
					}
					else
					{/* tmhwI2cStop, counter = 0, send NACK, byte has already been read
					    asynchronous caller event */
						nextStatus[i2cUnit]=TMHW_ERR_I2C_EVENT_SLV_REC_DONE;
						/* data read, however slave needs one more state to finish */
						i2cControl(i2cUnit, (tmhwI2cControl_t)tmhwI2cEnable);
					}
				}
				else
				{
					i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|tmhwI2cAcknowledge));
				}
				break;

			case tmhwI2cEventProceed:
				/* patch on hw ticket 68117*/
				if (i2cLastState [i2cUnit] == 0xa0)
				{
					i2cReceiveByte(i2cUnit);
					i2cLastState [i2cUnit] = 0x80;
				}
				/* tmhwI2cChain, buffer counter def>0 */
				if (i2cTransceiveData(i2cUnit, tmhwI2cDmaSlaveReceive) ==
						I2C_BUFFER_EMPTY)
				{
					if (gI2cData[i2cUnit].length == 0)
					{
						nextStatus[i2cUnit] = TMHW_ERR_I2C_EVENT_SLV_REC_DONE;
					}
					else
					{
						nextStatus[i2cUnit] = TMHW_ERR_I2C_EVENT_SLV_REC_ABORT;
					}
					i2cControl(i2cUnit, (tmhwI2cControl_t)tmhwI2cEnable);
				}
				else
				{
					i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable | tmhwI2cAcknowledge));
				}
				break;

			default:
				break;
		}
			break;

		case (tmhwI2cEvent0x88):    /* Previously addressed with own SLA.W; Data byte received, NOT ACK returned */
		case (tmhwI2cEvent0x98):    /* Previously addressed with generall call; Data byte recieved; NOT ACK returned */
			switch (request)
		{
			case tmhwI2cEventInterrupt:
				if (i2cLastState [i2cUnit] == 0xa0)
				{
					i2cLastState [i2cUnit] = 0x88;
					status = TM_OK;
				}
				else
				{
					/* all data has already been received.
					 * master has sent one byte too much.
					 */
					status = TMHW_ERR_I2C_EVENT_SLV_REC_ABORT;
				}
				i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|slaveMonitor));
				break;

			default:
				break;
		}
			break;

		case (tmhwI2cEvent0xa0):    /* A STOP condition or repeated START condition has been received ... */
			switch (request)
		{
			case tmhwI2cEventInterrupt:
				/* patch on hw ticket 68117*/
				if ( (i2cLastState [i2cUnit] == 0xa0) && (gI2cDirection[i2cUnit] == tmhwI2cReceive))
				{ /* zero byte transfer workaround
				     Missing 0x60 state*/
					i2cLastState [i2cUnit] = 0xa1;
					return TMHW_ERR_I2C_EVENT_SLV_REC_REQ;
				}

				i2cLastState [i2cUnit] = 0xa0;

				status = nextStatus[i2cUnit];
				if (gI2cDirection[i2cUnit] == tmhwI2cReceive)
				{
					if ((gI2cData[i2cUnit].counter == 0) && (status == TM_OK))
					{
						status = TMHW_ERR_I2C_EVENT_SLV_REC_DONE;
					}
					else
					{
						if (status == TM_OK)
						{
							status = TMHW_ERR_I2C_EVENT_SLV_REC_ABORT;
						}
					}
				}
				else
				{   /* transmit */
					if ((gI2cData[i2cUnit].counter == 0) && (status == TM_OK))
					{
						status = TMHW_ERR_I2C_EVENT_SLV_TRX_DONE;
					}
					else
					{
						if (gI2cByteWise[i2cUnit] == True)
						{/* transmit, bytewise
						    last written byte was invalid. Correct it. */
							(gI2cData[i2cUnit].counter)++;
						}

						if (status == TM_OK)
						{
							status = TMHW_ERR_I2C_EVENT_SLV_TRX_ABORT;
						}
					}
				}
				i2cSetDmaControl(i2cUnit, tmhwI2cDmaNone);
				i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|slaveMonitor));
				break;

			case tmhwI2cEventProceed:
				/* workaround h/w ticket 68117:
				 * return without clearing the interrupt. This will ensure
				 * a DONE event is returned to the Slave*/
				return TM_OK;
				break;

			default:
				break;
		}
			break;

		case (tmhwI2cEvent0xa8):    /* Own SLA.R is been received; ACK returned */
		case (tmhwI2cEvent0xb0):    /* Arbitration lost in SLA.R/W as master and own SLA.R received, ACK sent */
			i2cLastState [i2cUnit] = 0xa8;
			gI2cDirection[i2cUnit] = tmhwI2cTransmit;
			switch (request)
			{
				case tmhwI2cEventInterrupt:
					nextStatus[i2cUnit] = TM_OK;
					if (event == tmhwI2cEvent0xb0)
					{
						return TMHW_ERR_I2C_EVENT_SLV_TRX_REQ_MST_LOST;
					}

					/* synchronous caller event */
					return TMHW_ERR_I2C_EVENT_SLV_TRX_REQ;

				case tmhwI2cEventProceed:
					/* tmhwI2cChain */
					if (gI2cData[i2cUnit].counter == 0)
					{
						nextStatus[i2cUnit] =  TMHW_ERR_I2C_EVENT_SLV_TRX_ABORT;
						i2cWriteDat(i2cUnit, TMHW_I2C_DATA_DUMMY);
						i2cControl(i2cUnit, (tmhwI2cControl_t)tmhwI2cEnable);
					}
					else
					{/* counter > 0 */
						(Void) i2cTransceiveData(i2cUnit, tmhwI2cDmaSlaveTransmit);
						i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable | tmhwI2cAcknowledge));
					}
					break;

				default:
					break;
			}
			break;

		case (tmhwI2cEvent0xb8):    /* Data byte in I2CDAT has been transmitted; ACK sent */
			switch (request)
		{
			case tmhwI2cEventInterrupt:
				i2cSetDmaControl(i2cUnit, tmhwI2cDmaNone);
				if ( i2cTransceiveData(i2cUnit, tmhwI2cDmaSlaveTransmit) ==
						I2C_BUFFER_EMPTY )
				{/* buffer empty, counter = 0 */
					if (gI2cData[i2cUnit].attach == tmhwI2cChain)
					{
						/* synchronous caller event */
						return TMHW_ERR_I2C_EVENT_SLV_TRX_REQ;
					}
					else
					{/* tmhwI2cStop */
						/*  asynchronous caller event */
						nextStatus[i2cUnit] = TMHW_ERR_I2C_EVENT_SLV_TRX_DONE;
						/* indicate that when next state is tmhwI2cEvent0xa0,
						 * all data have already been sent
						 */

						/* Send dummy */
						i2cWriteDat(i2cUnit, TMHW_I2C_DATA_DUMMY);
						i2cControl(i2cUnit, tmhwI2cEnable);
					}
				}
				else
				{
					i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|tmhwI2cAcknowledge));
				}
				break;

			case tmhwI2cEventProceed:
				/* tmhwI2cChain */
				if (i2cTransceiveData(i2cUnit, tmhwI2cDmaSlaveTransmit) ==
						I2C_BUFFER_EMPTY)
				{
					if (gI2cData[i2cUnit].length == 0)
					{
						nextStatus[i2cUnit] = TMHW_ERR_I2C_EVENT_SLV_TRX_DONE;
					}
					else
					{
						nextStatus[i2cUnit] = TMHW_ERR_I2C_EVENT_SLV_TRX_ABORT;
					}
					i2cControl(i2cUnit, tmhwI2cEnable);
				}
				else
				{
					i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable | tmhwI2cAcknowledge));
				}
				break;

			default:
				break;
		}
			break;

		case (tmhwI2cEvent0xc0):    /* Data bytes in I2CDAT transmitted, NOT ACK is received */
			switch (request)
		{
			case tmhwI2cEventInterrupt:
				/* PR eh04#2776.Missing 0xc0 state. Missing 0xC0 state in SlvTrx*/
				if ( (i2cLastState [i2cUnit] == 0xc0) )
				{ /* zero byte transfer workaround
				     Missing 0xa8 state*/
					i2cLastState [i2cUnit] = 0xc1;
					return TMHW_ERR_I2C_EVENT_SLV_TRX_REQ;
				}

				i2cLastState [i2cUnit] = 0xc0;

				i2cSetDmaControl(i2cUnit, tmhwI2cDmaNone);
				if (gI2cData[i2cUnit].counter == 0)
				{
					status = TMHW_ERR_I2C_EVENT_SLV_TRX_DONE;
				}
				else
				{
					status = TMHW_ERR_I2C_EVENT_SLV_TRX_ABORT;
				}
				i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|slaveMonitor));
				break;

			default:
				break;
		}
			break;

		case (tmhwI2cEvent0xc8):    /* Last data byte in I2CDAT has sent ACK received ... */
			switch (request)
		{
			case tmhwI2cEventInterrupt:
				/* slave has sent dummy while master requires more data
				   error in master, nack had been expected after last byte */
				status = TMHW_ERR_I2C_EVENT_SLV_TRX_ABORT;
				i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable|slaveMonitor));
				break;

			default:
				break;
		}
		default:
			break;
	}

	(Void) tmhwI2cIntClear(i2cUnit);
	return status;
} /* tmhwI2cEvent */


/* ---------------------------------------------------------------------------
 * Internal functions for Driver initialization
 * ---------------------------------------------------------------------------
 */


/* ---------------------------------------------------------------------------
 * FUNCTION:    i2cDriverInit:
 *
 * DESCRIPTION: Initializes the I2c HwAPI driver.
 *
 * RETURN:      -
 *
 * PRE:         state = not initialised
 *
 * POST:        initialised
 *
 * NOTES:       must be called immediately after the first call to a
 *              HwApi function
 *              Re-entrant.
 * ---------------------------------------------------------------------------
 */
	static Void
i2cDriverInit(Void)
{
	ptmhwI2c_Cfg_t      pCfg = Null;
	UInt32              unit = 0;
	ptmhwI2c_Cfg_t      gpI2c_Cfg = Null;
	UInt32              I2c_CfgNumUnits = 0;

#ifndef TMHWI2C_BSL_INDEPENDANT
	(Void) tmbslCoreGetTicksPerSec(&gTimeTicks20us);
	gTimeTicks20us /= 50000; /* For 0105 STOP HW issue check */
#endif

	gpI2c_Cfg = tmhwI2c_CfgGet();   /*  retrieve configuration structure */
	I2c_CfgNumUnits = tmhwI2c_CfgGetNumUnits();
	gI2cUnitCount = (tmUnitSelect_t)I2c_CfgNumUnits;
	/* throughout the code. */
	while (unit < I2c_CfgNumUnits)
	{
		pCfg = &gpI2c_Cfg[unit];

		if(pCfg->moduleID == I2C_IP3203_HWMODULE_ID)
		{
			gI2cModule[unit].I2cType = I2cIP3203;
			/* configure unit */
			tmhwI2cCfgInit( (tmUnitSelect_t)unit, &(gI2cCfg[unit]) );

			if (
#ifndef LINUX_BUILD
					(gI2cRegisterMmFunc[unit].gpVirtToPhys == Null) ||
					(gI2cRegisterMmFunc[unit].gpCacheFlush == Null) ||
					(gI2cRegisterMmFunc[unit].gpCacheInvalidate == Null) ||
#endif
					(gI2cRegisterMmFunc[unit].length <= 1) ||
					(gI2cRegisterMmFunc[unit].bufferPtr == Null))
			{
				gI2cCfg[unit].bDma = False;
			}
		}
		else if(pCfg->moduleID == IIC_IP0105_HWMODULE_ID)
		{
			gI2cModule[unit].I2cType = I2cIP0105;
			/* configure unit */
			tmhwI2cCfgInit( (tmUnitSelect_t)unit, &(gI2cCfg[unit]) );
			gI2cCfg[unit].bDma = False;
		}
		else
		{
			/* Only 2 IP numbers we support now */
		}
		gI2cModule[unit].pRegBase = pCfg->baseAddress;
		unit++;
	}

} /* i2cDriverInit */


/* ---------------------------------------------------------------------------
 * Internal functions for ISR layer:
 * ---------------------------------------------------------------------------
 */


/* ---------------------------------------------------------------------------
 * FUNCTION:    i2cConvertSpeedToReg:
 *
 * DESCRIPTION: This function derives the peripheral register values, given
 *              the requested actual speed at which the specific master will
 *              communicate when it controls the I2C-bus.
 *              For the 0105 return the value to be written in ctrl_cr[2:0] in
 *              *pFSBIR is returned.
 *
 * RETURN:      SFMode
 *
 * PRE:         -
 *
 * NOTES:       Re-entrant.
 * ---------------------------------------------------------------------------
 */
static Void
i2cConvertSpeedToReg (
		tmUnitSelect_t          i2cUnit,
		UInt32                  fsSpeed,
		UInt32                  hsSpeed,
		pUInt32                 pFSBIR,
		pUInt32                 pHSBIR
		)
{
	UInt32 fsBitRate, hsBitRate, fsbir = 0, hsbir = 0;
	UInt32 uiClockFreqKHz;
	UInt32 divfact, divider=0;


	/* limit upper boundary of frequency range
	   SS/FS */
	if (fsSpeed > TMHW_I2C_MAX_FS_SPEED)
	{
		fsSpeed=TMHW_I2C_MAX_FS_SPEED;
	}

	if (fsSpeed > gI2cCfg[i2cUnit].maxFSSpeedKhz)
	{
		fsSpeed = gI2cCfg[i2cUnit].maxFSSpeedKhz;
	}

	uiClockFreqKHz = gI2cCfg[i2cUnit].clockFreqKHz;

	if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
	{
		/* HS */
		if (hsSpeed > TMHW_I2C_MAX_HS_SPEED)
		{
			hsSpeed = TMHW_I2C_MAX_HS_SPEED;
		}

		if (hsSpeed > gI2cCfg[i2cUnit].maxHsSpeedKhz)
		{
			hsSpeed = gI2cCfg[i2cUnit].maxHsSpeedKhz;
		}

		/* prevent devide by zero. */
		if (fsSpeed == 0)
		{
			fsSpeed = 1;
		}/* this will not influence the result of the total calculation */

		if (hsSpeed == 0)
		{
			hsSpeed = 1;
		}/* this will not influence the result of the total calculation */

		/* calculate register values */
		fsBitRate = (uiClockFreqKHz/(8*fsSpeed)) - 1;
		if ((uiClockFreqKHz%(8*fsSpeed)) != 0)
		{
			fsBitRate++;
		}

		hsBitRate = (uiClockFreqKHz/(3*hsSpeed)) - 1;
		if ((uiClockFreqKHz%(3*hsSpeed)) != 0)
		{
			hsBitRate++;
		}

		/* limit upper boundary of register value = lower limit of speed */
		if (fsBitRate > 127)
		{
			fsBitRate = 127;
		}

		if (hsBitRate > 31)
		{
			hsBitRate = 31;
		}

		fsbir = ((fsBitRate << TMVH_I2C3203_FSBIR_BITRATE_POS) & TMVH_I2C3203_FSBIR_BITRATE_MSK);
		hsbir = ((hsBitRate << TMVH_I2C3203_HSBIR_BITRATE_POS) & TMVH_I2C3203_HSBIR_BITRATE_MSK);


		if (fsSpeed > TMHW_I2C_MAX_SS_SPEED)
		{/* SFMode = FS mode */
			/* i2cFSBIR.bits.SFMode = I2C_FS_MODE; */
			fsbir &= ~TMVH_I2C3203_FSBIR_MODE_MSK;
			fsbir |= ((I2C_FS_MODE << TMVH_I2C3203_FSBIR_MODE_POS) & TMVH_I2C3203_FSBIR_MODE_MSK);
		}
		else
		{/* SFMode = SS mode */
			/* i2cFSBIR.bits.SFMode = I2C_SS_MODE; */
			fsbir &= ~TMVH_I2C3203_FSBIR_MODE_MSK;
			fsbir |= ((I2C_SS_MODE << TMVH_I2C3203_FSBIR_MODE_POS) & TMVH_I2C3203_FSBIR_MODE_MSK);
		}

#ifndef MINUS_SPEED_PATCH
		/* IP3203 version 2.c and 2.d patch */
		switch (fsbir)
		{
			case (0x00):
				fsbir++;
				break;
			case (0x02):
				fsbir = 0x04;
				break;
			case (0x03):
				fsbir++;
				break;
			case (0x08):
				fsbir =0xa;
				break;
			case (0x09):
				fsbir++;
				break;
			case (0x0b):
				fsbir++;
				break;
			case (0x0e):
				fsbir++;
				break;
			case (0x28):
				fsbir++;
				break;
			case (0x4f):
				fsbir++;
				break;
			case (0x80):
				fsbir++;
				break;
			case (0x83):
				fsbir = 0x85;
				break;
			case (0x84):
				fsbir++;
				break;
			case (0x87):
				fsbir = 0x8a;
				break;
			case (0x88):
				fsbir = 0x8a;
				break;
			case (0x89):
				fsbir++;
				break;
			default:
				if (fsbir >= 0xaa)
				{
					fsbir = 0xa9;
				}
				break;
		}

		switch (hsbir)
		{
			case (0x00):
				hsbir++;
				break;
			case (0x0c):
				hsbir++;
				break;
			default:
				break;
		}
		/* end IP3203 version 2.c and 2.d patch */
#endif /* if MINUS_SPEED_PATCH */

		*pFSBIR = fsbir;
		*pHSBIR = hsbir;
	}
	else /* for the 0105 return the value to be written in ctrl_cr[2:0] */
	{
		divfact = uiClockFreqKHz/fsSpeed;
		for (divider = 0; (divfact > gDivFact0105[divider]) && (divider < 7); divider++)
		{
		}
		*pFSBIR=divider;
		*pHSBIR = 0;
	}
} /* i2cConvertSpeedToReg */


/* ---------------------------------------------------------------------------
 * FUNCTION:    i2cConvertSpeedToActual:
 *
 * DESCRIPTION: This function derives the actual speed at which the specific
 *              master will communicate when it controls the I2C-bus, given
 *              the peripheral register values.
 *              In case of the 0105 FSBIR should contain the 3 bitvalue to be
 *              written to  ctrl_cr[2:0].
 *
 * RETURN:      -
 *
 * PRE:         -
 *
 * NOTES:       Re-entrant.
 * ---------------------------------------------------------------------------
 */
static Void
i2cConvertSpeedToActual (
		tmUnitSelect_t          i2cUnit,
		UInt32                  FSBIR,
		UInt32                  HSBIR,
		pUInt32                 pFsSpeed,
		pUInt32                 pHsSpeed
		)
{
	UInt32 uiClockFreqKHz, fsBitRate, hsBitRate;

	uiClockFreqKHz = gI2cCfg[i2cUnit].clockFreqKHz;

	if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
	{
		fsBitRate = ((FSBIR & TMVH_I2C3203_FSBIR_BITRATE_MSK) >> TMVH_I2C3203_FSBIR_BITRATE_POS);
		hsBitRate = ((HSBIR & TMVH_I2C3203_HSBIR_BITRATE_MSK) >> TMVH_I2C3203_HSBIR_BITRATE_POS);

		*pFsSpeed = uiClockFreqKHz / (8 * (fsBitRate + 1) );
		if ( ( uiClockFreqKHz%(8*(fsBitRate+1)) ) != 0 )
		{
			(*pFsSpeed)++;
		}

		*pHsSpeed = uiClockFreqKHz / (3 * (hsBitRate + 1) );
		if ( ( uiClockFreqKHz%(3*(hsBitRate+1)) ) != 0)
		{
			(*pHsSpeed)++;
		}

		if (gI2cCfg[i2cUnit].bHS == False)
		{/* indicate not to use HS */
			*pHsSpeed = 0;
		}
	}
	else
	{
		*pFsSpeed = uiClockFreqKHz / gDivFact0105[FSBIR];
		*pHsSpeed = 0;
	}
} /* i2cConvertSpeedToActual */


/* ---------------------------------------------------------------------------
 * FUNCTION:    i2cIP0105SetSpeed:
 *
 * DESCRIPTION: This function sets the speed of a 0105 module and should only
 *              be called from the initialisation function.
 *              The 0105 requires the module to be stopped before any
 *              adjustment to the speed can be done.
 *              The speed is set to the value according to tmhwI2cCfg.c.
 *
 * RETURN:      -
 *
 * PRE:         -
 *
 * NOTES:       Re-entrant.
 * ---------------------------------------------------------------------------
 */
static Void
i2cIP0105SetSpeed (
		tmUnitSelect_t i2cUnit
		)
{
	UInt32 speed, dummy;
	UInt32 reg_value;

	i2cConvertSpeedToReg ( i2cUnit,
			gI2cCfg[i2cUnit].maxFSSpeedKhz,
			0,
			&speed,
			&dummy );
	I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2C_STOP_OFFSET, 1);

	I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2CCONTROL_OFFSET, reg_value);
	reg_value &= ~TMVH_IIC0105_I2CCONTROL_CR2_MSK;
	reg_value |= ( (speed << TMVH_IIC0105_I2CCONTROL_CR2_POS) & TMVH_IIC0105_I2CCONTROL_CR2_MSK);
	I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2CCONTROL_OFFSET, reg_value);
} /* i2cIP0105SetSpeed */


/* ---------------------------------------------------------------------------
 * FUNCTION:        i2cControl:
 *
 * DESCRIPTION:     The control bits that are set to 1 (all or'ed control bits) will
 *                  be set. All other control bits will be reset.
 *                  For the 0105 the clockrate is unaffected.
 *
 * RETURN:          -
 *
 * PRE:             state != idle
 *
 * POST:            state change possible
 *
 * NOTES:           re-entrant
 * -------------------------------------------------------------------------------
 */
#ifdef LINUX_BUILD
Void
#else
static Void
#endif
i2cControl (
		tmUnitSelect_t      i2cUnit,
		tmhwI2cControl_t    control
	   )
{
#ifndef TMHWI2C_BSL_INDEPENDANT
	static UInt32 gTimeStopIssue;
	UInt32 wait20us;
#endif
	UInt32 reg_value, temp;

	if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
	{
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_I2CCON_OFFSET, (control & 0xf4));
	}
	else
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2CCONTROL_OFFSET, reg_value);

		temp = (UInt32)((control & TMVH_I2C3203_I2CCON_START_MSK) >> TMVH_I2C3203_I2CCON_START_POS);
		reg_value &= ~TMVH_IIC0105_I2CCONTROL_START_MSK;
		reg_value |= ((temp << TMVH_IIC0105_I2CCONTROL_START_POS) & TMVH_IIC0105_I2CCONTROL_START_MSK);

		if(reg_value & TMVH_IIC0105_I2CCONTROL_START_MSK)  /* Start cycle bit field set ? */
		{ /* look if STOP was at least 20 us ago */
#ifndef TMHWI2C_BSL_INDEPENDANT
			do
			{
				(Void) tmbslCoreGetTickCount(&wait20us);
			}
			while ((wait20us - gTimeStopIssue) < gTimeTicks20us);
#endif
		}
		temp = (UInt32)((control & TMVH_I2C3203_I2CCON_EN_I2C_MSK) >> TMVH_I2C3203_I2CCON_EN_I2C_POS);
		reg_value &= ~TMVH_IIC0105_I2CCONTROL_EN_I2C_MSK;
		reg_value |= ((temp << TMVH_IIC0105_I2CCONTROL_EN_I2C_POS) & TMVH_IIC0105_I2CCONTROL_EN_I2C_MSK);

		temp = (UInt32)(((UInt32)control & TMVH_I2C3203_I2CCON_AA_MSK) >> TMVH_I2C3203_I2CCON_AA_POS);
		reg_value &= ~TMVH_IIC0105_I2CCONTROL_AA_MSK;
		reg_value |= ((temp << TMVH_IIC0105_I2CCONTROL_AA_POS) & TMVH_IIC0105_I2CCONTROL_AA_MSK);

		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2CCONTROL_OFFSET, reg_value);

		if ( ((control & TMVH_I2C3203_I2CCON_SETSTOP_MSK) >> TMVH_I2C3203_I2CCON_SETSTOP_POS)
				== 1)
		{
			I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2C_STOP_OFFSET, 1);
#ifndef TMHWI2C_BSL_INDEPENDANT
			(Void) tmbslCoreGetTickCount(&gTimeStopIssue);
#endif
		}
	}
} /* i2cIP3203Control */


/* ---------------------------------------------------------------------------
 * FUNCTION:    i2cSetDmaControl:
 *
 * DESCRIPTION: Sets the specified DMA control bits.
 *
 * RETURN:      -
 *
 * PRE:
 *
 * NOTES:       Re-entrant.
 * ---------------------------------------------------------------------------
 */
static Void
i2cSetDmaControl (
		tmUnitSelect_t      i2cUnit,
		tmhwI2cDmaControl_t     dmaControl
		)
{
	if (gI2cModule[i2cUnit].I2cType != I2cIP0105)
	{
		if (dmaControl == tmhwI2cDmaNone)
		{
			gI2cDma[i2cUnit] = False;
		}
		else
		{
			gI2cDma[i2cUnit] = True;
		}
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_DMA_CONTROL_OFFSET, dmaControl);
	}
} /* i2cSetDmaControl */


/* ---------------------------------------------------------------------------
 * FUNCTION:    i2cGetStatus:
 *
 * DESCRIPTION: Gets the current I2c module interrupt state.
 *
 * RETURN:      TMHW_ERR_I2C_BUSY, TM_OK
 *
 * PRE:      state != idle
 *
 * NOTES:       Re-entrant.
 * ---------------------------------------------------------------------------
 */
static Void
i2cGetStatus (
		tmUnitSelect_t  i2cUnit,
		ptmhwI2cEvent_t pI2cEvent
	     )
{
	UInt32 reg_value, iicstatus;

	if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_INT_STATUS_OFFSET, reg_value);

		if ( ((reg_value & TMVH_I2C3203_INT_STATUS_STA0_MSK) >> TMVH_I2C3203_INT_STATUS_STA0_POS)
				== True)
		{
			*pI2cEvent = (tmhwI2cEvent_t)
				((reg_value & TMVH_I2C3203_INT_STATUS_CODE_MSK) >> TMVH_I2C3203_INT_STATUS_CODE_POS);
		}
		else
		{
			*pI2cEvent = tmhwI2cEvent0xf8;
		}
	}
	else
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_INT_STATUS_OFFSET, reg_value);
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2CSTATUS_OFFSET, iicstatus);

		if ( ((reg_value & TMVH_IIC0105_INT_STATUS_STA_MSK) >> TMVH_IIC0105_INT_STATUS_STA_POS)
				== True)
		{
			*pI2cEvent = (tmhwI2cEvent_t)
				((iicstatus & TMVH_IIC0105_I2CSTATUS_STATUS_CODE_MSK) >>TMVH_IIC0105_I2CSTATUS_STATUS_CODE_POS);
		}
		else
		{
			*pI2cEvent = tmhwI2cEvent0xf8;
		}
	}
} /* i2cGetStatus */


/* DMA */
/* ---------------------------------------------------------------------------
 * FUNCTION:    i2cPrepareDma:
 *
 * DESCRIPTION: Starts data communication using DMA
 *              Checks whether to include the last byte.
 *              If buffer is empty or one byte left while last byte needs
 *              not be included, returns I2C_BUFFER_EMPTY.
 *              I2cDMA_CONTROL is loaded.
 *              I2cI2CON register is ->not<- loaded.
 *
 * RETURN:      an indication whether DMA is started successfully, is of no
 *              interrest. No further action is needed anyhow.
 *              When this function is used, no byte-wise action is needed.
 *              Otherwise the function i2cTransceiveData() should be used
 *
 * PRE:         buffer != empty
 *
 * POST:        a block of data is retreived from buffer
 *              DMA is initialised
 *
 * NOTES:       Not re-entrant.
 * ---------------------------------------------------------------------------
 */
static Void
i2cPrepareDma (
		tmUnitSelect_t      i2cUnit,
		tmhwI2cDmaControl_t dmaControl
	      )
{
	Bool        bIncludeLastByte;
	pUInt8      pAddressLocal;
#ifndef LINUX_BUILD
	pUInt8      pPhysical;
#endif
	pUInt8      pDmaBuffer;
	UInt32      i, uiCounter;


	if (gI2cModule[i2cUnit].I2cType != I2cIP0105)
	{

		bIncludeLastByte = (i2cIncludeLastByte(i2cUnit, dmaControl))
			&& ((Bool)(gI2cData[i2cUnit].counter > 0));

		if (!bIncludeLastByte)
		{/* act as if the buffer is one byte shorter */
			gI2cData[i2cUnit].length--;
			gI2cData[i2cUnit].counter--;
		}

		if ( (gI2cCfg[i2cUnit].bDma == True) &&
				(gI2cData[i2cUnit].counter > 0) )
		{/* DMA, counter > 0 */
			uiCounter = gI2cData[i2cUnit].counter;
			if(uiCounter > gI2cRegisterMmFunc[i2cUnit].length  )
			{/* limit number of bytes to buffer size */
				uiCounter = gI2cRegisterMmFunc[i2cUnit].length;
			}

			if ( (dmaControl == tmhwI2cDmaMasterTransmit) ||
					(dmaControl == tmhwI2cDmaSlaveTransmit) )
			{
				pDmaBuffer = gI2cRegisterMmFunc[i2cUnit].bufferPtr;
				pAddressLocal = gI2cData[i2cUnit].pAddress +
					(gI2cData[i2cUnit].length -
					 gI2cData[i2cUnit].counter);
				for(i = 0; i < uiCounter; i++)
				{
					*pDmaBuffer = *pAddressLocal;
					pDmaBuffer++;
					pAddressLocal++;
				}
			}

#ifndef LINUX_BUILD

			gI2cRegisterMmFunc[i2cUnit].gpVirtToPhys((gI2cRegisterMmFunc[i2cUnit].bufferPtr), (pVoid *)(&pPhysical));
#ifdef I2C_DMA_WORD_ALINED
			if ((UInt32) pPhysical % 4 == 0)
			{/* word alined only */
#endif /* I2C_DMA_WORD_ALINED */
				I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_DMA_ADDR_OFFSET, (UInt32) pPhysical);

				I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_DMA_LEN_OFFSET, (UInt32) uiCounter);

				gI2cData[i2cUnit].counter = gI2cData[i2cUnit].counter - uiCounter;
				i2cSetDmaControl(i2cUnit, dmaControl);

				if ( (dmaControl == tmhwI2cDmaMasterTransmit) ||
						(dmaControl == tmhwI2cDmaSlaveTransmit ))
				{
					/* flush memory before DMA */
					gI2cRegisterMmFunc[i2cUnit].gpCacheFlush(gI2cRegisterMmFunc[i2cUnit].bufferPtr, uiCounter);
				}
#ifdef I2C_DMA_WORD_ALINED
			}/* word alined only */
#endif /* I2C_DMA_WORD_ALINED */

#else /* LINUX_BUILD */

			I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_DMA_ADDR_OFFSET,
					(UInt32)( gI2cRegisterMmFunc[ i2cUnit ].phyAddr));
			I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_DMA_LEN_OFFSET,
					(UInt32) uiCounter);
			gI2cData[i2cUnit].counter = gI2cData[i2cUnit].counter - uiCounter;
			i2cSetDmaControl(i2cUnit, dmaControl);

#endif /* LINUX_BUILD */
		}

		if (!bIncludeLastByte)
		{/* actually, the buffer is one byte longer */
			(gI2cData[i2cUnit].length)++;
			(gI2cData[i2cUnit].counter)++;
		}
	}   /* if(gI2cModule[i2cUnit].I2cType != I2cIP0105 */
} /* i2cPrepareDma */


/* ---------------------------------------------------------------------------
 * FUNCTION:    i2cCheckDma:
 *
 * DESCRIPTION: Check's if DMA transfer was successfull and if not, corrects
 *              the value gI2cData[i2cUnit].counter
 *
 * RETURN:      TM_OK if transaction was successfull
 *
 * PRE:
 *
 * POST:
 *
 * NOTES:       re-entrant.
 * ---------------------------------------------------------------------------
 */
static tmErrorCode_t
i2cCheckDma (
		tmUnitSelect_t      i2cUnit,
		tmhwI2cRequest_t    request
	    )
{
	tmErrorCode_t       status = TM_OK;
	pUInt8              pDmaBuffer, pAddressLocal;
	UInt32              i;

	/* accelerator: indirection pass */
	UInt32          dmaCounter, dmaLength, dmaControl;

	if (request == tmhwI2cEventInterrupt)
	{
		if (gI2cDma[i2cUnit] == True)
		{
			status = TM_OK;
			gI2cByteWise[i2cUnit] = False;

			/* flush memory after DMA */
			I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_DMA_COUNTER_OFFSET, dmaCounter);
			I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_DMA_CONTROL_OFFSET, dmaControl);

			if ((dmaControl == tmhwI2cDmaMasterReceive) ||
					(dmaControl == tmhwI2cDmaSlaveReceive))
			{

				pDmaBuffer = gI2cRegisterMmFunc[i2cUnit].bufferPtr;
				I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_DMA_LEN_OFFSET, dmaLength);


				pAddressLocal  = ((gI2cData[i2cUnit].pAddress +
							gI2cData[i2cUnit].length) -
						gI2cData[i2cUnit].counter) -
					dmaLength;
#ifndef LINUX_BUILD
				gI2cRegisterMmFunc[i2cUnit].gpCacheInvalidate (pDmaBuffer, dmaLength - dmaCounter);
#endif
				for(i=0; i < (dmaLength - dmaCounter); i++)
				{
					*pAddressLocal = *pDmaBuffer;
					pAddressLocal++;
					pDmaBuffer++;
				}

			}

			/* correct gI2cData[i2cUnit].counter value */
			if (dmaCounter != 0)
			{
				gI2cData[i2cUnit].counter += dmaCounter;
				status = I2C_DMA_NOT_SUCCESSFUL;

				/* switch off DMA mode */
				i2cSetDmaControl(i2cUnit, tmhwI2cDmaNone);
			}
		}
		else
		{/* gI2cDma[i2cUnit] = False */
			gI2cByteWise[i2cUnit] = True;
		}
	}
	else
	{/* request != tmhwI2cEventInterrupt */
		gI2cByteWise[i2cUnit] = False;
	}

	/* switch off DMA mode */
	if ( gI2cData[i2cUnit].counter == 0 )
	{
		i2cSetDmaControl(i2cUnit, tmhwI2cDmaNone);
	}

	return status;
} /* i2cCheckDma */


/* byte transfer */

/* ---------------------------------------------------------------------------
 * FUNCTION:        i2cWriteDat:
 *
 * DESCRIPTION:     Loads the value dat into the data register
 *
 * RETURN:          -
 *
 * NOTES:           Re-entrant.
 * ---------------------------------------------------------------------------
 */
static Void
i2cWriteDat (
		tmUnitSelect_t          i2cUnit,
		UInt8                   dat
	    )
{
	UInt32 reg_value;
	if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_I2CDAT_OFFSET, reg_value);
		reg_value &= ~TMVH_I2C3203_I2CDAT_DATA_MSK;
		reg_value |= (UInt32)((dat << TMVH_I2C3203_I2CDAT_DATA_POS) & TMVH_I2C3203_I2CDAT_DATA_MSK);
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_I2CDAT_OFFSET, reg_value);
	}
	else
	{
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2CDAT_OFFSET, reg_value);
		reg_value &= ~TMVH_IIC0105_I2CDAT_DATA_MSK;
		reg_value |= (UInt32)((dat << TMVH_IIC0105_I2CDAT_DATA_POS) & TMVH_IIC0105_I2CDAT_DATA_MSK);
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2CDAT_OFFSET, reg_value);
	}
} /* i2cWriteDat */


/* ---------------------------------------------------------------------------
 * FUNCTION:    i2cTransmitByte:
 *
 * DESCRIPTION: Retreives a byte from buffer and loads it into the data
 *              register
 *
 * RETURN:      -
 *
 * PRE:         gI2cData[i2cUnit].counter>0
 *
 * NOTES:       Not re-entrant.
 * ---------------------------------------------------------------------------
 */
static Void
i2cTransmitByte (
		tmUnitSelect_t  i2cUnit
		)
{
	UInt32 reg_value, dat;

	if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
	{
		dat = *( gI2cData[i2cUnit].pAddress +
				(gI2cData[i2cUnit].length - gI2cData[i2cUnit].counter) );
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_I2CDAT_OFFSET, reg_value);
		reg_value &= ~TMVH_I2C3203_I2CDAT_DATA_MSK;
		reg_value |= ((dat << TMVH_I2C3203_I2CDAT_DATA_POS) & TMVH_I2C3203_I2CDAT_DATA_MSK);
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_I2CDAT_OFFSET, reg_value);
	}
	else
	{
		dat = *( gI2cData[i2cUnit].pAddress +
				(gI2cData[i2cUnit].length - gI2cData[i2cUnit].counter) );
		I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2CDAT_OFFSET, reg_value);
		reg_value &= ~TMVH_IIC0105_I2CDAT_DATA_MSK;
		reg_value |= ((dat << TMVH_IIC0105_I2CDAT_DATA_POS) & TMVH_IIC0105_I2CDAT_DATA_MSK);
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2CDAT_OFFSET, reg_value);
	}

#ifdef I2C_PRINTK
	printk("\twrite: \t\t0x%x counter %d\n", *( gI2cData[i2cUnit].pAddress +
				(gI2cData[i2cUnit].length - gI2cData[i2cUnit].counter)), gI2cData[i2cUnit].counter);
#endif /* I2C_PRINTK */

	gI2cData[i2cUnit].counter--;
} /* i2cTransmitByte */


/* ---------------------------------------------------------------------------
 * FUNCTION:    i2cReceiveByte:
 *
 * DESCRIPTION: retreives a byte from the data register and stores it into
 *              the data buffer
 *
 * RETURN:      -
 *
 * PRE:         gI2cData[i2cUnit].counter>0
 *
 * NOTES:       Not re-entrant.
 * ---------------------------------------------------------------------------
 */
static Void
i2cReceiveByte (
		tmUnitSelect_t      i2cUnit
	       )
{
	UInt32 reg_value;
	if (gI2cData[i2cUnit].counter > 0)
	{

		if (gI2cModule[i2cUnit].I2cType == I2cIP3203)
		{
			I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_I2CDAT_OFFSET, reg_value);
			reg_value = ((reg_value & TMVH_I2C3203_I2CDAT_DATA_MSK) >> TMVH_I2C3203_I2CDAT_DATA_POS);
			*( gI2cData[i2cUnit].pAddress + (gI2cData[i2cUnit].length -
						gI2cData[i2cUnit].counter) ) = (UInt8)reg_value;
		}
		else
		{
			I2C_READ( gI2cModule[i2cUnit].pRegBase + TMVH_IIC0105_I2CDAT_OFFSET, reg_value);
			reg_value = ((reg_value & TMVH_IIC0105_I2CDAT_DATA_MSK) >> TMVH_IIC0105_I2CDAT_DATA_POS);
			*( gI2cData[i2cUnit].pAddress + (gI2cData[i2cUnit].length -
						gI2cData[i2cUnit].counter) ) = (UInt8)reg_value;
		}

#ifdef I2C_PRINTK
		printk("\tread: \t0x%x counter %d \n", *( gI2cData[i2cUnit].pAddress +
					(gI2cData[i2cUnit].length - gI2cData[i2cUnit].counter)), gI2cData[i2cUnit].counter);
#endif /* I2C_PRINTK */

		gI2cData[i2cUnit].counter--;
	}
} /* i2cReceiveByte */


/* Block transfer */

/* ---------------------------------------------------------------------------
 * FUNCTION:    i2cTransceiveData:
 *
 * DESCRIPTION: starts data communication, transmit or receive. Automatically
 *              chooses between DMA or byte-wise,
 *              checks whether to include the last byte,
 *              if buffer is empty or one byte is left while last byte need
 *              not be included, returns I2C_IP3203_BUFFER_EMPTY,
 *              if one byte is left while last byte need not be included,
 *              retreives last byte to or from data register
 *              I2cDMA_CONTROL is loaded,
 *              I2cI2CON register is ->not<- loaded.
 *
 * RETURN:      I2C_IP3203_BUFFER_EMPTY, TM_OK
 *
 * NOTES:       Re-entrant.
 * ---------------------------------------------------------------------------
 */
static tmErrorCode_t
i2cTransceiveData (
		tmUnitSelect_t      i2cUnit,
		tmhwI2cDmaControl_t dmaControl
		)
{

	Bool            bIncludeLastByte;
	pUInt8          pAddressLocal;
#ifndef LINUX_BUILD
	pUInt8          pPhysical;
#endif
	pUInt8          pDmaBuffer;
	UInt32          i, uiCounter;

	if ( ((gI2cByteWise[i2cUnit] == True) &&
				((dmaControl == tmhwI2cDmaMasterReceive) ||
				 (dmaControl == tmhwI2cDmaSlaveReceive)) )&&
			(gI2cData[i2cUnit].length != 0) )
	{	
		i2cReceiveByte(i2cUnit);
	}
	if (gI2cData[i2cUnit].counter == 0)
	{
		return I2C_BUFFER_EMPTY;
	}

	bIncludeLastByte = (Bool)((i2cIncludeLastByte(i2cUnit, dmaControl)) &&
			(gI2cData[i2cUnit].counter > 0));
	if (!bIncludeLastByte)
	{
		gI2cData[i2cUnit].length--;
		gI2cData[i2cUnit].counter--;
	}

	if (gI2cData[i2cUnit].counter == 0)
	{/* one byte left - return MasterReceive: return Nack */
		(gI2cData[i2cUnit].length)++;
		(gI2cData[i2cUnit].counter)++;
		return I2C_LAST_BYTE;
	}
	if (( (dmaControl == tmhwI2cDmaMasterTransmit) && (gI2cDma[i2cUnit] == False) ) ||
			(dmaControl == tmhwI2cDmaSlaveTransmit) )
	{
		i2cTransmitByte(i2cUnit);
	}
	if ( (gI2cCfg[i2cUnit].bDma == True) &&
			(gI2cData[i2cUnit].counter > 0) )
	{/* DMA, counter > 0 */
		uiCounter = gI2cData[i2cUnit].counter;
		if(uiCounter > gI2cRegisterMmFunc[i2cUnit].length  )
		{/* limit number of bytes to buffer size */
			uiCounter = gI2cRegisterMmFunc[i2cUnit].length;
		}

		if ( (dmaControl == tmhwI2cDmaMasterTransmit) ||
				(dmaControl == tmhwI2cDmaSlaveTransmit) )
		{
			pDmaBuffer = gI2cRegisterMmFunc[i2cUnit].bufferPtr;
			pAddressLocal = gI2cData[i2cUnit].pAddress +
				(gI2cData[i2cUnit].length -
				 gI2cData[i2cUnit].counter);
			for(i=0; i< uiCounter; i++)
			{
				*pDmaBuffer = *pAddressLocal;
				pDmaBuffer++;
				pAddressLocal++;
			}
		}

#ifndef LINUX_BUILD

		gI2cRegisterMmFunc[i2cUnit].gpVirtToPhys(gI2cRegisterMmFunc[i2cUnit].bufferPtr, (pVoid *)(&pPhysical));
#ifdef I2C_DMA_WORD_ALINED
		if ((UInt32) pPhysical % 4 == 0)
		{/* word alined only */
#endif /* I2C_DMA_WORD_ALINED */
			I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_DMA_ADDR_OFFSET, (UInt32) pPhysical);
			I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_DMA_LEN_OFFSET, (UInt32) uiCounter);

			gI2cData[i2cUnit].counter = gI2cData[i2cUnit].counter - uiCounter;
			i2cSetDmaControl(i2cUnit, dmaControl);

			if ( (dmaControl == tmhwI2cDmaMasterTransmit) ||
					(dmaControl == tmhwI2cDmaSlaveTransmit) )
			{
				/* flush memory before DMA */
				gI2cRegisterMmFunc[i2cUnit].gpCacheFlush(gI2cRegisterMmFunc[i2cUnit].bufferPtr, uiCounter);
			}

#ifdef I2C_DMA_WORD_ALINED
		}/* word alined only */
#endif /* I2C_DMA_WORD_ALINED */

#else /* LINUX_BUILD */
		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_DMA_ADDR_OFFSET,
				(UInt32)( gI2cRegisterMmFunc[ i2cUnit ].phyAddr));

		I2C_WRITE( gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_DMA_LEN_OFFSET,
				(UInt32) uiCounter);

		gI2cData[i2cUnit].counter = gI2cData[i2cUnit].counter - uiCounter;
		// printk("DMA Control is %d\n",dmaControl);
		i2cSetDmaControl(i2cUnit, dmaControl);

#endif /* LINUX_BUILD */

	}

	if (!bIncludeLastByte)
	{/* actually, the buffer is one byte longer */
		(gI2cData[i2cUnit].length)++;
		(gI2cData[i2cUnit].counter)++;
	}

	return TM_OK;
} /* i2cTransceiveData */


/* ---------------------------------------------------------------------------
 * FUNCTION:    i2cSendSlaveAddress:
 *
 * DESCRIPTION: sends slave address + r/w
 *              initializes DMA when possible
 *              I2cDMA_CONTROL is loaded.
 *              I2cI2CON register is ->not<- loaded.
 *
 * RETURN:      -
 *
 * PRE:
 *
 * NOTES:       Not re-entrant.
 * ---------------------------------------------------------------------------
 */
static Void i2cSendSlaveAddress (
		tmUnitSelect_t              i2cUnit
		)
{
	/* the request has already been passed
	   the data have already been loaded into the registers
	   so, just continue! */
	i2cWriteDat(i2cUnit, (UInt8)(gI2cData[i2cUnit].direction) |
			(gI2cData[i2cUnit].slaveAddress << 1) );
	/* prepare DMA any way. When DMA does not start here,
	   it will correct itself. */
	if (gI2cData[i2cUnit].counter > 0)
	{
		if (gI2cData[i2cUnit].direction == tmhwI2cTransmit)
		{/* transmit */
			i2cPrepareDma(i2cUnit, tmhwI2cDmaMasterTransmit);
		}
		/* for receive, wait until state 0x40 */
	}
} /* i2cSendSlaveAddress */


/* ---------------------------------------------------------------------------
 * FUNCTION:    i2cIncludeLastByte:
 *
 * DESCRIPTION: determines whether last byte must be included in block data
 *              transfer. A last must be excluded e.g. when a Nack must be
 *              returned when last data byte is received, while other bytes
 *              are returned with an Ack.
 *
 * RETURN:      True if last byte must be included
 *              False: otherwise
 *
 * NOTES:       Re-entrant.
 * ---------------------------------------------------------------------------
 */
static Bool
i2cIncludeLastByte (
		tmUnitSelect_t      i2cUnit,
		tmhwI2cDmaControl_t dmaControl
		)
{
	Bool bIncludeLastByte = True;

	if ( (dmaControl == tmhwI2cDmaMasterReceive) &&
			(gI2cData[i2cUnit].attach != tmhwI2cChain) )
	{/* tmhwI2cStop || tmhwI2cRestart */
		bIncludeLastByte = False;
	}

	return bIncludeLastByte;
} /* i2cIncludeLastByte */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cIP3202SetByteWise:
 *
 * DESCRIPTION: After calling this function, I2C data transfer will be done
 *              byte by byte.
 *              (undocumented feaure)
 *
 * RETURN:      -
 *
 * NOTES:       Not re-entrant.
 * ---------------------------------------------------------------------------
 */
Void
tmhwI2cIP3203SetByteWise (
		tmUnitSelect_t              i2cUnit
		)
{

	gI2cCfg[i2cUnit].bDma = False;

} /* tmhwI2cIP3203SetByteWise */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cIP3203SetDMA:
 *
 * DESCRIPTION: After calling this function, I2C data transfer will be
 *              done through DMA in case of the IP3203 module, for the IP0105
 *              calling this function has no effect.
 *              (undocumented feature)
 *
 * RETURN:      -
 *
 * NOTES:       Not re-entrant.
 * ---------------------------------------------------------------------------
 */
Void
tmhwI2cIP3203SetDMA (
		tmUnitSelect_t              i2cUnit
		)
{
	tmhwI2cCfg_t    i2cCfg;

	if(
#ifndef LINUX_BUILD
			(gI2cRegisterMmFunc[i2cUnit].gpVirtToPhys != Null) &&
			(gI2cRegisterMmFunc[i2cUnit].gpCacheFlush != Null) &&
			(gI2cRegisterMmFunc[i2cUnit].gpCacheInvalidate != Null) &&
#endif
			(gI2cRegisterMmFunc[i2cUnit].length > 1) &&
			(gI2cRegisterMmFunc[i2cUnit].bufferPtr != Null))

	{
		tmhwI2cCfgInit( i2cUnit, &i2cCfg );
		gI2cCfg[i2cUnit].bDma = i2cCfg.bDma;
	}
	else
	{
		gI2cCfg[i2cUnit].bDma = False;
	}

} /* tmhwI2cIP3203SetDMA */


/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cHotBoot
 *
 * DESCRIPTION: Terminates any ongoing transfers and STOPs the MASTER
 *          Puts the slave to non-addressed mode.
 *          De-inits the hw unit.
 *
 * RETURN:      -
 *
 * NOTES:      Only for IP3203. re-entrant.
 *          If DMA tansaction is ongoing, no register write commands can be issued.
 *          Register Readcan be issued only to DMA STATUS and INT STATUS registers.
 *          Other register reads will result in DEADABBA. Bus busy or Free
 *          Status cannot be determined. Hence waitforInt until timeout unconditionally.
 * ---------------------------------------------------------------------------
 */
Void
tmhwI2cHotBoot (
		tmUnitSelect_t  i2cUnit
	       )
{

	tmhwI2cEvent_t event;
	tmhwI2cDirection_t direction = tmhwI2cTransmit;
	UInt8 slaveaddress = 0xEE;/* dummy slave*/

	/* wait for an interrupt */
	i2cWaitForInt(i2cUnit);
	i2cGetStatus(i2cUnit, &event);

	switch(event)
	{
		case tmhwI2cEvent0x08 :
		case tmhwI2cEvent0x10 : /* Master mode, Send a dummy slave address*/
			i2cWriteDat(i2cUnit, (UInt8)(direction | (slaveaddress << 1)) );
			(Void) tmhwI2cIntClear(i2cUnit);
			break;
		case tmhwI2cEvent0x40 :
		case tmhwI2cEvent0x50 : /* MASTER mode, NACK will be programmed*/
			i2cControl(i2cUnit, (tmhwI2cControl_t)(tmhwI2cEnable));
			(Void) tmhwI2cIntClear(i2cUnit);
			break;
		case tmhwI2cEvent0xf8 :
			/* no interrupt may be due to clock stretching by Slave*/
			/* Stop the slave & Geenrate STOP condition*/
			tmhwI2cStopSlave(i2cUnit);
			i2cControl(i2cUnit,
					(tmhwI2cControl_t)(tmhwI2cEnable|tmhwI2cStopCond));
			(Void) tmhwI2cIntClear(i2cUnit);
			/* WAIT for STOP to be set*/
			i2cWaitForSTO(i2cUnit);
			break;
		default : /* for other states just generate a STOP condition.*/
			break;
	}
	i2cWaitForInt(i2cUnit);
	tmhwI2cStopSlave(i2cUnit);
	/* Generate STOP condition*/
	i2cControl(i2cUnit,
			(tmhwI2cControl_t)(tmhwI2cEnable|tmhwI2cStopCond));
	(Void) tmhwI2cIntClear(i2cUnit);

	/* WAIT for STOP to be set*/
	i2cWaitForSTO(i2cUnit);

	(Void) tmhwI2cDeinit(i2cUnit);
}

/* ---------------------------------------------------------------------------
 * FUNCTION:    i2cWaitForInt
 *
 * DESCRIPTION: waits for an interrupt
 *
 * RETURN:      -
 *
 * NOTES:      Only for IP3203. re-entrant.
 * ---------------------------------------------------------------------------
 */
static Void
i2cWaitForInt(
		tmUnitSelect_t i2cUnit
	     )
{
	tmhwI2cStatusType_t  intstatus;
	int timeout = HWI2C_RESET_TIMEOUT;
	while(timeout > 0)
	{
#ifdef LINUX_BUILD
		udelay(100); /* sleep 100us*/
#endif
		timeout--;
		tmhwI2cIntGetStatus(i2cUnit,  &intstatus);
		if ( intstatus != tmhwI2cNoInterrupt )
			break;
	}
}

/* ---------------------------------------------------------------------------
 * FUNCTION:    i2cWaitForSTO
 *
 * DESCRIPTION: waits for STO bit to be set
 *
 * RETURN:      -
 *
 * NOTES:      Only for IP3203. re-entrant.
 * ---------------------------------------------------------------------------
 */
static Void
i2cWaitForSTO(
		tmUnitSelect_t  i2cUnit
	     )
{
	UInt32 reg_value;
	int timeout = HWI2C_RESET_TIMEOUT;
	while(timeout > 0)
	{
#ifdef LINUX_BUILD
		udelay(100); /* sleep 100us*/
#endif
		timeout--;
		I2C_READ( (gI2cModule[i2cUnit].pRegBase + TMVH_I2C3203_INTROG_OFFSET),
				reg_value);
		if ( (reg_value & TMVH_I2C3203_INTROG_INTRO_STO_MSK)
				== TMVH_I2C3203_INTROG_INTRO_STO_MSK )
			break;
	}
}


#ifdef LINUX_BUILD

/* Reset unit definitions. Only for pnx8535*/
#define RESET_UNIT_BASE     0x00060000
#define RESET_I2C_UNIT0     10
#define RESET_I2C_UNIT1     11
#define RESET_CONTROL_REG   0x100

/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cSoftReset
 *
 * DESCRIPTION: Resets the i2c hardware unit using reset generation unit
 *
 * RETURN:      -
 *
 * NOTES:
 * ---------------------------------------------------------------------------
 */
Void
tmhwI2cSoftReset (
		tmUnitSelect_t              i2cUnit
		)
{
	/* Do a hard reset of the i2c unit */
	unsigned long lockstat = 0;
	unsigned long greset = 0;

	/* Unlock the RGU Lock Status register */
	writel(0x000000F8, RST_LOCKCMD_REG);
	writel(0x0000002B, RST_LOCKCMD_REG);

	/* Unlock the Global Reset registers */
	lockstat = readl(RST_LOCKSTAT_REG);
	lockstat &= ~RST_LOCKSTAT_LOCK;
	writel(lockstat, RST_LOCKSTAT_REG);

	/* Issue reset */
	switch(i2cUnit)
	{
		case 0:
			greset = readl(RST_GRESET0_REG);
			greset |= RST_GRESET0_IIC1; /* issue reset */
			writel(greset, RST_GRESET0_REG);
			msleep(1); /* revisit this 1 ms delay with hw teams. */
			greset &= ~RST_GRESET0_IIC1; /* clear reset */
			writel(greset, RST_GRESET0_REG);
			break;
		case 1:
			greset = readl(RST_GRESET0_REG);
			greset |= RST_GRESET0_IIC2;
			writel(greset, RST_GRESET0_REG);
			msleep(1);
			greset &= ~RST_GRESET0_IIC2;
			writel(greset, RST_GRESET0_REG);
			break;
		case 2:
			greset = readl(RST_GRESET0_REG);
			greset |= RST_GRESET0_IIC3;
			writel(greset, RST_GRESET0_REG);
			msleep(1);
			greset &= ~RST_GRESET0_IIC3;
			writel(greset, RST_GRESET0_REG);
			break;
		case 3:
			greset = readl(RST_GRESET2_REG);
			greset |= RST_GRESET2_IIC4;
			writel(greset, RST_GRESET2_REG);
			msleep(1);
			greset &= ~RST_GRESET2_IIC4;
			writel(greset, RST_GRESET2_REG);
			break;
		default:
			printk(KERN_ERR "Invalid I2C unit %d\n", i2cUnit);
			break;
	}

	/* Lock the RGU Lock Status register */
	writel(0x00000000, RST_LOCKCMD_REG);
}
#endif


#ifndef LINUX_BUILD
/* ---------------------------------------------------------------------------
 * FUNCTION:    tmhwI2cDoRequest:
 *
 * DESCRIPTION: Sequence master IIC machine states for both polled and
 *              interrupt mode.
 *
 * RETURN:      TMHW_ERR_IIC_BAD_UNIT_ID
 *              TMHW_ERR_IIC_BAD_PARAMETER
 *              TMHW_ERR_IIC_MASTER_WRITE
 *              TMHW_ERR_IIC_MASTER_READ
 *              TMHW_ERR_IIC_NO_SLA_ACK
 *              TMHW_ERR_IIC_NO_DATA_ACK
 *              TMHW_ERR_IIC_ARB_LOST
 *              TM_OK
 *
 * PRE:         Master only can call the function
 *
 * NOTES:       For either polled or interrupt mode, caller should set
 *              bStart to TRUE when calling the function the first time.
 *
 *              For either mode, caller should set pFinished variable to
 *              FALSE.
 *
 *              - For polled mode, pFinished is always TRUE when
 *                function returns to caller.
 *
 *              - For interrupt mode, pFinished is only TRUE when all
 *                states of one IIC transfer are completed.
 *
 *              Not re-entrant.
 * ---------------------------------------------------------------------------
 */
tmErrorCode_t
tmhwI2cLIBDoRequest(
		tmUnitSelect_t        unitID,     /* I: iic unit */
		Bool                  bStart,     /* I: True if 1st call */
		Bool                  bDoPoll,    /* I: IGNORED */
		ptmhwI2cLIBRequest_t  pHwReq,     /* I: data for sequenceing state machine */
		Bool                  *pFinished  /* O: set to True if this step finished */
		/*    a transaction */
		)
{
	(void) bStart;     /* Why are these not used ? */
	(void) bDoPoll;
	(void) pFinished;

	return tmhwI2cDoRequest(unitID, (ptmhwI2cDoRequest_t)pHwReq);
}



tmErrorCode_t
tmhwI2cDoRequest(
		tmUnitSelect_t        unitID,       /* I: iic unit */
		ptmhwI2cDoRequest_t  pHwReq         /* I: data for sequenceing state machine */
		)
{
	UInt32       numReq = 1;
	tmhwI2cData_t masterData1, masterData2;
	Bool done = False;
	tmErrorCode_t status;
	UInt32       uiTimeout;
	UInt32       ticksPerSec;
	UInt32       ticksPer1ms;
	UInt32       ticksPerTimeout;
	UInt32       currTicks;
	UInt32       entryTicks;
	UInt32       timediff;
	UInt8        switchVar =0;

	masterData1.bHS = False;
	masterData2.bHS = False;
	masterData1.slaveAddress = pHwReq->address;
	masterData2.slaveAddress = pHwReq->address;
	if (gI2cModule[unitID].I2cType == I2cIP3203)
	{
		tmhwI2cIP3203SetByteWise(unitID);
	}
	status = tmhwI2cIntDisable(unitID);

	if ((pHwReq->pWriteData == Null) && (pHwReq->pReadData == Null))
	{
		return (TMHW_ERR_I2C_INVALID_REQUEST);
	}

	if(pHwReq->numReadBytes !=0)
	{
		switchVar |= (1<<3);
	}
	if(pHwReq->numWriteBytes !=0)
	{
		switchVar |= (1<<2);
	}
	if(pHwReq->pReadData != Null)
	{
		switchVar |= (1<<1);
	}
	if(pHwReq->pWriteData != Null)
	{
		switchVar |= 0x1;
	}
	switch(switchVar)
	{
		case 0x00:
		case 0x04:
		case 0x08:
		case 0x0c:
			/*  Error */
			return (TMHW_ERR_I2C_INVALID_REQUEST);
			/* break; */

		case 0x01:
		case 0x03:
		case 0x05:
		case 0x07:
		case 0x09:
		case 0x0d:
			/* Write */
			masterData1.attach = tmhwI2cStop;
			masterData1.direction = tmhwI2cTransmit;
			masterData1.pAddress = pHwReq->pWriteData;
			masterData1.length = pHwReq->numWriteBytes;
			break;

		case 0x02:
		case 0x06:
		case 0x0a:
		case 0x0b:
		case 0x0e:
			/* Read */
			masterData1.attach = tmhwI2cStop;
			masterData1.direction = tmhwI2cReceive;
			masterData1.length = pHwReq->numReadBytes;
			masterData1.pAddress = pHwReq->pReadData;
			break;

		case 0x0f:
			/* Write + Read */
			/* read + subaddressing */
			masterData1.attach = tmhwI2cRestart;
			masterData1.direction = tmhwI2cTransmit;
			masterData1.pAddress = pHwReq->pWriteData;
			masterData1.length = pHwReq->numWriteBytes;

			masterData2.attach = tmhwI2cStop;
			masterData2.direction = tmhwI2cReceive;
			masterData2.length = pHwReq->numReadBytes;
			masterData2.pAddress = pHwReq->pReadData;
			break;

		default :
			return (TMHW_ERR_I2C_INVALID_REQUEST);
	}

	uiTimeout = gI2cCfg[unitID].timeout;
	status = tmbslCoreGetTicksPerSec(&ticksPerSec);
	ticksPer1ms = (ticksPerSec/1000);
	ticksPerTimeout = (uiTimeout * ticksPer1ms); /* timeout is in milisecond */

	/* start master */
	tmhwI2cStartMaster(unitID, 0, (tmhwI2cDirection_t)0);

	/* poll until done when timeout == 0 , otherwise wait for timeout */
	timediff = 0;
	status = tmbslCoreGetTickCount(&entryTicks);
	do
	{
		status = tmhwI2cEvent (unitID, tmhwI2cEventInterrupt, False);
		if ((status == TMHW_ERR_I2C_EVENT_MST_REQ) && (numReq == 1))
		{ /* first part of subaddressing or simple transfer */
			tmhwI2cSetData(unitID, &masterData1);
			status = tmhwI2cEvent (unitID, tmhwI2cEventProceed, False);
			numReq = 2;
		}
		if ((status == TMHW_ERR_I2C_EVENT_MST_TRX_REQ) ||
				(status == TMHW_ERR_I2C_EVENT_MST_REC_REQ) ||
				((status == TMHW_ERR_I2C_EVENT_MST_REQ) && (numReq == 2)))
		{/* second part of subaddressing */
			tmhwI2cSetData(unitID, &masterData2);
			status = tmhwI2cEvent (unitID, tmhwI2cEventProceed, False);
		}
		if ((status == TMHW_ERR_I2C_EVENT_MST_TRX_DONE) ||
				(status == TMHW_ERR_I2C_EVENT_MST_REC_DONE))
		{
			done = True;
			status = TM_OK;
		}
		if(status == TMHW_ERR_I2C_EVENT_MST_LOST)
		{
			/* start master once again*/
			tmhwI2cStartMaster(unitID, 0, (tmhwI2cDirection_t)0);
		}
		if ((status == TMHW_ERR_I2C_EVENT_MST_TRX_ABORT) ||
				(status == TMHW_ERR_I2C_EVENT_MST_REC_ABORT) ||
				(status == TMHW_ERR_I2C_EVENT_MST_ABORT)     ||
				(status == TMHW_ERR_I2C_EVENT_BUS_ERROR) )
		{
			done = True;
		}
		if(uiTimeout != 0)
		{
			(Void) tmbslCoreGetTickCount(&currTicks);
			timediff = (currTicks - entryTicks);
		}
	}while( (done != True ) && (timediff <= ticksPerTimeout));

	(Void) tmhwI2cIntEnable(unitID);
	tmhwI2cIP3203SetDMA(unitID);
	if(done != True)
	{
		return TMHW_ERR_I2C_TIMEOUT;
	}

	return status;
} /* tmhwI2cDoRequest */
#endif /* LINUX_BUILD */


#ifdef LINUX_BUILD
#ifdef CONFIG_I2C_DEBUG

void I2c_Error_Trigger( void )
{
	printk( KERN_EMERG "I2c error \n" );
	DumpI2c();
	printk( KERN_EMERG"I2c error - done\n");
}

static void DumpI2c( void )
{
	int i, j;
	unsigned int data, address;

	j = G_logI2cIndex - 1;
	for( i = 0; i < LOG_SIZE; i++ )
	{
		if( j < 0 )
			j = LOG_SIZE - 1;

		data    = G_logI2c[j--];
		address = G_logI2c[j--];
		if( address & 0x80000000 )
		{
			address &= 0x7fffffff;
			printk(KERN_EMERG "I2c WRITE 0x%08x @ 0x%08x (%s)\n", data, address, RegToName(address));
		}
		else
		{
			printk(KERN_EMERG "I2c read  0x%08x @ 0x%08x (%s)\n", data, address, RegToName(address));
		}
	}
}

	static const char*
RegToName( unsigned long offset)
{
	struct name_number {
		const char* pRegName;
		unsigned long regOffset;
	} NameNumberTable[] =
	{
		{ "I2CCON     ",    0x0000},
		{ "I2CSTAT    ",    0x0004},
		{ "I2CDAT     ",    0x0008},
		{ "I2CSLA     ",    0x000C},
		{ "HSBIR      ",    0x0010},
		{ "FSBIR      ",    0x0014},
		{ "INTROG     ",    0x0018},
		{ "DMA_ADDR   ",    0x0020},
		{ "DMA_LEN    ",    0x0024},
		{ "DMA_COUNTER",    0x0028},
		{ "DMA_CONTROL",    0x002C},
		{ "DMA_STATUS ",    0x0030},
		{ "INT_STATUS ",    0x0FE0},
		{ "INT_ENABLE ",    0x0FE4},
		{ "INT_CLEAR  ",    0x0FE8},
		{ "INT_SET    ",    0x0FEC},
		{ "POWERDOWN  ",    0x0FF4},
		{ "MODULE_ID  ",    0x0FFC }
	};
	int i;

	offset &= 0xfff;  // only look at offset part

	for( i = 0;
			i < (sizeof(NameNumberTable)/sizeof(NameNumberTable[0]) );
			i++)
	{
		if( NameNumberTable[i].regOffset == offset )
		{
			return NameNumberTable[i].pRegName;
		}
	}
	return "unknown register";
}

#endif /* CONFIG_I2C_DEBUG*/
#endif /* LINUX_BUILD*/
