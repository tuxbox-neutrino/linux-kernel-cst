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


#ifndef TMHW_I2C_H
#define TMHW_I2C_H

/*
===============================================================================
Standard include files:
===============================================================================
*/
#ifdef LINUX_BUILD

#include <HwAPI/tmNxTypes.h>
#include <HwAPI/tmNxCompId.h>

#else

#include <tmNxTypes.h>
#include <tmNxCompId.h>

#endif
/*
===============================================================================
Project include files:
===============================================================================
*/


#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

/*
   ===============================================================================
   Types and defines:
   ===============================================================================
 */
/*  compatibility and version numbers */
#define TMHW_I2C_COMPATIBILITY_NR       1       /* Module software version */
#define TMHW_I2C_MAJOR_VERSION_NR       1
#define TMHW_I2C_MINOR_VERSION_NR       0
#define TMHW_I2C_INTERFACE_VERSION      1

	/* I2c HWAPI Status/Error Codes */
#define TMHW_ERR_I2C_BASE                           (CID_COMP_IIC | CID_LAYER_HWAPI)
#define TMHW_ERR_I2C_COMP                           (TMHW_ERR_I2C_BASE | TM_ERR_COMP_UNIQUE_START)

	/* I2c HWAPI Standard Status Codes */
#define TMHW_ERR_I2C_BUSY                                                     \
	/* !ErrorTag: busy in tmhwI2c                                              */ \
	(TMHW_ERR_I2C_COMP+0x0000)
#define TMHW_ERR_I2C_INVALID_REQUEST                                          \
	/* !ErrorTag: invalid request error in tmhwI2c                             */ \
	(TMHW_ERR_I2C_COMP+0x0001)
#define TMHW_ERR_I2C_BAD_UNIT_ID                                              \
	/* !ErrorTag: bad unit id in tmhwI2c                                       */ \
	(TMHW_ERR_I2C_COMP+0x0002)
#define TMHW_ERR_I2C_EVENT_MST_REQ                                            \
	/* !ErrorTag: mst req event in tmhwI2c                                     */ \
	(TMHW_ERR_I2C_COMP+0x0003)
#define TMHW_ERR_I2C_EVENT_MST_LOST                                           \
	/* !ErrorTag: mst lost event in tmhwI2c                                    */ \
	(TMHW_ERR_I2C_COMP+0x0004)
#define TMHW_ERR_I2C_EVENT_MST_ABORT                                          \
	/* !ErrorTag: mst abort event in tmhwI2c                                   */ \
	(TMHW_ERR_I2C_COMP+0x0005)
#define TMHW_ERR_I2C_EVENT_MST_TRX_REQ                                        \
	/* !ErrorTag: mst trx req event in tmhwI2c                                 */ \
	(TMHW_ERR_I2C_COMP+0x0006)
#define TMHW_ERR_I2C_EVENT_MST_TRX_DONE                                       \
	/* !ErrorTag: mst trx done event in tmhwI2c                                */ \
	(TMHW_ERR_I2C_COMP+0x0007)
#define TMHW_ERR_I2C_EVENT_MST_TRX_ABORT                                      \
	/* !ErrorTag: mst trx abort event in tmhwI2c                               */ \
	(TMHW_ERR_I2C_COMP+0x0008)
#define TMHW_ERR_I2C_EVENT_MST_REC_REQ                                        \
	/* !ErrorTag: mst rec req event in tmhwI2c                                 */ \
	(TMHW_ERR_I2C_COMP+0x0009)
#define TMHW_ERR_I2C_EVENT_MST_REC_DONE                                       \
	/* !ErrorTag: mst rec done event in tmhwI2c                                */ \
	(TMHW_ERR_I2C_COMP+0x000a)
#define TMHW_ERR_I2C_EVENT_MST_REC_ABORT                                      \
	/* !ErrorTag: mst rec abort event in tmhwI2c                               */ \
	(TMHW_ERR_I2C_COMP+0x000b)
#define TMHW_ERR_I2C_EVENT_SLV_REC_REQ_MST_LOST                               \
	/* !ErrorTag: slv rec req mst lost event in tmhwI2c                        */ \
	(TMHW_ERR_I2C_COMP+0x000c)
#define TMHW_ERR_I2C_EVENT_SLV_REC_REQ                                        \
	/* !ErrorTag: slv rec req event in tmhwI2c                                 */ \
	(TMHW_ERR_I2C_COMP+0x000d)
#define TMHW_ERR_I2C_EVENT_SLV_GC_REQ_MST_LOST                                \
	/* !ErrorTag: slv gc req mst lost event in tmhwI2c                         */ \
	(TMHW_ERR_I2C_COMP+0x000e)
#define TMHW_ERR_I2C_EVENT_SLV_GC_REQ                                         \
	/* !ErrorTag: slv gc req event in tmhwI2c                                  */ \
	(TMHW_ERR_I2C_COMP+0x000f)
#define TMHW_ERR_I2C_EVENT_SLV_REC_DONE                                       \
	/* !ErrorTag: slv rec done event in tmhwI2c                                */ \
	(TMHW_ERR_I2C_COMP+0x0010)
#define TMHW_ERR_I2C_EVENT_SLV_REC_ABORT                                      \
	/* !ErrorTag: slv rec abort event in tmhwI2c                               */ \
	(TMHW_ERR_I2C_COMP+0x0011)
#define TMHW_ERR_I2C_EVENT_SLV_TRX_REQ                                        \
	/* !ErrorTag: slv trx req event in tmhwI2c                                 */ \
	(TMHW_ERR_I2C_COMP+0x0012)
#define TMHW_ERR_I2C_EVENT_SLV_TRX_REQ_MST_LOST                               \
	/* !ErrorTag: slv trx req mst lost event in tmhwI2c                        */ \
	(TMHW_ERR_I2C_COMP+0x0013)
#define TMHW_ERR_I2C_EVENT_SLV_TRX_DONE                                       \
	/* !ErrorTag: slv trx done event in tmhwI2c                                */ \
	(TMHW_ERR_I2C_COMP+0x0014)
#define TMHW_ERR_I2C_EVENT_SLV_TRX_ABORT                                      \
	/* !ErrorTag: slv trx abort event in tmhwI2c                               */ \
	(TMHW_ERR_I2C_COMP+0x0015)
#define TMHW_ERR_I2C_EVENT_BUS_ERROR                                          \
	/* !ErrorTag: bus error event in tmhwI2c                                   */ \
	(TMHW_ERR_I2C_COMP+0x0016)
#define TMHW_ERR_I2C_TIMEOUT                                                 \
	/* !ErrorTag: no respons on I2C-bus                                   */     \
	(TMHW_ERR_I2C_COMP+0x0017)

#define TMHW_ERR_I2C_EVENT_A0                                                 \
	/* !ErrorTag: bus error event in tmhwI2c                                   */ \
	(TMHW_ERR_I2C_COMP+0x0018)

	/* device */
	/* the maximum numbers of I2c hardware units that can be configured */
#define TMHW_I2C_UNIT_MAX               (5)

	/* i2c */
#define TMHW_I2C_MAX_SS_SPEED           100     /* max speed in SS mode (kHz) */
#define TMHW_I2C_MAX_FS_SPEED           400     /* max speed in FS mode (kHz) */
#define TMHW_I2C_MAX_HS_SPEED           3400    /* max speed in HS mode (kHz) */

#define TMHW_I2C_DATA_DUMMY             0xff
#define TMHW_I2C_NO_MEANING             (-1)

typedef enum
{
	tmhwI2cChain                = 0,
	tmhwI2cRestart              = 1,            /* for master only */
	tmhwI2cStop                 = 2
} tmhwI2cAttach_t, *ptmhwI2cAttach_t;

typedef enum
{
	tmhwI2cTransmit             = 0,
	tmhwI2cReceive              = 1,
	tmhwI2cNoMeaning            = -1
} tmhwI2cDirection_t, *ptmhwI2cDirection_t; /* for master only */

typedef enum
{
	tmhwI2cEventInterrupt,
	tmhwI2cEventProceed
} tmhwI2cRequest_t, *ptmhwI2cRequest_t;

typedef enum
{
	tmhwI2cMaster = 0,
	tmhwI2cSlave,
	tmhwI2cBusError,
	tmhwI2cNoInterrupt,
	tmhwI2cBusBusy,
	tmhwI2cBusFree
}
tmhwI2cStatusType_t, *ptmhwI2cStatusType_t;

typedef struct _tmhwI2cCapabilities_t
{
	UInt32        i2cModuleID;              /* I2c hardware module ID */
	UInt32        i2cUnitCount;             /* Total I2c unit count supported */
	UInt32        i2cMaxSpeedKHZ;           /* I2c unit maximum transfer speed */
	UInt32        i2cMaxHsSpeedKHZ;         /* I2c unit maximum transfer speed */
} tmhwI2cCapabilities_t, *ptmhwI2cCapabilities_t;

typedef struct _tmhwI2cData_t
{
	pUInt8                  pAddress;       /* data base address */
	UInt32                  length;         /* length of message(w) or buffer(r) */
	UInt32                  counter;        /* num of bytes left to transceive + */
	tmhwI2cAttach_t         attach;         /* how to finish */
	UInt8                   slaveAddress;   /* I2c slave address of slave being
						   addressed (master only!!!) */
	tmhwI2cDirection_t      direction;      /* i2c communication direction
						   (master mode only!!!) */
	Bool                    bHS;            /* if True: data will be transferred
						   in HS, otherwise FS/SS
						   (master only!!!) */
} tmhwI2cData_t, *ptmhwI2cData_t;


typedef Void (*ptmhwI2cVirtToPhys_t) (pVoid pVirt, pVoid *ppPhys);
typedef tmErrorCode_t (*ptmhwI2cCacheFlush_t) (pVoid pVirt, UInt32 length);
typedef tmErrorCode_t (*ptmhwI2cCacheInvalidate_t) (pVoid pVirt, UInt32 length);

typedef struct tmhwI2cMmFunc_t
{
	ptmhwI2cVirtToPhys_t        pI2cVirtToPhys;
	ptmhwI2cCacheFlush_t        pI2cCacheFlush;
	ptmhwI2cCacheInvalidate_t   pI2cCacheInvalidate;
	pUInt8                          pI2cDynamicMemPtr;
#ifdef DMA_MOD
	UInt32                          phyAddr;
#endif
	UInt32                          i2cDynamicMemLength;
} tmhwI2cMmFunc_t, *ptmhwI2cMmFunc_t;

typedef struct tmhwI2cDoRequest
{
	UInt8         address;               /* iic 7-bit address of slave dev */
	UInt32        numWriteBytes;         /* numbers of bytes */
	UInt32        numReadBytes;          /* numbers of bytes */
	pUInt8        pWriteData;            /* pointer to Write data */
	pUInt8        pReadData;             /* pointer to Read data */
} tmhwI2cDoRequest_t, *ptmhwI2cDoRequest_t;


/*
   ===============================================================================
   External function/data references:
   ===============================================================================
 */

/*
   ===============================================================================
   Exported functions:
   ===============================================================================
 */

tmErrorCode_t
tmhwI2cRegisterMmFunc (
		tmUnitSelect_t          i2cUnit,
		ptmhwI2cMmFunc_t        pFuncStruct
		);

tmErrorCode_t
tmhwI2cGetSWVersion (
		ptmSWVersion_t          pI2cVersion
		);

tmErrorCode_t
tmhwI2cGetCapabilities (
		tmUnitSelect_t          i2cUnit,
		ptmhwI2cCapabilities_t  pI2cCaps
		);

tmErrorCode_t
tmhwI2cGetBlockId(
		tmUnitSelect_t          i2cUnit,
		pUInt32                 pBlockId
		);

tmErrorCode_t
tmhwI2cInit (
		tmUnitSelect_t          i2cUnit
	    );

tmErrorCode_t
tmhwI2cDeinit (
		tmUnitSelect_t          i2cUnit
	      );

tmErrorCode_t
tmhwI2cGetPowerState (
		tmUnitSelect_t          i2cUnit,
		ptmPowerState_t         pPowerState
			);

tmErrorCode_t
tmhwI2cSetPowerState (
		tmUnitSelect_t          i2cUnit,
		tmPowerState_t          powerState
		);

tmErrorCode_t
tmhwI2cSetIntEnable (
		tmUnitSelect_t          i2cUnit,
		bool                    enable
		);

Void
tmhwI2cEnableGeneralCall (
		tmUnitSelect_t          i2cUnit
		);

Void
tmhwI2cDisableGeneralCall (
		tmUnitSelect_t          i2cUnit
		);

Void
tmhwI2cSetSlaveAddr (
		tmUnitSelect_t          i2cUnit,
		UInt8                   moduleSlaveAddr
		);

Void
tmhwI2cSetSpeed (
		tmUnitSelect_t          i2cUnit,
			UInt32                  fsSpeed,        /* I: FSspeed, */
			UInt32                  hsSpeed         /* I: HSspeed, 0: if no HS */
			);

Void
tmhwI2cGetSpeed (
		tmUnitSelect_t          i2cUnit,
		pUInt32                 pFsSpeed,       /* I: pFSspeed, */
		pUInt32                 pHsSpeed        /* I: pHSspeed, 0: if no HS */
		);

Void
tmhwI2cConvertSpeed (
		tmUnitSelect_t          i2cUnit,
		pUInt32                 pFsSpeed,
		pUInt32                 pHsSpeed        /*0: if no HS */
		);

Void
tmhwI2cGetBusStatus (
		tmUnitSelect_t          i2cUnit,
		ptmhwI2cStatusType_t    pStatusType
		);


Void
tmhwI2cIntGetStatus (
		tmUnitSelect_t          i2cUnit,
		ptmhwI2cStatusType_t    pStatusType
		);

tmErrorCode_t
tmhwI2cIntEnable (
		tmUnitSelect_t          i2cUnit
		);

tmErrorCode_t
tmhwI2cIntDisable (
		tmUnitSelect_t          i2cUnit
		);

tmErrorCode_t
tmhwI2cIntClear (
		tmUnitSelect_t          i2cUnit
		);

Void
tmhwI2cSetData (
		tmUnitSelect_t          i2cUnit,
		ptmhwI2cData_t          pData
	       );

Void
tmhwI2cGetData (
		tmUnitSelect_t          i2cUnit,
		ptmhwI2cData_t          pdata
	       );

Void
tmhwI2cStartMaster (
		tmUnitSelect_t          i2cUnit,
		UInt8                   slaveAddress,   /* I2c slave address of slave being */
		tmhwI2cDirection_t      direction       /* i2c communication direction */
		);

Void
tmhwI2cStartSlave (
		tmUnitSelect_t          i2cUnit
		);

Void
tmhwI2cStopSlave (
		tmUnitSelect_t          i2cUnit
		);

tmErrorCode_t
tmhwI2cEventHandle0xa0(
		tmUnitSelect_t          i2cUnit,
			Bool                    bSlaveMonitor
			);

tmErrorCode_t
tmhwI2cEvent (
		tmUnitSelect_t          i2cUnit,
		tmhwI2cRequest_t        request,
		Bool                    bSlaveMonitor
		     );

tmErrorCode_t
tmhwI2cEventHandle0xa0(
		tmUnitSelect_t i2cUnit,
		Bool bSlaveMonitor
			);

Void
tmhwI2cHotBoot (
		tmUnitSelect_t              i2cUnit
	       );

#ifdef LINUX_BUILD

Void
tmhwI2cSoftReset (
		tmUnitSelect_t              i2cUnit
		);
#endif

/* fit to need I2C function intended for system start-up
 * it does not use any OS or memory management instructions
 * As a consequence, this function does not use DMA

 * The driver supports two hardware peripherals: IP_3203 and IP_0105
	 * Some boards contain both peripherals, some only one
	 * the unit numbers are assigned as follows:
	 * {tmUnit0 - tmUnit<A-1>=> IP_3203, tmUnit<A> - tmUnit<A+B>=> IP_0105}
	 * When no IP_3203 devices are available in the system, tmUnit0 corresponds with the first IP_0105 deice.
	 */
#ifndef DMA_MOD
tmErrorCode_t
tmhwI2cDoRequest(
		tmUnitSelect_t        unitID,           // I: iic unit
		ptmhwI2cDoRequest_t  pHwReq     // I: data for sequenceing state machine
		);
#endif


#ifdef LINUX_BUILD
void debugSMU(void);
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* TMHW_I2C_H */
