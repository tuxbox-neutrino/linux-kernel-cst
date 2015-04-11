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

#ifndef TMHWEFMC_H
#define TMHWEFMC_H

/*-----------------------------------------------------------------------------
 * Project include files:
 *----------------------------------------------------------------------------*/
#include "tmNxTypes.h"
#include "tmNxCompId.h"
#include "tmhwEfmc_Cfg.h"

/*-----------------------------------------------------------------------------
* Component specific include files:
*-----------------------------------------------------------------------------*/

#if defined(__cplusplus)
extern "C"
{
#endif


/*-----------------------------------------------------------------------------
* Typedefs and Macros
*-----------------------------------------------------------------------------*/

/* Typedefs */

/*! \addtogroup Basic
* \{
*/

typedef UInt8 tmhwEfmc_DeviceNumber_t; /*!< This typedef is for denoting the device number that is connected to the memory controller */
typedef UInt8 *ptmhwEfmc_DeviceNumber_t; /*!< This typedef is for denoting pointer to the device number that is connected to the memory controller */

/*! \} */ /* addtogroup Basic */

#if (TMFL_EFMCSD_INT || TMFL_SD_ALL)

/*! \addtogroup InterruptSupport
*  \{
*/

typedef UInt32 tmhwEfmc_IntMask_t; /*!< This typedef is for denoting the interrupt mask */
typedef UInt32 *ptmhwEfmc_IntMask_t ; /*!< This typedef is for denoting pointer to the interrupt mask */

/*! \} */ /* addtogroup InterruptSupport */

#endif


/* Software Version information */
#define TMHW_EFMC_COMPATIBILITY_NR     (1) /*!< HwAPI compatibility number */
#define TMHW_EFMC_MAJOR_VERSION_NR     (1) /*!< HwAPI major version number */
#define TMHW_EFMC_MINOR_VERSION_NR     (0) /*!< HwAPI minor version number */

#if (TMFL_EFMCSD_INT || TMFL_SD_ALL)

/*! \addtogroup InterruptSupport
*  \{
*/

/* interrupt masks */
#define TMHW_EFMC_INT_OOB_READ              ((tmhwEfmc_IntMask_t)(0x00000001)) /*!< OOB block read request */
#define TMHW_EFMC_INT_OOB_WRITE             ((tmhwEfmc_IntMask_t)(0x00000002)) /*!< OOB block write request */
#define TMHW_EFMC_INT_512B_READ             ((tmhwEfmc_IntMask_t)(0x00000004)) /*!< 512byte block read request */
#define TMHW_EFMC_INT_512B_WRITE            ((tmhwEfmc_IntMask_t)(0x00000008)) /*!< 512byte block write request */
#define TMHW_EFMC_INT_512B_ENCODED          ((tmhwEfmc_IntMask_t)(0x00000010)) /*!< 512byte block encoded */
#define TMHW_EFMC_INT_512B_DECODED          ((tmhwEfmc_IntMask_t)(0x00000020)) /*!< 512byte block decoded */
#define TMHW_EFMC_INT_512B_DECODED_ERR_0    ((tmhwEfmc_IntMask_t)(0x00000040)) /*!< 512byte block decoded with 0 symbol errors */
#define TMHW_EFMC_INT_512B_DECODED_ERR_1    ((tmhwEfmc_IntMask_t)(0x00000080)) /*!< 512byte block decoded with 1 symbol errors */
#define TMHW_EFMC_INT_512B_DECODED_ERR_2    ((tmhwEfmc_IntMask_t)(0x00000100)) /*!< 512byte block decoded with 2 symbol errors */
#define TMHW_EFMC_INT_512B_DECODED_ERR_3    ((tmhwEfmc_IntMask_t)(0x00000200)) /*!< 512byte block decoded with 3 symbol errors */
#define TMHW_EFMC_INT_512B_DECODED_ERR_4    ((tmhwEfmc_IntMask_t)(0x00000400)) /*!< 512byte block decoded with 4 symbol errors */
#define TMHW_EFMC_INT_512B_DECODED_ERR_5    ((tmhwEfmc_IntMask_t)(0x00000800)) /*!< 512byte block decoded with 5 symbol errors */
#define TMHW_EFMC_INT_512B_UNCORR           ((tmhwEfmc_IntMask_t)(0x00001000)) /*!< 512byte block uncorrectable */
#define TMHW_EFMC_INT_512B_ENCRYPT          ((tmhwEfmc_IntMask_t)(0x00002000)) /*!< Reserved (when AES encryption is disabled), 512byte block decrypted (when AES encryption is enabled) */
#define TMHW_EFMC_INT_SEQ_PAGE_READ         ((tmhwEfmc_IntMask_t)(0x00004000)) /*!< Sequential page read done */
#define TMHW_EFMC_INT_SEQ_PAGE_WRITE        ((tmhwEfmc_IntMask_t)(0x00008000)) /*!< Sequential page write done */
#define TMHW_EFMC_INT_NAND_1_BUSY           ((tmhwEfmc_IntMask_t)(0x00010000)) /*!< Nand flash #1 busy */
#define TMHW_EFMC_INT_NAND_2_BUSY           ((tmhwEfmc_IntMask_t)(0x00020000)) /*!< Nand flash #2 busy */
#define TMHW_EFMC_INT_NAND_3_BUSY           ((tmhwEfmc_IntMask_t)(0x00040000)) /*!< Nand flash #3 busy */
#define TMHW_EFMC_INT_NAND_4_BUSY           ((tmhwEfmc_IntMask_t)(0x00080000)) /*!< Nand flash #4 busy */
#define TMHW_EFMC_INT_NAND_1_READY          ((tmhwEfmc_IntMask_t)(0x00100000)) /*!< Nand flash #1 ready */
#define TMHW_EFMC_INT_NAND_2_READY          ((tmhwEfmc_IntMask_t)(0x00200000)) /*!< Nand flash #2 ready */
#define TMHW_EFMC_INT_NAND_3_READY          ((tmhwEfmc_IntMask_t)(0x00400000)) /*!< Nand flash #3 ready */
#define TMHW_EFMC_INT_NAND_4_READY          ((tmhwEfmc_IntMask_t)(0x00800000)) /*!< Nand flash #4 ready */

/*! \} */ /* addtogroup InterruptSupport */

#endif


/*-----------------------------------------------------------------------------
* Error codes:
*-----------------------------------------------------------------------------*/
#define TMHW_ERR_EFMC_BASE              ((UInt32) CID_EFMC | (UInt32)CID_LAYER_HWAPI)


/*-----------------------------------------------------------------------------
* Global data
*-----------------------------------------------------------------------------*/


/*! \addtogroup Basic
* \{
*/

/*
 * enum definitions
 */


typedef enum tmhwEfmc_EnableDisable
/*! This enum is used to Enable or Disable a feature in EFMC controller */
{
  tmhwEfmc_Disable,       /*!< Disables the corresponding feature */
  tmhwEfmc_Enable         /*!< Enables the corresponding feature */
} tmhwEfmc_EnableDisable_t, *ptmhwEfmc_EnableDisable_t;

typedef enum tmhwEfmc_ChipEn
/*! This enum is used to Enable the corresponding flash chips connected to the EFMC controller */
{
  tmhwEfmc_ChipEn0 = 0x00000000,       /*!< Enables the flash chip 0 connected to the EFMC controller */
  tmhwEfmc_ChipEn1 = 0x00080000,       /*!< Enables the flash chip 1 connected to the EFMC controller */
  tmhwEfmc_ChipEn2 = 0x00100000,       /*!< Enables the flash chip 2 connected to the EFMC controller */
  tmhwEfmc_ChipEn3 = 0x00180000        /*!< Enables the flash chip 3 connected to the EFMC controller */
} tmhwEfmc_ChipEn_t, *ptmhwEfmc_ChipEn_t;

typedef enum tmhwEfmc_CycleType
/*! This enum is used to send the appropriate command/address cycle to the flash chip connected to the EFMC controller */
{
  tmhwEfmc_AddrCycle = 0x00000000,     /*!< Address cycle for the flash chip connected to the EFMC controller */
  tmhwEfmc_CmdCycle = 0x00010000,      /*!< command cycle for the flash chip connected to the EFMC controller */
  tmhwEfmc_PostWrCmdCycle = 0x00020000 /*!< post-write command cycle; a post-write command will not be issued until the last write cycle of a sequential page write has been executed. IP_2017 will always consider a post-write command cycle as the last cycle of a command / address sequence */
} tmhwEfmc_CycleType_t, *ptmhwEfmc_CycleType_t;

typedef enum tmhwEfmc_RWSel
/*! This enum is used for sending the page_read or page_write command sequence to the flash chip connected to the EFMC controller */
{
  tmhwEfmc_Nothing = 0x00,   /*!< This will notify the nand flash controller that a HW controlled page read is to be executed after the next command/address sequence. */
  tmhwEfmc_PageRead = 0x01,   /*!< This will notify the nand flash controller that a HW controlled page read is to be executed after the next command/address sequence. */
  tmhwEfmc_PageWrite = 0x02   /*!< This will notify the nand flash controller that a HW controlled page write is to be executed after the next command/address sequence. */
} tmhwEfmc_RWSel_t, *ptmhwEfmc_RWSel_t;

typedef enum tmhwEfmc_DataWidth
/*! This enum indicates data width of the correspondig nand flash device connected to EFMC */
{
  tmhwEfmc_8Bit,          /*!< 8 bit width */
  tmhwEfmc_16Bit,         /*!< 16 bit width */
  tmhwEfmc_Pseudo_16Bit   /*!< Pseudo 16 bit width. Two 8-bit devices connected in parallel */
} tmhwEfmc_DataWidth_t, *ptmhwEfmc_DataWidth_t;

typedef enum tmhwEfmc_PageSize
/*! This enum specifies the page size of the corresponding NAND flash device connected to the EFMC controller */
{
#if (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
tmhwEfmc_512Byte = 0x0,  /*!< Page size = 512 bytes */
tmhwEfmc_1024Byte = 0x1,  /*!< Page size = 1024 bytes */
tmhwEfmc_2048Byte = 0x2, /*!< Page size = 2048 bytes */
tmhwEfmc_4096Byte = 0x3, /*!< Page size = 4096 bytes */
tmhwEfmc_8192Byte = 0x4  /*!< Page size = 8192 bytes */
#else
tmhwEfmc_512Byte = 0x0,  /*!< Page size = 512 bytes */
tmhwEfmc_2048Byte = 0x1, /*!< Page size = 2048 bytes */
tmhwEfmc_4096Byte = 0x2, /*!< Page size = 4096 bytes */
#endif
} tmhwEfmc_PageSize_t, *ptmhwEfmc_PageSize_t;

typedef enum tmhwEfmc_SubPageSize
/*! This enum specifies the sub page size of the corresponding NAND flash device connected to the EFMC controller */
{
  tmhwEfmc_SubPage512Byte = 0x0,  /*!< sub page size = 512 bytes */
  tmhwEfmc_SubPage1024Byte = 0x1, /*!< sub page size = 1024 bytes */
} tmhwEfmc_SubPageSize_t, *ptmhwEfmc_SubPageSize_t;


typedef enum tmhwEfmc_RdyDelay
/*! This enum defines the data read delay timing in AXI/AHB clock cycles */
{
  tmhwEfmc_Cycles_0 =   0x00,   /*!< 0 AXI/AHB clock cycles */
  tmhwEfmc_Cycles_0_H = 0x01, /*!< Half AXI/AHB clock cycles */
  tmhwEfmc_Cycles_1 =   0x02,   /*!< 1 AXI/AHB clock cycles */
  tmhwEfmc_Cycles_1_H = 0x03, /*!< 1 and Half AXI/AHB clock cycles */
  tmhwEfmc_Cycles_2 =   0x04,   /*!< 2 AXI/AHB clock cycles */
  tmhwEfmc_Cycles_2_H = 0x05, /*!< 2 and Half AXI/AHB clock cycles */
  tmhwEfmc_Cycles_3 =   0x06,   /*!< 3 AXI/AHB clock cycles */
  tmhwEfmc_Cycles_3_H = 0x07, /*!< 3 and Half AXI/AHB clock cycles */
  tmhwEfmc_Cycles_4 =   0x08,   /*!< 4 AXI/AHB clock cycles */
  tmhwEfmc_Cycles_4_H = 0x09, /*!< 4 and Half AXI/AHB clock cycles */
  tmhwEfmc_Cycles_5 =   0x0A,  /*!< 5 AXI/AHB clock cycles */
  tmhwEfmc_Cycles_5_H = 0x0B,/*!< 5 and Half AXI/AHB clock cycles */
  tmhwEfmc_Cycles_6 =   0x0C,  /*!< 6 AXI/AHB clock cycles */
  tmhwEfmc_Cycles_6_H = 0x0D,/*!< 6 and Half AXI/AHB clock cycles */
  tmhwEfmc_Cycles_7 =   0x0E,  /*!< 7 AXI/AHB clock cycles */
  tmhwEfmc_Cycles_7_H = 0x0F /*!< 7 and Half AXI/AHB clock cycles */
} tmhwEfmc_RdyDelay_t, *ptmhwEfmc_RdyDelay_t;

typedef struct tmhwEfmc_ReadBusySignal
/*! this structure contains status of R/Bn signal for the NAND flash devices connected to the EFMC controller */
{
  Bool   rbEdge_Rise;            /*!< This represent occurance of the rising edge (if TM_TRUE) on the R/Bn signal on the given NAND flash device */
  Bool   rbEdge_Fall;            /*!< This represent occurance of the falling edge (if TM_TRUE) on the R/Bn signal on the given NAND flash device */
  Bool   rbEdge_Status_Ready;    /*!< This represent R/Bn signal on the given NAND flash device. TM_TRUE --> device is ready, TM_FALSE --> device is busy. */
} tmhwEfmc_ReadBusySignal_t, *ptmhwEfmc_ReadBusySignal_t;


/*
 * structures definitions
 */
typedef struct tmhwEfmc_CmdAddr
/*! This structure contains the command/address and the corresponding data to be put onto the IO lines for the flash devices connected to the EFMC controller */
{
  /* -- Kronos specific definitions starts -- */
  Bool                 immediate;      /* 0-Wait until RB# is 1 (default), 1-Don't care */
  Bool                 includeOOB;     /* 0-OOB block not included (default), 1-OOB block included */
  Bool                 includeECC;     /* 0-HW ECC disabled (default), 1-enabled */
  Bool                 page_transfer;  /* 0-Page xfer (default), 1-Not a Page transfer */
  Bool                 direction;      /* 0-Read (default), 1-Write */
  /* -- Kronos specific definitions ends -- */

  tmhwEfmc_ChipEn_t    deviceNum;  /*!< The value of CEn lines during command/address/read-write cycles */
  Bool                 lastCycle;  /*!< The specified command/address is the last on the sequence. When TRUE, nand flash controller will start sending all the commands and/or address present in the FIFO to the nand flash device */
  tmhwEfmc_CycleType_t cycleType;  /*!< Address/command/PostWrite command cycle type */
  UInt16               data;       /*!< data on the IO lines during address/command cycle */
}tmhwEfmc_CmdAddr_t, *ptmhwEfmc_CmdAddr_t;

typedef struct tmhwEfmc_SetModeCtrl
{
  Bool                 rb_n_bypass;
  Bool                 rd_stall;
} tmhwEfmc_ModeCtrl_t, *ptmhwEfmc_ModeCtrl_t;

typedef struct tmhwEfmc_PageConfig
/*! this structure contains the page related read/write configuration for the flash device */
{
  Bool                 includeOOB; /*!< Specifies whether OOB block to be included in the page read/write transfer. TM_TRUE --> OOB block included, FALSE --> OOB block is not included */
  Bool                 includeAES; /*!< Specifies whether AES IP needs to be included. TM_TRUE --> AES IP is included, TM_FALSE --> AES IP is not included */
  Bool                 includeECC; /*!< Specifies whether hardware ECC to be enabled/disabled. TM_TRUE --> hardware ECC is enabled, TM_FALSE --> hardware ECC is disabled. */
  tmhwEfmc_RWSel_t operType;   /*!< Operation type. Hardware page read or hardware page write operation */
} tmhwEfmc_PageConfig_t, *ptmhwEfmc_PageConfig_t;

typedef struct tmhwEfmc_TimingConfig
/*! This structure contains the timing configuration for the corresponding NAND flash device connected to the EFMC controller */
{
  UInt8                   tCenSetup;    /*!< CEn pre-setup time. Range: 1-16 AXI/AHB cycles. Number of AXI/AHB clock cycles between
                                           - falling edge of CEn and rising edge of ALE at the beginning of an address latch
                                           - falling edge of CEn and rising edge of CLE at the beginning of a command latch
                                           - falling edge of CEn and falling edge of WEn/REn at the beginning of a write/read */
  UInt8                   tCenHold;     /*!< CEn post-hold time. Range: 1-16 AXI/AHB cycles. Number of AXI/AHB clock cycles between
                                           - falling edge of ALE and rising edge of CEn at the end of an address latch
                                           - falling edge of CLE and rising edge of CEn at the end of a command latch
                                           - rising edge of WEn/REn and rising edge of CEn at the end of a write/read */
  UInt8                   tCleSetup;    /*!< defines the CLE setup time, i.e. the number of AXI/AHB clock cycles between rising/falling  edge of CLE and falling edge of WEn. Range: 1-16 AXI/AHB clock cycles */
  UInt8                   tCleHold;     /*!< defines the CLE hold time, i.e. the number of AXI/AHB clock cycles between rising edge of WEn and falling edge of CLE.  Range: 1-16 AXI/AHB clock cycles */
  UInt8                   tAleSetup;    /*!< defines the ALE setup time, i.e. the number of AXI/AHB clock cycles between rising/falling edge of ALE and falling edge of WEn.  Range: 1-16 AXI/AHB clock cycles */
  UInt8                   tAleHold;     /*!< defines the ALE hold time, i.e. the number of AXI/AHB clock cycles between rising edge of WEn and falling edge of ALE.  Range: 1-16 AXI/AHB clock cycles  */
  UInt8                   tWaitForRdy;  /*!< defines the wait until ready time, i.e. the number of AXI/AHB clock cycles between rising edge of R/Bn and falling edge of REn. Range: 4-16 AXI/AHB clock cycles. */
  //tmhwEfmc_RdyDelay_t tRdDelay;     /*!< defines the data read delay, i.e. the number of AXI/AHB clock cycles divided by two between rising edge of REn and clocking of the read data. This delay can be used to compensate for IO pad delay. */
  UInt8                   tRdDelay;  /*!< defines the data read delay, i.e. the number of AXI/AHB clock cycles divided by two between rising edge of REn and clocking of the read data. This delay can be used to compensate for IO pad delay. */
  UInt8                   tWaitForBusy; /*!< defines the wait until busy time, i.e. the number of AXI/AHB clock cycles between rising edge of WEn and falling edge of R/Bn. Range: 1-64 AXI/AHB cycles */
  UInt8                   tWenWidth;    /*!< defines the WEn pulse width, i.e. the number of AXI/AHB clock cycles [3] that WEn is asserted low. Range: 1-16 AXI/AHB clock cycles */
  UInt8                   tWenHigh;     /*!< defines the WEn high hold time, i.e. the number of AXI/AHB clock cycles [3] between rising and falling edge of WEn. Range: 1-16 AXI/AHB clock cycles */
  UInt8                   tRenWidth;    /*!< defines the REn pulse width, i.e. the number of clock AXI/AHB clock cycles [3] that REn is asserted low. Range: 1-16 AXI/AHB clock cycles */
  UInt8                   tRenHigh;     /*!< defines the REn high hold time, i.e. the number of AXI/AHB clock cycles [3] between rising and falling edge of REn. Range: 1-16 AXI/AHB clock cycles */
} tmhwEfmc_TimingConfig_t, *ptmhwEfmc_TimingConfig_t;

typedef struct tmhwEfmc_FlashConfig
/*! This structure specifies the flash configuration for the correspondign flash device connected to the EFMC controller */
{
  UInt8                    erasedPageThres;   /* threshold for the erased-page detector */
  UInt8                    eccLevel;          /* number of bits that can be corrected by the ECC engine */
  UInt16                   oobSize;           /* number of bytes available in the OOB area of one page */
  tmhwEfmc_SubPageSize_t   subpageSize;       /* 0:512 bytes,1:1024 bytes */
  tmhwEfmc_PageSize_t      pageSize;          /*!< Page size of the NAND flash device. */
  tmhwEfmc_EnableDisable_t enableWrProtect;   /*!< Data write protection enabled/disabled */
  tmhwEfmc_EnableDisable_t enableCENDontCare; /*!< defines whether the nand flash device supports "CEn don.t care". The "CEn don.t care" feature allows another memory controller like e.g. the IP_2016 to interrupt an ongoing sequential read/write. */
  tmhwEfmc_DataWidth_t     dataWidth;         /*!< data width of the NAND flash device */
  tmhwEfmc_TimingConfig_t  devTiming;         /*!< timing parameters for the NAND flash device */
} tmhwEfmc_FlashConfig_t, *ptmhwEfmc_FlashConfig_t;

typedef struct tmhwEfmc_OobInfo
/*! This structure specifies the flash oob partition info for the correspondign flash device connected to the EFMC controller */
{
   UInt16   extended_oob;
   UInt8    unprotected_bytes;
   UInt8    parity_bytes;
   UInt8    protected_oob_min;
} tmhwEfmc_OobInfo_t, *ptmhwEfmc_OobInfo_t;

typedef struct tmhwEfmc_PageRdStatus
/*! This structure specifies the page read status for the correspondign flash device connected to the EFMC controller */
{
   UInt16   tag;
   Bool     erased;
   Bool     ecc_off;
   Bool     uncorrectable;
   Bool     corrected;
   Bool     error_free;
   UInt8    num_errors;
} tmhwEfmc_PageRdStatus_t, *ptmhwEfmc_PageRdStatus_t;

typedef struct tmhwEfmc_PageWrStatus
/*! This structure specifies the page read status for the correspondign flash device connected to the EFMC controller */
{
   UInt16   tag;
} tmhwEfmc_PageWrStatus_t, *ptmhwEfmc_PageWrStatus_t;

#define MAX_PAGE_WR_STATUS_FIFO_DEPTH 8
#define MAX_PAGE_RD_STATUS_FIFO_DEPTH 16

typedef struct tmhwEfmc_PageRWStatusFifo
/*! This structure specifies the page read status fifo for the correspondign flash device connected to the EFMC controller */
{
   UInt8                   PageRdStatusLevel;
   UInt8                   PageWrStatusLevel;
   tmhwEfmc_PageRdStatus_t PageRdStatus[MAX_PAGE_RD_STATUS_FIFO_DEPTH];
   tmhwEfmc_PageWrStatus_t PageWrStatus[MAX_PAGE_WR_STATUS_FIFO_DEPTH];
} tmhwEfmc_PageRWStatusFifo_t, *ptmhwEfmc_PageRWStatusFifo_t;

typedef enum tmhwEfmc_StatusFifoMode
{
   tmhwEfmc_StatusFifoMode_Normal      = 0x0,
   tmhwEfmc_StatusFifoMode_Protected   = 0x1,
} tmhwEfmc_StatusFifoMode_t, *ptmhwEfmc_StatusFifoMode_t;

typedef struct tmhwEfmc_PageRWStatusFifoMode
/*! This structure specifies the page read status for the correspondign flash device connected to the EFMC controller */
{
   tmhwEfmc_StatusFifoMode_t  status_fifo_mode;
   Bool                       status_fifo_pop;
} tmhwEfmc_PageRWStatusFifoMode_t, *ptmhwEfmc_PageRWStatusFifoMode_t;

/*! \} */ /* addtogroup Basic */


#if (TMFL_EFMCSD_EBI || TMFL_SD_ALL)

/*! \addtogroup EBIPinSharingMode
* \{
*/

/*
 * enum definitions
 */

typedef enum tmhwEfmc_BackoffMode
/*! This enum defines the backoff mode that specifies when to backoff when the backoff-timeout occurs for the EFMC */
{
  tmhwEfmc_Always = 0x00, /*!< always complete current AHB burst and then backoff */
  tmhwEfmc_Current = 0x02 /*!< complete only the current memory transaction and then backoff */
} tmhwEfmc_BackoffMode_t, *ptmhwEfmc_BackoffMode_t;

/*
 * structure definitions
 */

typedef struct tmhwEfmc_BackoffConfig
/*! This structure defines the Back-Off configuration for EBI pin-sharing for the EFMC */
{
  tmhwEfmc_EnableDisable_t    defaultReqEnable; /*!< EBI default request mode: When enabled, NAND flash controller will do a default request for the memory bus whenever it is idle. In overall system there shall only be one memory controller with default request enabled. */
  UInt8                           timeOut; /*!< Backoff timout value after which the EBI backoff singal will be taken into account. Range 0 - 0xFF where in 0x00--> timeout immediately, otherwise timeout after n AHB clock cycles  */
  tmhwEfmc_BackoffMode_t      mode; /*!< When backoff timeout is reached, this defines when to backoff */
} tmhwEfmc_BackoffConfig_t, *ptmhwEfmc_BackoffConfig_t;


/*! \} */ /* addtogroup EBIPinSharingMode */

#endif

#if (TMFL_EFMCSD_DMA || TMFL_SD_ALL)
/*! \addtogroup DmaSupport
 *  \{
 */

/*
 * structure definitions
 */


typedef struct tmhwEfmc_DmaConfig
/*! This structure is used to configure the flow control for the DMA operation for EFMC */
{
  tmhwEfmc_EnableDisable_t   enableM2PDma;  /*!< Enables/disables flow control for memory-to-peripheral DMA (used when writing to Nand flash)  */
  tmhwEfmc_EnableDisable_t   enableP2MDma;  /*!< Enables/disables flow control for peripheral-to-memory DMA (used when reading from Nand flash) */
} tmhwEfmc_DmaConfig_t, *ptmhwEfmc_DmaConfig_t;

/*! \} */ /* end of DmaSupport */

#endif


#if (TMFL_EFMCSD_AES || TMFL_SD_ALL)
/*! \addtogroup AESSupport
 *  \{
 */

/*
 * enum definitions
 */

typedef enum tmhwEfmc_AESStatus
/*! This enum specifies the status of the AES IP */
{
  tmhwEfmc_AESBusy = 0x00,     /*!< AES IP is busy condition */
  tmhwEfmc_AESSetupKey = 0x02, /*!< AES IP is in key-setup condition, where in new data (to AES IP) will not be accepted until a key has been expanded */
  tmhwEfmc_AESIdle = 0x03      /*!< AES IP is idle condition and can accept new key for expansion or for more data processing */
} tmhwEfmc_AESStatus_t, *ptmhwEfmc_AESStatus_t;

/*
 * structure definitions
 */

typedef struct tmhwEfmc_AESConfig
/*! This structure stores the AES configuration that need to be sent to AES IP */
{
  UInt32   keyAES[4];  /*!< 128-bit AES key value to be transferred to AES IP */
  UInt32   initAES[4]; /*!< 128-bit AES initial value to be transferred to AES IP */
} tmhwEfmc_AESConfig_t, *ptmhwEfmc_AESConfig_t;


/*! \} */ /* addtogroup AESSupport */

#endif

#if (TMFL_EFMCSD_APB || TMFL_SD_ALL)

/*! \addtogroup APBConfig
*  \{
*/

/*
 * enum definitions
 */

typedef enum tmhwEfmc_Apb_Latency
/*! This enum defines the APB latency for read/write on APB */
{
  tmhwEfmc_Apb_Latency_1_Cycle = 0x00, /*!< No additional wait state insertion. Minimum read/write latency is 1 APB clock cycle */
  tmhwEfmc_Apb_Latency_2_Cycle = 0x01  /*!< Insert one additional wait state. Minimum read/write latency is 2 APB clock cycles */
} tmhwEfmc_Apb_Latency_t, *ptmhwEfmc_Apb_Latency_t;

/*! \} */ /* end of APBConfig */
#endif


#if (TMFL_EFMCSD_AHB || TMFL_SD_ALL)
/*! \addtogroup AHBConfig
 *  \{
 */


typedef enum tmhwEfmc_SelRegistering
/*! This enum defines the registering of combinational data and the read-data registering */
{
  tmhwEfmc_Combinational = 0x00, /*!< AHB/APB command is combinational */
  tmhwEfmc_Registered = 0x01 /*!< AHB/APB command or read data is registered */
} tmhwEfmc_SelRegistering_t, *ptmhwEfmc_SelRegistering_t;


/*! \} */ /* addtogroup AHBConfig */

#endif


#if (TMFL_EFMCSD_OTHERS || TMFL_SD_ALL)

/*! \addtogroup Others
 *  \{
 */

/*
 * enum definitions
 */

typedef enum tmhwEfmc_SelConfig
/*! This enum specifies the mode of relevant APB feature whether it is a configurable or programmable */
{
  tmhwEfmc_Configurable = 0x00, /*!< APB configuration is configurable i.e Read-only */
  tmhwEfmc_Programmable = 0x01 /*!< APB configuration is programmable i.e Read/write */
} tmhwEfmc_SelConfig_t, *ptmhwEfmc_SelConfig_t;

typedef enum tmhwEfmc_DataBusType
/*! This enum specifies the data bus type supported by EFMC */
{
  tmhwEfmc_DataBus_32BitAHB = 0x0000 /*!< 32-bit AHB data bus type */
} tmhwEfmc_DataBusType_t, *ptmhwEfmc_DataBusType_t;

/*
 * structure definitions
 */

typedef struct tmhwEfmc_Capabilities
/*! This structure provides the capabilities of the EFMC */
{
  tmhwEfmc_SelConfig_t        progApbLatency;  /*!< APB latency configuration */
  tmhwEfmc_SelConfig_t        progApbErrConfig;          /*!< APB error configuration */
  tmhwEfmc_DataBusType_t      busType;/*!< data bus type supported by EFMC */
  tmhwEfmc_SelConfig_t        dataBusLatency;  /*!< Data bus latency configuration */
  Bool                            supportAES;      /*!< Support for AES included or not. TM_TRUE if included, TM_FALSE if not included */
  Bool                            enEBIPinShare;     /*!< Support for EBI pin-sharing included or not. TM_TRUE if included, TM_FALSE if not included */
  Bool                            bootFromNAND;    /*!< Support boot from NAND flash device. TM_TRUE if supported, TM_FALSE if not supported */
  Bool                            localRstSync;       /*!< Reset local synchronizers. TM_TRUE if yes, TM_FALSE if no */
  UInt8                           maxDevices;      /*!< Maximum number of flash devices supported. Range 1-4 */
  UInt16                          efmcModuleId;    /*!< EFMC Module ID */
  UInt8                           efmcRevMajor;    /*!< EFMC Major revision number */
  UInt8                           efmcRevMinor;    /*!< EFMC Minor revision number */
  UInt8                           efmcAperSize;    /*!< EFMC MMIO register aperture */
} tmhwEfmc_Capabilities_t, *ptmhwEfmc_Capabilities_t;

/*! \} */ /* addtogroup Others */

#endif



/* Function definitions */

/*! \addtogroup Basic
* \{
*/

/*! \fn tmErrorCode_t tmhwEfmc_Init (tmUnitSelect_t  efmcUnitID);
*    This function initializes the EFM Device and powers ON the EFM device. This API should be called before any access to the EFM device.
*    \param[in] efmcUnitID -> EFMC Unit Number
*    \return TM_OK - Successful.
*/
tmErrorCode_t
tmhwEfmc_Init(
    tmUnitSelect_t                 efmcUnitID
);


/*!\fn tmErrorCode_t tmhwEfmc_WriteCmdAddr( tmUnitSelect_t efmcUnitID, const tmhwEfmc_CmdAddr_t * pCmdAddr);
*     This function issues the appropriate cmd/addr cycle to the appropriate NAND flash device connected to the EFMC.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[in] pCmdAddr -> pointer to the tmhwEfmc_CmdAddr_t structure that contains the flash command and address to be given for the relevant NAND flash device.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_WriteCmdAddr(
    tmUnitSelect_t                  efmcUnitID,
    const tmhwEfmc_CmdAddr_t *         pCmdAddr
);

/*!\fn tmErrorCode_t tmhwEfmc_InitPageOp( tmUnitSelect_t efmcUnitID, const tmhwEfmc_PageConfig_t * pPageOper);
*     This function configures the page read/write operation for the relevant NAND flash device.
*     Note: This API should be called before relevant command/address/post-commands sequence put into the FIFO. Please refer to the use
*     cases of page read/write.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[in] pPageOper -> pointer to the tmhwEfmc_PageConfig_t structure that contains the page configuration for the relevant NAND flash device.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_InitPageOp(
    tmUnitSelect_t                  efmcUnitID,
    const tmhwEfmc_PageConfig_t *   pPageOper
);

tmErrorCode_t
tmhwEfmc_SetModeCtrl(
    tmUnitSelect_t               efmcUnitID,
    const tmhwEfmc_ModeCtrl_t    *pModeCtrl
);


/*!\fn tmErrorCode_t tmhwEfmc_SetFlashConfig( tmUnitSelect_t efmcUnitID, tmhwEfmc_DeviceNumber_t deviceNum, const tmhwEfmc_FlashConfig_t * pFlashConfig);
*     This function configures the given NAND flash device for timing parameters, data width and page size and also for write-protect of NAND flash device.
*     Note: This API should be called before relevant command/address/post-commands sequence put into the FIFO. Please refer to the initialization sequence of the EFMC.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[in] deviceNum -> Nand flash device number for which the configuration to be set
*     \param[in] pFlashConfig -> pointer to the tmhwEfmc_FlashConfig_t structure that contains the flash configuration for the relevant NAND flash device.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_SetFlashConfig(
    tmUnitSelect_t                  efmcUnitID,
    tmhwEfmc_DeviceNumber_t                           deviceNum,
    const tmhwEfmc_FlashConfig_t *     pFlashConfig
);

tmErrorCode_t
tmhwEfmc_GetOobInfo (
    tmUnitSelect_t               efmcUnitID,
    tmhwEfmc_DeviceNumber_t      deviceNum,
    tmhwEfmc_OobInfo_t           *pOobInfo);

tmErrorCode_t
tmhwEfmc_GetPageRWStatusFifo (
    tmUnitSelect_t               efmcUnitID,
    tmhwEfmc_PageRWStatusFifo_t *pPageRWStatusFifo);

tmErrorCode_t
tmhwEfmc_SetPageRWStatusFifoMode (
    tmUnitSelect_t                  efmcUnitID,
    tmhwEfmc_PageRWStatusFifoMode_t *pPageRWStatusFifoCfg);

/*!\fn tmErrorCode_t tmhwEfmc_ReadSingleData( tmUnitSelect_t efmcUnitID, pUInt16 pDataRead);
*     This function reads a single 16-bit data from the relevant NAND flash device.
*     Note: This function should be called after proper command/address cycle is issued to the NAND flash device by calling tmhwEfmc_WriteCmdAddr function.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[out] pDataRead -> pointer to the UInt16, where the data read from the NAND flash device needs to be stored.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_ReadSingleData(
    tmUnitSelect_t                  efmcUnitID,
    pUInt16                         pDataRead
);


/*!\fn tmErrorCode_t tmhwEfmc_WriteSingleData( tmUnitSelect_t efmcUnitID, UInt16 dataWrite);
*     This function writes a single 16-bit data to the relevant NAND flash device.
*     Note: This function should be called after proper command/address cycle is issued to the NAND flash device by calling tmhwEfmc_WriteCmdAddr function.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[in] dataWrite -> data that needs to be written to the relevant NAND flash device.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_WriteSingleData(
    tmUnitSelect_t                  efmcUnitID,
    UInt16                          dataWrite
);


/*!\fn tmErrorCode_t tmhwEfmc_ReadBufData( tmUnitSelect_t efmcUnitID, tmhwEfmc_DeviceNumber_t deviceNum, pVoid pDataRead);
*     This function reads 512 bytes of data from the EFMC buffer. If the NAND flash has a page size more than 512 bytes, then
*     this API should be called number of times to get the whole page size and also memory location address should be provided
*     appropriately.
*     Note: 1. This function should be called after proper command/address cycle is issued to the NAND flash device by calling tmhwEfmc_WriteCmdAddr function.
*     2. The pointer provided should have the memory space of 512 bytes.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[in] deviceNum -> NAND Flash device number from which the data to be read
*     \param[out] pDataRead -> pointer to the memory location, where the data read from the NAND flash device needs to be stored.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_ReadBufData(
    tmUnitSelect_t                  efmcUnitID,
    tmhwEfmc_DeviceNumber_t                           deviceNum,
    pVoid                           pDataRead
);


/*!\fn tmErrorCode_t tmhwEfmc_ReadOobData( tmUnitSelect_t efmcUnitID, tmhwEfmc_DeviceNumber_t deviceNum, pVoid pDataRead);
*     This function reads OOB data of the corresponding page of NAND nand flash device.
*     Note: 1. This function should be called after proper command/address cycle is issued to the NAND flash device by calling tmhwEfmc_WriteCmdAddr function.
*     2. This API must be called only after the whole PAGE data is read from the buffer by calling tmhwEfmc_ReadBufData function.
*     3. The pointer provided should have the memory space for the OOB data whose size depends upon the PAGE size of the given NAND flash device.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[in] deviceNum -> NAND Flash device number from which the data to be read
*     \param[out] pDataRead -> pointer to the memory location, where the data read from the NAND flash device needs to be stored.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_ReadOobData(
    tmUnitSelect_t                  efmcUnitID,
    tmhwEfmc_DeviceNumber_t                           deviceNum,
    pVoid                           pDataRead
);


/*!\fn tmErrorCode_t tmhwEfmc_WriteBufData( tmUnitSelect_t efmcUnitID, tmhwEfmc_DeviceNumber_t deviceNum, pVoid pDataWrite);
*     This function writes 512 bytes of data from the EFMC buffer. If the NAND flash has a page size more than 512 bytes, then
*     this API should be called number of times to write the whole page size and also memory location address should be provided
*     appropriately.
*     Note: 1. This function should be called after proper command/address cycle is issued to the NAND flash device by calling tmhwEfmc_WriteCmdAddr function.
*     2. The pointer provided should have the memory space of 512 bytes.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[in] deviceNum -> NAND Flash device number to which the data needs to be written.
*     \param[out] pDataWrite -> pointer to the memory location, from where the data is read and is written onto the NAND flash.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_WriteBufData(
    tmUnitSelect_t                  efmcUnitID,
    tmhwEfmc_DeviceNumber_t                           deviceNum,
    pVoid                           pDataWrite
);


/*!\fn tmErrorCode_t tmhwEfmc_WriteOobData( tmUnitSelect_t efmcUnitID, tmhwEfmc_DeviceNumber_t deviceNum, pVoid pDataWrite);
*     This function writes OOB data to the NAND flash page.
*     Note: 1. This function should be called after proper command/address cycle is issued to the NAND flash device by calling tmhwEfmc_WriteCmdAddr function.
*     2. This API must be called only after the whole page data is written onto the NAND flash device by tmhwEfmc_WriteBufData function.
*     3. The pointer provided should have the memory space for the OOB data whose size depends upon the PAGE size of the given NAND flash device.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[in] deviceNum -> NAND Flash device number to which the OOB data needs to be written.
*     \param[out] pDataWrite -> pointer to the memory location, from where the OOB data is read and is written onto the NAND flash.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_WriteOobData(
    tmUnitSelect_t                  efmcUnitID,
    tmhwEfmc_DeviceNumber_t                           deviceNum,
    pVoid                           pDataWrite
);

/*!\fn tmErrorCode_t tmhwEfmc_GetBusyStatus( tmUnitSelect_t efmcUnitID, tmhwEfmc_DeviceNumber_t deviceNum, ptmhwEfmc_ReadBusySignal_t pBusyStatus);
*     This function gets the status of the given flash device connected to the EFMC.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[in] deviceNum -> Nand flash device number
*     \param[out] pBusyStatus -> pointer to the tmhwEfmc_ReadBusySignal_t structure that contains the status of NAND flash device ready/busy signal.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_GetBusyStatus(
    tmUnitSelect_t                  efmcUnitID,
    tmhwEfmc_DeviceNumber_t                           deviceNum,
    ptmhwEfmc_ReadBusySignal_t  pBusyStatus
);


/*! \} */ /* addtogroup Basic */



#if (TMFL_EFMCSD_DMA || TMFL_SD_ALL)
/*! \addtogroup DmaSupport
 *  \{
 */


/*!\fn tmErrorCode_t tmhwEfmc_SetDmaConfig( tmUnitSelect_t efmcUnitID, const tmhwEfmc_DmaConfig_t * pDmaConfig);
*     This function will setup the DMA configuration for the data transfer to/from NAND flash devices connected the EFMC.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[in] pDmaConfig -> pointer to the tmhwEfmc_DmaConfig_t structure that contains the DMA configuration for the data transfer to/from the NAND flash devices.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_SetDmaConfig(
    tmUnitSelect_t                  efmcUnitID,
    const tmhwEfmc_DmaConfig_t *       pDmaConfig
);


/*!\fn tmErrorCode_t tmhwEfmc_GetDmaConfig( tmUnitSelect_t efmcUnitID, ptmhwEfmc_DmaConfig_t pDmaConfig);
*     This function will retrieve the DMA configuration that is used for the data transfer to/from NAND flash devices connected the EFMC.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[out] pDmaConfig -> pointer to the tmhwEfmc_DmaConfig_t structure where in the read DMA configuration to be stored.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_GetDmaConfig(
    tmUnitSelect_t                  efmcUnitID,
    ptmhwEfmc_DmaConfig_t       pDmaConfig
);


/*! \} */ /* end of DmaSupport */

#endif



#if (TMFL_EFMCSD_AES || TMFL_SD_ALL)
/*! \addtogroup AESSupport
 *  \{
 */


/*!\fn tmErrorCode_t tmhwEfmc_SetAESConfig( tmUnitSelect_t efmcUnitID, const tmhwEfmc_AESConfig_t * pAESConfig);
*     This function will setup the AES configuration required for AES IP.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[in] pAESConfig -> pointer to the tmhwEfmc_AESConfig_t structure that contains the AES configuration data to be sent to the AES IP.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_SetAESConfig(
    tmUnitSelect_t                  efmcUnitID,
    const tmhwEfmc_AESConfig_t *       pAESConfig
);


/*!\fn tmErrorCode_t tmhwEfmc_GetAESConfig( tmUnitSelect_t efmcUnitID, ptmhwEfmc_AESConfig_t pAESConfig);
*     This function will retrieve the AES configuration set in the AES IP.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[out] pAESConfig -> pointer to the tmhwEfmc_AESConfig_t structure where in the configuration data read from AES IP needs to be stored.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_GetAESConfig(
    tmUnitSelect_t                  efmcUnitID,
    ptmhwEfmc_AESConfig_t       pAESConfig
);


/*!\fn tmErrorCode_t tmhwEfmc_GetAESStatus( tmUnitSelect_t efmcUnitID, ptmhwEfmc_AESStatus_t pAESStatus);
*     This function will retrieve the status of AES IP whether it is busy or idle.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[out] pAESStatus -> pointer to the tmhwEfmc_AESStatus_t enum where in the status of AES IP needs to be stored.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_GetAESStatus(
    tmUnitSelect_t                  efmcUnitID,
    ptmhwEfmc_AESStatus_t       pAESStatus
);


/*! \} */ /* end of AESSupport */

#endif




#if (TMFL_EFMCSD_POWER || TMFL_SD_ALL)

/*! \addtogroup PowerState
*  \{
*/


/*!\fn tmErrorCode_t tmhwEfmc_SetPowerState (tmUnitSelect_t  efmcUnitID, tmPowerState_t  powerState);
*     Function will set the EFMC Device to a specified power state. Only D0 (Device Powered on) and D3 (Device Powered off) state are valid.
*
*    \param[in] efmcUnitID -> EFMC Unit number
*    \param[in] powerState -> Power state to which EFMC device needs to be set. Only D0 (Device Powered On) and D3 (Device Powered Off) states are valid.
*    \return TM_OK - Successfull.
*/
tmErrorCode_t
tmhwEfmc_SetPowerState (
    tmUnitSelect_t                 efmcUnitID,
    tmPowerState_t                 powerState
);


/*!\fn tmErrorCode_t tmhwEfmc_GetPowerState (tmUnitSelect_t  efmcUnitID, ptmPowerState_t  pPowerState);
*     This function gets the present Power state of the Device.
*
*    \param[in] efmcUnitID -> EFMC Unit Number
*    \param[out] pPowerState -> Pointer to tmPowerState_t typedefinition.
*    \return TM_OK - Successfull.
*/
tmErrorCode_t
tmhwEfmc_GetPowerState (
    tmUnitSelect_t                  efmcUnitID,
    ptmPowerState_t                 pPowerState
);

/*! \} */ /* end of PowerState */

#endif



#if (TMFL_EFMCSD_INT || TMFL_SD_ALL)

/*! \addtogroup InterruptSupport
*  \{
*/


/*!\fn tmErrorCode_t tmhwEfmc_IntEnable(tmUnitSelect_t  efmcUnitID, tmhwEfmc_IntMask_t efmIntEnable);
*    This function will enable the relevant interrupts defined by the given interrupt mask value.
*
*    \param[in] efmcUnitID -> EFMC Unit number
*    \param[in] efmIntEnable -> Interrupt enable mask bits for which relevant interrupts to be enabled.
*    \return TM_OK - Successfull.
*/
tmErrorCode_t
tmhwEfmc_IntEnable(
    tmUnitSelect_t                 efmcUnitID,
    tmhwEfmc_IntMask_t         efmIntEnable
);


/*!\fn tmErrorCode_t tmhwEfmc_IntDisable(tmUnitSelect_t  efmcUnitID, tmhwEfmc_IntMask_t efmIntDisable);
*    This function will disable the relevant interrupts defined by the given interrupt mask value.
*
*    \param[in] efmcUnitID -> EFMC Unit number
*    \param[in] efmIntDisable -> Interrupt disable mask bits for which relevant interrupts to be disabled.
*    \return TM_OK - Successfull.
*/
tmErrorCode_t
tmhwEfmc_IntDisable(
    tmUnitSelect_t                 efmcUnitID,
    tmhwEfmc_IntMask_t         efmIntDisable
);


/*!\fn tmErrorCode_t tmhwEfmc_IntGetStatus(tmUnitSelect_t  efmcUnitID, ptmhwEfmc_IntMask_t pEfmIntGetStatus);
*    This function will get the current interrupts status.
*
*    \param[in] efmcUnitID -> EFMC Unit number
*    \param[in] pEfmIntGetStatus -> Pointer to tmhwEfmc_IntMask_t where in read interrupt status needs to be stored.
*    \return TM_OK - Successfull.
*/
tmErrorCode_t
tmhwEfmc_IntGetStatus(
    tmUnitSelect_t                 efmcUnitID,
    ptmhwEfmc_IntMask_t        pEfmIntGetStatus
);


/*!\fn tmErrorCode_t tmhwEfmc_IntClear(tmUnitSelect_t  efmcUnitID, tmhwEfmc_IntMask_t efmIntClear);
*    This function will clear the relevent interrupts status. The given maks will provide which interrupt status to be cleared.
*
*    \param[in] efmcUnitID -> EFMC Unit number
*    \param[in] efmIntClear -> Interrupt mask for which the relevent interrupts status to be cleared.
*    \return TM_OK - Successfull.
*/
tmErrorCode_t
tmhwEfmc_IntClear(
    tmUnitSelect_t                 efmcUnitID,
    tmhwEfmc_IntMask_t         efmIntClear
);


/*! \} */ /* end of InterruptSupport */

#endif




#if (TMFL_EFMCSD_EBI || TMFL_SD_ALL)
/*! \addtogroup EBIPinSharingMode
 *  \{
 */


/*!\fn tmErrorCode_t tmhwEfmc_SetBackoffConfig ( tmUnitSelect_t efmcUnitID, const tmhwEfmc_BackoffConfig_t * pBackOffConfig );
*     This function configures the EBI pin sharing (backoff mode) for the EFMC. The details include hte timeout for
*     backoff, mode of backoff. This function acts on the whole EFMC.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[in] pBackOffConfig -> pointer to the tmhwEfmc_BackoffConfig_t structure that contians the backoff config parameters.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_SetBackoffConfig (
    tmUnitSelect_t                  efmcUnitID,
    const tmhwEfmc_BackoffConfig_t *   pBackOffConfig
);


/*!\fn tmErrorCode_t tmhwEfmc_GetBackoffConfig ( tmUnitSelect_t efmcUnitID, ptmhwEfmc_BackoffConfig_t pBackOffConfig );
*     This function retrieves the confiuration of the EBI pin sharing (backoff mode) for the EFMC. The details include hte
*     timeout for backoff, mode of backoff. This function acts on the whole EFMC.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[in] pBackOffConfig -> pointer to the tmhwEfmc_BackoffConfig_t structure where retrieved information to be stored.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_GetBackoffConfig (
    tmUnitSelect_t                  efmcUnitID,
    ptmhwEfmc_BackoffConfig_t   pBackOffConfig
);


/*! \} */ /* end of EBIPinSharingMode */

#endif




#if (TMFL_EFMCSD_AHB || TMFL_SD_ALL)
/*! \addtogroup AHBConfig
 *  \{
 */


/*!\fn tmErrorCode_t tmhwEfmc_SetAhbConfig( tmUnitSelect_t efmcUnitID,tmhwEfmc_SelRegistering_t   commandRegistering, tmhwEfmc_SelRegistering_t   rdDataRegistering);
*     This function sets the AHB configuration for the command and read-data registering.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[in] commandRegistering -> AXI/AHB command registering
*     \param[in] rdDataRegistering -> AXI/AHB read-data registering
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_SetAhbConfig(
    tmUnitSelect_t                  efmcUnitID,
    tmhwEfmc_SelRegistering_t   commandRegistering,
    tmhwEfmc_SelRegistering_t   rdDataRegistering
);


/*!\fn tmErrorCode_t tmhwEfmc_GetAhbConfig( tmUnitSelect_t efmcUnitID, ptmhwEfmc_SelRegistering_t   pCommandRegistering, ptmhwEfmc_SelRegistering_t   pRdDataRegistering);
*     This function gets the AHB configuration for the command and read-data registering.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[out] pCommandRegistering -> AXI/AHB command registering configuration to be stored
*     \param[out] pRdDataRegistering -> AXI/AHB read-data registering configuration to be stored
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_GetAhbConfig(
    tmUnitSelect_t                  efmcUnitID,
    ptmhwEfmc_SelRegistering_t  pCommandRegistering,
    ptmhwEfmc_SelRegistering_t  pRdDataRegistering
);


/*! \} */ /* addtogroup AHBConfig */

#endif




#if (TMFL_EFMCSD_APB || TMFL_SD_ALL)

/*! \addtogroup APBConfig
*  \{
*/


/*!\fn tmErrorCode_t tmhwEfmc_SetApbErrConfig ( tmUnitSelect_t efmcUnitID, tmhwEfmc_EnableDisable_t enableWrErr, tmhwEfmc_EnableDisable_t enableRdErr);
*     This function will configure the APB error generation. Please note that this function should be called only
*     when the IP is capable of programming the APB error generation. Call tmhwEfmc_GetCapabilities() function
*     to get the capability of APB error generation in pCaps->progApbErrConfig. The application should call this function only
*     if pCaps->progApbErrConfig == tmhwEfmc_Programmable.
*
*    \param[in] efmcUnitID -> EFMC Unit number
*    \param[in] enableWrErr -> Enabling of APB write error generation
*    \param[in] enableRdErr -> Enabling of APB read error generation
*    \return TM_OK - Successfull.
*/
tmErrorCode_t
tmhwEfmc_SetApbErrConfig (
    tmUnitSelect_t                 efmcUnitID,
    tmhwEfmc_EnableDisable_t   enableWrErr,
    tmhwEfmc_EnableDisable_t   enableRdErr
);


/*!\fn tmErrorCode_t tmhwEfmc_GetApbErrConfig ( tmUnitSelect_t efmcUnitID, ptmhwEfmc_EnableDisable_t pEnableWrErr, ptmhwEfmc_EnableDisable_t pEnableRdErr);
*     This function will get the APB error generation status
*
*    \param[in] efmcUnitID -> EFMC Unit number
*    \param[in] pEnableWrErr -> APB write error generation status
*    \param[in] pEnableRdErr -> APB read error generation status
*    \return TM_OK - Successfull.
*/
tmErrorCode_t
tmhwEfmc_GetApbErrConfig (
    tmUnitSelect_t                 efmcUnitID,
    ptmhwEfmc_EnableDisable_t  pEnableWrErr,
    ptmhwEfmc_EnableDisable_t  pEnableRdErr
);


/*!\fn tmErrorCode_t tmhwEfmc_SetApbTimeoutConfig ( tmUnitSelect_t efmcUnitID, UInt16 timeout);
*     This function will configure the APB operation timeout.
*
*    \param[in] efmcUnitID -> EFMC Unit number
*    \param[in] timeout -> APB operation timeout value. Range : 0x00 - 0xFFFF. If 0x00 represent, timeout is disabled, else timeout value is n APB cycles.
*    \return TM_OK - Successfull.
*/
tmErrorCode_t
tmhwEfmc_SetApbTimeoutConfig (
    tmUnitSelect_t                 efmcUnitID,
    UInt16                         timeout
);


/*!\fn tmErrorCode_t tmhwEfmc_GetApbTimeoutConfig ( tmUnitSelect_t efmcUnitID, pUInt16 pTimeout);
*     This function will get the APB operation timeout value.
*
*    \param[in] efmcUnitID -> EFMC Unit number
*    \param[in] pTimeout -> APB operation timeout value.
*    \return TM_OK - Successfull.
*/
tmErrorCode_t
tmhwEfmc_GetApbTimeoutConfig (
    tmUnitSelect_t                 efmcUnitID,
    pUInt16                        pTimeout
);


/*!\fn tmErrorCode_t tmhwEfmc_SetApbLatency ( tmUnitSelect_t efmcUnitID, tmhwEfmc_Apb_Latency_t insertWait);
*     This function will configure the APB read/write latency. Please note that this function should be called only
*     when the IP is capable of programming the APB error generation. Call tmhwEfmc_GetCapabilities() function
*     to get the capability of APB error generation in pCaps->progApbLatency. The application should call this function only
*     if pCaps->progApbLatency == tmhwEfmc_Programmable.
*
*    \param[in] efmcUnitID -> EFMC Unit number
*    \param[in] insertWait -> APB read/write latency value
*    \return TM_OK - Successfull.
*/
tmErrorCode_t
tmhwEfmc_SetApbLatency (
    tmUnitSelect_t                 efmcUnitID,
    tmhwEfmc_Apb_Latency_t     insertWait
);


/*!\fn tmErrorCode_t tmhwEfmc_GetApbLatency ( tmUnitSelect_t efmcUnitID, ptmhwEfmc_Apb_Latency_t pInsertWait);
*     This function will get the APB read/write latency value.
*
*    \param[in] efmcUnitID -> EFMC Unit number
*    \param[in] pInsertWait -> APB read/write latency value that to be stored in the location pointed by this variable.
*    \return TM_OK - Successfull.
*/
tmErrorCode_t
tmhwEfmc_GetApbLatency (
    tmUnitSelect_t                 efmcUnitID,
    ptmhwEfmc_Apb_Latency_t    pInsertWait
);


/*! \} */ /* end of APBConfig */

#endif


#if (TMFL_EFMCSD_OTHERS || TMFL_SD_ALL)

/*! \addtogroup Others
*  \{
*/


/*!\fn tmErrorCode_t tmhwEfmc_GetSWVersion (ptmSWVersion_t  pEfmVersionInfo);
*     This function gets the Software version of the HwAPI.Function can be called used by the
*     application to know whether correct version of the software is used.
*
*    \param[out] pEfmVersionInfo -> pointer to tmSWVersion_t structure.
*    \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_GetSWVersion (
    ptmSWVersion_t                 pEfmVersionInfo
);


/*!\fn tmErrorCode_t tmhwEfmc_Deinit(tmUnitSelect_t  efmcUnitID);
*     This function will deinitialize the EFMC device. This function serves no purpose.
*
*    \param[in] efmcUnitID -> EFMC unit number.
*    \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_Deinit(
    tmUnitSelect_t                 efmcUnitID
);

/*!\fn tmErrorCode_t tmhwEfmc_GetCapabilities (tmUnitSelect_t  efmcUnitID, ptmhwEfmc_Capabilities_t  pCaps);
*     This function gets capabilities of the EFMC Hardware device and the structure elements specifies whether the certain
*     features of EFMC are configured in the EFMC IP or not.
*
*    \param[in] efmcUnitID -> EFMC unit Number.
*    \param[out] pCaps -> Pointer to tmhwEfmc_Capabilities_t structure.
*    \return TM_OK - Successful.
*/
tmErrorCode_t
tmhwEfmc_GetCapabilities (
    tmUnitSelect_t                 efmcUnitID,
    ptmhwEfmc_Capabilities_t   pCaps
);


/*!\fn tmErrorCode_t tmhwEfmc_GetFlashConfig( tmUnitSelect_t efmcUnitID, tmhwEfmc_DeviceNumber_t deviceNum, ptmhwEfmc_FlashConfig_t pFlashConfig);
*     This function retrieves the given NAND flash device configuration. The configuration such as timing parameters, data width and page size and also for write-protect of NAND flash device is stored in the parameter.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[in] deviceNum -> Nand flash device number for which the configuration to be retrieved.
*     \param[out] pFlashConfig -> pointer to the tmhwEfmc_FlashConfig_t structure where in the read configuration needs to be stored.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_GetFlashConfig(
    tmUnitSelect_t                  efmcUnitID,
    tmhwEfmc_DeviceNumber_t                           deviceNum,
    ptmhwEfmc_FlashConfig_t     pFlashConfig
);

/*!\fn tmErrorCode_t tmhwEfmc_SoftReset ( tmUnitSelect_t efmcUnitID);
*     This function provides a soft reset to the EFMC unit.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_SoftReset (
    tmUnitSelect_t                  efmcUnitID
);

/*! \} */ /* end of Others */

#endif

#if defined(__cplusplus)
}
#endif

#endif /* TMHWEFMC_H */

