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

#include <linux/string.h>
#include <linux/io.h>
#include "tmNxTypes.h"
#include "tmNxCompId.h"
#include "tmhwEfmc_Vhip.h"
#include "tmhwEfmc_Cfg.h"
#include "tmhwEfmc.h"


#if (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
/* Kronos specific - Cmd, Addr, Page config structure */
static tmhwEfmc_CmdAddr_t Cmd_Addr_Fifo_Cfg;
#endif

#define TMVH_EFMC_HWMODULE_MAJOR_REV        0x2
#define TMVH_EFMC_HWMODULE_MINOR_REV        0x1

#define ATTRIBUTE_UNUSED __attribute__ ((__unused__))

/*! \fn tmErrorCode_t tmhwEfmc_Init (tmUnitSelect_t  efmcUnitID);
*    This function initializes the EFMC and powers ON the EFM device. This API should be called before any access to the EFM device.
*    \param[in] efmcUnitID -> EFMC Unit Number
*    \return TM_OK - Successful.
*/
tmErrorCode_t
tmhwEfmc_Init(
    tmUnitSelect_t                 efmcUnitID ATTRIBUTE_UNUSED
)
{
  /* nothing to be done here */
  return TM_OK;
}

/*!\fn tmErrorCode_t tmhwEfmc_WriteCmdAddr( tmUnitSelect_t efmcUnitID, const tmhwEfmc_CmdAddr_t * pCmdAddr);
*     This function issues the appropriate cmd/addr cycle to the appropriate NAND flash device connected to the EFMC.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[in] pCmdAddr -> pointer to the tmhwEfmc_CmdAddr_t structure that contains the flash command and address to be given for the relevant NAND flash device.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_WriteCmdAddr(
    tmUnitSelect_t                  efmcUnitID,
    const tmhwEfmc_CmdAddr_t *      pCmdAddr
)
{
  UInt32              regs    = 0 ;
  UInt32              regVal  = 0 ;
  
  regs = (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_CMD_ADDR_FIFO_OFFSET );

#if (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
  /* -- Kronos specific definitions starts -- */
  if (tmhwEfmc_CmdCycle == pCmdAddr->cycleType)
  {
    if ((/*NAND_CMD_RESET*/0xff==pCmdAddr->data) || 
        (/*NAND_CMD_STATUS*/0x70== pCmdAddr->data))
    {
      Cmd_Addr_Fifo_Cfg.immediate = TM_TRUE;
    }
  }
  if( TM_TRUE == Cmd_Addr_Fifo_Cfg.immediate )
  {
    regVal |= (UInt32) TMVH_EFMC_CMD_ADDR_FIFO_IMMEDIATE_MSK ;
  }
  if( TM_TRUE == Cmd_Addr_Fifo_Cfg.includeOOB )
  {
    /* OOB block included */
    regVal |= (UInt32) TMVH_EFMC_CMD_ADDR_FIFO_OOB_MSK ;
  }
  if( TM_TRUE == Cmd_Addr_Fifo_Cfg.includeECC )
  {
    /* ECC block included */
    regVal |= (UInt32) TMVH_EFMC_CMD_ADDR_FIFO_ECC_MSK ;
  }
  if( TM_TRUE == Cmd_Addr_Fifo_Cfg.page_transfer )
  {
     /* Transfer page */
    regVal |= (UInt32) TMVH_EFMC_CMD_ADDR_FIFO_PAGE_XFER_MSK;
  }
  if( TM_TRUE == Cmd_Addr_Fifo_Cfg.direction )
  {
    /* Write page */
    regVal |= (UInt32) TMVH_EFMC_CMD_ADDR_FIFO_DIR_MSK;
  }
  /* -- Kronos specific definitions ends -- */
#endif

  regVal |= (UInt32) (pCmdAddr->deviceNum) ;

  if( TM_TRUE == pCmdAddr->lastCycle )
  {
    regVal |= (UInt32) TMVH_EFMC_CMD_ADDR_FIFO_LAST_CYCLE_MSK ;
#if (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
    memset(&Cmd_Addr_Fifo_Cfg, 0, sizeof(Cmd_Addr_Fifo_Cfg));
#endif
  }
  
  regVal |= (UInt32) (pCmdAddr->cycleType) ;
  regVal |= (UInt32) ( pCmdAddr->data & TMVH_EFMC_CMD_ADDR_FIFO_IO_MSK) ;
  TMVH_GEN_WRITE( regs, regVal );
  return TM_OK;
}


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
    const tmhwEfmc_PageConfig_t *      pPageOper
)
{
#if defined (CONFIG_ARCH_APOLLO)
  UInt32              regs  = 0 ;
  UInt32              regVal    = 0 ;

  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_PAGE_RW_OFFSET );
  /* Just write the values directly into the register */
  if( TM_TRUE == pPageOper->includeOOB  )
  {
    regVal |= (UInt32) (TMVH_EFMC_PAGE_RW_OOB_MSK) ;
  }
  if( TM_TRUE == pPageOper->includeAES )
  {
    regVal |= (UInt32) (TMVH_EFMC_PAGE_RW_AES_MSK) ;
  }
  if( TM_TRUE == pPageOper->includeECC )
  {
    regVal |= (UInt32) (TMVH_EFMC_PAGE_RW_ECC_MSK) ;
  }
  regVal |= (UInt32) (pPageOper->operType) ;

  TMVH_GEN_WRITE( regs, regVal );
#else
  /* Write the values into the global cmd_addr_fifo structure */
  if( TM_TRUE == pPageOper->includeOOB  )
  {
    Cmd_Addr_Fifo_Cfg.includeOOB = TM_TRUE;
  }
  if( TM_TRUE == pPageOper->includeECC )
  {
    Cmd_Addr_Fifo_Cfg.includeECC = TM_TRUE;
  }
  if (tmhwEfmc_PageRead == pPageOper->operType)
  {
    Cmd_Addr_Fifo_Cfg.page_transfer = TM_TRUE;
    Cmd_Addr_Fifo_Cfg.direction     = TM_FALSE;
  }
  else if (tmhwEfmc_PageWrite == pPageOper->operType)
  {
     Cmd_Addr_Fifo_Cfg.page_transfer   = TM_TRUE;
     Cmd_Addr_Fifo_Cfg.direction       = TM_TRUE;
  }
  else
  {
     Cmd_Addr_Fifo_Cfg.page_transfer   = TM_FALSE;
     Cmd_Addr_Fifo_Cfg.direction       = TM_FALSE;
  }
#endif
  return TM_OK;
}

tmErrorCode_t
tmhwEfmc_SetModeCtrl(
   tmUnitSelect_t             efmcUnitID,
   const tmhwEfmc_ModeCtrl_t *pModeCtrl
   )
{
   UInt32   regs     = 0 ;
   UInt32   regVal   = 0 ;

   regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + 
      TMVH_EFMC_MODE_CTRL_OFFSET );

   /* Just write the values directly into the register */

   if( TM_TRUE == pModeCtrl->rb_n_bypass )
   {
      regVal |= (UInt32) (TMVH_EFMC_MODE_CTRL_RBN_BYPASS_MSK);
   }
   else
   {
      regVal &= (UInt32) ~(TMVH_EFMC_MODE_CTRL_RBN_BYPASS_MSK);
   }
   if( TM_TRUE == pModeCtrl->rd_stall )
   {
      regVal |= (UInt32) (TMVH_EFMC_MODE_CTRL_RD_STALL_MSK);
   }
   else
   {
      regVal &= (UInt32) ~(TMVH_EFMC_MODE_CTRL_RD_STALL_MSK);
   }
   TMVH_GEN_WRITE( regs, regVal );

   /* enable normal fifo operation i.e. reading from the FIFO increments 
    * the FIFO pointer as a side effect. */
   regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + 
      TMVH_EFMC_STATUS_FIFO_MODE_OFFSET );
   regVal = (UInt32) (tmhwEfmc_StatusFifoMode_Normal);
   TMVH_GEN_WRITE( regs, regVal );

   return TM_OK;
}

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
   tmUnitSelect_t                efmcUnitID,
   tmhwEfmc_DeviceNumber_t       deviceNum,
   const tmhwEfmc_FlashConfig_t *pFlashConfig
)
{
   UInt32   regs     = 0 ;
   UInt32   regVal   = 0 ;

   regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_DEV_TYPE0_OFFSET +
      ( deviceNum * TMVH_EFMC_DEV_TYPE0_DIFF ) );

   /* Just write the values directly into the register */
   regVal = (UInt32) (pFlashConfig->dataWidth) ;
   if( tmhwEfmc_Enable == pFlashConfig->enableCENDontCare )
   {
      regVal |= (UInt32) (TMVH_EFMC_DEV_TYPE0_CEN_DONT_MSK);
   }
   if( tmhwEfmc_Enable == pFlashConfig->enableWrProtect )
   {
      regVal |= (UInt32) (TMVH_EFMC_DEV_TYPE0_WP_MSK);
   }
   regVal |= ((UInt32) (pFlashConfig->pageSize) << 
      TMVH_EFMC_DEV_TYPE0_PAGE_SIZE_POS ) ;

#if (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
   /* -- Kronos specific definitions starts -- */
   regVal |=  ((UInt32)(pFlashConfig->subpageSize) << 
      TMVH_EFMC_DEV_TYPE0_SUB_PAGE_SIZE_POS);

   regVal |=  ((UInt32)(pFlashConfig->oobSize) << 
      TMVH_EFMC_DEV_TYPE0_OOB_SIZE_POS);

   regVal |=  ((UInt32)(pFlashConfig->eccLevel) << 
      TMVH_EFMC_DEV_TYPE0_ECC_LEVEL_POS);

   regVal |=  ((UInt32)(pFlashConfig->erasedPageThres) << 
      TMVH_EFMC_DEV_TYPE0_ERASED_PAGE_THRES_POS);
#endif

   /* -- Kronos specific definitions ends -- */
   TMVH_GEN_WRITE( regs, regVal );

   /* configure the timing0 register */
   regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_DEV_TIMING0_OFFSET +
      ( deviceNum * TMVH_EFMC_DEV_TIMING0_DIFF ) );
   regVal = (UInt32) (pFlashConfig->devTiming.tWaitForRdy & TMVH_EFMC_DEV_TIMING0_TRR_MSK);

   /* please note that, 0x00 meaning 16 cycles, so when masking, if given is 16,then it will turn to be 0x00 which is correct */
   regVal |= ((((UInt32)(pFlashConfig->devTiming.tAleHold)) << TMVH_EFMC_DEV_TIMING0_TALH_POS ) & TMVH_EFMC_DEV_TIMING0_TALH_MSK );
   regVal |= ((((UInt32)(pFlashConfig->devTiming.tAleSetup)) << TMVH_EFMC_DEV_TIMING0_TALS_POS ) & TMVH_EFMC_DEV_TIMING0_TALS_MSK );
   regVal |= ((((UInt32)(pFlashConfig->devTiming.tCleHold)) << TMVH_EFMC_DEV_TIMING0_TCLH_POS ) & TMVH_EFMC_DEV_TIMING0_TCLH_MSK );
   regVal |= ((((UInt32)(pFlashConfig->devTiming.tCleSetup)) << TMVH_EFMC_DEV_TIMING0_TCLS_POS ) & TMVH_EFMC_DEV_TIMING0_TCLS_MSK );
   regVal |= ((((UInt32)(pFlashConfig->devTiming.tCenHold))<< TMVH_EFMC_DEV_TIMING0_TCPH_POS ) & TMVH_EFMC_DEV_TIMING0_TCPH_MSK );
   regVal |= ((((UInt32)(pFlashConfig->devTiming.tCenSetup)) << TMVH_EFMC_DEV_TIMING0_TCPS_POS ) & TMVH_EFMC_DEV_TIMING0_TCPS_MSK );

   TMVH_GEN_WRITE( regs, regVal );

   /* configure the timing1 register */
   regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_DEV_TIMING1_OFFSET +
      ( deviceNum * TMVH_EFMC_DEV_TIMING1_DIFF ) );

   regVal = ((((UInt32)(pFlashConfig->devTiming.tRdDelay)) << TMVH_EFMC_DEV_TIMING1_TDRD_POS ) &  TMVH_EFMC_DEV_TIMING1_TDRD_MSK );
   /* please note that, 0x00 meaning 16 cycles, so when masking, if given is 16,then it will turn to be 0x00 which is correct */
   regVal |= ((((UInt32)(pFlashConfig->devTiming.tWaitForBusy)) << TMVH_EFMC_DEV_TIMING1_TWB_POS ) & TMVH_EFMC_DEV_TIMING1_TWB_MSK );
   regVal |= ((((UInt32)(pFlashConfig->devTiming.tWenWidth)) << TMVH_EFMC_DEV_TIMING1_TWP_POS ) & TMVH_EFMC_DEV_TIMING1_TWP_MSK );
   regVal |= ((((UInt32)(pFlashConfig->devTiming.tWenHigh)) << TMVH_EFMC_DEV_TIMING1_TWH_POS ) & TMVH_EFMC_DEV_TIMING1_TWH_MSK );
   regVal |= ((((UInt32)(pFlashConfig->devTiming.tRenWidth)) << TMVH_EFMC_DEV_TIMING1_TRP_POS ) & TMVH_EFMC_DEV_TIMING1_TRP_MSK );
   regVal |= ((((UInt32)(pFlashConfig->devTiming.tRenHigh)) << TMVH_EFMC_DEV_TIMING1_TRH_POS ) &  TMVH_EFMC_DEV_TIMING1_TRH_MSK );

   TMVH_GEN_WRITE( regs, regVal );
   return TM_OK;
}

tmErrorCode_t
tmhwEfmc_GetOobInfo (
    tmUnitSelect_t            efmcUnitID,
    tmhwEfmc_DeviceNumber_t   deviceNum,
    tmhwEfmc_OobInfo_t       *pOobInfo
)
{
   UInt32   regs     = 0 ;
   UInt32   regVal   = 0 ;

   regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + 
      TMVH_EFMC_OOB_PARTITION_N_OFFSET + ( deviceNum * TMVH_EFMC_DEV_TYPE0_DIFF ) );
   TMVH_GEN_READ( regs, regVal );

   pOobInfo->extended_oob = 
      (UInt16) ((regVal & TMVH_EFMC_OOB_PARTITION_N_EXTENDED_OOB_MSK)
         >> TMVH_EFMC_OOB_PARTITION_N_EXTENDED_OOB_POS);

   pOobInfo->unprotected_bytes = 
      (UInt8) ((regVal & TMVH_EFMC_OOB_PARTITION_N_UNPROTECTED_BYTES_MSK)
         >> TMVH_EFMC_OOB_PARTITION_N_UNPROTECTED_BYTES_POS);

   pOobInfo->parity_bytes = 
      (UInt8) ((regVal & TMVH_EFMC_OOB_PARTITION_N_PARITY_BYTES_MSK)
         >> TMVH_EFMC_OOB_PARTITION_N_PARITY_BYTES_POS);

   pOobInfo->protected_oob_min = 
      (UInt8) ((regVal & TMVH_EFMC_OOB_PARTITION_N_PROTECTED_OOB_MIN_MSK)
         >> TMVH_EFMC_OOB_PARTITION_N_PROTECTED_OOB_MIN_POS);

  return TM_OK;
}

tmErrorCode_t
tmhwEfmc_GetPageRWStatusFifo (
    tmUnitSelect_t               efmcUnitID,
    tmhwEfmc_PageRWStatusFifo_t *pPageRWStatusFifo
)
{
   UInt32   regs     = 0;
   UInt32   regVal   = 0;
   UInt32   i        = 0;

   regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + 
      TMVH_EFMC_FIFO_STATUS_OFFSET );
   TMVH_GEN_READ( regs, regVal );

   pPageRWStatusFifo->PageRdStatusLevel =  
      ((regVal & TMVH_EFMC_FIFO_PAGE_RD_STATUS_LEVEL_MSK)
         >> TMVH_EFMC_FIFO_PAGE_RD_STATUS_LEVEL_POS);

   pPageRWStatusFifo->PageWrStatusLevel = 
      ((regVal & TMVH_EFMC_FIFO_PAGE_WR_STATUS_LEVEL_MSK)
         >> TMVH_EFMC_FIFO_PAGE_WR_STATUS_LEVEL_POS);

   for (i = 0; i < pPageRWStatusFifo->PageRdStatusLevel; i++)
   {
      regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + 
         TMVH_EFMC_PAGE_RD_STATUS_FIFO_OFFSET );
      TMVH_GEN_READ( regs, regVal );

      pPageRWStatusFifo->PageRdStatus[i].tag             = 
         ((regVal & TMVH_EFMC_PAGE_RD_STATUS_FIFO_TAG_MSK)
            >> TMVH_EFMC_PAGE_RD_STATUS_FIFO_TAG_POS);

      pPageRWStatusFifo->PageRdStatus[i].erased          = 
         ((regVal & TMVH_EFMC_PAGE_RD_STATUS_FIFO_ERASED_MSK)
            >> TMVH_EFMC_PAGE_RD_STATUS_FIFO_ERASED_POS);

      pPageRWStatusFifo->PageRdStatus[i].ecc_off         = 
         ((regVal & TMVH_EFMC_PAGE_RD_STATUS_FIFO_ECC_OFF_MSK)
            >> TMVH_EFMC_PAGE_RD_STATUS_FIFO_ECC_OFF_POS);

      pPageRWStatusFifo->PageRdStatus[i].uncorrectable   = 
         ((regVal & TMVH_EFMC_PAGE_RD_STATUS_FIFO_UNCORRECTABLE_MSK)
            >> TMVH_EFMC_PAGE_RD_STATUS_FIFO_UNCORRECTABLE_POS);

      pPageRWStatusFifo->PageRdStatus[i].corrected       = 
         ((regVal & TMVH_EFMC_PAGE_RD_STATUS_FIFO_CORRECTED_MSK)
            >> TMVH_EFMC_PAGE_RD_STATUS_FIFO_CORRECTED_POS);

      pPageRWStatusFifo->PageRdStatus[i].error_free      = 
         ((regVal & TMVH_EFMC_PAGE_RD_STATUS_FIFO_ERROR_FREE_MSK)
            >> TMVH_EFMC_PAGE_RD_STATUS_FIFO_ERROR_FREE_POS);

      pPageRWStatusFifo->PageRdStatus[i].num_errors      =
         ((regVal & TMVH_EFMC_PAGE_RD_STATUS_FIFO_NUM_ERROR_MSK)
            >> TMVH_EFMC_PAGE_RD_STATUS_FIFO_NUM_ERROR_POS);
   }

   for (i = 0; i < pPageRWStatusFifo->PageWrStatusLevel; i++)
   {
      regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + 
         TMVH_EFMC_PAGE_WR_STATUS_FIFO_OFFSET );
      TMVH_GEN_READ( regs, regVal );

      pPageRWStatusFifo->PageWrStatus[i].tag      =
         ((regVal & TMVH_EFMC_PAGE_WR_STATUS_FIFO_TAG_MSK)
            >> TMVH_EFMC_PAGE_WR_STATUS_FIFO_TAG_POS);
   }

   return TM_OK;
}

tmErrorCode_t
tmhwEfmc_SetPageRWStatusFifoMode (
    tmUnitSelect_t                  efmcUnitID,
    tmhwEfmc_PageRWStatusFifoMode_t  *pPageRWStatusFifoMode
)
{
   UInt32   regs     = 0;
   UInt32   regVal   = 0;

   regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + 
      TMVH_EFMC_STATUS_FIFO_MODE_OFFSET );
   regVal = (UInt32) (pPageRWStatusFifoMode->status_fifo_mode);
   TMVH_GEN_WRITE( regs, regVal );

   regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + 
      TMVH_EFMC_STATUS_FIFO_POP_OFFSET );
   regVal = (UInt32) (pPageRWStatusFifoMode->status_fifo_pop);
   TMVH_GEN_WRITE( regs, regVal );

   return TM_OK;
}

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
)
{
  UInt32              regs  = 0 ;
  UInt32                         regVal    = 0 ;
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_SINGLE_READ_OFFSET );
  
  TMVH_GEN_READ( regs, regVal );
  *pDataRead = (UInt16) (regVal & TMVH_EFMC_SINGLE_READ_DATA_MSK );
  return TM_OK;
}


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
)
{
  UInt32              regs  = 0 ;
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_SINGLE_WRITE_OFFSET );

  TMVH_GEN_WRITE( regs, ( (UInt32) dataWrite & TMVH_EFMC_SINGLE_WRITE_DATA_MSK ) );
  return TM_OK;
}


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
    tmhwEfmc_DeviceNumber_t                           deviceNum ATTRIBUTE_UNUSED,
    pVoid                           pDataRead
)
{
  pUInt8   pSrc   = (pUInt8) 0 ;
  pUInt8   pTgt   = (pUInt8) 0 ;
  UInt32   ctr    = TMVH_EFMC_MAX_AHB_MAIN_MEM_SIZE ;
  UInt32   i      = 0 ;

  /* source - AHB memory buffer */
  /* target - user buffer */
  pSrc = ( pUInt8 ) TMHW_EFMC_GET_AHB_ADDRESS( efmcUnitID ) ;
  pTgt = ( pUInt8 ) pDataRead ;

  /* regardless of data width, just copy the data present in source buffer to target buffer */
  while( ctr > 0 )
  {
    pTgt[i] = pSrc[i] ;
    i++;
    ctr--;
  }

  return TM_OK;
}


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
)
{
  pUInt8   pSrc   = (pUInt8) 0 ;
  pUInt8   pTgt   = (pUInt8) 0 ;
  UInt32   ctr    = 0 ;
  UInt32   i      = 0 ;
  UInt32   regs   = 0 ;
  UInt32   regVal = 0 ;

  regs =  (UInt32) (TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_DEV_TYPE0_OFFSET +
                                   ( deviceNum * TMVH_EFMC_DEV_TYPE0_DIFF ) );
  TMVH_GEN_READ( regs, regVal ) ;

  regVal = ( ( regVal & TMVH_EFMC_DEV_TYPE0_PAGE_SIZE_MSK ) >> TMVH_EFMC_DEV_TYPE0_PAGE_SIZE_POS );
  switch( regVal )
  {
    case 0x00 :
      {
        ctr = ( 16 ) ; /* page size = 512 bytes */
        break;
      }
    case 0x01 :
      {
        ctr = ( 64 ) ; /* page size = 2048 bytes */
        break;
      }
    case 0x02 :
      {
        ctr = ( 128 ) ; /* page size = 4096 bytes */
        break;
      }
    default:
      {
        break;
      }
  }

  /* source - AHB memory buffer */
  /* target - user buffer */
  pSrc = ( pUInt8 ) ( TMHW_EFMC_GET_AHB_ADDRESS( efmcUnitID) ) ;
  pSrc = &pSrc[TMVH_EFMC_MAX_AHB_MAIN_MEM_SIZE];
  pTgt = ( pUInt8 ) pDataRead ;

  /* regardless of data width, just copy the data present in source buffer to target buffer */
  while( ctr > 0 )
  {
    pTgt[i] = pSrc[i] ;
    i++;
    ctr--;
  }

  return TM_OK;
}


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
    tmhwEfmc_DeviceNumber_t         deviceNum ATTRIBUTE_UNUSED,
    pVoid                           pDataWrite
)
{
  pUInt8   pSrc   = (pUInt8) 0 ;
  pUInt8   pTgt   = (pUInt8) 0 ;
  UInt32   ctr    = TMVH_EFMC_MAX_AHB_MAIN_MEM_SIZE ;
  UInt32   i      = 0 ;

  /* source - user buffer */
  /* target - AHB memory buffer */

  pSrc = ( pUInt8 ) pDataWrite ;
  pTgt = ( pUInt8 ) ( TMHW_EFMC_GET_AHB_ADDRESS( efmcUnitID) ) ;

  /* regardless of data width, just copy the data present in source buffer to target buffer */
  while( ctr > 0 )
  {
    pTgt[i] = pSrc[i] ;
    i++;
    ctr--;
  }

  return TM_OK;
}




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
    tmhwEfmc_DeviceNumber_t         deviceNum,
    pVoid                           pDataWrite
)
{
  pUInt8   pSrc   = (pUInt8) 0 ;
  pUInt8   pTgt   = (pUInt8) 0 ;
  UInt32   ctr    = 0 ;
  UInt32   i      = 0 ;
  UInt32   regs   = 0 ;
  UInt32   regVal = 0 ;

  regs =  (UInt32) (TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_DEV_TYPE0_OFFSET +
                                   ( deviceNum * TMVH_EFMC_DEV_TYPE0_DIFF ) );
  TMVH_GEN_READ( regs, regVal ) ;

  regVal = ( ( regVal & TMVH_EFMC_DEV_TYPE0_PAGE_SIZE_MSK ) >> TMVH_EFMC_DEV_TYPE0_PAGE_SIZE_POS );
  switch( regVal )
  {
    case 0x00 :
      {
        ctr = ( 16 ) ; /* page size = 512 bytes */
        break;
      }
    case 0x01 :
      {
        ctr = ( 64 ) ; /* page size = 2048 bytes */
        break;
      }
    case 0x02 :
      {
        ctr = ( 128 ) ; /* page size = 4096 bytes */
        break;
      }
    default:
      {
        break;
      }
  }

  /* source - user buffer */
  /* target - AHB memory buffer */
  pSrc = ( pUInt8 ) pDataWrite ;
  pTgt = ( pUInt8 ) ( TMHW_EFMC_GET_AHB_ADDRESS( efmcUnitID) ) ;
  pTgt = &pTgt[TMVH_EFMC_MAX_AHB_MAIN_MEM_SIZE];

  /* regardless of data width, just copy the data present in source buffer to target buffer */
  while( ctr > 0 )
  {
    pTgt[i] = pSrc[i] ;
    i++;
    ctr--;
  }

  return TM_OK;
}


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
)
{
  UInt32              regs = 0 ;
  UInt32                         regVal     = 0 ;
  UInt32                         regMsk    = 0 ;
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_NAND_STATUS_OFFSET );

  /* get the mask for the register */
  regMsk = (UInt32) ( 0x01U << ( deviceNum & TMVH_EFMC_MOD_CONFIG_NUM_DEV_MSK ));

  /* read the data present in module ID register */
  TMVH_GEN_READ( regs, regVal ) ;

  
  pBusyStatus->rbEdge_Rise = TM_FALSE ;
  pBusyStatus->rbEdge_Fall = TM_FALSE ;
  pBusyStatus->rbEdge_Status_Ready = TM_FALSE ;

  if( 0x00 != ( regVal & regMsk ) )
  {
    pBusyStatus->rbEdge_Status_Ready = TM_TRUE ;
  }

  if( 0x00 != ( regVal & ( regMsk << TMVH_EFMC_NAND_STATUS_RBN_FALL_POS ) ) )
  {
    pBusyStatus->rbEdge_Fall = TM_TRUE ;
  }

  if( 0x00 != ( regVal & ( regMsk << TMVH_EFMC_NAND_STATUS_RBN_RISE_POS )) )
  {
    pBusyStatus->rbEdge_Rise = TM_TRUE ;
  }

  return TM_OK;
}

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
)
{
  UInt32              regs  = 0 ;
  UInt32                         regVal   = 0 ;
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_DMA_CTRL_OFFSET );
  
  if( tmhwEfmc_Enable == pDmaConfig->enableM2PDma )
  {
    regVal |= TMVH_EFMC_DMA_CTRL_MEM_2_DMA_MSK;
  }
  if( tmhwEfmc_Enable == pDmaConfig->enableP2MDma )
  {
    regVal |= TMVH_EFMC_DMA_CTRL_DMA_2_MEM_MSK;
  }
  
  TMVH_GEN_WRITE( regs, regVal );
  return TM_OK;
}


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
)
{
  UInt32              regs  = 0 ;
  UInt32                         regVal   = 0 ;
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_DMA_CTRL_OFFSET );
  
  TMVH_GEN_READ( regs, regVal );
  
  pDmaConfig->enableP2MDma = ( tmhwEfmc_EnableDisable_t ) ( regVal & TMVH_EFMC_DMA_CTRL_DMA_2_MEM_MSK );
  if( TMVH_EFMC_DMA_CTRL_MEM_2_DMA_MSK == ( regVal & TMVH_EFMC_DMA_CTRL_MEM_2_DMA_MSK )  )
  {
    pDmaConfig->enableM2PDma = tmhwEfmc_Enable;
  }
  else
  {
    pDmaConfig->enableM2PDma = tmhwEfmc_Disable;
  }
  return TM_OK;
}


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
)
{
  UInt32              regs  = 0 ;
  
  /* write into the AES Key registers */
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_AES_KEY1_OFFSET );
  TMVH_GEN_WRITE( regs, pAESConfig->keyAES[0]);
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_AES_KEY2_OFFSET );
  TMVH_GEN_WRITE( regs, pAESConfig->keyAES[1]);
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_AES_KEY3_OFFSET );
  TMVH_GEN_WRITE( regs, pAESConfig->keyAES[2]);
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_AES_KEY4_OFFSET );
  TMVH_GEN_WRITE( regs, pAESConfig->keyAES[3]);
  
  /* write into the AES Initial Value registers */
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_AES_IV1_OFFSET );
  TMVH_GEN_WRITE( regs, pAESConfig->initAES[0]);
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_AES_IV2_OFFSET );
  TMVH_GEN_WRITE( regs, pAESConfig->initAES[1]);
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_AES_IV3_OFFSET );
  TMVH_GEN_WRITE( regs, pAESConfig->initAES[2]);
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_AES_IV4_OFFSET );
  TMVH_GEN_WRITE( regs, pAESConfig->initAES[3]);
  
  return TM_OK;
}


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
)
{
  UInt32              regs  = 0 ;
  
  /* read from the registers and assign it into the structures members */
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_AES_KEY1_OFFSET );
  TMVH_GEN_READ( regs, pAESConfig->keyAES[0]);
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_AES_KEY2_OFFSET );
  TMVH_GEN_READ( regs, pAESConfig->keyAES[1]);
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_AES_KEY3_OFFSET );
  TMVH_GEN_READ( regs, pAESConfig->keyAES[2]);
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_AES_KEY4_OFFSET );
  TMVH_GEN_READ( regs, pAESConfig->keyAES[3]);
  
  /* write into the AES Initial Value registers */
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_AES_IV1_OFFSET );
  TMVH_GEN_READ( regs, pAESConfig->initAES[0]);
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_AES_IV2_OFFSET );
  TMVH_GEN_READ( regs, pAESConfig->initAES[1]);
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_AES_IV3_OFFSET );
  TMVH_GEN_READ( regs, pAESConfig->initAES[2]);
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_AES_IV4_OFFSET );
  TMVH_GEN_READ( regs, pAESConfig->initAES[3]);
  
  return TM_OK;
}


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
)
{
  UInt32              regs  = 0 ;
  UInt32                         regVal         = 0 ;
  
  /* read from the registers and assign it into the structures members */
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_AES_STATUS_OFFSET );
  TMVH_GEN_READ( regs, regVal);
  *pAESStatus = ( tmhwEfmc_AESStatus_t ) ( regVal & ( TMVH_EFMC_AES_STATUS_ACCEPT_KEY_MSK | TMVH_EFMC_AES_STATUS_ACCEPT_IN_MSK ) );
  
  return TM_OK;
}


/*! \} */ /* end of AESSupport */

#endif



#if (TMFL_EFMCSD_POWER || TMFL_SD_ALL)

/** \addtogroup PowerState
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
)
{
  UInt32              regs = 0 ;
  UInt32                         regVal        = 0 ;
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_POWER_DOWN_OFFSET );
  
  /* read the register and modify the timeout value */
  TMVH_GEN_READ( regs, regVal ) ;
  regVal &= (UInt32) ~( (UInt32)TMVH_EFMC_POWER_DOWN_POWER_DOWN_MSK ) ;
  /* valid values are : tmPowerOff and tmPowerOn */
  if( tmPowerOff == powerState )
  {
    regVal |= TMVH_EFMC_POWER_DOWN_POWER_DOWN_MSK ;
  }
  
  TMVH_GEN_WRITE( regs, regVal );
  return TM_OK;
}


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
)
{
  UInt32              regs = 0 ;
  UInt32                         regVal        = 0 ;
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_POWER_DOWN_OFFSET );
  
  /* read the register and modify the timeout value */
  TMVH_GEN_READ( regs, regVal ) ;
  if( ( regVal & TMVH_EFMC_POWER_DOWN_POWER_DOWN_MSK ) == TMVH_EFMC_POWER_DOWN_POWER_DOWN_MSK )
  {
    *pPowerState = tmPowerOff;
  }
  else
  {
    *pPowerState = tmPowerOn;
  }
  return TM_OK;
}

/*! \} */ /* end of PowerState */

#endif



#if (TMFL_EFMCSD_INT || TMFL_SD_ALL)

/** \addtogroup InterruptSupport
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
)
{
  UInt32              regs = 0 ;
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_INT_SET_ENABLE_OFFSET );
  
  /* write the value directly */
  TMVH_GEN_WRITE( regs, ( efmIntEnable & TMVH_EFMC_INT_MSK ) );
  
  return TM_OK;
}


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
)
{
  UInt32              regs = 0 ;
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_INT_CLR_ENABLE_OFFSET );
  
  /* write the value directly */
  TMVH_GEN_WRITE( regs, ( efmIntDisable & TMVH_EFMC_INT_MSK ) );
  
  return TM_OK;
}

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
)
{
  UInt32              regs = 0 ;
  UInt32                         regVal           = 0 ;
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_INT_STATUS_OFFSET );
  
  /* write the value directly */
  TMVH_GEN_READ( regs, regVal ) ;
  *pEfmIntGetStatus = ( tmhwEfmc_IntMask_t ) ( regVal & TMVH_EFMC_INT_MSK ) ;
  
  return TM_OK;
}


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
)
{
  UInt32              regs = 0 ;
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_INT_CLR_STATUS_OFFSET );
  
  /* write the value directly */
  TMVH_GEN_WRITE( regs, ( efmIntClear & TMVH_EFMC_INT_MSK ) ) ;
  
  return TM_OK;
}


/*! \} */ /* end of InterruptSupport */

#endif





#if (TMFL_EFMCSD_EBI || TMFL_SD_ALL)
/*! \addtogroup EBIBackOffMode
 *  \{
 */


/*!\fn tmErrorCode_t tmhwEfmc_SetBackoffConfig ( tmUnitSelect_t efmcUnitID, const tmhwEfmc_BackoffConfig_t * pBackOffConfig );
*     This function configures the EBI pin sharing (backoff mode) for the EFMC. The details include hte timeout for
*     backoff, mode of backoff. This function acts on the whole EFMC.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[in] pBackConfig -> pointer to the tmhwEfmc_BackoffConfig_t structure that contians the backoff config parameters.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_SetBackoffConfig (
    tmUnitSelect_t                  efmcUnitID,
    const tmhwEfmc_BackoffConfig_t *   pBackOffConfig
)
{
  UInt32              regs = 0 ;
  UInt32                         regVal  = 0 ;
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_EBI_CTRL_OFFSET );
  
  /* write the data to EBI register directly */
  regVal = pBackOffConfig->defaultReqEnable ;
  regVal |= (UInt32) (pBackOffConfig->mode & TMVH_EFMC_EBI_CTRL_MODE_MSK );
  regVal |= (UInt32)  ( ((UInt32)pBackOffConfig->timeOut << TMVH_EFMC_EBI_CTRL_TIMEOUT_POS ) & TMVH_EFMC_EBI_CTRL_TIMEOUT_MSK ) ;
  TMVH_GEN_WRITE( regs, regVal ) ;
  return TM_OK;
}


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
)
{
  UInt32              regs = 0 ;
  UInt32                         regVal  = 0 ;
  
  regs =  (UInt32) ( (UInt32)  TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_EBI_CTRL_OFFSET );
  
  TMVH_GEN_READ( regs, regVal ) ;
  pBackOffConfig->defaultReqEnable = ( tmhwEfmc_EnableDisable_t ) ( regVal & TMVH_EFMC_EBI_CTRL_DEF_REQ_MSK ) ;
  pBackOffConfig->mode = ( tmhwEfmc_BackoffMode_t ) ( regVal & TMVH_EFMC_EBI_CTRL_MODE_MSK );
  pBackOffConfig->timeOut = ( UInt8 ) ( ( regVal & TMVH_EFMC_EBI_CTRL_TIMEOUT_MSK ) >> TMVH_EFMC_EBI_CTRL_TIMEOUT_POS ) ;
  return TM_OK;
}



/*! \} */ /* end of EBIBackOffMode */

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
)

{
  UInt32              regs = 0 ;
  UInt32                         regVal  = 0 ;
  
  regs =  (UInt32) ( (UInt32)  TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_AHB_LATENCY_OFFSET );
  
  /* write the data to EBI register directly */
  if( tmhwEfmc_Registered == commandRegistering )
  {
    regVal |= TMVH_EFMC_AHB_LATENCY_CMD_MSK ;
  }
  regVal |= (UInt32) ( rdDataRegistering );
  
  TMVH_GEN_WRITE( regs, regVal ) ;
  
  return TM_OK;
}


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
)
{
  UInt32              regs = 0 ;
  UInt32                         regVal  = 0 ;
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_AHB_LATENCY_OFFSET );
  
  TMVH_GEN_READ( regs, regVal );
  
  *pCommandRegistering = ( tmhwEfmc_SelRegistering_t ) ( ( regVal & TMVH_EFMC_AHB_LATENCY_CMD_MSK ) >> TMVH_EFMC_AHB_LATENCY_CMD_POS ) ;
  *pRdDataRegistering = ( tmhwEfmc_SelRegistering_t ) ( ( regVal & TMVH_EFMC_AHB_LATENCY_RD_DATA_MSK ) );
  
  return TM_OK;
}


/*! \} */ /* addtogroup AHBConfig */

#endif




#if (TMFL_EFMCSD_APB || TMFL_SD_ALL)

/** \addtogroup APBConfig
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
)
{
  UInt32              regs = 0 ;
  UInt32                         regVal  = 0 ;
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_APB_ERROR_GEN_OFFSET );
  
  if( tmhwEfmc_Enable == enableWrErr )
  {
    regVal |= TMVH_EFMC_APB_ERROR_GEN_WR_MSK ;
  }
  regVal |= (UInt32) enableRdErr;
  
  TMVH_GEN_WRITE( regs, regVal );
  
  return TM_OK;
}



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
)
{
  UInt32              regs = 0 ;
  UInt32                         regVal  = 0 ;
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_APB_ERROR_GEN_OFFSET );
  
  TMVH_GEN_READ( regs, regVal );
  
  *pEnableWrErr = ( tmhwEfmc_EnableDisable_t) ( (regVal & TMVH_EFMC_APB_ERROR_GEN_WR_MSK) >> TMVH_EFMC_APB_ERROR_GEN_WR_POS );
  *pEnableRdErr = ( tmhwEfmc_EnableDisable_t ) (regVal & TMVH_EFMC_APB_ERROR_GEN_RD_MSK) ;
  
  return TM_OK;
}



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
)
{
  UInt32              regs = 0 ;
  UInt32                         regVal        = 0 ;
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_POWER_DOWN_OFFSET );
  
  /* read the register and modify the timeout value */
  TMVH_GEN_READ( regs, regVal ) ;
  regVal &= TMVH_EFMC_POWER_DOWN_POWER_DOWN_MSK;
  regVal |= (UInt32) ( ( timeout << TMVH_EFMC_POWER_DOWN_TIMEOUT_POS ) & TMVH_EFMC_POWER_DOWN_TIMEOUT_MSK );
  
  TMVH_GEN_WRITE( regs, regVal ) ;
  return TM_OK;
}


/*!\fn tmErrorCode_t tmhwEfmc_GetApbTimeoutConfig ( tmUnitSelect_t efmcUnitID, pUInt16 ptimeout);
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
)
{
  UInt32              regs = 0 ;
  UInt32                         regVal        = 0 ;
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_POWER_DOWN_OFFSET );
  
  /* read the register and modify the timeout value */
  TMVH_GEN_READ( regs, regVal ) ;
  *pTimeout = ( UInt16 ) ( ( regVal & TMVH_EFMC_POWER_DOWN_TIMEOUT_MSK ) >> TMVH_EFMC_POWER_DOWN_TIMEOUT_POS );
  
  return TM_OK;
}


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
)
{
  UInt32              regs = 0 ;
  UInt32                         regVal         = 0 ;
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_APB_LATENCY_OFFSET );
  
  regVal = insertWait;
  
  TMVH_GEN_WRITE( regs, regVal ) ;
  
  return TM_OK;
}


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
)
{
  UInt32              regs = 0 ;
  UInt32                         regVal        = 0 ;
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_APB_LATENCY_OFFSET );
  
  TMVH_GEN_READ( regs, regVal ) ;
  *pInsertWait = ( tmhwEfmc_Apb_Latency_t) ( regVal & TMVH_EFMC_APB_LATENCY_WS_MSK );
  
  return TM_OK;
}


/*! \} */ /* end of APBConfig */

#endif


#if (TMFL_EFMCSD_OTHERS || TMFL_SD_ALL)

/** \addtogroup Others
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
)
{
  pEfmVersionInfo->compatibilityNr = TMHW_EFMC_COMPATIBILITY_NR;
  pEfmVersionInfo->majorVersionNr  = TMHW_EFMC_MAJOR_VERSION_NR;
  pEfmVersionInfo->minorVersionNr  = TMHW_EFMC_MINOR_VERSION_NR;
  return TM_OK;
}



/*!\fn tmErrorCode_t tmhwEfmc_Deinit(tmUnitSelect_t  efmcUnitID);
*     This function will deinitialize the EFMC device. This function serves no purpose.
*
*    \param[in] efmcUnitID -> EFMC unit number.
*    \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_Deinit(
    tmUnitSelect_t                 efmcUnitID ATTRIBUTE_UNUSED
)
{
  /* This function does nothing.. */
  return TM_OK;
}


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
)
{
  UInt32              regs   = 0 ;
  UInt32                         regVal    = 0 ;
  
  regs = (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_MOD_CONFIG_OFFSET );
  
  /* read the data present in the config register */
  TMVH_GEN_READ( regs, regVal ) ;

  pCaps->progApbLatency =   tmhwEfmc_Configurable ;
  pCaps->progApbErrConfig =   tmhwEfmc_Configurable ;
  pCaps->dataBusLatency =   tmhwEfmc_Configurable ;
  pCaps->supportAES = (Bool) TM_FALSE ;
  pCaps->enEBIPinShare = (Bool) TM_FALSE ;
  pCaps->bootFromNAND = (Bool) TM_FALSE ;
  pCaps->localRstSync = (Bool) TM_FALSE ;
  
  if( 0x00 != ( regVal & TMVH_EFMC_MOD_CONFIG_APB_LAT_MSK ) )
  {
    pCaps->progApbLatency =   tmhwEfmc_Programmable ;
  }
  if( 0x00 != ( regVal & TMVH_EFMC_MOD_CONFIG_APB_ERR_MSK ) )
  {
    pCaps->progApbErrConfig =   tmhwEfmc_Programmable ;
  }
  if( 0x00 != ( regVal & TMVH_EFMC_MOD_CONFIG_DATA_BUS_LAT_MSK ) )
  {
    pCaps->dataBusLatency =   tmhwEfmc_Programmable ;
  }
  
  pCaps->busType = ( tmhwEfmc_DataBusType_t ) ( regVal & TMVH_EFMC_MOD_CONFIG_DATA_BUS_TYPE_MSK );
  pCaps->maxDevices = ( UInt8 ) ( ( regVal & TMVH_EFMC_MOD_CONFIG_NUM_DEV_MSK ) + 1 );
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_MODID_OFFSET );
  
  /* read the data present in module ID register */
  TMVH_GEN_READ( regs, regVal ) ;
  
  pCaps->efmcModuleId = (UInt16) ( ( regVal & TMVH_EFMC_MODID_MODID_MSK ) >> TMVH_EFMC_MODID_MODID_POS);
  pCaps->efmcRevMajor = ( UInt8 ) (( regVal & TMVH_EFMC_MODID_MAJOR_VER_MSK ) >> TMVH_EFMC_MODID_MAJOR_VER_POS);
  pCaps->efmcRevMinor = ( UInt8 ) (( regVal & TMVH_EFMC_MODID_MINOR_VER_MSK ) >> TMVH_EFMC_MODID_MINOR_VER_POS);
  pCaps->efmcAperSize = ( UInt8 ) ( regVal & TMVH_EFMC_MODID_APERTURE_MSK ) ;
  
  return TM_OK;
}


/*!\fn tmErrorCode_t tmhwEfmc_GetFlashConfig( tmUnitSelect_t efmcUnitID, tmhwEfmc_DeviceNumber_t deviceNum, ptmhwEfmc_FlashConfig_t pFlashConfig);
*     This function retrieves the given NAND flash device configuration. The configuration such as timing parameters, data width and page size and also for write-protect of NAND flash device is stored in the parameter.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \param[in] deviceNum -> Nand flash device number for which the configuration to be retrieved.
*     \param[out] pFlashConfig -> pointer to the tmhwEfmc_FlashConfig_t structure where in the read configuration needs to be stored.
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_GetFlashConfig(
    tmUnitSelect_t          efmcUnitID,
    tmhwEfmc_DeviceNumber_t deviceNum,
    ptmhwEfmc_FlashConfig_t pFlashConfig
)
{
  UInt32 regs   = 0 ;
  UInt32 regVal = 0 ;

  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_DEV_TYPE0_OFFSET +
                    ( deviceNum * TMVH_EFMC_DEV_TYPE0_DIFF ) );

  TMVH_GEN_READ( regs, regVal );
  if( ( regVal & TMVH_EFMC_DEV_TYPE0_CEN_DONT_MSK ) == 0 )
  {
    pFlashConfig->enableCENDontCare = tmhwEfmc_Disable;
  }
  else
  {
    pFlashConfig->enableCENDontCare = tmhwEfmc_Enable;
  }
  if( ( regVal & TMVH_EFMC_DEV_TYPE0_WP_MSK ) == 0 )
  {
    pFlashConfig->enableWrProtect = tmhwEfmc_Disable;
  }
  else
  {
    pFlashConfig->enableWrProtect = tmhwEfmc_Enable;
  }
  pFlashConfig->dataWidth = (tmhwEfmc_DataWidth_t) ( regVal & TMVH_EFMC_DEV_TYPE0_DATA_WIDTH_MSK ) ;
  pFlashConfig->pageSize = (tmhwEfmc_PageSize_t) 
    ((regVal & TMVH_EFMC_DEV_TYPE0_PAGE_SIZE_MSK) >>
                TMVH_EFMC_DEV_TYPE0_PAGE_SIZE_POS);

#if (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
  /* -- Kronos specific definitions starts -- */
  pFlashConfig->subpageSize = (tmhwEfmc_SubPageSize_t) 
    ((regVal & TMVH_EFMC_DEV_TYPE0_SUB_PAGE_SIZE_MSK) >> 
                TMVH_EFMC_DEV_TYPE0_SUB_PAGE_SIZE_POS);
  pFlashConfig->oobSize = (UInt16) 
    ((regVal & TMVH_EFMC_DEV_TYPE0_OOB_SIZE_MSK) >> 
                TMVH_EFMC_DEV_TYPE0_OOB_SIZE_POS);
  pFlashConfig->eccLevel = (UInt8) 
    ((regVal & TMVH_EFMC_DEV_TYPE0_ECC_LEVEL_MSK) >>
                TMVH_EFMC_DEV_TYPE0_ECC_LEVEL_POS);
  pFlashConfig->erasedPageThres = (UInt8) 
    ((regVal & TMVH_EFMC_DEV_TYPE0_ERASED_PAGE_THRES_MSK) >>
                TMVH_EFMC_DEV_TYPE0_ERASED_PAGE_THRES_POS);
#endif

  /* now read timing 0 parameters and assign appropriately */
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_DEV_TIMING0_OFFSET +
    ( deviceNum * TMVH_EFMC_DEV_TIMING0_DIFF ) );
  TMVH_GEN_READ( regs, regVal );

  pFlashConfig->devTiming.tWaitForRdy = (UInt8) ( regVal & TMVH_EFMC_DEV_TIMING0_TRR_MSK );
  if( 0x00 == pFlashConfig->devTiming.tWaitForRdy )
  {
    pFlashConfig->devTiming.tWaitForRdy = (UInt8) (TMVH_EFMC_DEV_TIMING0_MAX_CYCLES) ;
  }

  /* please note that, 0x00 meaning 16 cycles, so when masking, if given is 16,then it will turn to be 0x00 which is correct */
  pFlashConfig->devTiming.tAleHold = (UInt8) ( ( regVal & TMVH_EFMC_DEV_TIMING0_TALH_MSK ) >> TMVH_EFMC_DEV_TIMING0_TALH_POS );
  if( 0x00 == pFlashConfig->devTiming.tAleHold )
  {
    pFlashConfig->devTiming.tAleHold = (UInt8) (TMVH_EFMC_DEV_TIMING0_MAX_CYCLES) ;
  }
  pFlashConfig->devTiming.tAleSetup = (UInt8) ( ( regVal & TMVH_EFMC_DEV_TIMING0_TALS_MSK ) >> TMVH_EFMC_DEV_TIMING0_TALS_POS );
  if( 0x00 == pFlashConfig->devTiming.tAleSetup )
  {
    pFlashConfig->devTiming.tAleSetup = (UInt8) (TMVH_EFMC_DEV_TIMING0_MAX_CYCLES) ;
  }
  pFlashConfig->devTiming.tCleHold = (UInt8) ( ( regVal & TMVH_EFMC_DEV_TIMING0_TCLH_MSK ) >> TMVH_EFMC_DEV_TIMING0_TCLH_POS );
  if( 0x00 == pFlashConfig->devTiming.tCleHold )
  {
    pFlashConfig->devTiming.tCleHold = (UInt8) (TMVH_EFMC_DEV_TIMING0_MAX_CYCLES) ;
  }
  pFlashConfig->devTiming.tCleSetup = (UInt8) ( ( regVal & TMVH_EFMC_DEV_TIMING0_TCLS_MSK ) >> TMVH_EFMC_DEV_TIMING0_TCLS_POS );
  if( 0x00 == pFlashConfig->devTiming.tCleSetup )
  {
    pFlashConfig->devTiming.tCleSetup = (UInt8) (TMVH_EFMC_DEV_TIMING0_MAX_CYCLES) ;
  }
  pFlashConfig->devTiming.tCenHold = (UInt8) ( ( regVal & TMVH_EFMC_DEV_TIMING0_TCPH_MSK ) >> TMVH_EFMC_DEV_TIMING0_TCPH_POS );
  if( 0x00 == pFlashConfig->devTiming.tCenHold )
  {
    pFlashConfig->devTiming.tCenHold = (UInt8) (TMVH_EFMC_DEV_TIMING0_MAX_CYCLES) ;
  }
  pFlashConfig->devTiming.tCenSetup = (UInt8) ( ( regVal & TMVH_EFMC_DEV_TIMING0_TCPS_MSK ) >> TMVH_EFMC_DEV_TIMING0_TCPS_POS );
  if( 0x00 == pFlashConfig->devTiming.tCenSetup )
  {
    pFlashConfig->devTiming.tCenSetup = (UInt8) (TMVH_EFMC_DEV_TIMING0_MAX_CYCLES) ;
  }

  /* configure the timing1 register */
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_DEV_TIMING1_OFFSET +
    ( deviceNum * TMVH_EFMC_DEV_TIMING1_DIFF ) );

  TMVH_GEN_READ( regs, regVal );

  pFlashConfig->devTiming.tRdDelay = (UInt8) ( ( regVal & TMVH_EFMC_DEV_TIMING1_TDRD_MSK ) >> TMVH_EFMC_DEV_TIMING1_TDRD_POS );
  pFlashConfig->devTiming.tWaitForBusy = (UInt8) ( ( regVal & TMVH_EFMC_DEV_TIMING1_TWB_MSK ) >> TMVH_EFMC_DEV_TIMING1_TWB_POS );

  if( 0x00 == pFlashConfig->devTiming.tWaitForBusy )
  {
    pFlashConfig->devTiming.tWaitForBusy = TMVH_EFMC_DEV_TIMING1_TWB_MAX_CYCLES;
  }
  pFlashConfig->devTiming.tWenWidth = (UInt8) ( ( regVal & TMVH_EFMC_DEV_TIMING1_TWP_MSK ) >> TMVH_EFMC_DEV_TIMING1_TWP_POS );
  if( 0x00 == pFlashConfig->devTiming.tWenWidth )
  {
    pFlashConfig->devTiming.tWenWidth = TMVH_EFMC_DEV_TIMING1_MAX_CYCLES;
  }
  pFlashConfig->devTiming.tWenHigh = (UInt8) ( ( regVal & TMVH_EFMC_DEV_TIMING1_TWH_MSK ) >> TMVH_EFMC_DEV_TIMING1_TWH_POS );
  if( 0x00 == pFlashConfig->devTiming.tWenHigh )
  {
    pFlashConfig->devTiming.tWenHigh = TMVH_EFMC_DEV_TIMING1_MAX_CYCLES;
  }
  pFlashConfig->devTiming.tRenWidth = (UInt8) ( ( regVal & TMVH_EFMC_DEV_TIMING1_TRP_MSK ) >> TMVH_EFMC_DEV_TIMING1_TRP_POS );
  if( 0x00 == pFlashConfig->devTiming.tRenWidth )
  {
    pFlashConfig->devTiming.tRenWidth = TMVH_EFMC_DEV_TIMING1_MAX_CYCLES;
  }
  pFlashConfig->devTiming.tRenHigh = (UInt8) ( ( regVal & TMVH_EFMC_DEV_TIMING1_TRH_MSK ));
  if( 0x00 == pFlashConfig->devTiming.tRenHigh )
  {
    pFlashConfig->devTiming.tRenHigh = TMVH_EFMC_DEV_TIMING1_MAX_CYCLES;
  }
  return TM_OK;
}

/*!\fn tmErrorCode_t tmhwEfmc_SoftReset ( tmUnitSelect_t efmcUnitID);
*     This function provides a soft reset to the EFMC unit.
*     \param[in] efmcUnitID -> EFMC Unit Number
*     \return TM_OK - Successful
*/
tmErrorCode_t
tmhwEfmc_SoftReset (
    tmUnitSelect_t                  efmcUnitID
)
{
  UInt32              regs = 0 ;
  
  regs =  (UInt32) ( TMHW_EFMC_GET_BASE( efmcUnitID) + TMVH_EFMC_SWRESET_OFFSET );
  
  TMVH_GEN_WRITE( regs, TMVH_EFMC_SWRESET_SW_RESET_MSK );
  return TM_OK;
}



/*! \} */ /* end of Others */
#endif

