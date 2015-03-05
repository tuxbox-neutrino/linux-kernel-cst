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

#ifndef TMHWEFMC_VHIP_H
#define TMHWEFMC_VHIP_H

#if defined(__cplusplus)
extern "C"
{
#endif /* defined(__cplusplus) */

#ifndef TMVH_GEN_READ
#define TMVH_GEN_READ(_address_,_result_) {_result_ = readl(IOMEM(_address_));}
#endif

#ifndef TMVH_GEN_WRITE
#define TMVH_GEN_WRITE(_address_,_value_) writel(_value_,IOMEM(_address_))
#endif

/**---------------------------------------------------------------------------*/
/**
 * Register offsets from base address
 */
/**---------------------------------------------------------------------------*/
#define TMVH_EFMC_CMD_ADDR_FIFO_OFFSET          (0x000) /* EFMC Nand Flash Command / Address FIFO register */

#define TMVH_EFMC_PAGE_RW_OFFSET                (0x004) /* (Apollo) EFMC Nand Flash Page Read/Write register */
#define TMVH_EFMC_MODE_CTRL_OFFSET              (0x004) /* (Kronos) Determines the page read operation mode */

#define TMVH_EFMC_NAND_STATUS_OFFSET            (0x008) /* Nand Flash Status registers */
#define TMVH_EFMC_DMA_CTRL_OFFSET               (0x00C) /* DMA Control Register */
#define TMVH_EFMC_EBI_CTRL_OFFSET               (0x010) /* EBI Pin-Sharing Control Register*/

#define TMVH_EFMC_AHB_LATENCY_OFFSET            (0x014) /* (Reserved) AX/AHB Latency Register */
#define TMVH_EFMC_APB_LATENCY_OFFSET            (0x018) /* (Reserved) APB Latency Register*/
#define TMVH_EFMC_APB_ERROR_GEN_OFFSET          (0x01C) /* (Reserved) APB Error Register */
#define TMVH_EFMC_AES_KEY1_OFFSET               (0x020) /* (Reserved) AES Key Register(1st 32bit) */
#define TMVH_EFMC_AES_KEY2_OFFSET               (0x024) /* (Reserved) AES Key Register(2nd 32bit) */
#define TMVH_EFMC_AES_KEY3_OFFSET               (0x028) /* (Reserved) AES Key Register (3rd 32bit) */
#define TMVH_EFMC_AES_KEY4_OFFSET               (0x02C) /* (Reserved) AES Key Register (4th 32bit) */
#define TMVH_EFMC_AES_IV1_OFFSET                (0x030) /* (Reserved) AES Initial Value Register (1st 32bit)*/
#define TMVH_EFMC_AES_IV2_OFFSET                (0x034) /* (Reserved) AES Initial Value Register (2nd32bit) */
#define TMVH_EFMC_AES_IV3_OFFSET                (0x038) /* (Reserved) AES Initial Value Register (3rd 32bit) */
#define TMVH_EFMC_AES_IV4_OFFSET                (0x03C) /* (Reserved) AES Initial Value Register (4th 32bit) */
#define TMVH_EFMC_AES_STATUS_OFFSET             (0x040) /* (Reserved) AES Status Register */

#define TMVH_EFMC_SINGLE_READ_OFFSET            (0x044) /* Nand Flash SInge Read Register*/
#define TMVH_EFMC_SINGLE_WRITE_OFFSET           (0x048) /* Nand Flash SInge Write Register */
#define TMVH_EFMC_DEV_TYPE0_OFFSET              (0x04C) /* Nand Flash Device Type Register for CEn[n]*/
#define TMVH_EFMC_DEV_TYPE0_DIFF                (0x00C) /* */
#define TMVH_EFMC_DEV_TIMING0_OFFSET            (0x050) /* Nand Flash Device Timing Register #0 for CEn[n]  */
#define TMVH_EFMC_DEV_TIMING0_DIFF              (0x00C) /* */
#define TMVH_EFMC_DEV_TIMING1_OFFSET            (0x054) /* Nand Flash Device Timing Register #1 for CEn[n] */
#define TMVH_EFMC_DEV_TIMING1_DIFF              (0x00C) /* */

#define TMVH_EFMC_OOB_PARTITION_N_OFFSET        (0x07C) /* (Kronos) OOB Partitioning Register */
#define TMVH_EFMC_PAGE_RD_STATUS_FIFO_OFFSET    (0x080) /* (Kronos) Page read status fifo Register*/
#define TMVH_EFMC_PAGE_WR_STATUS_FIFO_OFFSET    (0x084) /* (Kronos) Page write status fifo Register */
#define TMVH_EFMC_PAGE_TAG_OFFSET               (0x088) /* (Kronos) Page Tag Register */
#define TMVH_EFMC_FIFO_STATUS_OFFSET            (0x08C) /* (Kronos) FIFO Status Register*/
#define TMVH_EFMC_STATUS_FIFO_POP_OFFSET        (0x090) /* (Kronos) PAGE RD/WR STATUS status fifo pop Register */
#define TMVH_EFMC_STATUS_FIFO_MODE_OFFSET       (0x094) /* (Kronos) FIFO mode selection Register */
#define TMVH_EFMC_CMD_ADDR_FIFO_DATA_OFFSET     (0x098) /* (Kronos) Command address fifo contents register */
#define TMVH_EFMC_CMD_ADDR_FIFO_DATA_DIFF       (0x004) /* (Kronos) */

#define TMVH_EFMC_MOD_CONFIG_OFFSET             (0xFD4) /* Module Configuration Register */
#define TMVH_EFMC_INT_CLR_ENABLE_OFFSET         (0xFD8) /* Interrupt Clear Enable Register */
#define TMVH_EFMC_INT_SET_ENABLE_OFFSET         (0xFDC) /* Interrupt Set Enable Register */
#define TMVH_EFMC_INT_STATUS_OFFSET             (0xFE0) /* Interrupt Status Register */
#define TMVH_EFMC_INT_ENABLE_OFFSET             (0xFE4) /* Interrupt Enable Register */
#define TMVH_EFMC_INT_CLR_STATUS_OFFSET         (0xFE8) /* Interrupt Clear Status Set Register */
#define TMVH_EFMC_INT_SET_STATUS_OFFSET         (0xFEC) /* Interrupt Set Status Set Register */
#define TMVH_EFMC_SWRESET_OFFSET                (0xFF0) /* Software reset register */
#define TMVH_EFMC_POWER_DOWN_OFFSET             (0xFF4) /* Power down register */
#define TMVH_EFMC_EXT_MODID_OFFSET              (0xFF8) /* Dynamic memory write recovery time register */
#define TMVH_EFMC_MODID_OFFSET                  (0xFFC) /* Dynamic memory write recovery time register */

#define TMVH_EFMC_MAX_AHB_MAIN_MEM_SIZE         (0x200)

/**---------------------------------------------------------------------------*/
/**
 * Register details
 */
/**---------------------------------------------------------------------------*/

/**
 * (Apollo, Kronos) CMD_ADDR_FIFO register
 */
/* -- Kronos specific definitions starts -- */
#define TMVH_EFMC_CMD_ADDR_FIFO_IMMEDIATE_MSK   (0x04000000)
#define TMVH_EFMC_CMD_ADDR_FIFO_IMMEDIATE_POS   (26)

#define TMVH_EFMC_CMD_ADDR_FIFO_OOB_MSK         (0x02000000)
#define TMVH_EFMC_CMD_ADDR_FIFO_OOB_POS         (25)

#define TMVH_EFMC_CMD_ADDR_FIFO_ECC_MSK         (0x01000000)
#define TMVH_EFMC_CMD_ADDR_FIFO_ECC_POS         (24)

#define TMVH_EFMC_CMD_ADDR_FIFO_PAGE_XFER_MSK   (0x00800000)
#define TMVH_EFMC_CMD_ADDR_FIFO_PAGE_XFER_POS   (23)

#define TMVH_EFMC_CMD_ADDR_FIFO_DIR_MSK         (0x00400000)
#define TMVH_EFMC_CMD_ADDR_FIFO_DIR_POS         (22)
/* -- Kronos specific definitions ends -- */

#define TMVH_EFMC_CMD_ADDR_FIFO_CEN_MSK         (0x00380000)
#define TMVH_EFMC_CMD_ADDR_FIFO_CEN_POS         (19)

#define TMVH_EFMC_CMD_ADDR_FIFO_LAST_CYCLE_MSK  (0x00040000)
#define TMVH_EFMC_CMD_ADDR_FIFO_LAST_CYCLE_POS  (18)

#define TMVH_EFMC_CMD_ADDR_FIFO_CYCLE_TYPE_MSK  (0x00030000)
#define TMVH_EFMC_CMD_ADDR_FIFO_CYCLE_TYPE_POS  (16)

#define TMVH_EFMC_CMD_ADDR_FIFO_IO_MSK          (0x0000FFFF)
#define TMVH_EFMC_CMD_ADDR_FIFO_IO_POS          (0)

/**
 * (Kronos) MODE_CTRL register
 */
#define TMVH_EFMC_MODE_CTRL_RBN_BYPASS_MSK      (0x00000002)
#define TMVH_EFMC_MODE_CTRL_RBN_BYPASS_POS      (1)

#define TMVH_EFMC_MODE_CTRL_RD_STALL_MSK        (0x00000001)
#define TMVH_EFMC_MODE_CTRL_RD_STALL_POS        (0)

/**
 * (Apollo) PAGE_RW register
 */
#define TMVH_EFMC_PAGE_RW_OOB_MSK               (0x00000010)
#define TMVH_EFMC_PAGE_RW_OOB_POS               (4)

#define TMVH_EFMC_PAGE_RW_AES_MSK               (0x00000008)
#define TMVH_EFMC_PAGE_RW_AES_POS               (3)

#define TMVH_EFMC_PAGE_RW_ECC_MSK               (0x00000004)
#define TMVH_EFMC_PAGE_RW_ECC_POS               (2)

#define TMVH_EFMC_PAGE_RW_WRITE_PAGE_MSK        (0x00000002)
#define TMVH_EFMC_PAGE_RW_WRITE_PAGE_POS        (1)

#define TMVH_EFMC_PAGE_RW_READ_PAGE_MSK         (0x00000001)
#define TMVH_EFMC_PAGE_RW_READ_PAGE_POS         (0)

/**
 * (Apollo, Kronos) NAND_STATUS register
 */
#define TMVH_EFMC_NAND_STATUS_RBN_RISE_MSK      (0x00000F00)
#define TMVH_EFMC_NAND_STATUS_RBN_RISE_POS      (8)

#define TMVH_EFMC_NAND_STATUS_RBN_FALL_MSK      (0x000000F0)
#define TMVH_EFMC_NAND_STATUS_RBN_FALL_POS      (4)

#define TMVH_EFMC_NAND_STATUS_RBN_MSK           (0x0000000F)
#define TMVH_EFMC_NAND_STATUS_RBN_POS           (0)

/**
 * (Apollo, Kronos) DMA_CTRL register
 */
#define TMVH_EFMC_DMA_CTRL_MEM_2_DMA_MSK        (0x00000002)
#define TMVH_EFMC_DMA_CTRL_MEM_2_DMA_POS        (1)

#define TMVH_EFMC_DMA_CTRL_DMA_2_MEM_MSK        (0x00000001)
#define TMVH_EFMC_DMA_CTRL_DMA_2_MEM_POS        (0)

/**
 * (Apollo, Kronos) EBI_CTRL register
 */
#define TMVH_EFMC_EBI_CTRL_TIMEOUT_MSK          (0x000003FC)
#define TMVH_EFMC_EBI_CTRL_TIMEOUT_POS          (2)

#define TMVH_EFMC_EBI_CTRL_MODE_MSK             (0x00000002)
#define TMVH_EFMC_EBI_CTRL_MODE_POS             (1)

#define TMVH_EFMC_EBI_CTRL_DEF_REQ_MSK          (0x00000001)
#define TMVH_EFMC_EBI_CTRL_DEF_REQ_POS          (0)

/**
 * (Reserved) AHB_LATENCY register
 */
#define TMVH_EFMC_AHB_LATENCY_CMD_MSK           (0x00000002)
#define TMVH_EFMC_AHB_LATENCY_CMD_POS           (1)

#define TMVH_EFMC_AHB_LATENCY_RD_DATA_MSK       (0x00000001)
#define TMVH_EFMC_AHB_LATENCY_RD_DATA_POS       (0)

/**
 * (Reserved) APB_LATENCY register
 */
#define TMVH_EFMC_APB_LATENCY_WS_MSK            (0x00000001)
#define TMVH_EFMC_APB_LATENCY_WS_POS            (0)

/**
 * (Reserved) APB_ERROR_GEN register
 */
#define TMVH_EFMC_APB_ERROR_GEN_WR_MSK          (0x00000002)
#define TMVH_EFMC_APB_ERROR_GEN_WR_POS          (1)

#define TMVH_EFMC_APB_ERROR_GEN_RD_MSK          (0x00000001)
#define TMVH_EFMC_APB_ERROR_GEN_RD_POS          (0)

/**
 * (Reserved) AES_KEY1 register
 */
#define TMVH_EFMC_AES_KEY1_AES_KEY_MSK          (0xFFFFFFFFU)
#define TMVH_EFMC_AES_KEY1_AES_KEY_POS          (0)

/**
 * (Reserved) AES_KEY2 register
 */
#define TMVH_EFMC_AES_KEY2_AES_KEY_MSK          (0xFFFFFFFFU)
#define TMVH_EFMC_AES_KEY2_AES_KEY_POS          (0)

/**
 * (Reserved) AES_KEY3 register
 */
#define TMVH_EFMC_AES_KEY3_AES_KEY_MSK          (0xFFFFFFFFU)
#define TMVH_EFMC_AES_KEY3_AES_KEY_POS          (0)

/**
 * (Reserved) AES_KEY4 register
 */
#define TMVH_EFMC_AES_KEY4_AES_KEY_MSK          (0xFFFFFFFFU)
#define TMVH_EFMC_AES_KEY4_AES_KEY_POS          (0)

/**
 * (Reserved) AES_IV1 register
 */
#define TMVH_EFMC_AES_IV1_AES_IV_MSK            (0xFFFFFFFFU)
#define TMVH_EFMC_AES_IV1_AES_IV_POS            (0)

/**
 * (Reserved) AES_IV2 register
 */
#define TMVH_EFMC_AES_IV2_AES_IV_MSK            (0xFFFFFFFFU)
#define TMVH_EFMC_AES_IV2_AES_IV_POS            (0)

/**
 * (Reserved) AES_IV3 register
 */
#define TMVH_EFMC_AES_IV3_AES_IV_MSK            (0xFFFFFFFFU)
#define TMVH_EFMC_AES_IV3_AES_IV_POS            (0)

/**
 * (Reserved) AES_IV4 register
 */
#define TMVH_EFMC_AES_IV4_AES_IV_MSK            (0xFFFFFFFFU)
#define TMVH_EFMC_AES_IV4_AES_IV_POS            (0)

/**
 * (Reserved) AES_STATUS register
 */
#define TMVH_EFMC_AES_STATUS_ACCEPT_KEY_MSK     (0x00000002)
#define TMVH_EFMC_AES_STATUS_ACCEPT_KEY_POS     (1)

#define TMVH_EFMC_AES_STATUS_ACCEPT_IN_MSK      (0x00000001)
#define TMVH_EFMC_AES_STATUS_ACCEPT_IN_POS      (0)

/**
 * (Apollo, Kronos) SINGLE_READ register
 */
#define TMVH_EFMC_SINGLE_READ_DATA_MSK          (0x0000FFFF)
#define TMVH_EFMC_SINGLE_READ_DATA_POS          (0)

/**
 * (Apollo, Kronos) SINGLE_WRITE register
 */
#define TMVH_EFMC_SINGLE_WRITE_DATA_MSK         (0x0000FFFF)
#define TMVH_EFMC_SINGLE_WRITE_DATA_POS         (0)

/**
 * (Apollo, Kronos) DEV_TYPE0 register
 */
/* -- Kronos specific definitions starts -- */
#define TMVH_EFMC_DEV_TYPE0_ERASED_PAGE_THRES_MSK  (0xFC000000)
#define TMVH_EFMC_DEV_TYPE0_ERASED_PAGE_THRES_POS  (26)

#define TMVH_EFMC_DEV_TYPE0_ECC_LEVEL_MSK       (0x03F00000)
#define TMVH_EFMC_DEV_TYPE0_ECC_LEVEL_POS       (20)

#define TMVH_EFMC_DEV_TYPE0_OOB_SIZE_MSK        (0x000FFF00)
#define TMVH_EFMC_DEV_TYPE0_OOB_SIZE_POS        (8)

#define TMVH_EFMC_DEV_TYPE0_SUB_PAGE_SIZE_MSK   (0x00000080)
#define TMVH_EFMC_DEV_TYPE0_SUB_PAGE_SIZE_POS   (7)
/* -- Kronos specific definitions ends -- */

#define TMVH_EFMC_DEV_TYPE0_PAGE_SIZE_MSK       (0x00000070)
#define TMVH_EFMC_DEV_TYPE0_PAGE_SIZE_POS       (4)

#define TMVH_EFMC_DEV_TYPE0_WP_MSK              (0x00000008)
#define TMVH_EFMC_DEV_TYPE0_WP_POS              (3)

#define TMVH_EFMC_DEV_TYPE0_CEN_DONT_MSK        (0x00000004)
#define TMVH_EFMC_DEV_TYPE0_CEN_DONT_POS        (2)

#define TMVH_EFMC_DEV_TYPE0_DATA_WIDTH_MSK      (0x00000003)
#define TMVH_EFMC_DEV_TYPE0_DATA_WIDTH_POS      (0)

/**
 * (Apollo, Kronos) DEV_TIMING0 register
 */
#define TMVH_EFMC_DEV_TIMING0_TCPS_MSK          (0x0F000000)
#define TMVH_EFMC_DEV_TIMING0_TCPS_POS          (24)

#define TMVH_EFMC_DEV_TIMING0_TCPH_MSK          (0x00F00000)
#define TMVH_EFMC_DEV_TIMING0_TCPH_POS          (20)

#define TMVH_EFMC_DEV_TIMING0_TCLS_MSK          (0x000F0000)
#define TMVH_EFMC_DEV_TIMING0_TCLS_POS          (16)

#define TMVH_EFMC_DEV_TIMING0_TCLH_MSK          (0x0000F000)
#define TMVH_EFMC_DEV_TIMING0_TCLH_POS          (12)

#define TMVH_EFMC_DEV_TIMING0_TALS_MSK          (0x00000F00)
#define TMVH_EFMC_DEV_TIMING0_TALS_POS          (8)

#define TMVH_EFMC_DEV_TIMING0_TALH_MSK          (0x000000F0)
#define TMVH_EFMC_DEV_TIMING0_TALH_POS          (4)

#define TMVH_EFMC_DEV_TIMING0_TRR_MSK           (0x0000000F)
#define TMVH_EFMC_DEV_TIMING0_TRR_POS           (0)
#define TMVH_EFMC_DEV_TIMING0_MAX_CYCLES        (16)

/**
 * (Apollo, Kronos) DEV_TIMING1 register
 */
#define TMVH_EFMC_DEV_TIMING1_TDRD_MSK          (0x03C00000)
#define TMVH_EFMC_DEV_TIMING1_TDRD_POS          (22)

#define TMVH_EFMC_DEV_TIMING1_TWB_MSK           (0x003F0000)
#define TMVH_EFMC_DEV_TIMING1_TWB_POS           (16)
#define TMVH_EFMC_DEV_TIMING1_TWB_MAX_CYCLES    (64)

#define TMVH_EFMC_DEV_TIMING1_TWP_MSK           (0x0000F000)
#define TMVH_EFMC_DEV_TIMING1_TWP_POS           (12)

#define TMVH_EFMC_DEV_TIMING1_TWH_MSK           (0x00000F00)
#define TMVH_EFMC_DEV_TIMING1_TWH_POS           (8)

#define TMVH_EFMC_DEV_TIMING1_TRP_MSK           (0x000000F0)
#define TMVH_EFMC_DEV_TIMING1_TRP_POS           (4)

#define TMVH_EFMC_DEV_TIMING1_TRH_MSK           (0x0000000F)
#define TMVH_EFMC_DEV_TIMING1_TRH_POS           (0)
#define TMVH_EFMC_DEV_TIMING1_MAX_CYCLES        (16)

/**
 * (Kronos) OOB_PARTITION_N register
 */
#define TMVH_EFMC_OOB_PARTITION_N_EXTENDED_OOB_MSK       (0xFFFF0000)
#define TMVH_EFMC_OOB_PARTITION_N_EXTENDED_OOB_POS       (16)

#define TMVH_EFMC_OOB_PARTITION_N_UNPROTECTED_BYTES_MSK  (0x0000C000)
#define TMVH_EFMC_OOB_PARTITION_N_UNPROTECTED_BYTES_POS  (14)

#define TMVH_EFMC_OOB_PARTITION_N_PARITY_BYTES_MSK       (0x00003F00)
#define TMVH_EFMC_OOB_PARTITION_N_PARITY_BYTES_POS       (8)

#define TMVH_EFMC_OOB_PARTITION_N_PROTECTED_OOB_MIN_MSK  (0x000000FF)
#define TMVH_EFMC_OOB_PARTITION_N_PROTECTED_OOB_MIN_POS  (0)

/**
 * (Kronos) PAGE_RD_STATUS_FIFO register
 */
#define TMVH_EFMC_PAGE_RD_STATUS_FIFO_TAG_MSK            (0xFFFF0000)
#define TMVH_EFMC_PAGE_RD_STATUS_FIFO_TAG_POS            (16)

#define TMVH_EFMC_PAGE_RD_STATUS_FIFO_ERASED_MSK         (0x00000400)
#define TMVH_EFMC_PAGE_RD_STATUS_FIFO_ERASED_POS         (10)

#define TMVH_EFMC_PAGE_RD_STATUS_FIFO_ECC_OFF_MSK        (0x00000200)
#define TMVH_EFMC_PAGE_RD_STATUS_FIFO_ECC_OFF_POS        (9)

#define TMVH_EFMC_PAGE_RD_STATUS_FIFO_UNCORRECTABLE_MSK  (0x00000100)
#define TMVH_EFMC_PAGE_RD_STATUS_FIFO_UNCORRECTABLE_POS  (8)

#define TMVH_EFMC_PAGE_RD_STATUS_FIFO_CORRECTED_MSK      (0x00000080)
#define TMVH_EFMC_PAGE_RD_STATUS_FIFO_CORRECTED_POS      (7)

#define TMVH_EFMC_PAGE_RD_STATUS_FIFO_ERROR_FREE_MSK     (0x00000040)
#define TMVH_EFMC_PAGE_RD_STATUS_FIFO_ERROR_FREE_POS     (6)

#define TMVH_EFMC_PAGE_RD_STATUS_FIFO_NUM_ERROR_MSK      (0x0000003F)
#define TMVH_EFMC_PAGE_RD_STATUS_FIFO_NUM_ERROR_POS      (0)

/**
 * (Kronos) PAGE_WR_STATUS_FIFO register
 */
#define TMVH_EFMC_PAGE_WR_STATUS_FIFO_TAG_MSK            (0xFFFF0000)
#define TMVH_EFMC_PAGE_WR_STATUS_FIFO_TAG_POS            (16)

/**
 * (Kronos) PAGE_TAG register
 */
#define TMVH_EFMC_PAGE_TAG_TAG_MSK              (0x000000FF)
#define TMVH_EFMC_PAGE_TAG_TAG_POS              (0)

/**
 * (Kronos) FIFO_STATUS register
 */
#define TMVH_EFMC_FIFO_PAGE_RD_STATUS_LEVEL_MSK (0x0000FF00)
#define TMVH_EFMC_FIFO_PAGE_RD_STATUS_LEVEL_POS (8)
#define TMVH_EFMC_FIFO_PAGE_WR_STATUS_LEVEL_MSK (0x00FF0000)
#define TMVH_EFMC_FIFO_PAGE_WR_STATUS_LEVEL_POS (16)

/**
 * (Kronos) STATUS_FIFO_POP register
 */
#define TMVH_EFMC_STATUS_FIFO_POP_MSK (0x00000001)
#define TMVH_EFMC_STATUS_FIFO_POP_POS (0)

/**
 * (Kronos) STATUS_FIFO_MODE register
 */
#define TMVH_EFMC_STATUS_FIFO_MODE_MSK (0x00000001)
#define TMVH_EFMC_STATUS_FIFO_MODE_POS (0)

/**
 * (Kronos) CMD_ADDR_FIFO register
 */
#define TMVH_EFMC_CMD_ADDR_FIFO_MSK (0xFFFFFFFF)
#define TMVH_EFMC_CMD_ADDR_FIFO_POS (0)

/**
 * (Apollo, Kronos) MOD_CONFIG register
 */
#define TMVH_EFMC_MOD_CONFIG_APB_LAT_MSK        (0x00000400)
#define TMVH_EFMC_MOD_CONFIG_APB_LAT_POS        (10)

#define TMVH_EFMC_MOD_CONFIG_APB_ERR_MSK        (0x00000200)
#define TMVH_EFMC_MOD_CONFIG_APB_ERR_POS        (9)

#define TMVH_EFMC_MOD_CONFIG_DATA_BUS_TYPE_MSK  (0x00000180)
#define TMVH_EFMC_MOD_CONFIG_DATA_BUS_TYPE_POS  (7)

#define TMVH_EFMC_MOD_CONFIG_DATA_BUS_LAT_MSK   (0x00000040)
#define TMVH_EFMC_MOD_CONFIG_DATA_BUS_LAT_POS   (6)

#define TMVH_EFMC_MOD_CONFIG_AES_MSK            (0x00000020)
#define TMVH_EFMC_MOD_CONFIG_AES_POS            (5)

#define TMVH_EFMC_MOD_CONFIG_EBI_PIN_MSK        (0x00000010)
#define TMVH_EFMC_MOD_CONFIG_EBI_PIN_POS        (4)

#define TMVH_EFMC_MOD_CONFIG_BOOT_NAND_MSK      (0x00000008)
#define TMVH_EFMC_MOD_CONFIG_BOOT_NAND_POS      (3)

#define TMVH_EFMC_MOD_CONFIG_RST_SYNC_MSK       (0x00000004)
#define TMVH_EFMC_MOD_CONFIG_RST_SYNC_POS       (2)

#define TMVH_EFMC_MOD_CONFIG_NUM_DEV_MSK        (0x00000003)
#define TMVH_EFMC_MOD_CONFIG_NUM_DEV_POS        (0)

/**
 * (Apollo, Kronos) INT registers
 */
#define TMVH_EFMC_INT_MSK                       (0x00FFFFFF)
#define TMVH_EFMC_INT_POS                       (0)

/**
 * (Apollo, Kronos) SWRESET registers
 */
#define TMVH_EFMC_SWRESET_SW_RESET_MSK          (0x00000001)
#define TMVH_EFMC_SWRESET_SW_RESET_POS          (0)

/**
 * (Apollo, Kronos) POWER_DOWN registers
 */
#define TMVH_EFMC_POWER_DOWN_TIMEOUT_MSK        (0x0001FFFE)
#define TMVH_EFMC_POWER_DOWN_TIMEOUT_POS        (1)

#define TMVH_EFMC_POWER_DOWN_POWER_DOWN_MSK     (0x00000001)
#define TMVH_EFMC_POWER_DOWN_POWER_DOWN_POS     (0)

/**
 * (Apollo, Kronos) MODID register
 */
#define TMVH_EFMC_MODID_MODID_MSK               (0xFFFF0000U)
#define TMVH_EFMC_MODID_MODID_POS               (16)

#define TMVH_EFMC_MODID_MAJOR_VER_MSK           (0x0000F000)
#define TMVH_EFMC_MODID_MAJOR_VER_POS           (12)

#define TMVH_EFMC_MODID_MINOR_VER_MSK           (0x00000F00)
#define TMVH_EFMC_MODID_MINOR_VER_POS           (8)

#define TMVH_EFMC_MODID_APERTURE_MSK            (0x000000FF)
#define TMVH_EFMC_MODID_APERTURE_POS            (0)

/**
 * (Reserved) EXT_MODID register
 */
#define TMVH_EFMC_EXT_MODID_MODID_MSK           (0xFFFF0000U)
#define TMVH_EFMC_EXT_MODID_MODID_POS           (16)

#define TMVH_EFMC_EXT_MODID_MAJOR_VER_MSK       (0x0000F000)
#define TMVH_EFMC_EXT_MODID_MAJOR_VER_POS       (12)

#define TMVH_EFMC_EXT_MODID_MINOR_VER_MSK       (0x00000F00)
#define TMVH_EFMC_EXT_MODID_MINOR_VER_POS       (8)

#define TMVH_EFMC_EXT_MODID_APERTURE_MSK        (0x000000FF)
#define TMVH_EFMC_EXT_MODID_APERTURE_POS        (0)

#if defined(__cplusplus)
}
#endif /* defined(__cplusplus) */
#endif /* TMHWEFMC_VHIP_H*/
