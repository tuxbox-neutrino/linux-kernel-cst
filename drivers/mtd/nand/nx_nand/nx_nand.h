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

#ifndef _NX_NAND_H
#define _NX_NAND_H

#ifdef __KERNEL__

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#if defined(CONFIG_MTD_NX_NAND_DMAC) && defined (CONFIG_PLAT_STB)
#define NX_NAND_AHB_BUF     (0x40000000) /* DMAC Flow conttrol */
#elif defined(CONFIG_MTD_NX_NAND_DMAC) && !defined(CONFIG_ARCH_APOLLO)
#define NX_NAND_AHB_BUF    (0x00000000) /* DMAC Flow conttrol */
#else
#define NX_NAND_AHB_BUF    (NX_NAND_AHB_INTFC_BUF) /* Interrupt Flow control */
#endif

/**
* NAND control structure
*/
struct nx_nand_ctrl {
   struct mtd_info      mtd;           /* MTD information structure        */
   struct nand_chip     chip;          /* NAND chip structure              */
   struct nand_hw_control  nandctrl;   /* NAND control lock                */
   uint8_t              *dmabuf;       /* DMA buffer                       */
   dma_addr_t           dmabuf_phy;    /* DMA buffer PHY address           */
   int                  slots;         /* # of chips                       */
   int                  slotid;        /* current chip number              */
   uint32_t             slotbase;      /* Chip start address               */
   int                  unitid;        /* Current unit ID of controller    */
   int                  cur_col;       /* Current column address           */
   int                  cur_page;      /* Current page address             */
   int                  cur_cmd;       /* Current page address             */
   int                  lb_chip;       /* Large Block chip flag            */
   void __iomem         *ctrl_base;    /* Controller base address          */
   void __iomem         *ahb_buf;      /* Controller base address          */
   bool                 aes;           /* AES decryption support           */
   bool                 hwecc;         /* Use HW ECC                       */
   uint32_t             aes_key[4];    /* AES 128 bit key                  */
   uint32_t             aes_val[4];    /* AES initial 128 bit value        */
   int                  blk_size;      /* Block size                       */
   int                  num_blks;      /* # of blocks per page             */
   int                  blk_index;     /* Block index                      */
   int                  *ecc_status;   /* ECC status array                 */
   bool                 done;          /* Read, write, erase done flag     */
   wait_queue_head_t    nand_queue;    /* NAND queue                       */
   int                  offset;        /* Offset in inetrnal driver buffer */
   bool                 cedontcare;    /* CE dont care support             */
   tmhwEfmc_PageRWStatusFifo_t page_rw_status_fifo;
   tmhwEfmc_FlashConfig_t flash_config;
};

/* Max Data buffer size */
#if (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
#define NX_NAND_BLK_SIZE   (1024)
#else 
#define NX_NAND_BLK_SIZE   (512)
#endif

/* Max OOB buffer size */
#define NX_NAND_MAX_OOB_SIZE  (128)

/* Maximum AHB buffer size */
#define NX_NAND_AHB_BUF_MAX_SIZE (NX_NAND_BLK_SIZE+NX_NAND_MAX_OOB_SIZE)

/* NAND CONTROLLER register definitions */
#define NX_NAND_INT_ENA_OFFSET   (0xFE4)

#define NX_NAND_INT_READY_START  (20)

/* Interrupt status bits */
#define NX_NAND_INT_OOB_READ                 (1<<0)
#define NX_NAND_INT_OOB_WRITE                (1<<1)
#define NX_NAND_INT_BLK_READ                 (1<<2)
#define NX_NAND_INT_BLK_WRITE                (1<<3)
#define NX_NAND_INT_ENC                      (1<<4)
#define NX_NAND_INT_DEC                      (1<<5)
#define NX_NAND_INT_DEC_0_ERR                (1<<6)
#if defined(CONFIG_ARCH_APOLLO)
#define NX_NAND_INT_DEC_1_ERR                (1<<7)
#define NX_NAND_INT_DEC_2_ERR                (1<<8)
#define NX_NAND_INT_DEC_3_ERR                (1<<9)
#define NX_NAND_INT_DEC_4_ERR                (1<<10)
#define NX_NAND_INT_DEC_5_ERR                (1<<11)
#define NX_NAND_INT_DEC_UNCOR                (1<<12)
#define NX_NAND_INT_AES_DEC                  (1<<13)
#elif (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
#define NX_NAND_INT_DEC_ERR_CORR             (1<<7)
#define NX_NAND_INT_PAGE_READ_STS_FIFO_FULL  (1<<9)
#define NX_NAND_INT_PAGE_WRITE_STS_FIFO_FULL (1<<10)
#define NX_NAND_INT_CMD_ADR_FIFO_EMPTY       (1<<11)
#define NX_NAND_INT_SUBPAGE_UNCOR            (1<<12)
#endif
#define NX_NAND_INT_SEQ_READ                 (1<<14)
#define NX_NAND_INT_SEQ_WRITE                (1<<15)
#define NX_NAND_INT_BUSY1                    (1<<16)
#define NX_NAND_INT_BUSY2                    (1<<17)
#define NX_NAND_INT_BUSY3                    (1<<18)
#define NX_NAND_INT_BUSY4                    (1<<19)
#define NX_NAND_INT_READY1                   (1<<20)
#define NX_NAND_INT_READY2                   (1<<21)
#define NX_NAND_INT_READY3                   (1<<22)
#define NX_NAND_INT_READY4                   (1<<23)

/* Position of address */
#define NX_NAND_SP_ADDR_MASK  (0xFF)
#define NX_NAND_SP_ADDR_MASK1 (0x03)

#define NX_NAND_SP_ADDR1_POS  (0)
#define NX_NAND_SP_ADDR2_POS  (9)
#define NX_NAND_SP_ADDR3_POS  (17)
#define NX_NAND_SP_ADDR4_POS  (29)

/* Cmd FIFO bit information */
#define NX_NAND_CMD_FIFO_CE_START   (19)
#define NX_NAND_CMD_FIFO_ADDR_CYC   (0x0)
#define NX_NAND_CMD_FIFO_CMD_CYC    (0x1)
#define NX_NAND_CMD_FIFO_POST_CMD   (0x2)

#define NX_NAND_POWER_DOWN_CFG_OFFSET  (0xFF4)
#define NX_NAND_POWER_DOWN_MASK        (0x00000001)
#define NX_NAND_POWER_DOWN_ENABLE      (1UL<<0)
#define NX_NAND_POWER_DOWN_DISABLE     (0UL<<0)

#endif /* __KERNEL__ */
#endif /* _NX_NAND_H */
