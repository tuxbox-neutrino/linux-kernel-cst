/*
 * Copyright (C) 2014, Entropic Communications. All Rights Reserved
 * Author: Srinivas Rao L <srinivas.rao@entropic.com>
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

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/sched.h>
#include <mach/gcs.h>
#include "nx_sfc.h"

#define FLASH_PAGESIZE        	256
#define SFC_PREFETCH_BUFSIZE	64
#define FLASH_READ_SIZE		4092

#define SFC_DCSN_SIZE		0x8000000

/* Flash opcodes. */
#define  OPCODE_WREN       	0x06  /* Write enable */
#define  OPCODE_RDSR       	0x05  /* Read status register */
#define  OPCODE_WRSR       	0x01  /* Write status register 1 byte */
#define  OPCODE_NORM_READ  	0x03  /* Read data bytes (low frequency) */
#define  OPCODE_FAST_READ  	0x0b  /* Read data bytes (high frequency) */
#define  OPCODE_PP         	0x02  /* Page program (up to 256 bytes) */
#define  OPCODE_BE_4K      	0x20  /* Erase 4KiB block */
#define  OPCODE_BE_32K     	0x52  /* Erase 32KiB block */
#define  OPCODE_CHIP_ERASE 	0xc7  /* Erase whole flash chip */
#define  OPCODE_SE         	0xd8  /* Sector erase (usually 64KiB) */
#define  OPCODE_RDID       	0x9f  /* Read JEDEC ID */
#ifdef CONFIG_MTD_NX_SFC_QUAD_MODE
#define OPCODE_QUADIO_READ 	0xeb  /* fast read quad I/O */
#endif

/* Status Register bits. */
#define  SR_WIP         1  /* Write in progress */
#define  SR_WEL         2  /* Write enable latch */
/* meaning of other SR_* bits may differ between vendors */
#define  SR_BP0         4  /* Block protect 0 */
#define  SR_BP1         8  /* Block protect 1 */
#define  SR_BP2         0x10  /* Block protect 2 */
#define  SR_SRWD        0x80  /* SR write protect */

/* Define max times to check status register before we give up. */
#define	MAX_READY_WAIT_JIFFIES	(10 * HZ) 
#define	MAX_READY_WAIT_CHIP_ERASE_JIFFIES	(400 * HZ)	/* MX25L25635E specs 400s max chip erase */

#define  MACRONIX_ID_MIN   0xc22016
#define  MACRONIX_ID_MAX   0xc22619


#define  MICRON_ID_MIN     0x20ba17
#define  MICRON_ID_MAX     0x20ba19

#define  MAN_ID_MACRONIX   0xc2
#define  MAN_ID_SPANSION   0x01
#define  MAN_ID_MICRON     0x20

#define  CMD_SIZE    4

#ifdef CONFIG_MTD_NX_SFC_QUAD_MODE
#define OPCODE_READ  OPCODE_QUADIO_READ
#define FAST_READ_DUMMY_BYTE 1
#else
#define OPCODE_READ  OPCODE_NORM_READ
#define FAST_READ_DUMMY_BYTE 0
#endif

#ifdef CONFIG_MTD_NX_SFC_DMAC_EN
extern void nx_sfc_map_copy_to (unsigned long to, const void *from, ssize_t 
len);
extern void nx_sfc_map_copy_from(void *to, unsigned long from, ssize_t len);
extern int nx_sfc_dma_init(struct platform_device *pdev);
extern void nx_sfc_dma_exit(struct platform_device *pdev);
#endif
/****************************************************************************/

static inline struct nx_sfc_mtd *mtd_to_nx_sfc_mtd(struct mtd_info *mtd)
{
   return container_of(mtd, struct nx_sfc_mtd, mtd);
}

/****************************************************************************/
/* IP2070 GCS: SFC Routines                                                 */
/****************************************************************************/

/*
  * for fast read quad I/O operations
  */
#ifdef CONFIG_MTD_NX_SFC_QUAD_MODE
ssize_t nx_sfc_read_quad(struct nx_sfc_mtd *flash)
{
   u32 round_len =0;
   unsigned long pinconfig, prtdelay,cmd;
#ifdef CONFIG_MTD_NX_SFC_DMAC_EN
   unsigned long dma_from, dma_sfc_base;
#else
   u32 cnt;
   unsigned long *buf_base, buf_store;
   u8 *buf_ptr =flash->cd.buffer;
   u8 *mem_buf_ptr = (u8*)&buf_store;
#endif

#ifdef CONFIG_MTD_NX_SFC_DMAC_EN
   dma_sfc_base =  readl(NX_DMA_SFC_BASE_ADDR_REG);
   dma_from =  dma_sfc_base +flash->cd.offset;
#else
   buf_base = (( unsigned long *)(flash->io_base+flash->cd.offset));
#endif

   if((flash->info.jedec_id >= MACRONIX_ID_MIN)&&(flash->info.jedec_id <= MACRONIX_ID_MAX))
   {
      pinconfig = SFC_PIN_CONFIG_QUAD_READ_MX;
      prtdelay =  SFC_PROTCOL_DELAY_QUADRD_MX;
      cmd = SFC_DEV_CMD_QUAD_READ_MX;
   }
   else if ((flash->info.jedec_id >= MICRON_ID_MIN)&&(flash->info.jedec_id <= MICRON_ID_MAX))
   {
      pinconfig = SFC_PIN_CONFIG_QUAD_READ;
      prtdelay =  SFC_PROTCOL_DELAY_QUADRD_MN;
      cmd = SFC_DEV_CMD_QUAD_READ;
   }
   else
   {
      pinconfig = SFC_PIN_CONFIG_QUAD_READ;
      prtdelay =  SFC_PROTCOL_DELAY_QUADRD;
      cmd = SFC_DEV_CMD_QUAD_READ;
   }

   if ((flash->cd.offset + flash->cd.length) >= SZ_16M)
   {
      nx_sfc_32b_en(flash, 1);
   }
   else
   {   /* Program the protocol delay regsister */
      writel( prtdelay, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
   }
   /* Program the Pin Configuration Register */
   writel( pinconfig, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_PIN_CONFIG_REG);
   round_len = flash->cd.length % 4;
   if(round_len)
   {
      round_len = flash->cd.length + (4 - round_len);
      writel(round_len,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
   }
   else
   {
      writel(flash->cd.length,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
   }

#ifdef CONFIG_MTD_NX_SFC_DMAC_EN
   if(!(dma_from & 3))
      writel(SFC_CSR_QUAD_READ_32_BIT, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);
   else if(!(dma_from & 1))
      writel(SFC_CSR_QUAD_READ_16_BIT, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);
   else
      writel(SFC_CSR_QUAD_READ_8_BIT, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);

   /* Program the Device Command Register with Quad read command */
   writel(cmd, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);
   nx_sfc_map_copy_from(flash->cd.buffer, dma_from, flash->cd.length);
#else 
   writel(SFC_CSR_QUAD_READ_32_BIT, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);
   /* Program the Device Command Register with Quad read command */
   writel(cmd, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);

   for (cnt=0;cnt<flash->cd.length/4; cnt++)
   {
      buf_store=readl(buf_base);
      memcpy(buf_ptr, mem_buf_ptr, 4);
      buf_base++;
      buf_ptr +=0x04;
   }

   if(flash->cd.length%4)
   {
      buf_store = readl(buf_base);
      memcpy(buf_ptr, mem_buf_ptr, flash->cd.length%4);
   }
#endif

   /* Set the default settings back */
   writel( SFC_PIN_CONFIG_DEFAULT, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_PIN_CONFIG_REG);
   if((flash->cd.offset + flash->cd.length) >= SZ_16M)
   {
      nx_sfc_32b_dis(flash);
   }
   else
   {
      writel( SFC_PROTOCOL_DELAY_VAL_24B, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
   }
   return flash->cd.length;
}
#endif

ssize_t nx_sfc_prefetch_read(struct nx_sfc_mtd *flash) 
{
   volatile unsigned long *prefetch_buf_base;
   volatile unsigned long prefetch_buf_store;
   u32 i =0;
   u32 round_len =0;
   u8 *prefetch_buf_ptr = (u8*)&prefetch_buf_store;
   u8 *buf_ptr = flash->cd.buffer;

    if ((flash->cd.offset + flash->cd.length) >= SZ_16M)
   {
        nx_sfc_32b_en(flash, 1);
   }
   else
   {
      /* Program the protocol delay regsister */
      writel(SFC_PROTOCOL_DELAY_VAL_24B,
         flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
   }

   /* Program the CSR Register */
   writel(SFC_CSR_READ, flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);

   /* Program the SPL CMD ADDR Register */
   writel(flash->cd.offset, flash->mmio_base +0x7C);

   /* Program the DATA COUNT Register in multiples of 4 bytes */
   round_len = flash->cd.length % 4;
   if(round_len)
   {
      round_len = flash->cd.length + (4 - round_len);
      writel(round_len,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
   }
   else
      writel(flash->cd.length,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
      //writel(0x40,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);

   /* Write the CMD to the DEV_CMD Register */
   writel(SFC_DEV_CMD_READ,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);
   nx_sfc_wait(flash->mmio_base);

   /* Copy the data from the Prefetch Buffers */
   prefetch_buf_base = ((volatile unsigned long *)(flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PRE_FETCH_BUFF_0_REG));
   for (i = 0; i < flash->cd.length/4; i++) 
   {
      prefetch_buf_store = readl(prefetch_buf_base);
      prefetch_buf_base++;
      memcpy(buf_ptr, prefetch_buf_ptr, 4);
      buf_ptr += 4;
   }
   if (flash->cd.length % 4)
   {
      prefetch_buf_store = readl(prefetch_buf_base);
      memcpy(buf_ptr, prefetch_buf_ptr, flash->cd.length % 4);
   }

    if((flash->cd.offset + flash->cd.length) >= SZ_16M)
      nx_sfc_32b_dis(flash);

   return flash->cd.length;
}

ssize_t nx_sfc_prefetch_write(struct nx_sfc_mtd *flash) 
{
   volatile unsigned long *prefetch_buf_base;
   volatile unsigned long prefetch_buf_store;
   u32 i =0;
   u8 *prefetch_buf_ptr = (u8*)&prefetch_buf_store;
   u8 *buf_ptr = flash->cd.buffer;

    if ((flash->cd.offset + flash->cd.length) >= SZ_16M)
   {
        nx_sfc_32b_en(flash, 0); 
   }
   else
   {
      /* Program the protocol delay regsister */
      writel(SFC_PROTOCOL_DELAY_VAL_24B,
         flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
   }

   /* Program the CSR Register */
   writel(SFC_CSR_WRITE, flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);

   /* Program the SPL CMD ADDR Register */
   writel(flash->cd.offset, flash->mmio_base +0x7C);

   /* Program the DATA COUNT Register */
   writel(flash->cd.length << 12, flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);

   /* Setup the Prefetch Buffers */
   prefetch_buf_base = ((volatile unsigned long *)(flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PRE_FETCH_BUFF_0_REG));
   for (i = 0; i < flash->cd.length/4; i++) 
   {
      memcpy(prefetch_buf_ptr, buf_ptr, 4);
      writel(prefetch_buf_store, prefetch_buf_base);
      prefetch_buf_base++;
      buf_ptr += 4;
   }
   if (flash->cd.length % 4)
   {
      memcpy(prefetch_buf_ptr, buf_ptr, flash->cd.length % 4);
      writel(prefetch_buf_store, prefetch_buf_base);
   }

   /* -------Write the CMD to the DEV_CMD Register---------- */
   writel(SFC_DEV_CMD_WRITE,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);
   nx_sfc_wait(flash->mmio_base);

   if ((flash->cd.offset + flash->cd.length) >= SZ_16M)
      nx_sfc_32b_dis(flash);

   return flash->cd.length;
}

#ifdef SFC_DMAC_ENABLE
ssize_t nx_sfc_read(struct nx_sfc_mtd *flash) 
{

   /* Program the protocol delay regsister */
   writel(0x10008000,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
   
   /* Program the CSR Register ------------*/
   writel(0x0000167, flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);

   /* Program the DATA COUNT Register ------------*/
   writel(flash->cd.length,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
   
   /* Write the CMD to the DEV_CMD Register */
   writel(0x00000203,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);
   nx_sfc_wait(flash->mmio_base);

   /* Map the data buffer to a DMA bounce buffer */

   /* Setup the DMA Xfer ------------------*/
   nx_sfc_map_copy_from(flash->cd.buffer, flash->cd.offset, flash->cd.length);

   /* Kick Start the DMA xfer and wait for the completion event */
}

ssize_t nx_sfc_write(struct nx_sfc_mtd *flash) 
{
   /* Program the protocol delay regsister */
   writel(0x10000000, flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
  
   /* Program the CSR Register ------------*/
   writel(0x03C20001, flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);
  
   /* Program the DATA COUNT Register ------------*/
   writel(flash->cd.length << 12 ,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
  
   /* Write the CMD to the DEV_CMD Register */
   writel(0x00840203 ,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);
   nx_sfc_wait(flash->mmio_base);
  
   /* Map the data buffer to a DMA bounce buffer */
  
   /* Setup the DMA Xfer ------------------*/
   nx_sfc_map_copy_to(flash->cd.offset, flash->cd.buffer, flash->cd.length);
  
   /* Kick Start the DMA xfer and wait for the completion event */
}
#endif

ssize_t nx_sfc_erase(struct nx_sfc_mtd *flash) 
{
   u32 cmd=0;

   if(flash->cd.command[0] == OPCODE_CHIP_ERASE)
   {
      cmd = 0x0D0203 | (flash->cd.command[0] << IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_SP_CMD_SHIFT);

      /* Program the CSR Register */
      writel(0x67,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);

      /* Write the CMD to the DEV_CMD Register */
      writel(cmd, flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);
      nx_sfc_wait(flash->mmio_base);

      return flash->mtd.size;
   }
   else
   {
      cmd = 0x810203 | (flash->cd.command[0] << IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_SP_CMD_SHIFT);

      /* Program the protocol delay regsister */
       if ((flash->cd.offset + flash->cd.length) >= SZ_16M)
          nx_sfc_32b_en(flash, 0);
   
      /* Program the CSR Register */
      writel(SFC_CSR_ERASE,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);

      /* Program the SPL CMD ADDR Register */
      writel(flash->cd.offset, flash->mmio_base +0x7C);
  
      /* Write the CMD to the DEV_CMD Register */
      writel(cmd, flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);
      nx_sfc_wait(flash->mmio_base);

      if ((flash->cd.offset + flash->cd.length) >= SZ_16M)
           nx_sfc_32b_dis(flash);

      return flash->cd.length;
   }
}

void nx_sfc_32b_en(struct nx_sfc_mtd *flash, bool isread)
{
   if(isread) {
      write_enable(flash);
      wait_till_ready_wr(flash);
   }
   if((flash->info.jedec_id >> 16 == MAN_ID_MACRONIX) ||
      (flash->info.jedec_id >> 16 == MAN_ID_MICRON)) {
      writel(SFC_PIN_CONFIG_DEFAULT, flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PIN_CONFIG_REG);
      writel(SFC_CSR_WRITE_32B_EN, flash->mmio_base+ IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);
      writel(SFC_PROTOCOL_DELAY_32B_EN, flash->mmio_base+ IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
      writel(SFC_DEV_CMD_32B_EN_MX, flash->mmio_base+ IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);

   } else if(flash->info.jedec_id >> 16 == MAN_ID_SPANSION){
      writel(SFC_PIN_CONFIG_DEFAULT, flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PIN_CONFIG_REG);
      writel(SFC_CSR_WRITE_BNK_SPA, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);
      writel(SFC_PROTOCOL_DELAY_32B_EN, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
      writel((0x1 << 12), flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
      writel(0x80, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_SP_CMD_RD_WR0_DATA_REG);
      writel(SFC_DEV_CMD_WR_BNK_SPA, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);
   }

   nx_sfc_wait(flash->mmio_base);
   wait_till_ready(flash);

   if(isread)
   {
#ifdef CONFIG_MTD_NX_SFC_QUAD_MODE
      if(flash->info.jedec_id >> 16 == MAN_ID_MACRONIX) {
         writel(SFC_PROTOCOL_DELAY_QUADVAL_32B_MX,
				flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
      } else if(flash->info.jedec_id >> 16 == MAN_ID_MICRON){
         writel(SFC_PROTOCOL_DELAY_QUADVAL_32B_MN,
				flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
	  }else {
         writel(SFC_PROTOCOL_DELAY_QUADVAL_32B,
				flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
      }
#else
      writel(SFC_PROTOCOL_DELAY_VAL_32B,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
#endif
   }
   else
   {
      write_enable(flash);
      wait_till_ready_wr(flash);
      writel(SFC_PROTOCOL_DELAY_VAL_32B,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
   }
}

void nx_sfc_32b_dis(struct nx_sfc_mtd *flash)
{
   wait_till_ready(flash);
   write_enable(flash);
   wait_till_ready_wr(flash);

   if((flash->info.jedec_id >> 16 == MAN_ID_MACRONIX) ||
      (flash->info.jedec_id >> 16 == MAN_ID_MICRON)) {
      writel(SFC_PIN_CONFIG_DEFAULT, flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PIN_CONFIG_REG);
      writel(SFC_CSR_WRITE_32B_EN, flash->mmio_base+ IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);
      writel(SFC_PROTOCOL_DELAY_32B_DIS, flash->mmio_base+ IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
      writel(SFC_DEV_CMD_32B_DIS_MX, flash->mmio_base+ IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);

   } else if(flash->info.jedec_id >> 16 == MAN_ID_SPANSION){
      writel(SFC_PIN_CONFIG_DEFAULT, flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PIN_CONFIG_REG);
      writel(SFC_CSR_WRITE_BNK_SPA, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);
      writel(SFC_PROTOCOL_DELAY_32B_DIS, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
      writel((0x1 << 12), flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
      writel(0x00, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_SP_CMD_RD_WR0_DATA_REG);
      writel(SFC_DEV_CMD_WR_BNK_SPA, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);
   }

   nx_sfc_wait(flash->mmio_base);
   writel(SFC_PROTOCOL_DELAY_VAL_24B,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
}

u32 nx_sfc_read_RSCUR(void __iomem  * sfc_regs)
{
   u32 status;
   writel(0x7, sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);
   writel(0x1, sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
   writel(0x2B0D0203, sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);
   nx_sfc_wait(sfc_regs);
   status = readl(sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_SP_CMD_RD_WR0_DATA_REG);

   return status;
}
void nx_sfc_write_en(void __iomem  * sfc_regs)
{
   //Program the CSR register, enable prefetch, tx_en, tx_hold_en
   writel(SFC_CSR_WREN, sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);
   // Do SPL CMD- "Write Enable" - 06h
   writel(SFC_DEV_CMD_WREN,sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);
   nx_sfc_wait(sfc_regs);
}

void  nx_sfc_read_id(void __iomem * sfc_regs, u32 *id )
{
   writel(SFC_CSR_READ_ID, sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_CSR_REG); 
   writel(0x8, sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
   writel(SFC_DEV_CMD_ID, sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);
   nx_sfc_wait(sfc_regs);
   *id = readl(sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_SP_CMD_RD_WR0_DATA_REG);
   *(id+1) = readl(sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_SP_CMD_RD_WR1_DATA_REG);

}

u32 nx_sfc_read_sr(void __iomem * sfc_regs)
{
   u32 status;
   writel(SFC_CSR_READ_STATUS, sfc_regs +IPBGCSFLSHNTWK_SFC_SFC_CSR_REG); 
   writel(0x1,sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
   writel(SFC_DEV_CMD_READ_STATUS,sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);
   nx_sfc_wait(sfc_regs);
   status = readl(sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_SP_CMD_RD_WR0_DATA_REG);

   return status;
}

/*
 *nx_sfc_status2_read : to read status register 2 for Winbond serial flash
 */

#if 0  
u32 nx_sfc_status2_read(void __iomem *sfc_regs)
{  
   u32 status=0;
   writel( 0xa900001b, sfc_regs+IPBGCSFLSHNTWK_SFC_SFC_PIN_CONFIG_REG); 
   writel( 0x10008000, sfc_regs+IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG); 
   writel(0x00000007, sfc_regs+IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);
   writel(0x01, sfc_regs+IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
   writel(0x350D0203, sfc_regs+IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);
   nx_sfc_wait(sfc_regs);
   status= readl(sfc_regs+IPBGCSFLSHNTWK_SFC_SFC_SP_CMD_RD_WR0_DATA_REG);
   return status;    
}

#endif

void nx_sfc_write_sr(struct nx_sfc_mtd * flash)
{
   void __iomem * sfc_regs = flash->mmio_base;

   /* Wait for Write In Progress bit to clear (from possible previous operation) */
   /* Added since we can't rely on caller to have called wait_till_ready() before calling this function */
   while (nx_sfc_read_sr(sfc_regs) & SR_WIP) ;   

   writel(SFC_CSR_WRITE_STATUS,sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_CSR_REG); 
   writel((0x1 << 12),sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
   writel(flash->cd.command[1], sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_SP_CMD_RD_WR0_DATA_REG);
   writel(SFC_DEV_CMD_WRITE_STATUS,sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);
   nx_sfc_wait(sfc_regs);

   /* Wait for Write In Progress bit to clear for SR register write (expect 5 to 40 msec) */
   while (nx_sfc_read_sr(sfc_regs) & SR_WIP) ;  
   return;
}

void nx_sfc_wait(void __iomem * sfc_regs)
{
   volatile u32        status;
   
   // Wait for SPL CMD to complete
   do
   {
      // Read SFC Status Register - Wait for SPL CMD to complete
      status = readl(sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);
   }
   while (status & 0x10000);
}


void nx_sfc_reset(void __iomem * sfc_regs)
{
   u32 tmp=100;	
   writel(0x80000000,sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_CSR_REG); 
   do
   {
      tmp=tmp-1;
   }
   while(tmp>0);
   writel(0x00000000,sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_CSR_REG); 
   return;
}

void nx_sfc_set_quad_bit(struct nx_sfc_mtd *flash)
{
   nx_sfc_write_en(flash->mmio_base);
   writel(0x00020005, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);
   writel(0x10000000, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
   if((flash->info.jedec_id >= MACRONIX_ID_MIN)&&(flash->info.jedec_id <= MACRONIX_ID_MAX))
   {
      writel(0x00001000, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
      writel(0x00000040, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_SP_CMD_RD_WR0_DATA_REG);
   }
   else
   {
      writel(0x00002000, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
      writel(0x00000200, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_SP_CMD_RD_WR0_DATA_REG);
   }
   writel(0x00000000, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_SP_CMD_RD_WR1_DATA_REG);
   writel(0x010D0203, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);
   nx_sfc_wait(flash->mmio_base);
}


void nx_sfc_reset_quad_bit(struct nx_sfc_mtd *flash)
{
   nx_sfc_reset(flash->mmio_base);
   nx_sfc_write_en(flash->mmio_base);
   writel(0x00020005, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);
   writel(0x10000000, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
   if((flash->info.jedec_id >= MACRONIX_ID_MIN)&&(flash->info.jedec_id <= MACRONIX_ID_MAX))
   {
      writel(0x00001000, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
      writel(0x00000000, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_SP_CMD_RD_WR0_DATA_REG);
   }
   else if ((flash->info.jedec_id >= MICRON_ID_MIN)&&(flash->info.jedec_id <= MICRON_ID_MAX))
   {
      writel(0x00001000, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
      writel(0x00000000, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_SP_CMD_RD_WR0_DATA_REG);
   }
   else
   {
      writel(0x00002000, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
      writel(0x00000000, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_SP_CMD_RD_WR0_DATA_REG);
   }
   writel(0x00000000, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_SP_CMD_RD_WR1_DATA_REG);
   writel(0x010D0203, flash->mmio_base+IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);
   nx_sfc_wait(flash->mmio_base);
}

void nx_sfc_init(void __iomem * sfc_regs)
{
   writel(0x1,sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_SCK_CSR_REG); 
   writel(SFC_PROTOCOL_DELAY_VAL_24B, sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
   writel(0x01, IP2070_GCS_CLOCK_MUX);
   return;
}

/*
 * Internal helper functions
 */

/*
 * Read the status register, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */
 
static int read_sr(struct nx_sfc_mtd *flash)
{
   ssize_t retval;
   //u8 code = OPCODE_RDSR;
   
   retval = nx_sfc_read_sr(flash->mmio_base);

   if (retval < 0) {
      dev_err(&flash->sfc->dev, "error %d reading SR\n",
            (int) retval);
      return retval;
   }
  
   return retval;
}

/*
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
int write_enable(struct nx_sfc_mtd *flash)
{
   //u8  code = OPCODE_WREN;

   nx_sfc_write_en(flash->mmio_base);
   return 0;
}

/*
 * Service routine to read status register until ready, or timeout occurs.
 * Returns non-zero if error.
 */
int wait_till_ready(struct nx_sfc_mtd *flash)
{
   unsigned long deadline;
   int sr;

   if(flash->cd.command[0] == OPCODE_CHIP_ERASE)
     deadline = jiffies + MAX_READY_WAIT_CHIP_ERASE_JIFFIES;
   else
      deadline = jiffies + MAX_READY_WAIT_JIFFIES;

   do {
      if ((sr = read_sr(flash)) < 0)
         break;
      else if (!(sr & SR_WIP))
         return 0;
      cond_resched();
   } while (!time_after_eq(jiffies, deadline));


   return 1;
}

int wait_till_ready_wr(struct nx_sfc_mtd *flash)
{
   unsigned long deadline;
   int sr;

   if(flash->cd.command[0] == OPCODE_CHIP_ERASE)
     deadline = jiffies + MAX_READY_WAIT_CHIP_ERASE_JIFFIES;
   else
      deadline = jiffies + MAX_READY_WAIT_JIFFIES;

   do {
      if ((sr = read_sr(flash)) < 0)
         break;
      else if ((sr & SR_WEL))
         return 0;
      cond_resched();
   } while (!time_after_eq(jiffies, deadline));

   return 1;
}

/*
 * Write status register 1 byte
 * Returns negative if error occurred.
 */
static int write_sr(struct nx_sfc_mtd *flash, u8 val)
{
   /* Wait until finished previous write command (if any) */
   if (wait_till_ready(flash))
      return 1;

   flash->cd.command[0] = OPCODE_WRSR;
   flash->cd.command[1] = val;

   nx_sfc_write_sr(flash);
   return 0;

}

/*
 * Erase the whole flash memory
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_chip(struct nx_sfc_mtd *flash)
{
   pr_debug(" %s %lldKiB\n",
          __func__,
         flash->mtd.size / 1024);

   /* Wait until finished previous write command. */
   if (wait_till_ready(flash))
      return 1;

   /* Send write enable, then erase commands. */
   write_enable(flash);

   /* Set up command buffer. */
   flash->cd.command[0] = OPCODE_CHIP_ERASE;

   nx_sfc_erase(flash);

   return 0;
}

/*
 * Erase one sector of flash memory at offset ``offset'' which is any
 * address within the sector which should be erased.
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_sector(struct nx_sfc_mtd *flash, u32 offset)
{
   pr_debug(" %s %dKiB at 0x%08x\n",
          __func__,
         flash->mtd.erasesize / 1024, offset);

   /* Wait until finished previous write command. */
   if (wait_till_ready(flash))
      return 1;

   /* Send write enable, then erase commands. */
   write_enable(flash);
   wait_till_ready_wr(flash);

   /* Set up command buffer. */
   flash->cd.command[0] = flash->erase_opcode;
   flash->cd.offset = offset;

   nx_sfc_erase(flash);

   return 0;
}

/****************************************************************************/

/*
 * MTD implementation
 */

/*
 * Erase an address range on the flash chip.  The address range may extend
 * one or more erase sectors.  Return an error is there is a problem erasing.
 */
static int nx_sfc_mtd_erase(struct mtd_info *mtd, struct erase_info *instr)
{
   struct nx_sfc_mtd *flash = mtd_to_nx_sfc_mtd(mtd);
   u32 addr,len;
   uint32_t rem;

   pr_debug(" %s %s 0x%08x, len %lld\n",
          __func__, "at",
         (u32)instr->addr, instr->len);

   /* sanity checks */
   if (instr->addr + instr->len > flash->mtd.size)
      return -EINVAL;
   div_u64_rem(instr->len, mtd->erasesize, &rem);
   if (rem)
      return -EINVAL;

   addr = instr->addr;
   len = instr->len;

   mutex_lock(&flash->lock);

   /* whole-chip erase? */
   if (len == flash->mtd.size && erase_chip(flash)) {
      instr->state = MTD_ERASE_FAILED;
      mutex_unlock(&flash->lock);
      return -EIO;

   /* REVISIT in some cases we could speed up erasing large regions
    * by using OPCODE_SE instead of OPCODE_BE_4K.  We may have set up
    * to use "small sector erase", but that's not always optimal.
    */

   /* "sector"-at-a-time erase */
   } else {
      while (len) {
         if (erase_sector(flash, addr)) {
            instr->state = MTD_ERASE_FAILED;
            mutex_unlock(&flash->lock);
            return -EIO;
         }

         addr += mtd->erasesize;
         len -= mtd->erasesize;
      }
   }

   mutex_unlock(&flash->lock);

   instr->state = MTD_ERASE_DONE;
   mtd_erase_callback(instr);
   return 0;
}


inline void sfc_sync(struct nx_sfc_mtd *flash, loff_t offset, const u_char *buf, 
   u32 page_size, u32 pgoffset, bool bwrite)
{
   flash->cd.offset = offset + pgoffset;
   flash->cd.length = page_size;
   flash->cd.buffer = (u_char*)(buf + pgoffset);
   
   wait_till_ready(flash);

   if(bwrite)
   {
      write_enable(flash);
      wait_till_ready_wr(flash);
#ifdef SFC_DMAC_ENABLE
      if (page_size == FLASH_PAGESIZE)
         nx_sfc_write(flash);
#endif
      nx_sfc_prefetch_write(flash);
   }
   else
   {
#ifdef CONFIG_MTD_NX_SFC_QUAD_MODE
      nx_sfc_read_quad(flash);
#else
      nx_sfc_prefetch_read(flash);
#endif
   }
}

/*
 * Read an address range from the flash chip.  The address range
 * may be any size provided it is within the physical boundaries.
 */
static int nx_sfc_mtd_read(struct mtd_info *mtd, loff_t from, size_t len,
   size_t *retlen, u_char *buf)
{
   struct nx_sfc_mtd *flash = mtd_to_nx_sfc_mtd(mtd);
   u32 pgoffset, page_size, i;
   
   pr_debug(" %s %s 0x%08x, len %zd\n",
          __func__, "from",
         (u32)from, len);
   
   /* sanity checks */
   if (!len)
      return 0;

   if (from + len > flash->mtd.size)
      return -EINVAL;

   /* NOTE:
    * OPCODE_FAST_READ (if available) is faster.
    * Should add 1 byte DUMMY_BYTE.
    */
   /* Byte count starts at zero. */
   if (retlen)
      *retlen = 0;
   mutex_lock(&flash->lock);

   /* Wait till previous write/erase is done. */
   if (wait_till_ready(flash)) {
      /* REVISIT status return?? */
      mutex_unlock(&flash->lock);
      return 1;
   }

   /* FIXME switch to OPCODE_FAST_READ.  It's required for higher
    * clocks; and at this writing, every chip this driver handles
    * supports that opcode.
    */

   /* Set up the write data buffer. */
   flash->cd.command[0] = OPCODE_READ;

#ifdef CONFIG_MTD_NX_SFC_QUAD_MODE
    for (i = 0, pgoffset = 0; i < len; i += page_size, pgoffset += page_size) 
   {
      page_size = len - i;

      if (page_size > FLASH_READ_SIZE)
         page_size = FLASH_READ_SIZE;

      sfc_sync(flash, from, buf, page_size, pgoffset, 0);
   }
#else
   /* write everything in PAGESIZE chunks */
   for (i = 0, pgoffset = 0; i < len; i += page_size, pgoffset += page_size) 
   {
      page_size = len - i;

      if (page_size > SFC_PREFETCH_BUFSIZE)
         page_size = SFC_PREFETCH_BUFSIZE;

      sfc_sync(flash, from, buf, page_size, pgoffset, 0);
   }	
#endif

   if (retlen)
      *retlen = len;
   mutex_unlock(&flash->lock);

   return 0;
}

/*
 * Write an address range to the flash chip.  Data must be written in
 * FLASH_PAGESIZE chunks.  The address range may be any size provided
 * it is within the physical boundaries.
 */
static int nx_sfc_mtd_write(struct mtd_info *mtd, loff_t to, size_t len,
   size_t *retlen, const u_char *buf)
{
   struct nx_sfc_mtd *flash = mtd_to_nx_sfc_mtd(mtd);
   u32 page_offset = 0, page_size = 0, first_page = 0, pgoffset = 0;
   u32 i;
   
   pr_debug(" %s %s 0x%08x, len %zd\n",
          __func__, "to",
         (u32)to, len);

   if (retlen)
      *retlen = 0;

   /* sanity checks */
   if (!len)
      return(0);

   if (to + len > flash->mtd.size)
      return -EINVAL;

   mutex_lock(&flash->lock);

   /* Wait until finished previous write command. */
   if (wait_till_ready(flash)) {
      mutex_unlock(&flash->lock);
      return 1;
   }

   /* Set up the opcode in the write buffer. */
   flash->cd.command[0] = OPCODE_PP;

   /* what page do we start with? */
   page_offset = to % FLASH_PAGESIZE;

   /* the size of data remaining on the first page */
   if(page_offset)
      first_page = FLASH_PAGESIZE - page_offset;

   page_offset = to & (FLASH_PAGESIZE - 1);

   /* do all the bytes fit onto one page? */
   if ((page_offset + len) <= FLASH_PAGESIZE) 
   {
      for ( i=0, pgoffset = 0; i < len; i += page_size, pgoffset += page_size) 
      {
         page_size = len-i;

         if (page_size > SFC_PREFETCH_BUFSIZE) 
            page_size = SFC_PREFETCH_BUFSIZE;

         sfc_sync(flash, to, buf, page_size, pgoffset, 1);
      }
   } 
   else 
   {
      /* the size of data remaining on the first page */
      first_page = FLASH_PAGESIZE - page_offset;
      
      for (i=0, pgoffset = 0;i < first_page;i += page_size, pgoffset += page_size) 
      {
         page_size = first_page - i;

         if (page_size > SFC_PREFETCH_BUFSIZE) 
            page_size = SFC_PREFETCH_BUFSIZE;

         sfc_sync(flash, to, buf, page_size, pgoffset, 1);
      }

      /* write everything in flash->page_size chunks */
      for (;i < len; i += page_size, pgoffset += page_size)
      {
         page_size = len - i;

         if (page_size > FLASH_PAGESIZE)
            page_size = FLASH_PAGESIZE;

         if (page_size > SFC_PREFETCH_BUFSIZE) 
            page_size = SFC_PREFETCH_BUFSIZE;

         sfc_sync(flash, to, buf, page_size, pgoffset, 1);
      }
   }
  
   if (retlen)
      *retlen = len;
   mutex_unlock(&flash->lock);

   return 0;
}

u32 nx_sfc_read_lockbit(void __iomem *sfc_regs)
{
    u32 status;
    writel(0x7, sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);
    writel(0x1, sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
    writel(SFC_DEV_CMD_RDSPBLK, sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);
    nx_sfc_wait(sfc_regs);
    status = readl(sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_SP_CMD_RD_WR0_DATA_REG);
    return status;
}

/*
 * Internal helper functions
 */

int sfc_permlockbit_read(struct nx_sfc_mtd *flash)
{
   u32 retval;
   retval = nx_sfc_read_lockbit(flash->mmio_base);

   /* returns 0 if Solid/Peristent Protection bits can be programmed/erased
    * returns 1 if Solid/Peristent Protection bits are locked
    */
   return retval? 0:1 ;
}

/* Read Configuration Register-1 */
u32 spansion_RDCR(void __iomem  * sfc_regs)
{
    u32 status;
    writel(0x7, sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);
    writel(0x1, sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
    writel(0x350D0203, sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);
    nx_sfc_wait(sfc_regs);
    status = readl(sfc_regs + IPBGCSFLSHNTWK_SFC_SFC_SP_CMD_RD_WR0_DATA_REG);

    return status;
}

int sfc_ASPRD(struct nx_sfc_mtd *flash)
{
    ssize_t retval;

    retval = nx_sfc_read_RSCUR(flash->mmio_base);

    if (retval < 0) {
        printk(KERN_ERR "error %d reading RSCUR\n",
                (int) retval);
        return retval;
    }

    if( (retval & 0x80) != 0 )
       return 1;
    else
       return 0;

}

void nx_sfc_DYBRD(struct nx_sfc_mtd *flash) /* not test */
{
   /*
       * Program the protocol delay regsister
       * FIXME: For spansion S25FL256S, its size is large than 16Mbytes, so it need 32 bit address.
       *            But if flash size is less than 16Mbytes, maybe it do not need change the setting.
       */
   u32 saved_delay_val = readl(flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
   writel(SFC_PROTOCOL_DELAY_VAL_32B,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);

   /* Program the CSR Register ------------*/
   writel(SFC_CSR_DYBRD,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);

   /* Set byte count for read to nx_sfc controller -----------*/
   writel(1,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);

   /* Program the SPL CMD ADDR Register ------------*/
   writel(flash->cd.offset, flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_SPL_CMD_ADDR_REG);

   /* Write the CMD to the DEV_CMD Register */
   writel(SFC_DEV_CMD_DYBRD, flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);

   /* Waiting complete ....*/
   nx_sfc_wait(flash->mmio_base);

   /* Read back returned value from nx_sfc controller */
   flash->cd.command[1] = (u8)readl(flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_SP_CMD_RD_WR0_DATA_REG);

   /* Recovery Program the protocol delay regsister */
   writel(saved_delay_val,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
}

void nx_sfc_DYBWR(struct nx_sfc_mtd *flash) /* not test */
{
   /*
       * Program the protocol delay regsister
       * FIXME: For spansion S25FL256S, its size is large than 16Mbytes, so it need 32 bit address.
       *            But if flash size is less than 16Mbytes, maybe it do not need change the setting.
       */
   u32 saved_delay_val = readl(flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
   writel(SFC_PROTOCOL_DELAY_VAL_32B,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);

   /* Program the CSR Register ------------*/
   writel(SFC_CSR_DYBWR,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);

   /* Write byte count and byte value to nx_sfc controller -----------*/
   writel((1<< IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_TX_COUNT_SHIFT),flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);
   writel(flash->cd.command[1], flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_SP_CMD_RD_WR0_DATA_REG);

   /* Program the SPL CMD ADDR Register ------------*/
   writel(flash->cd.offset, flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_SPL_CMD_ADDR_REG);

   /* Write the CMD to the DEV_CMD Register */
   writel(SFC_DEV_CMD_DYBWR, flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);

   /* Waiting complete ....*/
   nx_sfc_wait(flash->mmio_base);

   /* Recovery Program the protocol delay regsister */
   writel(saved_delay_val,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
}

void nx_sfc_SPBWR(struct nx_sfc_mtd *flash)
{
   /*
    * Program the protocol delay regsister
    */
   u32 saved_delay_val = readl(flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
   writel(SFC_PROTOCOL_DELAY_VAL_32B,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);

   /* Program the CSR Register ------------*/
   writel(SFC_CSR_DYBWR,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);

   /* Program the SPL CMD ADDR Register ------------*/
   writel(flash->cd.offset, flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_SPL_CMD_ADDR_REG);

   /* Write the CMD to the DEV_CMD Register */
   writel(SFC_DEV_CMD_SPBWR, flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);

   /* Waiting complete ....*/
   nx_sfc_wait(flash->mmio_base);

   /* Recovery Program the protocol delay regsister */
   writel(saved_delay_val,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
}

void nx_sfc_SPBRD(struct nx_sfc_mtd *flash)
{
   /*
    * Program the protocol delay regsister
    */
   u32 saved_delay_val = readl(flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
   writel(SFC_PROTOCOL_DELAY_VAL_32B,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);

   /* Program the CSR Register ------------*/
   writel(SFC_CSR_DYBRD,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_CSR_REG);

   /* Set byte count for read to nx_sfc controller -----------*/
   writel(1,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_TX_RX_DATA_CNT_REG);

   /* Program the SPL CMD ADDR Register ------------*/
   writel(flash->cd.offset, flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_SPL_CMD_ADDR_REG);

   /* Write the CMD to the DEV_CMD Register */
   writel(SFC_DEV_CMD_SPBRD, flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_DEVICE_CMD_REG);

   /* Waiting complete ....*/
   nx_sfc_wait(flash->mmio_base);

   /* Read back returned value from nx_sfc controller */
   flash->cd.command[0] = (u8)readl(flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_SP_CMD_RD_WR0_DATA_REG);

   /* Recovery Program the protocol delay regsister */
   writel(saved_delay_val,flash->mmio_base + IPBGCSFLSHNTWK_SFC_SFC_PROTOCOL_DELAY_REG);
}

void nx_sfc_spb_write(struct nx_sfc_mtd *flash,unsigned int offset, int lock)
{
   /* Wait until finished previous write command (if any) */
   wait_till_ready(flash);

   /* Send write enable, then erase commands. */
   write_enable(flash);
   wait_till_ready_wr(flash);
   /* Set up command buffer. */
   if(lock) {
      if(flash->info.jedec_id >> 16 == MAN_ID_MACRONIX)
         flash->cd.command[1] = 0xff;
      else if(flash->info.jedec_id >> 16 == MAN_ID_SPANSION)
         flash->cd.command[1] = 0x00;
      else {
          printk("Command not supported for this flash\n");
          return;
      }

   } else {
      if(flash->info.jedec_id >> 16 == MAN_ID_MACRONIX)
         flash->cd.command[1] = 0x00;
      else if(flash->info.jedec_id >> 16 == MAN_ID_SPANSION)
         flash->cd.command[1] = 0xff;
      else {
          printk("Command not supported for this flash\n");
          return;
      }

   }
   flash->cd.offset = offset;
   nx_sfc_SPBWR(flash);
   return;
}

u8 nx_sfc_spb_read(struct nx_sfc_mtd *flash,unsigned int offset)
{
      /* Wait until finished previous write command (if any) */
    wait_till_ready(flash);

    /* Set up command buffer. */
    flash->cd.command[0] = 0x33;
    flash->cd.offset = offset;

    nx_sfc_SPBRD(flash);
//    printk("SPB flash->cd.command[0] = 0x%x\n",flash->cd.command[0]);
        return flash->cd.command[0];
}

void nx_sfc_dyb_write(struct nx_sfc_mtd *flash,unsigned int offset, int lock)
{
   /* Wait until finished previous write command (if any) */
   wait_till_ready(flash);
   /* Send write enable, then erase commands. */
   write_enable(flash);
   wait_till_ready_wr(flash);
   /* Set up command buffer. */
   if(lock) {
      if(flash->info.jedec_id >> 16 == MAN_ID_MACRONIX)
         flash->cd.command[1] = 0xff;
      else if(flash->info.jedec_id >> 16 == MAN_ID_SPANSION)
         flash->cd.command[1] = 0x00;
      else {
          printk("Command not supported for this flash\n");
          return;
      }

   } else {
      if(flash->info.jedec_id >> 16 == MAN_ID_MACRONIX)
         flash->cd.command[1] = 0x00;
      else if(flash->info.jedec_id >> 16 == MAN_ID_SPANSION)
         flash->cd.command[1] = 0xff;
      else {
          printk("Command not supported for this flash\n");
          return;
      }

   }
   flash->cd.offset = offset;
   nx_sfc_DYBWR(flash);
   return;
}

u8 nx_sfc_dyb_read(struct nx_sfc_mtd *flash,unsigned int offset)
{
      /* Wait until finished previous write command (if any) */
    wait_till_ready(flash);

    /* Set up command buffer. */
    flash->cd.command[0] = 0x33;
    flash->cd.offset = offset;

    nx_sfc_DYBRD(flash);
//    printk(" DYB flash->cd.command[0] = 0x%x\n",flash->cd.command[0]);
        return flash->cd.command[1];
}

int nx_sfc_mtd_protect(struct mtd_info *mtd, loff_t from, uint64_t len)
{
   struct nx_sfc_mtd *flash = mtd_to_nx_sfc_mtd(mtd);
   uint32_t rem;
   uint32_t bsize;
   uint8_t  cr;

   pr_debug(" %s %s 0x%08x, len 0x%llx\n",
            __func__, "from",
            (u32)from, len);

   if (from + len > flash->mtd.size)
      return -EINVAL;

   div_u64_rem(len, mtd->erasesize, &rem);
   if (rem)
      return -EINVAL;

   mutex_lock(&flash->lock);

   if(flash->info.jedec_id >> 16 == MAN_ID_MACRONIX)
   {
      if(!sfc_ASPRD(flash))
      {
         printk("Locking Error: ASP mode not set\n");
         mutex_unlock(&flash->lock);
         return 1;
      }
   }

   while (len)
   {
      bsize = flash->mtd.erasesize;
      if(flash->info.jedec_id >> 16 == MAN_ID_MACRONIX)
      {
         /* First and Last 64K of Macronix are individually controlled as 4K Sectors */
         if((from >= (flash->mtd.size - (16*4*1024))) || (from < (16*4*1024)))
            bsize = (4*1024);
      }
      else if((flash->info.jedec_id == 0x10219) && ((flash->info.ext_id & 0xF) == 0x1))
      {
         cr = spansion_RDCR(flash->mmio_base);
         if( cr&0x4) {
             /* Top 32 sectors of 4K */
            if(from >= (flash->mtd.size - (4*1024*32)))
               bsize = (4*1024);
         } else {
            /* Bottom 32 sectors of 4K */
            if(from < (4*1024*32))
              bsize = (4*1024);
         }
      }

      nx_sfc_dyb_write(flash, (u32)from, 1);
      if (wait_till_ready(flash)) {
         mutex_unlock(&flash->lock);
         return 1;
      }

      from += bsize;
      len -= bsize;
   }

   mutex_unlock(&flash->lock);
   return 0;
}

int nx_sfc_mtd_unprotect(struct mtd_info *mtd, loff_t from, uint64_t len)
{
   struct nx_sfc_mtd *flash = mtd_to_nx_sfc_mtd(mtd);
   uint32_t rem;
   uint32_t bsize;
   uint8_t  cr;
   pr_debug(" %s %s 0x%08x, len 0x%08x\n",
            __func__, "from",(u32)from, (u32)len);

   if (from + len > flash->mtd.size)
      return -EINVAL;

   div_u64_rem(len, mtd->erasesize, &rem);
   if (rem)
      return -EINVAL;

   mutex_lock(&flash->lock);

   if(flash->info.jedec_id >> 16 == MAN_ID_MACRONIX)
   {
      if(!sfc_ASPRD(flash))
      {
         printk("Unlocking Error: ASP mode not set\n");
         mutex_unlock(&flash->lock);
         return 1;
      }
   }

   while (len)
   {
      bsize = flash->mtd.erasesize;

      if(flash->info.jedec_id >> 16 == MAN_ID_MACRONIX)
      {
         /* First and Last 64K of Macronix are individually controlled as 4K Sectors */
         if((from >= (flash->mtd.size - (16*4*1024))) || (from < (16*4*1024)))
            bsize = (4*1024);
      }
      else if((flash->info.jedec_id == 0x10219) && ((flash->info.ext_id & 0xF) == 0x1))
      {
         cr = spansion_RDCR(flash->mmio_base);
         if( cr&0x4) {
            /* Top 32 sectors of 4K */
            if(from >= (flash->mtd.size - (4*1024*32)))
               bsize = (4*1024);
         } else {
            /* Bottom 32 sectors of 4K */
            if(from < (4*1024*32))
               bsize = (4*1024);
         }
      }
      nx_sfc_dyb_write(flash, from, 0);
      if (wait_till_ready(flash)) {
         mutex_unlock(&flash->lock);
         return 1;
      }

      from += bsize;
      len -= bsize;
   }

   mutex_unlock(&flash->lock);
   return 0;
}

int nx_sfc_mtd_permprotect(struct mtd_info *mtd, loff_t from, size_t len)
{
   struct nx_sfc_mtd *flash = mtd_to_nx_sfc_mtd(mtd);
   uint32_t rem;
   uint32_t bsize;
   uint8_t  cr;

   pr_debug(" %s %s 0x%08x, len 0x%08x\n",
            __func__, "from", (u32)from, len);

   if (from + len > flash->mtd.size)
      return -EINVAL;

   div_u64_rem(len, mtd->erasesize, &rem);
   if (rem)
      return -EINVAL;

   mutex_lock(&flash->lock);

   if(flash->info.jedec_id >> 16 == MAN_ID_MACRONIX)
   {
      if(!sfc_ASPRD(flash))
      {
         printk("Locking Error: ASP mode not set\n");
         mutex_unlock(&flash->lock);
         return 1;
      }
   }

   /* Check if Solid/Peristent Lock Bits are locked */
   if(sfc_permlockbit_read(flash))
   {
     printk("Solid/Peristent Bits locked, Reset/Power-cylce the board\n");
     mutex_unlock(&flash->lock);
     return 1;
   }

   while (len)
   {
      bsize = flash->mtd.erasesize;

      if(flash->info.jedec_id >> 16 == MAN_ID_MACRONIX)
      {
         /* First and Last 64K of Macronix are individually controlled as 4K Sectors */
         if((from >= (flash->mtd.size - (16*4*1024))) || (from < (16*4*1024)))
            bsize = (4*1024);
      }
      else if((flash->info.jedec_id == 0x10219) && ((flash->info.ext_id & 0xF) == 0x1))
      {
         cr = spansion_RDCR(flash->mmio_base);
         if( cr&0x4) {
            /* Top 32 sectors of 4K */
            if(from >= (flash->mtd.size - (4*1024*32)))
               bsize = (4*1024);
         } else {
            /* Bottom 32 sectors of 4K */
            if(from < (4*1024*32))
               bsize = (4*1024);
         }
      }

      nx_sfc_spb_write(flash, (u32)from, 1);
      if (wait_till_ready(flash)) {
         mutex_unlock(&flash->lock);
         return 1;
      }

      from += bsize;
      len -= bsize;
   }

   mutex_unlock(&flash->lock);
   return 0;
}

int nx_sfc_mtd_permprotectstatus(struct mtd_info *mtd, struct otp_info *buf, size_t len)
{
   struct nx_sfc_mtd *flash = mtd_to_nx_sfc_mtd(mtd);
   loff_t ofs;
   uint32_t block, bsize;
   uint8_t  perm_status,temp_status,cr;
   pr_debug("%s: %s: len 0x%x\n",
           dev_name(&flash->sfc->dev), __func__, (u32)len);

   mutex_lock(&flash->lock);

   if(flash->info.jedec_id >> 16 == MAN_ID_MACRONIX)
   {
      if(!sfc_ASPRD(flash))
      {
         printk("Error: ASP mode not set\n");
         mutex_unlock(&flash->lock);
         return 1;
      }
   }

   for (ofs =0, block =0; ofs < flash->mtd.size; block++)
   {
      bsize = flash->mtd.erasesize;

      if(flash->info.jedec_id >> 16 == MAN_ID_MACRONIX)
      {
         /* First and Last 64K of Macronix are individually controlled as 4K Sectors */
         if((ofs >= (flash->mtd.size - (16*4*1024))) || (ofs < (16*4*1024)))
            bsize = (4*1024);
      }
      else if((flash->info.jedec_id == 0x10219) && ( (flash->info.ext_id & 0xF) == 0x1))
      {
         cr = spansion_RDCR(flash->mmio_base);
         if( cr&0x4) {
            /* Top 32 sectors of 4K */
            if( ofs >= (flash->mtd.size - (4*1024*32)))
               bsize = (4*1024);
         } else {
            /* Bottom 32 sectors of 4K */
            if(ofs < (4*1024*32))
               bsize = (4*1024);
         }
      }

      perm_status = nx_sfc_spb_read(flash, (u32)ofs);
      temp_status = nx_sfc_dyb_read(flash, (u32)ofs);
      if(flash->info.jedec_id >> 16 == MAN_ID_MACRONIX)
              buf[block].locked = (0xFF == perm_status) ? 1 :(0xFF == temp_status) ? 2 : 0;
      else if(flash->info.jedec_id >> 16 == MAN_ID_SPANSION)
              buf[block].locked = (0x0 == perm_status) ? 1 : (0x0 == temp_status) ? 2 : 0;

      /* save the lock status into the structure */
      buf[block].start  = ofs;
      buf[block].length = bsize;
      ofs += bsize;
   }

   mutex_unlock(&flash->lock);
   return sizeof(struct otp_info) * block;
}

/****************************************************************************/

/* NOTE: double check command sets and memory organization when you add
 * more flash chips.  This current list focusses on newer chips, which
 * have been converging on command sets which including JEDEC ID.
 */
static struct flash_info serial_flash_data[] = {
   /* Atmel -- some are (confusingly) marketed as "DataFlash" */
   { "at25fs010",  0x1f6601, 0, 32 * 1024, 4, SECT_4K, },
   { "at25fs040",  0x1f6604, 0, 64 * 1024, 8, SECT_4K, },

   { "at25df041a", 0x1f4401, 0, 64 * 1024, 8, SECT_4K, },
   { "at25df641",  0x1f4800, 0, 64 * 1024, 128, SECT_4K, },

   { "at26f004",   0x1f0400, 0, 64 * 1024, 8, SECT_4K, },
   { "at26df081a", 0x1f4501, 0, 64 * 1024, 16, SECT_4K, },
   { "at26df161a", 0x1f4601, 0, 64 * 1024, 32, SECT_4K, },
   { "at26df321",  0x1f4701, 0, 64 * 1024, 64, SECT_4K, },

   /* EON -- en25pxx */
   { "en25p32", 0x1c2016, 0, 64 * 1024,  64,  },
   { "en25p64", 0x1c2017, 0, 64 * 1024, 128,  },
   { "en25q32", 0x1c3016, 0, 64 * 1024, 64,   },
   { "en25q32", 0x1c3017, 0, 64 * 1024, 128,  },


   /* Intel/Numonyx -- xxxs33b */
   { "160s33b",  0x898911, 0, 64 * 1024,  32, },
   { "320s33b",  0x898912, 0, 64 * 1024,  64, },
   { "640s33b",  0x898913, 0, 64 * 1024, 128, },

   /* Spansion -- single (large) sector size only, at least
    * for the chips listed here (without boot sectors).
    */
   { "s25sl004a", 0x010212, 0, 64 * 1024, 8, },
   { "s25sl008a", 0x010213, 0, 64 * 1024, 16, },
   { "s25sl016a", 0x010214, 0, 64 * 1024, 32, },
   { "s25sl032a", 0x010215, 0, 64 * 1024, 64, },
   { "s25sl064a", 0x010216, 0, 64 * 1024, 128, },
   { "s25sl12800", 0x012018, 0x0300, 256 * 1024, 64, },
   { "s25sl12801", 0x012018, 0x0301, 64 * 1024, 256, },
   { "s25fl129p0", 0x012018, 0x4d00, 256 * 1024,  64, },
   { "s25fl129p1", 0x012018, 0x4d01, 64 * 1024, 256, },
   { "s25fl256s1", 0x010219, 0x4d01, 64  * 1024, 512, },
   { "s25fl016k",  0xef4015,      0,  64 * 1024,  32, SECT_4K, },
   { "s25fl064k",  0xef4017,      0,  64 * 1024, 128, SECT_4K, },

   /* SST -- large erase sizes are "overlays", "sectors" are 4K */
   { "sst25vf040b", 0xbf258d, 0, 64 * 1024, 8, SECT_4K, },
   { "sst25vf080b", 0xbf258e, 0, 64 * 1024, 16, SECT_4K, },
   { "sst25vf016b", 0xbf2541, 0, 64 * 1024, 32, SECT_4K, },
   { "sst25vf032b", 0xbf254a, 0, 64 * 1024, 64, SECT_4K, },
   { "sst25wf512",  0xbf2501, 0, 64 * 1024,  1, SECT_4K, },
   { "sst25wf010",  0xbf2502, 0, 64 * 1024,  2, SECT_4K, },
   { "sst25wf020",  0xbf2503, 0, 64 * 1024,  4, SECT_4K, },
   { "sst25wf040",  0xbf2504, 0, 64 * 1024,  8, SECT_4K, },

   /* ST Microelectronics -- newer production may have feature updates */
   { "m25p05",  0x202010,  0, 32 * 1024, 2, },
   { "m25p10",  0x202011,  0, 32 * 1024, 4, },
   { "m25p20",  0x202012,  0, 64 * 1024, 4, },
   { "m25p40",  0x202013,  0, 64 * 1024, 8, },
   { "m25p80",  0x202014,  0, 64 * 1024, 16, },
   { "m25p16",  0x202015,  0, 64 * 1024, 32, },
   { "m25p32",  0x202016,  0, 64 * 1024, 64, },
   { "m25p64",  0x202017,  0, 64 * 1024, 128, },
   { "m25p128", 0x202018, 0, 256 * 1024, 64, },

   { "m45pe10", 0x204011,  0, 64 * 1024,  2, },
   { "m45pe80", 0x204014,  0, 64 * 1024, 16, },
   { "m45pe16", 0x204015,  0, 64 * 1024, 32, },

   { "m25pe80", 0x208014,  0, 64 * 1024, 16, },
   { "m25pe16", 0x208015,  0, 64 * 1024, 32, SECT_4K, },

   /* Winbond -- w25x "blocks" are 64K, "sectors" are 4KiB */
   { "w25x10", 0xef3011, 0, 64 * 1024, 2, SECT_4K, },
   { "w25x20", 0xef3012, 0, 64 * 1024, 4, SECT_4K, },
   { "w25x40", 0xef3013, 0, 64 * 1024, 8, SECT_4K, },
   { "w25x80", 0xef3014, 0, 64 * 1024, 16, SECT_4K, },
   { "w25x16", 0xef3015, 0, 64 * 1024, 32, SECT_4K, },
   { "w25x32", 0xef3016, 0, 64 * 1024, 64, SECT_4K, },
   { "w25q32", 0xef4016, 0, 64 * 1024, 64, SECT_4K, },
   { "w25x64", 0xef3017, 0, 64 * 1024, 128, SECT_4K, },
   { "w25q64", 0xef4017, 0, 64 * 1024, 128, SECT_4K, },
   { "w25q128",0xef4018, 0, 64 * 1024, 256, },

   /* Macronix */
   { "mx25l4005a",  0xc22013, 0, 64 * 1024, 8, SECT_4K, },
   { "mx25l8005",   0xc22014, 0, 64 * 1024, 16, },
   { "mx25l1605d",  0xc22015, 0, 64 * 1024, 32, },
   { "mx25l3205d",  0xc22016, 0, 64 * 1024, 64, },
   { "mx25l6405d",  0xc22017, 0, 64 * 1024, 128, },
   { "mx25l12805d", 0xc22018, 0, 64 * 1024, 256, },
   { "mx25l12855e", 0xc22618, 0, 64 * 1024, 256, },
   { "mx25l25635e", 0xc22019, 0, 64 * 1024, 512, },
   { "mx25l25655e", 0xc22619, 0, 64 * 1024, 512, },
   { "mx25l3255d",  0xc29e16, 0xc29e, 64 * 1024, 64, },

   /* Micron */
   { "n25q064", 0x20ba17,  0, 64 * 1024, 128, },
   { "n25q256", 0x20ba19,  0, 64 * 1024, 512, },
};

static struct flash_info * jedec_probe(struct platform_device *spi)
{
   int         cnt;
   u32         tmp[2];
   u8          *id;
   u32         jedec;
   u16         ext_jedec;
   struct flash_info *info;

   /* JEDEC also defines an optional "extended device information"
   * string for after vendor-specific data, after the three bytes
   * we use here.  Supporting some chips might require using it.
   */
   struct nx_sfc_mtd *flash = dev_get_drvdata(&spi->dev);
   nx_sfc_read_id(flash->mmio_base, tmp);

   if (tmp[0] == 0xFFFFFFFF) {
      pr_debug("error %d reading JEDEC ID\n",
                tmp[0]);
      return NULL;
   }
   id = (u8 *) &tmp;
   jedec = id[0];
   jedec = jedec << 8;
   jedec |= id[1];
   jedec = jedec << 8;
   jedec |= id[2];

   ext_jedec = id[3];
   ext_jedec = ext_jedec << 8;
   ext_jedec |= id[4];
   printk(KERN_ERR "JEDEC ID = 0x%x , Extended Device ID = 0x%x \n", jedec, ext_jedec);

   for (cnt = 0, info = serial_flash_data; 
            cnt < ARRAY_SIZE(serial_flash_data);
            cnt++, info++) {
      if (info->jedec_id == jedec) {
         if (info->ext_id != 0 && info->ext_id != ext_jedec)
            continue;
         return info;
      }
   }
   dev_err(&spi->dev, "unrecognized JEDEC id %06x\n", jedec);
   return NULL;
}


/*
 * board specific setup should have ensured the SPI clock used here
 * matches what the READ command supports, at least until this driver
 * understands FAST_READ (for clocks over 25 MHz).
 */
static int nx_sfc_probe(struct platform_device *pdev)
{
   struct nx_sfc_info   *data;
   struct nx_sfc_mtd *flash;
   struct flash_info *info;
   unsigned    i;
   int err;
   struct resource *res;   
#ifdef CONFIG_MTD_NX_SFC_QUAD_MODE
   unsigned long sfc_dcsn_base;
   void __iomem *io_regs =NULL;
#endif
#ifdef CONFIG_MTD_CMDLINE_PARTS
   static const char *part_probes[] = { "cmdlinepart", NULL, };
#else 
   static const char *part_probes[] = NULL;
#endif
   
   void __iomem *mmio_regs = NULL;
   resource_size_t mmio_start, mmio_len;

   /* Get MMIO registers resource */
   res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   if (!res) {
         dev_err(&pdev->dev, "No MMIO resource found\n");
         return -ENXIO;
   }

   mmio_start = res->start;
   mmio_len   = res->end - res->start + 1;

   if (!request_mem_region(mmio_start, mmio_len, pdev->name)) {
         dev_err(&pdev->dev, "request_mem_region failure\n");
         return -EBUSY;
   }

   mmio_regs = ioremap(mmio_start, mmio_len);
   if(!mmio_regs) {
         dev_err(&pdev->dev, "ioremap failure\n");
         err = -ENOMEM;
         return err;
   }


#ifdef CONFIG_MTD_NX_SFC_QUAD_MODE
   sfc_dcsn_base = readl(SFC_DCSN_BASE_REG);
   if (!request_mem_region(sfc_dcsn_base, SFC_DCSN_SIZE, pdev->name))
   {
      dev_err(&pdev->dev, "request_mem_region for io failure\n");
      return -EBUSY;
   }

   io_regs = ioremap(sfc_dcsn_base, SFC_DCSN_SIZE );
   if(!io_regs)
   {
      dev_err(&pdev->dev, "ioremap for io failure\n");
      err = -ENOMEM;
      return err;
   }

 #endif

   nx_sfc_init(mmio_regs);

#ifdef CONFIG_MTD_NX_SFC_DMAC_EN
   err = nx_sfc_dma_init(pdev);
   if (err)
   return err;
#endif

   flash = kzalloc(sizeof *flash, GFP_KERNEL);
   if (!flash)
      return -ENOMEM;

   mutex_init(&flash->lock);
   dev_set_drvdata(&pdev->dev, flash);
   
   flash->sfc = pdev;
   flash->mtd.name = pdev->name;
   flash->mtd.type = MTD_NORFLASH;
   flash->mtd.writesize = 1;
   flash->mtd.flags = MTD_CAP_NORFLASH;
   flash->mtd._erase = nx_sfc_mtd_erase;
   flash->mtd._read = nx_sfc_mtd_read;
   flash->mtd._write = nx_sfc_mtd_write;
   flash->mtd._lock      = nx_sfc_mtd_protect;
   flash->mtd._unlock    = nx_sfc_mtd_unprotect;
   flash->mtd._read_fact_prot_reg = flash->mtd._read;
   flash->mtd._read_user_prot_reg = flash->mtd._read;
#ifdef CONFIG_MTD_OTP
   flash->mtd._get_user_prot_info = nx_sfc_mtd_permprotectstatus;
   flash->mtd._get_fact_prot_info = nx_sfc_mtd_permprotectstatus;
   flash->mtd._lock_user_prot_reg = nx_sfc_mtd_permprotect;
#endif
   flash->mmio_start    = mmio_start;
   flash->mmio_len      = mmio_len;
   flash->mmio_base     = mmio_regs;
#ifdef CONFIG_MTD_NX_SFC_QUAD_MODE
   flash->io_base    = io_regs;
#endif

   /* Platform data helps sort out which chip type we have, as
    * well as how this board partitions it.  If we don't have
    * a chip ID, try the JEDEC id commands; they'll work for most
    * newer chips, even if we don't recognize the particular chip.
    */
   data = pdev->dev.platform_data;
   if (data && data->type) {
      for (i = 0, info = serial_flash_data;
            i < ARRAY_SIZE(serial_flash_data);
            i++, info++) {
         if (strcmp(data->type, info->name) == 0)
            break;
      }

      /* unrecognized chip? */
      if (i == ARRAY_SIZE(serial_flash_data)) {
         pr_debug(": unrecognized id %s\n",
                data->type);
         info = NULL;

      /* recognized; is that chip really what's there? */
      } else if (info->jedec_id) {
         struct flash_info *chip = jedec_probe(pdev);

         if (!chip || chip != info) {
            dev_warn(&pdev->dev, "found %s, expected %s\n",
                  chip ? chip->name : "UNKNOWN",
                  info->name);
            info = NULL;
         }
      }
   } else
         info = jedec_probe(pdev);
      

   if (!info)
      return -ENODEV;

   flash->info = *info;

   flash->mtd.size = info->sector_size * info->n_sectors;
   /* prefer "small sector" erase if possible */
   if (info->flags & SECT_4K) {
      flash->erase_opcode = OPCODE_BE_4K;
      flash->mtd.erasesize = 4096;
   } else {
      flash->erase_opcode = OPCODE_SE;
      flash->mtd.erasesize = info->sector_size;
   }

   dev_info(&pdev->dev, "%s (%lld Kbytes)\n", info->name,
         (long long)flash->mtd.size >> 10);

   pr_debug(
      "mtd .name = %s, .size = 0x%llx (%lldMiB) "
         ".erasesize = 0x%.8x (%uKiB) .numeraseregions = %d\n",
      flash->mtd.name,
      (long long)flash->mtd.size, (long long)(flash->mtd.size >> 20),
      flash->mtd.erasesize, flash->mtd.erasesize / 1024,
      flash->mtd.numeraseregions);

   if (flash->mtd.numeraseregions)
      for (i = 0; i < flash->mtd.numeraseregions; i++)
         pr_debug(
            "mtd.eraseregions[%d] = { .offset = 0x%llx, "
            ".erasesize = 0x%.8x (%uKiB), "
            ".numblocks = %d }\n",
            i, (long long)flash->mtd.eraseregions[i].offset,
            flash->mtd.eraseregions[i].erasesize,
            flash->mtd.eraseregions[i].erasesize / 1024,
            flash->mtd.eraseregions[i].numblocks);


   #ifdef CONFIG_MTD_NX_SFC_QUAD_MODE
   nx_sfc_set_quad_bit( flash);
   #endif

   /*
    * Atmel serial flash tend to power up
    * with the software protection bits set
    */
   if (info->jedec_id >> 16 == 0x1f) {
      write_enable(flash);
      write_sr(flash, 0);
   }

   /* partitions should match sector boundaries; and it may be good to
    * use readonly partitions for writeprotected sectors (BP2..BP0).
    */

   return mtd_device_parse_register(&flash->mtd, part_probes, NULL, NULL, 0);
}

#if 0
static int nx_sfc_remove(struct platform_device *spi)
{
   struct nx_sfc_mtd *flash = dev_get_drvdata(&spi->dev);
   int      status;

   /* Clean up MTD stuff. */
   if (mtd_has_partitions() && flash->partitioned)
      status = del_mtd_partitions(&flash->mtd);
   else
      status = del_mtd_device(&flash->mtd);
   if (status == 0)
      kfree(flash);
#ifdef CONFIG_MTD_NX_SFC_DMAC_EN
   nx_sfc_dma_exit(spi);
#endif
   return 0;
}
#endif

#ifdef CONFIG_OF
static const struct of_device_id nx_sfc_dt_match[] = {
        { .compatible = "entr,stb-sfc" },
        {},
};
MODULE_DEVICE_TABLE(of, nx_sfc_dt_match);
#endif

static struct platform_driver nx_sfc_driver = {
	.driver   = {
		.name  = "nx_sfc",
		.owner = THIS_MODULE,
                .of_match_table = nx_sfc_dt_match,
	},
};

static int __init nx_sfc_drv_init(void)
{
	return platform_driver_probe(&nx_sfc_driver, nx_sfc_probe);
}

static void __exit nx_sfc_drv_exit(void)
{
	platform_driver_unregister(&nx_sfc_driver);
}

module_init(nx_sfc_drv_init);
module_exit(nx_sfc_drv_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MTD SFC driver for IP2070");
MODULE_ALIAS("platform:nx_sfc");

