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

#include <linux/wait.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/bbm.h>
#include <linux/nx_dmac.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <linux/platform_data/mtd-nx_nand.h>

#include "tmhwEfmc.h"
#include "tmhwEfmc_Cfg.h"
#include "tmhwEfmc_Vhip.h"
#include "nx_nand.h"

#define ATTRIBUTE_UNUSED __attribute__ ((__unused__))

#ifdef CONFIG_MTD_NX_NAND_HWECC
#ifdef CONFIG_ARCH_APOLLO
/**
 *  OOB structure
 */
/* For 4K LPF */
static struct nand_ecclayout nx_nand_oob_128 = {
	.eccbytes = 96,
	.eccpos = {
			  4,	5, 	6,   7,	 8,	9,  10,	11,  12,  13,	14,  15,
			 20,	21,  22,  23,	24,  25,  26,	27,  28,  29,	30,  31,
			 36,	37,  38,  39,	40,  41,  42,	43,  44,  45,	46,  47,
			 52,	53,  54,  55,	56,  57,  58,	59,  60,  61,	62,  63,
			 68,	69,  70,  71,	72,  73,  74,	75,  76,  77,	78,  79,
			 84,	85,  86,  87,	88,  89,  90,	91,  92,  93,	94,  95,
			100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111,
			116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127
		},

	.oobfree = {
			{.offset =	2, .length = 2},
			{.offset = 16, .length = 4},
			{.offset = 32, .length = 4},
			{.offset = 48, .length = 4},
			{.offset = 64, .length = 4},
			{.offset = 80, .length = 4},
			{.offset = 96, .length = 4},
			{.offset = 112,.length = 4},
		}
};

/* For LPF */
static struct nand_ecclayout nx_nand_oob_64 = {
	.eccbytes = 48,
	.eccpos = {
			 4,  5,	6,  7,  8,	9, 10, 11, 12, 13, 14, 15,
			20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
			36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
			52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63
		},

	.oobfree = {
			{.offset = 2, .length = 2},
			{.offset = 16,.length = 4},
			{.offset = 32,.length = 4},
			{.offset = 48,.length = 4},
		}
};

/* For SPF */
static struct nand_ecclayout nx_nand_oob_16 = {
	.eccbytes = 16,
	.eccpos = {
			4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
		},
	.oobfree = {
			{ .offset = 0, .length = 4},
		}
};
#else
static struct nand_ecclayout nx_nand_oob_448 = {
	.eccbytes = 416,
	.eccpos = {
		32, 33, 34, 35, 36, 37, 38, 39,
		40, 41, 42, 43, 44, 45, 46, 47,
		48, 49, 50, 51, 52, 53, 54, 55,
		56, 57, 58, 59, 60, 61, 62, 63,
		64, 65, 66, 67, 68, 69, 70, 71,
		72, 73, 74, 75, 76, 77, 78, 79,
		80, 81, 82, 83, 84, 85, 86, 87,
		88, 89, 90, 91, 92, 93, 94, 95,
		96, 97, 98, 99,100,101,102, 103,
		104,105,106,107,108,109,110,111,
		112,113,114,115,116,117,118,119,
		120,121,122,123,124,125,126,127,
	},
	.oobfree = {
		{.offset = 0,
		.length = 32},
	}
};
/* For 4K LPF of 224 Bytes oob size */
static struct nand_ecclayout nx_nand_oob_224 = {
	.eccbytes = 208,
	.eccpos = {
		16, 17, 18, 19, 20, 21, 22, 23,
		24, 25, 26, 27, 28, 29, 30, 31, 
		32, 33, 34, 35, 36, 37, 38, 39, 
		40, 41, 42, 43, 44, 45, 46, 47,
		48, 49, 50, 51, 52, 53, 54, 55, 
		56, 57, 58, 59, 60, 61, 62, 63,
		64, 65, 66, 67, 68, 69, 70, 71,
		72, 73, 74, 75, 76, 77, 78, 79,
		80, 81, 82, 83, 84, 85, 86, 87,
		88, 89, 90, 91, 92, 93, 94, 95,
		96, 97, 98, 99,100,101,102,103,
		104,105,106,107,108,109,110,111,
		/* limited by MTD_MAX_ECCPOS_ENTRIES */
		/*
		112,113,114,115,116,117,118,119,
		120,121,122,123,124,125,126,127,
		*/
		/* ...so on until 224 */
	},
	.oobfree = {
		{.offset = 0,
		.length = 16},
	}
};

/* For 4K LPF */
static struct nand_ecclayout nx_nand_oob_128 = {
	.eccbytes = 112,
	.eccpos = {
		16, 17, 18, 19, 20, 21, 22, 23,
		24, 25, 26, 27, 28, 29, 30, 31, 
		32, 33, 34, 35, 36, 37, 38, 39, 
		40, 41, 42, 43, 44, 45, 46, 47,
		48, 49, 50, 51, 52, 53, 54, 55, 
		56, 57, 58, 59, 60, 61, 62, 63,
		64, 65, 66, 67, 68, 69, 70, 71,
		72, 73, 74, 75, 76, 77, 78, 79,
		80, 81, 82, 83, 84, 85, 86, 87,
		88, 89, 90, 91, 92, 93, 94, 95,
		96, 97, 98, 99,100,101,102,103,
		104,105,106,107,108,109,110,111,
		/* limited by MTD_MAX_ECCPOS_ENTRIES */
		/*
		112,113,114,115,116,117,118,119,
		120,121,122,123,124,125,126,127
		*/
	},
	.oobfree = {
		{.offset = 0,
		.length = 16},
	}
};

/* For LPF */
static struct nand_ecclayout nx_nand_oob_64 = {
	.eccbytes = 56,
	.eccpos = {
		 8,  9, 10, 11, 12, 13, 14, 15, 
		16, 17, 18, 19, 20, 21, 22, 23,
		24, 25, 26, 27, 28, 29, 30, 31, 
		32, 33, 34, 35, 36, 37, 38, 39, 
		40, 41, 42, 43, 44, 45, 46, 47,
		48, 49, 50, 51, 52, 53, 54, 55, 
		56, 57, 58, 59, 60, 61, 62, 63
	},
	.oobfree = {
		{.offset = 0,
		.length = 8},
	}
};

/* For SPF */
static struct nand_ecclayout nx_nand_oob_16 = {
	.eccbytes = 16,
	.eccpos = {
		4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
	.oobfree = {
		{.offset = 0,
		.length = 4},
	}
};
#endif 
#endif

/**
 *  Flash based BBT information for LPF
 */

#ifdef CONFIG_MTD_NX_NAND_HWECC

static uint8_t nx_bbt_pattern[] = {'N', 'X', 'P' };
static uint8_t nx_mirror_pattern[] = {'P', 'X', 'N' };

static struct nand_bbt_descr nx_bbt_main = {
	.options =	NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE |
					NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs =	0,
	.len = 3,
	.veroffs = 3,
	.maxblocks = 4,
	.pattern = nx_bbt_pattern
};

static struct nand_bbt_descr nx_bbt_mirror = {
	.options =	NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE |
					NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs =	0,
	.len = 3,
	.veroffs = 3,
	.maxblocks = 4,
	.pattern = nx_mirror_pattern
};

#endif

/**
 * NAND control structure
 */
struct nx_nand_ctrl *nx_nc = NULL;
EXPORT_SYMBOL(nx_nc);


/**
 * MTD command line partition parser 
 */
/*We don't need nxpart parser */
/*const char *part_probes[] = { "cmdlinepart", "nxpart", NULL };*/

const char *part_probes[] = { "cmdlinepart", NULL };

/*----------------------------------------------------------------------------
* Internal functions 
------------------------------------------------------------------------------*/
#ifdef CONFIG_MTD_NX_NAND_DMAC
/**
 * nx_nand_dmac_init - Configure the DMAC scatter gather list
 * @nc: NAND control structure
 * @read: Read command
 * @req: Request structure
 * @stgt: Scatter gather array
 *
 * Initialise the DMAC scatter gather list
 */
static inline void nx_nand_dmac_init(struct nx_nand_ctrl *nc, 
	uint32_t cmd, nx_dmac_tfr_t *req, nx_dmac_stgt_t *stgt)
{
	int i;

	if(cmd) {

		/* Read Main Area */
		for(i=0; i < nc->num_blks; i++) {
			stgt[i].src_addr = NX_NAND_AHB_BUF;
			stgt[i].dst_addr = nc->dmabuf_phy + (i * nc->blk_size);
			stgt[i].tfr_size = nc->blk_size >> 2;	  
			stgt[i].flowctl = nx_dmac_per2mem_dma; 		  
			stgt[i].src_per = 0; 				 
			stgt[i].dst_per = 0;
			stgt[i].src_ahb = 1; 				 /* Source AHB master 1 */
			stgt[i].dst_ahb = 0; 				 /* Dest AHB master 0 */
			stgt[i].src_inc = true;
			stgt[i].dst_inc = true;
			if ( nc->blk_size == NX_NAND_BLK_SIZE ) {
				stgt[i].src_brst = nx_dmac_256;
				stgt[i].dst_brst = nx_dmac_256;
			} 
			else {
				stgt[i].src_brst = nx_dmac_128;
				stgt[i].dst_brst = nx_dmac_128;
			}
			stgt[i].src_width = nx_dmac_width_32;
			stgt[i].dst_width = nx_dmac_width_32;
		}

		/* Read OOB area */
		stgt[i].src_addr = NX_NAND_AHB_BUF + NX_NAND_BLK_SIZE;
		stgt[i].dst_addr = nc->dmabuf_phy + (i * nc->blk_size);
		stgt[i].tfr_size = nc->mtd.oobsize >> 2;	  
		stgt[i].flowctl = nx_dmac_per2mem_dma; 		  
		stgt[i].src_ahb = 1; 				 /* Source AHB master 1 */
		stgt[i].dst_ahb = 0; 				 /* Dest AHB master 0 */
		stgt[i].src_per = 0; 				 
		stgt[i].dst_per = 0;
		stgt[i].src_inc = true;
		stgt[i].dst_inc = true;
		if(nc->mtd.oobsize == 448) {
			stgt[i].src_brst = nx_dmac_128;
			stgt[i].dst_brst = nx_dmac_128;
		}
		else if(nc->mtd.oobsize == 224) {
			stgt[i].src_brst = nx_dmac_64;
			stgt[i].dst_brst = nx_dmac_64;
		}
		else if(nc->mtd.oobsize == 128) {
			stgt[i].src_brst = nx_dmac_32;
			stgt[i].dst_brst = nx_dmac_32;
		}
		else if(nc->mtd.oobsize == 64) {
			stgt[i].src_brst = nx_dmac_16;
			stgt[i].dst_brst = nx_dmac_16;
		}
		else {
			stgt[i].src_brst = nx_dmac_4;
			stgt[i].dst_brst = nx_dmac_4;
		}
		stgt[i].src_width = nx_dmac_width_32;
		stgt[i].dst_width = nx_dmac_width_32;
	} 
	else { 
		/* Write OOB area */
		i=0;
		stgt[i].dst_addr = NX_NAND_AHB_BUF + NX_NAND_BLK_SIZE;	  
		stgt[i].src_addr = nc->dmabuf_phy + (nc->num_blks * nc->blk_size);
		stgt[i].tfr_size = nc->mtd.oobsize >> 2;	  
		stgt[i].flowctl = nx_dmac_mem2per_dma; 		  
		stgt[i].src_ahb = 0; 				 /* Source AHB master 0 */
		stgt[i].dst_ahb = 1; 				 /* Dest AHB master 1 */
		stgt[i].src_per = 0; 				 
		stgt[i].dst_per = 0;
		stgt[i].src_inc = true;
		stgt[i].dst_inc = true;
		if(nc->mtd.oobsize == 448) {
			stgt[i].src_brst = nx_dmac_128;
			stgt[i].dst_brst = nx_dmac_128;
		}
		else if(nc->mtd.oobsize == 224) {
			stgt[i].src_brst = nx_dmac_64;
			stgt[i].dst_brst = nx_dmac_64;
		}
		else if(nc->mtd.oobsize == 128) {
			stgt[i].src_brst = nx_dmac_32;
			stgt[i].dst_brst = nx_dmac_32;
		}
		else if(nc->mtd.oobsize == 64) {
			stgt[i].src_brst = nx_dmac_16;
			stgt[i].dst_brst = nx_dmac_16;
		}
		else {
			stgt[i].src_brst = nx_dmac_4;
			stgt[i].dst_brst = nx_dmac_4;
		}
		stgt[i].src_width = nx_dmac_width_32;
		stgt[i].dst_width = nx_dmac_width_32;

		/* Write Main area */
		for(i=1; i < (nc->num_blks+1); i++) {
			stgt[i].src_addr = nc->dmabuf_phy + ( (i-1) * nc->blk_size);
			stgt[i].dst_addr = NX_NAND_AHB_BUF;
			stgt[i].tfr_size = nc->blk_size >> 2;
			stgt[i].flowctl = nx_dmac_mem2per_dma;
			stgt[i].src_per = 0;
			stgt[i].dst_per = 0;
			stgt[i].src_ahb = 0; 				 /* Source AHB master 0 */
			stgt[i].dst_ahb = 1; 				 /* Dest AHB master 1 */
			stgt[i].src_inc = true;
			stgt[i].dst_inc = true;
			if ( nc->blk_size == NX_NAND_BLK_SIZE ) {
				stgt[i].src_brst = nx_dmac_256;
				stgt[i].dst_brst = nx_dmac_256;
			} 
			else {
				stgt[i].src_brst = nx_dmac_128;
				stgt[i].dst_brst = nx_dmac_128;
			}
			stgt[i].src_width = nx_dmac_width_32;
			stgt[i].dst_width = nx_dmac_width_32;
		}
	}

	req->num_reqs = nc->num_blks + 1;
	req->req = &stgt[0];
}
#endif

/**
 * nx_nand_cmd_addr - Send cmd & address cycles to chip
 * @nc: NAND control structure
 * @cmd: Command to be send
 * @data: command or data
 * @last: Last cycle
 *
 * Send command & address cycles to chip for small page chips
 */
static inline void nx_nand_cmd_addr(struct nx_nand_ctrl *nc, uint32_t cmd,
	uint32_t data, int last)
{
	tmhwEfmc_CmdAddr_t	cmd_addr;

	/* Chip enable */
	cmd_addr.deviceNum = (0 << (nc->slotid + NX_NAND_CMD_FIFO_CE_START));

	/* Cmd or address */
	switch(cmd) {

		case NX_NAND_CMD_FIFO_ADDR_CYC:
			cmd_addr.cycleType = tmhwEfmc_AddrCycle;
		break;

		case NX_NAND_CMD_FIFO_CMD_CYC:
			cmd_addr.cycleType = tmhwEfmc_CmdCycle;
		break;

		case NX_NAND_CMD_FIFO_POST_CMD:
			cmd_addr.cycleType = tmhwEfmc_PostWrCmdCycle;
		break;

		default:
			printk(KERN_ERR "nx_nand: Invalid cmd \r\n"); 
		return;
	}

	/* Last cmd or addr cycle */
	if(last)
		cmd_addr.lastCycle = TM_TRUE;
	else
		cmd_addr.lastCycle = TM_FALSE;

	/* Addr or Cmd */
	cmd_addr.data = data;

	/* Send to chip */
	tmhwEfmc_WriteCmdAddr(0, &cmd_addr);
}

/*----------------------------------------------------------------------------
* NAND chip specific functions 
------------------------------------------------------------------------------*/
#ifdef CONFIG_MTD_NX_NAND_HWECC
/**
 * nx_nand_calculate_ecc - HW ECC calculate
 * @mtd: MTD information structure
 * @dat: Databuffer
 * @ecc_code: ECC code buffer
 *
 * Dummy function for HW ECC calculation
 */
static int nx_nand_calculate_ecc(struct mtd_info *mtd ATTRIBUTE_UNUSED, 
	const uint8_t *dat ATTRIBUTE_UNUSED,uint8_t *ecc_code ATTRIBUTE_UNUSED)
{
	return 0;
}

/**
 * nx_nand_correct_data - HW ECC correct
 * @mtd: MTD information structure
 * @dat: Databuffer
 * @read_ecc: Read ECC code buffer
 * @calc_ecc: Calculated ECC buffer
 *
 * Dummy function for HW ECC calculation
 */
static int nx_nand_correct_data(struct mtd_info *mtd ATTRIBUTE_UNUSED, 
	uint8_t *dat ATTRIBUTE_UNUSED, uint8_t *read_ecc ATTRIBUTE_UNUSED, 
	uint8_t *calc_ecc ATTRIBUTE_UNUSED)
{
	return 0;
}

/**
 * nx_nand_hwctl - HW ECC control function
 * @mtd: MTD information structure
 * @mode: Mode
 *
 * Dummy function for HW ECC calculation
 */
static void nx_nand_hwctl(struct mtd_info *mtd ATTRIBUTE_UNUSED, 
	int mode ATTRIBUTE_UNUSED)
{
	return;
}
#endif

/**
 * nx_nand_select_chip - Enable or Disable chip
 * @mtd: MTD information structure
 * @chipnr: Chip number
 *
 * Enable the chip if it chipnr >= 0, else disable the chip
 */
static void nx_nand_select_chip(struct mtd_info *mtd, int chipnr)
{
	struct nx_nand_ctrl *nc = container_of(mtd, struct nx_nand_ctrl, mtd);

	if(chipnr == -1)
		return;

	/* Store the value in nand control structure 
	 * Chip enable/disable done in command function */
	nc->slotid = chipnr;
}

/**
 * nx_nand_dev_ready - Check device ready
 * @mtd: MTD information structure
 *
 * Return true if the device is ready, false otherwise
 */
static int nx_nand_dev_ready(struct mtd_info *mtd)
{
	tmhwEfmc_ReadBusySignal_t dev_stat;
	struct nx_nand_ctrl *nc = container_of(mtd, struct nx_nand_ctrl, mtd);

	/* Get status from controller */
	tmhwEfmc_GetBusyStatus(nc->unitid, nc->slotid, &dev_stat);

	/* Return with R/B status */
	return (dev_stat.rbEdge_Status_Ready);
}

/**
 * nx_nand_read_byte - Read a byte from chip
 * @mtd: MTD information structure
 *
 * Read a byte from the nand chip
 */
static uint8_t nx_nand_read_byte(struct mtd_info *mtd)
{
	struct nx_nand_ctrl *nc = container_of(mtd, struct nx_nand_ctrl, mtd);
	uint16_t data;

	/* Read 16-bit word */
	tmhwEfmc_ReadSingleData(nc->unitid, &data);

	return (uint8_t) cpu_to_le16(data);
}

/**
 * nx_nand_read_byte16 - Read a byte from 16bit chip
 * @mtd: MTD information structure
 *
 * Read a byte from the 16bit nand chip
 */
static uint8_t nx_nand_read_byte16(struct mtd_info *mtd)
{
	struct nx_nand_ctrl *nc = container_of(mtd, struct nx_nand_ctrl, mtd);
	uint16_t data;

	/* Read byte word */
	tmhwEfmc_ReadSingleData(nc->unitid, &data);

	return (uint8_t) cpu_to_le16(data);
}

/**
 * nx_nand_read_buf - Read data from chip
 * @mtd: MTD information structure
 * @buf: Data buffer
 * @len: Transfer size
 *
 * Read specified number of bytes from the driver buffer
 */
static void nx_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nx_nand_ctrl *nc = container_of(mtd, struct nx_nand_ctrl, mtd);

	/* Copy from driver buffer */
	memcpy(buf, nc->dmabuf + nc->offset, len);
	nc->offset += len;
}

/**
 * nx_nand_read_page_raw - Read 1 page data from chip
 * @mtd: MTD information structure
 * @chip: Chip information structure
 * @buf: Data buffer
 *
 * Read a full page + oob into the buffer
 */
static int nx_nand_read_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int oob_required, int page)
{
	chip->read_buf(mtd, buf, mtd->writesize);

	if (oob_required)
		chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);

	return 0;
} 

#ifdef CONFIG_MTD_NX_NAND_HWECC
/**
 * nx_nand_read_page - Read 1 page data from chip with HWECC
 * @mtd: MTD information structure
 * @chip: Chip information structure
 * @buf: Data buffer
 *
 * Read a full page + oob into the buffer
 */
static int nx_nand_read_page(struct mtd_info *mtd, struct nand_chip *chip,
			uint8_t *buf, int oob_required, int page)
{
	int i, stat=0;
	struct nx_nand_ctrl *nc = container_of(mtd, struct nx_nand_ctrl, mtd);
	int eccsteps = chip->ecc.steps;
#ifdef CONFIG_ARCH_APOLLO
	uint32_t j;
	int no_all_ffs=0;
#endif

	/* Read page data */
	nx_nand_read_page_raw(mtd, chip, buf, oob_required, page);

	/* Check ECC status */
	for (i = 0 ; i<eccsteps; i++) {
#ifdef CONFIG_ARCH_APOLLO
		if(nc->ecc_status[i] == NX_NAND_INT_DEC_UNCOR) {

			for(j=0; j < mtd->writesize; j++) {
				if(buf[j] != 0xFF) {
					no_all_ffs = 1;
					break;
				}
			}

			if(no_all_ffs) {
				printk(KERN_INFO "nx_nand: [INFO] step %d: ECC failed [page = %x] \r\n",
					i, nc->cur_page);
				mtd->ecc_stats.failed++;
			}
		} 
		else {
			/* Update stats */
			switch (nc->ecc_status[i]) {
				case NX_NAND_INT_DEC_1_ERR:
					printk(KERN_INFO "nx_nand: [INFO] step %d:ECC 1 bit corrected [page = %x] \r\n", i, 
						nc->cur_page);
					stat = 1;
				break;

				case NX_NAND_INT_DEC_2_ERR:
					printk(KERN_INFO "nx_nand: [INFO] step %d:ECC 2 bits corrected [page = %x] \r\n", i, 
						nc->cur_page);
					stat = 2;
				break;

				case NX_NAND_INT_DEC_3_ERR:
					printk(KERN_INFO "nx_nand: [INFO] step %d:ECC 3 bits corrected [page = %x] \r\n", i, 
						nc->cur_page);
					stat = 3;
				break;

				case NX_NAND_INT_DEC_4_ERR:
					printk(KERN_INFO "nx_nand: [INFO] step %d:ECC 4 bits corrected [page = %x] \r\n", i, 
						nc->cur_page);
					stat = 4;
				break;

				case NX_NAND_INT_DEC_5_ERR:
					printk(KERN_INFO "nx_nand: [INFO] step %d:ECC 5 bits corrected [page = %x] \r\n", i, 
						nc->cur_page);
					stat = 5;
				break;
			}
			mtd->ecc_stats.corrected += stat;
		}
#elif (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
		if (nc->page_rw_status_fifo.PageRdStatus[i].uncorrectable) {
			printk(KERN_INFO "nx_nand: [INFO] step %d:uncorrectable ecc [page = %x] \r\n", i, 
				nc->cur_page);
			mtd->ecc_stats.failed++;
		} 
		/* High density MLC NAND flashes generate too many correctable 
		 * ecc errors, on detecting these errors filessystem aggressively 
		 * marks ecc correctable physical erase blocks as bad blocks.
		 * We devise a heuristic here by defining a correctable ecc error
		 * threshold equal to half of set ecc_level and logging errors only
		 *  when threshold is hit.
		 */
		else if (nc->page_rw_status_fifo.PageRdStatus[i].corrected) {
			stat = nc->page_rw_status_fifo.PageRdStatus[i].num_errors;
			if (stat > (nc->flash_config.eccLevel >> 1)) {
				printk(KERN_INFO "nx_nand: [INFO] step %d:ECC %d bits corrected [page = %x] \r\n", i, 
					stat,nc->cur_page );
				mtd->ecc_stats.corrected += stat;
			}
		}
#endif
	}
	return 0;
}
#endif

/**
 * nx_nand_read_oob - Read OOB data
 * @mtd: MTD information structure
 * @chip: Chip information structure
 * @page: Page address
 * @sndcmd: Send command flag
 *
 * Read OOB data into the buffer
 */
static int nx_nand_read_oob(struct mtd_info *mtd, 
							struct nand_chip *chip, int page)
{
	struct nx_nand_ctrl *nc = container_of(mtd, struct nx_nand_ctrl, mtd);
	uint8_t *buf = chip->oob_poi;
	int status = 0;
	int length = mtd->oobsize;
	int column, addr, i;
	uint16_t data;
	tmhwEfmc_PageConfig_t	page_cfg;

	if(mtd->flags & MTD_USE_DEV_OOB_LAYOUT) {

		/* Use Device OOB layout (Main page data followed by OOB data) */ 	
		/* No page operation for OOB */
		page_cfg.includeOOB = true;
		page_cfg.operType = tmhwEfmc_Nothing;
		page_cfg.includeAES = false;
		page_cfg.includeECC = false;
		tmhwEfmc_InitPageOp(nc->unitid, &page_cfg);	

		/* Send READOOB command */ 
		chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);

		/* Check if CE DON't care is supported */
		column = nc->cur_col;
		i = 0;
		while(length) {

			/* Send Address & cmd cycles */
			nx_nand_cmd_addr(nc, 1, nc->cur_cmd, 0);

			if(nc->lb_chip) {
				addr = column & NX_NAND_SP_ADDR_MASK;
				nx_nand_cmd_addr(nc, 0, addr, 0);

				addr = (column >> 8) & NX_NAND_SP_ADDR_MASK;
				nx_nand_cmd_addr(nc, 0, addr, 0);

				addr = nc->cur_page & NX_NAND_SP_ADDR_MASK;
				nx_nand_cmd_addr(nc, 0, addr, 0);

				addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
				nx_nand_cmd_addr(nc, 0, addr, 0);

				/* if > 2Gb, extra address cycle */
				if (nc->chip.chipsize >= (1 << 28)) {
					addr = (nc->cur_page >> 16) & NX_NAND_SP_ADDR_MASK;
					nx_nand_cmd_addr(nc, 0, addr, 0);
				}

				/* Send Read confirm command */
				nx_nand_cmd_addr(nc, 1, NAND_CMD_READSTART, 1);
			} 
			else {
				addr = column & NX_NAND_SP_ADDR_MASK;
				nx_nand_cmd_addr(nc, 0, addr, 0);

				addr = nc->cur_page & NX_NAND_SP_ADDR_MASK;
				nx_nand_cmd_addr(nc, 0, addr, 0);

				/* if > 64Mb, extra adddress cycle */
				if (nc->chip.chipsize > (32 << 20)) {
					addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
					nx_nand_cmd_addr(nc, 0, addr, 0);

					addr = (nc->cur_page >> 16) & NX_NAND_SP_ADDR_MASK;
					nx_nand_cmd_addr(nc, 0, addr, 1);
				} 
				else {
					addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
					nx_nand_cmd_addr(nc, 0, addr, 1);
				}
			}

			/* Wait for completion */
			udelay(chip->chip_delay);

			/* Write data into chip */
			status  = tmhwEfmc_ReadSingleData(nc->unitid, &data); 	
			buf[i] = (uint8_t) data; 
			//printk(KERN_INFO "nx_nand: ReadSingle data 0x%x 0x%x \r\n", buf[i], data);

			length--;
			i++;

			if(!nx_nc->cedontcare)
				column++;
		}
	} 
	else {
		/* Use IP_2017 OOB layout - (512 Bytes data + 16 bytes OOB data) */
		/* Use IP_2017 OOB layout - Read page */

		/* Send READOOB command */ 
		chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);

		/* Copy into user buffer */
		nc->offset = mtd->writesize;
		chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);
	}

	return 0;
}

/**
 * nx_nand_write_buf - Write data into chip
 * @mtd: MTD information structure
 * @buf: Data buffer
 * @len: Transfer size
 *
 * Write specified number of bytes into the nand chip
 */
static void nx_nand_write_buf(struct mtd_info *mtd, const uint8_t *buf, 
	int len)
{
	struct nx_nand_ctrl *nc = container_of(mtd, struct nx_nand_ctrl, mtd);

	/* Copy data to driver buffer */
	memcpy(nc->dmabuf + nc->offset, buf, len);
	nc->offset += len;
}

/**
 * nx_nand_write_page_raw - Write 1 page data into chip
 * @mtd: MTD information structure
 * @chip: Chip information structure
 * @buf: Data buffer
 *
 * Write a full page + oob into the buffer
 */
static int nx_nand_write_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
			const uint8_t *buf, int oob_required)
{
	struct nx_nand_ctrl *nc = container_of(mtd, struct nx_nand_ctrl, mtd);
	uint32_t intr=0;
	uint16_t addr;
	tmhwEfmc_PageConfig_t page_cfg;
#ifdef CONFIG_MTD_NX_NAND_DMAC
	int chanid, status;
	tmhwEfmc_DmaConfig_t dma_cfg;
	nx_dmac_tfr_t			req;
	nx_dmac_stgt_t 		stgt[(mtd->writesize/nc->blk_size)+1];
#endif

	/* Copy data into buffer */
	chip->write_buf(mtd, buf, mtd->writesize);
	chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);

#ifdef CONFIG_MTD_NX_NAND_DMAC	
	/* Scatter gather list for DMAC */
	nx_nand_dmac_init(nc, 0, &req, stgt);

	/* Flow control */
	dma_cfg.enableM2PDma = tmhwEfmc_Enable;
	dma_cfg.enableP2MDma = tmhwEfmc_Disable;
	tmhwEfmc_SetDmaConfig(nc->unitid, &dma_cfg);

	chanid = nx_dmac_tfr(&req);
	if(chanid < 0) {
		printk(KERN_ERR "nx_nand: nx_dmac_tfr \r\n"); 
		return -1;
	}

	/* Page operation */
	page_cfg.includeOOB = true;
	page_cfg.operType = tmhwEfmc_PageWrite;
	if(nc->aes) {
		page_cfg.includeAES = true;
	} 
	else {
		page_cfg.includeAES = false;
	}
	if(nc->hwecc) {
		page_cfg.includeECC = true;
	} 
	else {
		page_cfg.includeECC = false;
	}
	tmhwEfmc_InitPageOp(nc->unitid, &page_cfg);

	/* Clear the interrupts */
	intr = NX_NAND_INT_SEQ_WRITE;
	intr |= 1 << (NX_NAND_INT_READY_START + nc->slotid);
	tmhwEfmc_IntClear(nc->unitid, intr);

	/* Enable the SEQ READ PAGE DONE interrupt */
	tmhwEfmc_IntEnable(nc->unitid, intr);

	/* Send address cycles & command */
	nc->done = false;
	nx_nand_cmd_addr(nc, 1, NAND_CMD_SEQIN, 0);

	if(nc->lb_chip) {
		addr = nc->cur_col & NX_NAND_SP_ADDR_MASK;
		nx_nand_cmd_addr(nc, 0, addr, 0);

		addr = (nc->cur_col >> 8) & NX_NAND_SP_ADDR_MASK;
		nx_nand_cmd_addr(nc, 0, addr, 0);

		addr = nc->cur_page & NX_NAND_SP_ADDR_MASK;
		nx_nand_cmd_addr(nc, 0, addr, 0);

		addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
		nx_nand_cmd_addr(nc, 0, addr, 0);

		/* if > 2Gb, extra address cycle */
		if (nc->chip.chipsize >= (1 << 28)) {
			addr = (nc->cur_page >> 16) & NX_NAND_SP_ADDR_MASK;
			nx_nand_cmd_addr(nc, 0, addr, 0);
		}
	} 
	else {
		addr = nc->cur_col & NX_NAND_SP_ADDR_MASK;
		nx_nand_cmd_addr(nc, 0, addr, 0);

		addr = nc->cur_page & NX_NAND_SP_ADDR_MASK;
		nx_nand_cmd_addr(nc, 0, addr, 0);

		addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
		nx_nand_cmd_addr(nc, 0, addr, 0);

		/* if > 64Mb, extra adddress cycle */
		if (nc->chip.chipsize > (32 << 20)) {
			addr = (nc->cur_page >> 16) & NX_NAND_SP_ADDR_MASK;
			nx_nand_cmd_addr(nc, 0, addr, 0);
		}
	}

	/* Send post write command */
	nx_nand_cmd_addr(nc, 2, NAND_CMD_PAGEPROG, 1);

	/* Complete DMAC transfer */
	status = nx_dmac_tfr_comp(chanid);
	if(status) {
		printk(KERN_ERR "nx_nand: write_page_raw \r\n");
		return -1;
	}

	/* Wait for READY interrupt */	
	wait_event(nc->nand_queue, (nc->done != false));

	/* Disable interrupts */	
	tmhwEfmc_IntDisable(nc->unitid, intr);

	/* Disable Flow control */
	dma_cfg.enableM2PDma = tmhwEfmc_Disable;
	dma_cfg.enableP2MDma = tmhwEfmc_Disable;
	tmhwEfmc_SetDmaConfig(nc->unitid, &dma_cfg);

#else 

	/* Init page operation */
	nc->blk_index = 0;

	/* Send Cmd & address to chip */
	page_cfg.includeOOB = true;
	page_cfg.operType = tmhwEfmc_PageWrite;
	if(nc->aes) {
		intr |= NX_NAND_INT_AES_DEC;
		page_cfg.includeAES = true;
	}
	else {
		page_cfg.includeAES = false;
	}
	if(nc->hwecc) {
		intr |= NX_NAND_INT_ENC;
		page_cfg.includeECC = true;
	}
	else {
		page_cfg.includeECC = false;
		intr |= NX_NAND_INT_BLK_WRITE;
	}
	tmhwEfmc_InitPageOp(nc->unitid, &page_cfg);	

	/* Int Enable */
	intr |= (NX_NAND_INT_OOB_WRITE | 
		(1 << (NX_NAND_INT_READY_START + nc->slotid)));
	tmhwEfmc_IntClear(nc->unitid, intr);
	tmhwEfmc_IntEnable(nc->unitid, intr);

	/* Send the address commands to chip */
	nc->done = false;
	nx_nand_cmd_addr(nc,1, NAND_CMD_SEQIN, 0);

	if(nc->lb_chip) {
		addr = nc->cur_col & NX_NAND_SP_ADDR_MASK;
		nx_nand_cmd_addr(nc, 0, addr, 0);

		addr = (nc->cur_col >> 8) & NX_NAND_SP_ADDR_MASK;
		nx_nand_cmd_addr(nc, 0, addr, 0);

		addr = nc->cur_page & NX_NAND_SP_ADDR_MASK;
		nx_nand_cmd_addr(nc, 0, addr, 0);

		addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
		nx_nand_cmd_addr(nc, 0, addr, 0);

		if (nc->chip.chipsize >= (1 << 28)) {
			addr = (nc->cur_page >> 16) & NX_NAND_SP_ADDR_MASK;
			nx_nand_cmd_addr(nc, 0, addr, 0);
		}
	}
	else {
		addr = nc->cur_col & NX_NAND_SP_ADDR_MASK;
		nx_nand_cmd_addr(nc, 0, addr, 0);

		addr = nc->cur_page & NX_NAND_SP_ADDR_MASK;
		nx_nand_cmd_addr(nc, 0, addr, 0);

		addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
		nx_nand_cmd_addr(nc, 0, addr, 0);

		if (nc->chip.chipsize > (32 << 20)) {
			addr = (nc->cur_page >> 16) & NX_NAND_SP_ADDR_MASK;
			nx_nand_cmd_addr(nc, 0, addr, 0);
		}
	}

	/* Post write command */
	nx_nand_cmd_addr(nc, 2, NAND_CMD_PAGEPROG, 1);

	/* Wait for READY interrupt */	
	wait_event(nc->nand_queue, (nc->done != false));

	/* Disable interrupts */
	tmhwEfmc_IntDisable(nc->unitid, intr);
#endif
	return 0;
}

#ifdef CONFIG_MTD_NX_NAND_HWECC
/**
 * nx_nand_write_page - Write 1 page data into chip when HW ECC enabled
 * @mtd: MTD information structure
 * @chip: Chip information structure
 * @buf: Data buffer
 *
 * Write a full page + oob into the buffer
 */
static int nx_nand_write_page(struct mtd_info *mtd, struct nand_chip *chip,
			const uint8_t *buf, int oob_required)
{
	return nx_nand_write_page_raw(mtd, chip, buf, 0);
}
#endif

#ifdef CONFIG_ARCH_APOLLO
#define _NAND_MAX_PAGESIZE 4096
#define _NAND_MAX_OOBSIZE	128
static u_char temp_buf[_NAND_MAX_PAGESIZE+_NAND_MAX_OOBSIZE];
#endif

/**
 * nx_nand_write_oob - Write OOB data
 * @mtd: MTD information structure
 * @chip: Chip information structure
 * @page: Page address
 *
 * Write OOB data into the chip
 */
static int nx_nand_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
	int page)
{
	int status = 0;
#ifdef CONFIG_ARCH_APOLLO
	struct nx_nand_ctrl *nc = container_of(mtd, struct nx_nand_ctrl, mtd);
	int sndcmd = 1;
	const uint8_t *buf = chip->oob_poi;
	int length;
	int column, addr, i;
	uint16_t data;
	tmhwEfmc_PageConfig_t	page_cfg;
	bool	ecc_old, aes_old;
	u_char *temp1;
	uint8_t	 *oob_poi_orig;
	u_int32_t remaining_data;

	if(mtd->flags & MTD_USE_DEV_OOB_LAYOUT) {
		/* Use Device OOB layout -  write only OOB data */ 	
		/* No page operation for OOB */
		page_cfg.includeOOB = true;
		page_cfg.operType = tmhwEfmc_Nothing;
		page_cfg.includeAES = false;
		page_cfg.includeECC = false;
		tmhwEfmc_InitPageOp(nc->unitid, &page_cfg);

		/* Send SEQIN command */
		chip->cmdfunc(mtd, NAND_CMD_SEQIN, mtd->writesize, page);

		/* Write to driver buffer */
		length = mtd->oobsize;
		chip->write_buf(mtd, buf, length);

		/* Check if CE DON't care is supported */
		column = nc->cur_col;
		i = 0;
		while(length) {
			/* Send Address & cmd cycles */
			if(sndcmd) {
				nx_nand_cmd_addr(nc, 1, NAND_CMD_SEQIN, 0);

				if(nc->lb_chip) {
					addr = column & NX_NAND_SP_ADDR_MASK;
					nx_nand_cmd_addr(nc, 0, addr, 0);

					addr = (column >> 8) & NX_NAND_SP_ADDR_MASK;
					nx_nand_cmd_addr(nc, 0, addr, 0);

					addr = nc->cur_page & NX_NAND_SP_ADDR_MASK;
					nx_nand_cmd_addr(nc, 0, addr, 0);

					/* if > 2Gb, extra address cycle */
					if (nc->chip.chipsize >= (1 << 28)) {
						addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
						nx_nand_cmd_addr(nc, 0, addr, 0);

						addr = (nc->cur_page >> 16) & NX_NAND_SP_ADDR_MASK;
						nx_nand_cmd_addr(nc, 0, addr, 1);
					}
					else {
						addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
						nx_nand_cmd_addr(nc, 0, addr, 1);
					}
				}
				else {
					addr = column & NX_NAND_SP_ADDR_MASK;
					nx_nand_cmd_addr(nc, 0, addr, 0);

					addr = nc->cur_page & NX_NAND_SP_ADDR_MASK;
					nx_nand_cmd_addr(nc, 0, addr, 0);

					/* if > 64Mb, extra adddress cycle */
					if (nc->chip.chipsize > (32 << 20)) {
						addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
						nx_nand_cmd_addr(nc, 0, addr, 0);

						addr = (nc->cur_page >> 16) & NX_NAND_SP_ADDR_MASK;
						nx_nand_cmd_addr(nc, 0, addr, 1);
					}
					else {
						addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
						nx_nand_cmd_addr(nc, 0, addr, 1);
					}
				}
				sndcmd = 0;
			}

			/* Write data into chip */
			data = buf[i];
			status  = tmhwEfmc_WriteSingleData(nc->unitid, data); 	

			length--;
			i++;

			if(!nx_nc->cedontcare) {
				sndcmd = 1; 
				column++;
			}
		}

		/* write  confirm command */
		nx_nand_cmd_addr(nc, 1, NAND_CMD_PAGEPROG, 1);
	}
	else 
	{
		const u_int32_t OOB_BYTES_PER_BLK = (mtd->oobsize/nc->num_blks);

		/* Use IP_2017 OOB layout - write page */
		/* Store ECC,AES values & Disable */	
		ecc_old = nc->hwecc;
		aes_old = nc->aes;
		oob_poi_orig = chip->oob_poi;
		nc->hwecc = false;
		nc->aes = false;

		/* Initialise temp_buf */
		memset(temp_buf, 0xff, (mtd->writesize + mtd->oobsize));

		/* Send SEQIN command */
		chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0, page);

		/* copy OOB */ 
		temp1 = temp_buf;
		for(i=0; i < nc->num_blks; i++) {
			temp1 += nc->blk_size;
			memcpy(temp1, (chip->oob_poi + (i * OOB_BYTES_PER_BLK)), 
				OOB_BYTES_PER_BLK);
			temp1 += OOB_BYTES_PER_BLK;
		}
		/* copy remaining oob bytes */
		remaining_data = (mtd->oobsize - (nc->num_blks * OOB_BYTES_PER_BLK));
		if ( remaining_data ) {
			memcpy(temp1, (chip->oob_poi + (i * OOB_BYTES_PER_BLK)), 
				remaining_data);
		}
		chip->oob_poi = &temp_buf[mtd->writesize];

		/* Call write page raw */
		nx_nand_write_page_raw(mtd, chip, temp_buf, page);

		/* Restore ECC,AES values */	
		chip->oob_poi = oob_poi_orig;
		nc->hwecc = ecc_old;
		nc->aes = aes_old;

		/* Send command to program the OOB data */
		chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

		status = chip->waitfunc(mtd, chip);
	}
#elif (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
	status = chip->waitfunc(mtd, chip);
#endif
	return status & NAND_STATUS_FAIL ? -EIO : 0;
}

#if (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
/**
 * ESP Layout (Page Size > = 1K) :
 * 	Page Size			= 1 K
 * 	Sub page Size		= 512 Bytes
 * 	Oob Size 			= 128 Bytes
 * 	ECC Level			= 32
 * 	Erased Page Thres = 16
 *
 * As per ESP layout 1K of uldr image data is physically 
 * written in 2 pages of NAND flash
 */

/**
 * nx_nand_read_uldr - Read requested page of uldr MTD part
 * @mtd: MTD information structure
 *
 * Read requested page of uldr MTD part with ESP layout
 */
void nx_nand_read_uldr (struct mtd_info *mtd)
{
	struct nx_nand_ctrl *nc = container_of(mtd, struct nx_nand_ctrl, mtd);
	tmhwEfmc_PageConfig_t page_cfg;
	tmhwEfmc_DmaConfig_t dma_cfg;
	nx_dmac_tfr_t req;
	nx_dmac_stgt_t stgt[(SZ_1K/SZ_512)+1];
	uint32_t intr=0;
	uint16_t addr;
	int status, chanid;

	tmhwEfmc_FlashConfig_t pPrevConfig, pConfig;
	tmErrorCode_t ret=0;
	int prev_blk_size;
	int prev_num_blks;
	uint32_t prev_oobsize;

	/* Memset FF's in DMA buffer */
	memset(nc->dmabuf, 0xFF, mtd->writesize + mtd->oobsize);

	/* Save current config */
	ret = tmhwEfmc_GetFlashConfig(nc->unitid, nc->slotid, &pPrevConfig);
	pConfig = pPrevConfig;
	pConfig.pageSize = tmhwEfmc_1024Byte; /* page_size = 1024 Bytes  */
	pConfig.subpageSize = tmhwEfmc_SubPage512Byte; /* sub_page_size = 512 Bytes */
	pConfig.oobSize = 128; /* oob_size = 128 B */
	pConfig.eccLevel = 32; /* ecc_level = 32 */
	pConfig.erasedPageThres = 16; /* erased_page_threshold = ecc_level/2 */
	ret = tmhwEfmc_SetFlashConfig(nc->unitid,nc->slotid, &pConfig);

	prev_num_blks = nc->num_blks;
	nc->num_blks = (SZ_1K/SZ_512);
	prev_blk_size = nc->blk_size;
	nc->blk_size = SZ_512;
	prev_oobsize = nc->mtd.oobsize;
	nc->mtd.oobsize = 128;

	nc->cur_col = 0;

	/* Start the DMAC */
	nx_nand_dmac_init(nc, 1, &req, stgt);
	
	/* Configure Flow control */
	dma_cfg.enableM2PDma = tmhwEfmc_Disable;
	dma_cfg.enableP2MDma = tmhwEfmc_Enable;
	tmhwEfmc_SetDmaConfig(nc->unitid, &dma_cfg);
	
	/* Enable the SEQ READ PAGE DONE interrupt */
	intr = NX_NAND_INT_SEQ_READ;
	tmhwEfmc_IntClear(nc->unitid, intr);
	tmhwEfmc_IntEnable(nc->unitid, intr);
	
	/* Page operation */
	page_cfg.includeOOB = true;
	page_cfg.operType = tmhwEfmc_PageRead;
	if(nc->aes) {
		page_cfg.includeAES = true;
	}
	else {
		page_cfg.includeAES = false;
	}
	if(nc->hwecc) {
		page_cfg.includeECC = true;
	}
	else {
		page_cfg.includeECC = false;
	}
	tmhwEfmc_InitPageOp(nc->unitid, &page_cfg);
	
	chanid = nx_dmac_tfr(&req);
	if(chanid < 0) {
		printk(KERN_ERR "nx_nand: NAND_READ0 DMAC config \r\n"); 
		goto err;
	}
	
	/* Send address cycles & command */
	nc->done = false;
	nx_nand_cmd_addr(nc, 1, NAND_CMD_READ0, 0);
	
	addr = nc->cur_col & NX_NAND_SP_ADDR_MASK;
	nx_nand_cmd_addr(nc, 0, addr, 0);
	
	addr = (nc->cur_col >> 8) & NX_NAND_SP_ADDR_MASK;
	nx_nand_cmd_addr(nc, 0, addr, 0);
	
	addr = nc->cur_page & NX_NAND_SP_ADDR_MASK;
	nx_nand_cmd_addr(nc, 0, addr, 0);
	
	addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
	nx_nand_cmd_addr(nc, 0, addr, 0);
	
	if (nc->chip.chipsize >= (1 << 28)) {
		addr = (nc->cur_page >> 16) & NX_NAND_SP_ADDR_MASK;
		nx_nand_cmd_addr(nc, 0, addr, 0); 
	}
	
	nx_nand_cmd_addr(nc, 1, NAND_CMD_READSTART, 1);
	
	/* Wait for the completion */ 
	wait_event(nc->nand_queue, (nc->done != false));
	
	/* Complete DMAC transfer */
	status = nx_dmac_tfr_comp(chanid);
	if(status) {
		printk(KERN_ERR "nx_nand: NAND_READ0 DMAC complete\r\n");	
		goto err;
	}
	
	/* Disable interrupts */
	tmhwEfmc_IntDisable(nc->unitid, intr);
	
	/* Disable Flow control */
	dma_cfg.enableM2PDma = tmhwEfmc_Disable;
	dma_cfg.enableP2MDma = tmhwEfmc_Disable;
	tmhwEfmc_SetDmaConfig(nc->unitid, &dma_cfg);

err:
	/* Restore previos config */
	ret = tmhwEfmc_SetFlashConfig(nc->unitid,nc->slotid, &pPrevConfig);
	nc->num_blks = prev_num_blks;
	nc->blk_size = prev_blk_size;
	nc->mtd.oobsize = prev_oobsize;

	return;
}

/**
 * nx_nand_check_uldr_rw - Is this uldr MTD part ?
 * @mtd: MTD information structure
 *
 * Return true if page address falls in the uldr partition
 * Otherwise return false
 */
static inline bool nx_nand_check_uldr_rw (struct mtd_info *mtd)
{
	struct nx_nand_ctrl *nc = container_of(mtd, struct nx_nand_ctrl, mtd);
	struct mtd_erase_region_info *eraseregions = mtd->eraseregions;

	if (eraseregions) {
		struct nand_chip *chip = mtd->priv;
		uint64_t uldr_start, uldr_end;
		int page_offset;

		page_offset = (nc->cur_page << chip->page_shift);
		uldr_start = eraseregions->offset;
		uldr_end = (eraseregions->offset + mtd->size);

		if (((page_offset >= uldr_start) && (page_offset < uldr_end)) &&
				!strcmp(mtd->name, "uldr"))
			return true;
	}
	return false;
}
#endif

/**
 * nx_nand_command - Command function for small page chips
 * @mtd: MTD information structure
 * @cmd: Command
 * @column: Column address
 * @page_addr: Page address
 *
 * Command control function:
 */
static void nx_nand_command(struct mtd_info *mtd, unsigned int cmd,
	int column, int page_addr)
{
	struct nx_nand_ctrl *nc = container_of(mtd, struct nx_nand_ctrl, mtd);
	tmhwEfmc_PageConfig_t page_cfg;
	uint16_t addr;
	uint32_t intr=0;
	int i;

#ifdef CONFIG_MTD_NX_NAND_DMAC 
	nx_dmac_stgt_t stgt[(mtd->writesize/nc->blk_size)+1];
	tmhwEfmc_DmaConfig_t dma_cfg;
	nx_dmac_tfr_t req;
	int chanid;
	int status;
#endif

	/* Store the command, colmn & page address */
	nc->cur_cmd = cmd;
	if(column == -1)
		column = 0;
	nc->cur_col = column;
	if(page_addr == -1)
		page_addr = 0;
	nc->cur_page = page_addr;

	/*
	 * Issue the correct first command, when we write to
	 * the device.
	 */
	switch(cmd) {
		case NAND_CMD_SEQIN:
			nc->offset = 0;
			/* Address cycles & command will be sent in write_page_raw */
			break;

		case NAND_CMD_PAGEPROG:
			/* Post write command feature used */
			break;

		case NAND_CMD_RESET:
			/* No page operation */
			page_cfg.includeOOB = false;
			page_cfg.operType = tmhwEfmc_Nothing;
			page_cfg.includeAES = false;
			page_cfg.includeECC = false;
			tmhwEfmc_InitPageOp(nc->unitid, &page_cfg);

			nx_nand_cmd_addr(nc, 1, cmd, 1);
			udelay(20); /* FIXME */
			break;

		case NAND_CMD_STATUS:
			/* No page operation */
			page_cfg.includeOOB = false;
			page_cfg.operType = tmhwEfmc_Nothing;
			page_cfg.includeAES = false;
			page_cfg.includeECC = false;
			tmhwEfmc_InitPageOp(nc->unitid, &page_cfg);

			nx_nand_cmd_addr(nc, 1, cmd, 1);
			break;

		case NAND_CMD_ERASE1:
			/* Enable READY interupt */
			intr |= (1 << (NX_NAND_INT_READY_START + nc->slotid));
			tmhwEfmc_IntClear(nc->unitid, intr);
			tmhwEfmc_IntEnable(nc->unitid, intr);

			/* No page operation */
			page_cfg.includeOOB = false;
			page_cfg.operType = tmhwEfmc_Nothing;
			page_cfg.includeAES = false;
			page_cfg.includeECC = false;
			tmhwEfmc_InitPageOp(nc->unitid, &page_cfg);	

			/* Send address cycles & command */
			nc->done = false;

			nx_nand_cmd_addr(nc, 1, NAND_CMD_ERASE1, 0);

			addr = nc->cur_page & NX_NAND_SP_ADDR_MASK;
			nx_nand_cmd_addr(nc, 0, addr, 0);

			addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
			nx_nand_cmd_addr(nc, 0, addr, 0);

			if (nc->chip.chipsize > (32 << 20)) {
				addr = (nc->cur_page >> 16) & NX_NAND_SP_ADDR_MASK;
				nx_nand_cmd_addr(nc, 0, addr, 0);
			}

			nx_nand_cmd_addr(nc, 1, NAND_CMD_ERASE2, 1);

			/* Wait for the completion */ 
			wait_event(nc->nand_queue, (nc->done != false));

			/* Disable READY interrupt */
			tmhwEfmc_IntDisable(nc->unitid, intr);

			break;

		case NAND_CMD_ERASE2:
			/* Already done in CMD_ERASE1 */
			break;

		case NAND_CMD_READ0:
			nc->offset = 0;
#ifdef CONFIG_MTD_NX_NAND_DMAC
			/* Start the DMAC */
			nx_nand_dmac_init(nc, 1, &req, stgt);

			/* Configure Flow control */
			dma_cfg.enableM2PDma = tmhwEfmc_Disable;
			dma_cfg.enableP2MDma = tmhwEfmc_Enable;
			tmhwEfmc_SetDmaConfig(nc->unitid, &dma_cfg);

			/* Enable the SEQ READ PAGE DONE interrupt */
			intr |= NX_NAND_INT_SEQ_READ;
			tmhwEfmc_IntClear(nc->unitid, intr);
			tmhwEfmc_IntEnable(nc->unitid, intr);

			/* Page operation */
			page_cfg.includeOOB = true;
			page_cfg.operType = tmhwEfmc_PageRead;
			if(nc->aes) {
				page_cfg.includeAES = true;
			}
			else {
				page_cfg.includeAES = false;
			}
			if(nc->hwecc) {
				page_cfg.includeECC = true;
			}
			else {
				page_cfg.includeECC = false;
			}
			tmhwEfmc_InitPageOp(nc->unitid, &page_cfg);	

			chanid = nx_dmac_tfr(&req);
			if(chanid < 0) {
				printk(KERN_ERR "nx_nand: NAND_READ0 DMAC config \r\n"); 
				return;
			}

			/* Send address cycles & command */
			nx_nand_cmd_addr(nc, 1, NAND_CMD_READ0, 0);

			addr = nc->cur_col & NX_NAND_SP_ADDR_MASK;
			nx_nand_cmd_addr(nc, 0, addr, 0);

			addr = nc->cur_page & NX_NAND_SP_ADDR_MASK;
			nx_nand_cmd_addr(nc, 0, addr, 0);

			if (nc->chip.chipsize > (32 << 20)) {
				addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
				nx_nand_cmd_addr(nc, 0, addr, 0);
				addr = (nc->cur_page >> 16) & NX_NAND_SP_ADDR_MASK;
				nx_nand_cmd_addr(nc, 0, addr, 1);
			}
			else {
				addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
				nx_nand_cmd_addr(nc, 0, addr, 1);
			}

			/* Complete DMAC transfer */
			status = nx_dmac_tfr_comp(chanid);
			if(status) {
				printk(KERN_ERR "nx_nand: NAND_READ0 DMAC complete\r\n");	
				return;
			}

			/* Disable interrupts */	
			tmhwEfmc_IntDisable(nc->unitid, intr);

			/* Disable Flow control */
			dma_cfg.enableM2PDma = tmhwEfmc_Disable;
			dma_cfg.enableP2MDma = tmhwEfmc_Disable;
			tmhwEfmc_SetDmaConfig(nc->unitid, &dma_cfg);
#else
			nc->blk_index = 0;

			/* Init page operation */
			page_cfg.includeOOB = true;
			page_cfg.operType = tmhwEfmc_PageRead;
			if(nc->aes) {
				intr |= NX_NAND_INT_AES_DEC;
				page_cfg.includeAES = true;
			}
			else {
				page_cfg.includeAES = false;
			}

			if(nc->hwecc) {
				intr |= NX_NAND_INT_DEC;
				page_cfg.includeECC = true;
			}
			else {
				page_cfg.includeECC = false;
				intr |= NX_NAND_INT_BLK_READ;
			}
			tmhwEfmc_InitPageOp(nc->unitid, &page_cfg);	

			intr |= NX_NAND_INT_OOB_READ;
			tmhwEfmc_IntClear(nc->unitid, intr);
			tmhwEfmc_IntEnable(nc->unitid, NX_NAND_INT_OOB_READ);

			/* Send the address commands to chip */
			nc->done = false;

			nx_nand_cmd_addr(nc, 1, NAND_CMD_READ0, 0);
			addr = nc->cur_col & NX_NAND_SP_ADDR_MASK;

			nx_nand_cmd_addr(nc, 0, addr, 0);
			addr = nc->cur_page & NX_NAND_SP_ADDR_MASK;

			nx_nand_cmd_addr(nc, 0, addr, 0);

			if (nc->chip.chipsize > (32 << 20)) {
				addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
				nx_nand_cmd_addr(nc, 0, addr, 0);
				addr = (nc->cur_page >> 16) & NX_NAND_SP_ADDR_MASK;
				nx_nand_cmd_addr(nc, 0, addr, 1);
			}
			else {
				addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
				nx_nand_cmd_addr(nc, 0, addr, 1);
			}

			/* Wait for the completion */ 
			wait_event(nc->nand_queue, (nc->done != false));

			/* Disable interrupts */
			tmhwEfmc_IntDisable(nc->unitid, intr);
#endif
			break;

		case NAND_CMD_READOOB:
			/* Offset to OOB area in driver buffer */
			nc->offset = mtd->writesize;
			nc->blk_index = (nc->num_blks * nc->blk_size);

			/* Enable the OOB block request */
			tmhwEfmc_IntClear(nc->unitid, NX_NAND_INT_OOB_READ);
			tmhwEfmc_IntEnable(nc->unitid, NX_NAND_INT_OOB_READ);

			/* Init page operation command */
			page_cfg.includeOOB = true;
			page_cfg.operType = tmhwEfmc_PageRead;
			if(nc->aes) {
				page_cfg.includeAES = true;
			}
			else {
				page_cfg.includeAES = false;
			}
			if(nc->hwecc) {
				page_cfg.includeECC = true;
			}
			else {
				page_cfg.includeECC = false;
			}
			tmhwEfmc_InitPageOp(nc->unitid, &page_cfg);	

			/* Send cmd & address cycles */
			nc->done = false;

			nx_nand_cmd_addr(nc, 1, NAND_CMD_READOOB, 0);
			addr = nc->cur_col & NX_NAND_SP_ADDR_MASK;

			nx_nand_cmd_addr(nc, 0, addr, 0);
			addr = nc->cur_page & NX_NAND_SP_ADDR_MASK;

			nx_nand_cmd_addr(nc, 0, addr, 0);

			if (nc->chip.chipsize > (32 << 20)) {
				addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
				nx_nand_cmd_addr(nc, 0, addr, 0);
				addr = (nc->cur_page >> 16) & NX_NAND_SP_ADDR_MASK;
				nx_nand_cmd_addr(nc, 0, addr, 1);
			}
			else {
				addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
				nx_nand_cmd_addr(nc, 0, addr, 1);
			}

			/* Wait for the completion */ 
			wait_event(nc->nand_queue, (nc->done != false));

			/* Disable the OOB block request */
			tmhwEfmc_IntDisable(nc->unitid, NX_NAND_INT_OOB_READ);

			break;

		case NAND_CMD_READID:
			/* No page operation */
			page_cfg.includeOOB = false;
			page_cfg.operType = tmhwEfmc_Nothing;
			page_cfg.includeAES = false;
			page_cfg.includeECC = false;
			tmhwEfmc_InitPageOp(nc->unitid, &page_cfg);

			nx_nand_cmd_addr(nc, 1, cmd, 0);
			nx_nand_cmd_addr(nc, 0, column, 1);
			break;

		case NAND_CMD_PARAM:
			nc->offset = 0;

			/* No page operation */
			page_cfg.includeOOB = false;
			page_cfg.operType = tmhwEfmc_Nothing;
			page_cfg.includeAES = false;
			page_cfg.includeECC = false;
			tmhwEfmc_InitPageOp(nc->unitid, &page_cfg);

			nx_nand_cmd_addr(nc, 1, cmd, 0);
			nx_nand_cmd_addr(nc, 0, 0x0, 1);

			/* Read PARAM Page and 2 Redundant Parameter Pages */
			for (i=0; i < (3 * sizeof(struct nand_onfi_params)); i++)
			{
				nc->dmabuf[i]=nx_nand_read_byte(mtd);
			}
			break;

		default:
			printk(KERN_ERR "nx_nand: command not supported %d \n", cmd);
	}
}

/**
 * nx_nand_command_lp - Command function for large page chips
 * @mtd: MTD information structure
 * @cmd: Command
 * @column: Column address
 * @page_addr: Page address
 *
 * Command control function:
 */
static void nx_nand_command_lp(struct mtd_info *mtd, unsigned int cmd,
	int column, int page_addr)
{
	struct nx_nand_ctrl *nc = container_of(mtd, struct nx_nand_ctrl, mtd);
	tmhwEfmc_PageConfig_t page_cfg;
	uint16_t addr;
	uint32_t intr=0;
	int i;

#ifdef CONFIG_MTD_NX_NAND_DMAC 
	nx_dmac_stgt_t stgt[(mtd->writesize/nc->blk_size)+1];
	tmhwEfmc_DmaConfig_t dma_cfg;
	nx_dmac_tfr_t req;
	int status; 
	int chanid;
#endif

	/* Store the command, colmn & page address */
	if(cmd == NAND_CMD_READOOB) {
		cmd = NAND_CMD_READ0;

		/* If Device OOB layout, read data in read_OOB function */ 
		if(mtd->flags & MTD_USE_DEV_OOB_LAYOUT) {
			nc->cur_cmd = cmd;
			nc->cur_col = mtd->writesize;
			nc->cur_page = page_addr;
			return;
		}
	}

	/* Store cmd, addresses */
	nc->cur_cmd = cmd;
	if(column == -1)
		column = 0;
	nc->cur_col = column;
	if(page_addr == -1)
		page_addr = 0;
	nc->cur_page =  (page_addr);

	/*
	 * Issue the correct first command, when we write to
	 * the device.
	 */
	switch(cmd) {
		case NAND_CMD_SEQIN:
			nc->offset = column;
			/* Address cycles & command will be sent in write_page_raw */
			break;

		case NAND_CMD_PAGEPROG:
			/* Post write command feature used */
			break;

		case NAND_CMD_RESET:
			/* No page operation */
			page_cfg.includeOOB = false;
			page_cfg.operType = tmhwEfmc_Nothing;
			page_cfg.includeAES = false;
			page_cfg.includeECC = false;
			tmhwEfmc_InitPageOp(nc->unitid, &page_cfg);

			nx_nand_cmd_addr(nc, 1, cmd, 1);
			udelay(20); /* FIXME */
			break;

		case NAND_CMD_STATUS:
			/* No page operation */
			page_cfg.includeOOB = false;
			page_cfg.operType = tmhwEfmc_Nothing;
			page_cfg.includeAES = false;
			page_cfg.includeECC = false;
			tmhwEfmc_InitPageOp(nc->unitid, &page_cfg);

			nx_nand_cmd_addr(nc, 1, cmd, 1);
			break;

		case NAND_CMD_ERASE1:
			/* Enable READY interupt */
			intr = 1 << (NX_NAND_INT_READY_START + nc->slotid);
			tmhwEfmc_IntClear(nc->unitid, intr);
			tmhwEfmc_IntEnable(nc->unitid, intr);

			/* No page operation */
			page_cfg.includeOOB = false;
			page_cfg.operType = tmhwEfmc_Nothing;
			page_cfg.includeAES = false;
			page_cfg.includeECC = false;
			tmhwEfmc_InitPageOp(nc->unitid, &page_cfg);

			/* Send address cycles & command */
			nc->done = false;

			nx_nand_cmd_addr(nc, 1, NAND_CMD_ERASE1, 0);

			addr = nc->cur_page & NX_NAND_SP_ADDR_MASK;
			nx_nand_cmd_addr(nc, 0, addr, 0);

			addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
			nx_nand_cmd_addr(nc, 0, addr, 0);

			if (nc->chip.chipsize >= (1 << 28)) {
				addr = (nc->cur_page >> 16) & NX_NAND_SP_ADDR_MASK;
				nx_nand_cmd_addr(nc, 0, addr, 0);
			}
			nx_nand_cmd_addr(nc, 1, NAND_CMD_ERASE2, 1);

			/* Wait for the completion */ 
			wait_event(nc->nand_queue, (nc->done != false));

			/* Disable READY interrupt */
			tmhwEfmc_IntDisable(nc->unitid, intr);
			break;

		case NAND_CMD_ERASE2:
			/* Already done in CMD_ERASE1 */
			break;

		case NAND_CMD_READ0:
			nc->offset = column;
#ifdef CONFIG_MTD_NX_NAND_DMAC

#if (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
			if(nx_nand_check_uldr_rw(mtd)) {
				nx_nand_read_uldr (mtd);
				break;
			}
#endif

			/* Start the DMAC */
			nx_nand_dmac_init(nc, 1, &req, stgt);

			/* Configure Flow control */
			dma_cfg.enableM2PDma = tmhwEfmc_Disable;
			dma_cfg.enableP2MDma = tmhwEfmc_Enable;
			tmhwEfmc_SetDmaConfig(nc->unitid, &dma_cfg);

			/* Enable the SEQ READ PAGE DONE interrupt */
			intr = NX_NAND_INT_SEQ_READ;
			tmhwEfmc_IntClear(nc->unitid, intr);
			tmhwEfmc_IntEnable(nc->unitid, intr);

			/* Page operation */
			page_cfg.includeOOB = true;
			page_cfg.operType = tmhwEfmc_PageRead;
			if(nc->aes) {
				page_cfg.includeAES = true;
			}
			else {
				page_cfg.includeAES = false;
			}
			if(nc->hwecc) {
				page_cfg.includeECC = true;
			}
			else {
				page_cfg.includeECC = false;
			}
			tmhwEfmc_InitPageOp(nc->unitid, &page_cfg);	

			chanid = nx_dmac_tfr(&req);
			if(chanid < 0) {
				printk(KERN_ERR "nx_nand: NAND_READ0 DMAC config \r\n"); 
				return;
			}

			/* Send address cycles & command */
			nc->done = false;
			nx_nand_cmd_addr(nc, 1, NAND_CMD_READ0, 0);

			addr = column & NX_NAND_SP_ADDR_MASK;
			nx_nand_cmd_addr(nc, 0, addr, 0);

			addr = (column >> 8) & NX_NAND_SP_ADDR_MASK;
			nx_nand_cmd_addr(nc, 0, addr, 0);

			addr = nc->cur_page & NX_NAND_SP_ADDR_MASK;
			nx_nand_cmd_addr(nc, 0, addr, 0);

			addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
			nx_nand_cmd_addr(nc, 0, addr, 0);

			if (nc->chip.chipsize >= (1 << 28)) {
				addr = (nc->cur_page >> 16) & NX_NAND_SP_ADDR_MASK;
				nx_nand_cmd_addr(nc, 0, addr, 0); 
			}

			nx_nand_cmd_addr(nc, 1, NAND_CMD_READSTART, 1);

			/* Wait for the completion */ 
			wait_event(nc->nand_queue, (nc->done != false));

			/* Complete DMAC transfer */
			status = nx_dmac_tfr_comp(chanid);
			if(status) {
				printk(KERN_ERR "nx_nand: NAND_READ0 DMAC complete\r\n");	
				return;
			}

			/* Disable interrupts */	
			tmhwEfmc_IntDisable(nc->unitid, intr);

			/* Disable Flow control */
			dma_cfg.enableM2PDma = tmhwEfmc_Disable;
			dma_cfg.enableP2MDma = tmhwEfmc_Disable;
			tmhwEfmc_SetDmaConfig(nc->unitid, &dma_cfg);
#else
			nc->blk_index = 0;

			/* Init page operation */
			page_cfg.includeOOB = true;
			intr |= NX_NAND_INT_OOB_READ;
			page_cfg.operType = tmhwEfmc_PageRead;
			if(nc->aes) {
				intr |= NX_NAND_INT_AES_DEC;
				page_cfg.includeAES = true;
			}
			else {
				page_cfg.includeAES = false;
			}

			if(nc->hwecc) {
				intr |= NX_NAND_INT_DEC;
				page_cfg.includeECC = true;
			}
			else {
				page_cfg.includeECC = false;
				intr |= NX_NAND_INT_BLK_READ;
			}
			tmhwEfmc_IntClear(nc->unitid, intr);
			tmhwEfmc_IntEnable(nc->unitid, intr);
			tmhwEfmc_InitPageOp(nc->unitid, &page_cfg);	

			/* Send the address commands to chip */
			nc->done = false;
			/* Send address cycles & command */
			nx_nand_cmd_addr(nc, 1, NAND_CMD_READ0, 0);
			addr = nc->cur_col & NX_NAND_SP_ADDR_MASK;
			nx_nand_cmd_addr(nc, 0, addr, 0);
			addr = (nc->cur_col >> 8) & NX_NAND_SP_ADDR_MASK;
			nx_nand_cmd_addr(nc, 0, addr, 0);
			addr = nc->cur_page & NX_NAND_SP_ADDR_MASK;
			nx_nand_cmd_addr(nc, 0, addr, 0);
			addr = (nc->cur_page >> 8) & NX_NAND_SP_ADDR_MASK;
			nx_nand_cmd_addr(nc, 0, addr, 0);
			if (nc->chip.chipsize >= (1 << 28)) {
				addr = (nc->cur_page >> 16) & NX_NAND_SP_ADDR_MASK;
				nx_nand_cmd_addr(nc, 0, addr, 0);
			}
			nx_nand_cmd_addr(nc, 1, NAND_CMD_READSTART, 1);

			/* Wait for the completion */ 
			wait_event(nc->nand_queue, (nc->done != false));

			/* Disable interrupts */
			tmhwEfmc_IntDisable(nc->unitid, intr);
#endif
			break;

		case NAND_CMD_READID:
			/* No page operation */
			page_cfg.includeOOB = false;
			page_cfg.operType = tmhwEfmc_Nothing;
			page_cfg.includeAES = false;
			page_cfg.includeECC = false;
			tmhwEfmc_InitPageOp(nc->unitid, &page_cfg);

			nx_nand_cmd_addr(nc, 1, cmd, 0);
			nx_nand_cmd_addr(nc, 0, column, 1);
			break;

		case NAND_CMD_PARAM:
			nc->offset = 0;

			/* No page operation */
			page_cfg.includeOOB = false;
			page_cfg.operType = tmhwEfmc_Nothing;
			page_cfg.includeAES = false;
			page_cfg.includeECC = false;
			tmhwEfmc_InitPageOp(nc->unitid, &page_cfg);

			nx_nand_cmd_addr(nc, 1, cmd, 0);
			nx_nand_cmd_addr(nc, 0, 0, 1);

			/* Read PARAM Page and 2 Redundant Parameter Pages */
			for (i=0; i < (3 * sizeof(struct nand_onfi_params)); i++) {
				nc->dmabuf[i]=nx_nand_read_byte(mtd);
			}
			break;

		default:
			printk(KERN_ERR "nx_nand: command not supported %d \n", cmd);
	}
}

/**
 * nx_nand_block_bad - Read bad block marker from the chip
 * @mtd: MTD device structure
 * @ofs: offset from device start
 * @getchip:	0, if the chip is already selected
 *
 * Check, if the block is bad.
 */
static int nx_nand_block_bad(struct mtd_info *mtd, loff_t ofs, 
	int getchip ATTRIBUTE_UNUSED)
{
	struct mtd_oob_ops ops;
	uint8_t	buf[NAND_MAX_OOBSIZE];
	int ret;
	u8 bad;
	int res = 0;
	struct nand_chip *chip = mtd->priv;

	printk(KERN_INFO "nx_nand: Bad block check 0x%x \r\n", (int)ofs);

	mtd->flags |= MTD_USE_DEV_OOB_LAYOUT;

	/* Read OOB data */
	ops.ooblen = mtd->oobsize;
	ops.oobbuf = buf;
	ops.ooboffs = 0;
	ops.datbuf = NULL;
	ops.mode = MTD_OPS_PLACE_OOB;
	ret = mtd_read_oob(mtd, ofs, &ops);
	if (ret) {
		printk(KERN_INFO "nx_nand: READOOB failed 0x%x \r\n", ret);
		return ret;
	}
	mtd->flags &= ~MTD_USE_DEV_OOB_LAYOUT;

	/* Check the bad block marker */ 
	bad = buf[chip->badblockpos];
	if (bad != 0xff) {
		res = 1;
	}

	printk(KERN_INFO "nx_nand: Bad block res 0x%x \r\n", res);

	return res;
}

/**
 * nx_nand_ctrl_isr - NAND controller ISR function 
 * @irq_no: IRQ number
 * @dev_id: Device ID
 *
 * Handles the NAND Controller interrupt events
 */
static irqreturn_t nx_nand_ctrl_isr(int irq_no ATTRIBUTE_UNUSED, void *dev_id)
{
	uint32_t int_stat;
	uint32_t int_ena;
	struct nx_nand_ctrl	*nc=(struct nx_nand_ctrl *)dev_id;

	/* Read the interrupt status & chan ID */
	tmhwEfmc_IntGetStatus(nc->unitid, (ptmhwEfmc_IntMask_t) &int_stat);
	int_ena = readl(nx_nc->ctrl_base + NX_NAND_INT_ENA_OFFSET);

#ifdef CONFIG_MTD_NX_NAND_DMAC
	/* Clear the interrupt */
	tmhwEfmc_IntClear(nc->unitid, int_stat);

	if(nc->cur_cmd == NAND_CMD_READ0) {
#if defined (CONFIG_ARCH_APOLLO)
		if(nc->hwecc) {
			int i;
			for(i=0; i < nc->num_blks; i++)	{
				nc->ecc_status[i] = (int_stat & (NX_NAND_INT_DEC_UNCOR | 
															NX_NAND_INT_DEC_0_ERR | 
															NX_NAND_INT_DEC_1_ERR |
															NX_NAND_INT_DEC_2_ERR |
															NX_NAND_INT_DEC_3_ERR |
															NX_NAND_INT_DEC_4_ERR |
															NX_NAND_INT_DEC_5_ERR));
			}
		}
#endif

		if((int_stat & NX_NAND_INT_SEQ_READ) && 
			(int_ena & NX_NAND_INT_SEQ_READ)) {
#if (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
			tmhwEfmc_GetPageRWStatusFifo(nc->unitid, &nc->page_rw_status_fifo);
#endif
			nc->done = true; 
			wake_up(&nc->nand_queue); 
		}
	}

	if(nc->cur_cmd == NAND_CMD_SEQIN) {
		if((int_stat & NX_NAND_INT_SEQ_WRITE) && 
			(int_ena & NX_NAND_INT_SEQ_WRITE)) {
			tmhwEfmc_IntClear(nc->unitid, NX_NAND_INT_SEQ_WRITE);
		}

		if((int_stat & (1 << (NX_NAND_INT_READY_START + nc->slotid))) && 
			(int_ena & (1 << (NX_NAND_INT_READY_START + nc->slotid)))) {
			tmhwEfmc_IntClear(nc->unitid, 
				(1 << (NX_NAND_INT_READY_START + nc->slotid)));
#if (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
			tmhwEfmc_GetPageRWStatusFifo(nc->unitid, &nc->page_rw_status_fifo);
#endif
			nc->done = true;
			wake_up(&nc->nand_queue);
		}
	}

	/* Erase command */
	if(nc->cur_cmd == NAND_CMD_ERASE1) {
		if((int_stat & (1 << (NX_NAND_INT_READY_START + nc->slotid))) && 
			(int_ena & (1 << (NX_NAND_INT_READY_START + nc->slotid)))) {
			tmhwEfmc_IntClear(nc->unitid, 
				(1 << (NX_NAND_INT_READY_START + nc->slotid)));
			nc->done = true;
			wake_up(&nc->nand_queue);
		}
	}
#else
	/* Write command */
	if((nc->cur_cmd == NAND_CMD_SEQIN) || (nc->cur_cmd == NAND_CMD_PAGEPROG)) {
		if((int_stat & NX_NAND_INT_OOB_WRITE) && 
			(int_ena & NX_NAND_INT_OOB_WRITE)) {
			tmhwEfmc_WriteOobData(nc->unitid, nc->slotid, 
				nc->dmabuf+nc->mtd.writesize);
			tmhwEfmc_IntClear(nc->unitid, int_stat);
		}

		if(nc->hwecc) {
			if((int_stat & NX_NAND_INT_ENC) && (int_ena & NX_NAND_INT_ENC)) {
				tmhwEfmc_WriteBufData(nc->unitid, nc->slotid, 
					nc->dmabuf+(nc->blk_index*nc->blk_size));
				nc->blk_index++;
				tmhwEfmc_IntClear(nc->unitid, int_stat);
			}
		}
		else {
			if((int_stat & NX_NAND_INT_BLK_WRITE) && 
				(int_ena & NX_NAND_INT_BLK_WRITE)){
				tmhwEfmc_WriteBufData(nc->unitid, 
					nc->slotid, nc->dmabuf + (nc->blk_index*nc->blk_size));
				nc->blk_index++;
				tmhwEfmc_IntClear(nc->unitid, int_stat);
			}
		}

		if((int_stat & NX_NAND_INT_SEQ_WRITE) && 
			(int_ena & NX_NAND_INT_SEQ_WRITE)) {
			tmhwEfmc_IntClear(nc->unitid, int_stat);
		}

		if((int_stat & (1 << (NX_NAND_INT_READY_START + nc->slotid))) && 
			(int_ena & (1 << (NX_NAND_INT_READY_START + nc->slotid)))) {
			tmhwEfmc_IntClear(nc->unitid, int_stat);
			nc->done = true;
			wake_up(&nc->nand_queue);
		}
	}

	/* Read command */
	if((nc->cur_cmd == NAND_CMD_READ0) || (nc->cur_cmd == NAND_CMD_READOOB)) {
		if((int_stat & NX_NAND_INT_OOB_READ) && 
			(int_ena & NX_NAND_INT_OOB_READ)) {
			tmhwEfmc_ReadOobData(nc->unitid, nc->slotid, 
				nc->dmabuf+nc->mtd.writesize);
			tmhwEfmc_IntClear(nc->unitid, int_stat);
			nc->done = true;
			wake_up(&nc->nand_queue);
		}

		if(nc->hwecc) {
			if((int_stat & NX_NAND_INT_DEC) && (int_ena & NX_NAND_INT_DEC)) {
				tmhwEfmc_ReadBufData(nc->unitid, nc->slotid, 
					nc->dmabuf+(nc->blk_index * nc->blk_size));
				nc->ecc_status[nc->blk_index] = (int_stat & (NX_NAND_INT_DEC_UNCOR | 
																			NX_NAND_INT_DEC_0_ERR | 
																			NX_NAND_INT_DEC_1_ERR |
																			NX_NAND_INT_DEC_2_ERR |
																			NX_NAND_INT_DEC_3_ERR |
																			NX_NAND_INT_DEC_4_ERR |
																			NX_NAND_INT_DEC_5_ERR));
				nc->blk_index++;
				tmhwEfmc_IntClear(nc->unitid, int_stat);
			}
		}
		else {
			if((int_stat & NX_NAND_INT_BLK_READ) && 
				(int_ena & NX_NAND_INT_BLK_READ)) {
				tmhwEfmc_ReadBufData(nc->unitid, nc->slotid, 
					nc->dmabuf+(nc->blk_index * nc->blk_size));
				nc->blk_index++;
				tmhwEfmc_IntClear(nc->unitid, int_stat);
			}
		}

		if((int_stat & NX_NAND_INT_SEQ_READ) && 
			(int_ena & NX_NAND_INT_SEQ_READ) ) {
			tmhwEfmc_IntClear(nc->unitid, int_stat);
		}
	}

	/* Erase command */
	if(nc->cur_cmd == NAND_CMD_ERASE1) {
		if((int_stat & (1 << (NX_NAND_INT_READY_START + nc->slotid))) && 
			(int_ena & (1 << (NX_NAND_INT_READY_START + nc->slotid)))) {
			tmhwEfmc_IntClear(nc->unitid, int_stat);
			nc->done = true;
			wake_up(&nc->nand_queue);
		}
	}
#endif
	return IRQ_HANDLED;
}

#ifdef CONFIG_OF
static const struct of_device_id nx_nand_dt_match[] = {
	{ .compatible = "entr,stb-nand" },
	{},
};
MODULE_DEVICE_TABLE(of, nx_nand_dt_match);

#if (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
static struct nx_nand_pdata
	*nx_nand_get_pdata(struct platform_device *pdev)
{
	if (!pdev->dev.platform_data && pdev->dev.of_node) {
		struct nx_nand_pdata *pdata;
		u32 prop;

		pdata =  devm_kzalloc(&pdev->dev,
				sizeof(struct nx_nand_pdata),
				GFP_KERNEL);
		pdev->dev.platform_data = pdata;
		if (!pdata)
			return NULL;
		if (!of_property_read_u32(pdev->dev.of_node,
			"entr,stb-nand-dev_timing0", &prop))
			pdata->dev_timing0 = prop;
		else /* Maximum */
			pdata->dev_timing0 = 0x0FFFFFFF;
		if (!of_property_read_u32(pdev->dev.of_node,
			"entr,stb-nand-dev_timing1", &prop))
			pdata->dev_timing1 = prop;
		else /* Maximum */
			pdata->dev_timing1 = 0x003FFFFF;
	}

	return pdev->dev.platform_data;
}
#endif
#else
#if (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
static struct nx_nand_pdata
	*nx_nand_get_pdata(struct platform_device *pdev)
{
	return pdev->dev.platform_data;
}
#endif
#endif

/**
 * nx_nand_probe - Module probe function
 * @pdev: Device structure
 *
 * Probes the device & do the initialisation
 */
static int nx_nand_probe(struct platform_device *pdev)
{
	int			ret=0;
	struct resource		*res1, *res2;
	struct nand_chip	*chip;
	struct mtd_info		*mtd;
	tmhwEfmc_Capabilities_t pcaps;
	tmhwEfmc_FlashConfig_t	pconfig;
#if (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
	struct nx_nand_pdata	*pdata;
	tmhwEfmc_ModeCtrl_t	pModeCtrl;
	tmhwEfmc_FlashConfig_t	FlashConfig;
	tmhwEfmc_OobInfo_t	pOobInfo;
#endif

	/* Allocate memory for nand control structure */
	nx_nc = kzalloc(sizeof(struct nx_nand_ctrl), GFP_KERNEL);
	if (!nx_nc) {
		printk(KERN_ERR "nx_nand: NAND ctrl mem alloc \r\n");
		return -ENOMEM;
	}

	/* Unit ID */
	nx_nc->unitid = 0;

	/* Get I/O resource */
	res1 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res1) {
		printk(KERN_ERR "nx_nand: NAND get resource \r\n");	
		ret = -ENXIO;
		goto out_free1;
	}

	/* Ioremap controller base */
	nx_nc->ctrl_base = devm_ioremap(&pdev->dev, res1->start, 
		(res1->end-res1->start+1));
	if (!nx_nc->ctrl_base) {
		printk(KERN_ERR "nx_nand: NAND base devm_iormep \r\n");	
		ret = -ENOMEM;
		goto out_free1;
	}

	/* Get interrupt resource */
	res2 = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res2) {
		printk(KERN_ERR "nx_nand: NAND get resource \r\n");	
		ret = -ENXIO;
		goto out_free1;
	}

	/* Alloc IRQ */
	ret = devm_request_irq (&pdev->dev, res2->start, 
		nx_nand_ctrl_isr, IRQF_DISABLED, "nx_2070", nx_nc);
	if (ret < 0){
		printk(KERN_ERR "nx_nand: NAND irq alloc \r\n");	
		goto out_free1;
	}

#ifndef CONFIG_MTD_NX_NAND_DMAC
	/* Ioremap AHB buffer for interrupt flow control */
	nx_nc->ahb_buf = devm_ioremap(&pdev->dev, NX_NAND_AHB_BUF, 
		NX_NAND_AHB_BUF_MAX_SIZE);
	if (!nx_nc->ahb_buf) {
		goto out_free1;
	}
#endif

	/* Store in dev structure */
	dev_set_drvdata(&pdev->dev, nx_nc);

	/* Store in HwAPI config */
	gktmhwEfmc_Config[nx_nc->unitid].baseAddress = (uint32_t) nx_nc->ctrl_base;
	gktmhwEfmc_Config[nx_nc->unitid].ahbMemAddress = (uint8_t *)nx_nc->ahb_buf;

	/* Init wait queue */
	init_waitqueue_head(&nx_nc->nand_queue);

	/* Read the control configuration & store */
	ret = tmhwEfmc_GetCapabilities(nx_nc->unitid, &pcaps);
	nx_nc->aes = pcaps.supportAES;
	nx_nc->slots = pcaps.maxDevices;
	nx_nc->slotid = 0;
	nx_nc->blk_size = SZ_512; /* default block size */
	nx_nc->lb_chip = 0;	  /* initialise the lbchip flag to false */

	/* Flash configuration -- REVISIT */
	ret = tmhwEfmc_GetFlashConfig(nx_nc->unitid, nx_nc->slotid, &pconfig);
#if (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
	pModeCtrl.rb_n_bypass = TM_FALSE;
	pModeCtrl.rd_stall = TM_FALSE;
	ret = tmhwEfmc_SetModeCtrl(nx_nc->unitid, &pModeCtrl);
	ret = tmhwEfmc_GetOobInfo(nx_nc->unitid, nx_nc->slotid, &pOobInfo);
#endif

	/* CE don;t care support */
	nx_nc->cedontcare = pconfig.enableCENDontCare; 

	mtd = &nx_nc->mtd;
	mtd->owner = THIS_MODULE;
	/* mtd->name = "nx_2017"; This has to come from the platform driver*/
	mtd->name = pdev->name;
	chip = &nx_nc->chip;
	mtd->priv = chip;

	/* Initialize hardware controller structure */
	spin_lock_init(&nx_nc->nandctrl.lock);
	init_waitqueue_head(&nx_nc->nandctrl.wq);
	chip->controller = &nx_nc->nandctrl;

	/* Store reference to the nx_nand structure */
	chip->priv = nx_nc;

	/* Chip Information */
	chip->chip_delay = 0;
	chip->options = NAND_NO_SUBPAGE_WRITE;
	chip->select_chip = nx_nand_select_chip;
	chip->dev_ready = nx_nand_dev_ready;

	if (pconfig.dataWidth) {
		chip->options |= NAND_BUSWIDTH_16;
		chip->read_byte = nx_nand_read_byte16;
	} else {
		chip->read_byte = nx_nand_read_byte;
	}

	chip->cmdfunc = nx_nand_command;
	chip->read_buf = nx_nand_read_buf;
	chip->write_buf = nx_nand_write_buf;
	chip->block_bad = nx_nand_block_bad;

	/* Allocate temp driver buffer for ONFI PARAM page read */
	nx_nc->dmabuf = dmam_alloc_coherent(&pdev->dev,
										1024, 
										&nx_nc->dmabuf_phy, 
										GFP_DMA | GFP_KERNEL);
	if(!nx_nc->dmabuf) {
		printk(KERN_ERR "nx_nand: DMA buf alloc \r\n");
		ret = -ENOMEM;
		goto out_free1;
	}

	/* Call chip identify function */
	if(nand_scan_ident(mtd, 1, NULL)) {
		printk(KERN_ERR "nx_nand: NAND scan ident \r\n");	
		ret = -ENXIO;
		goto out_free1;
	}

	/* Free temp driver buffer */
	dmam_free_coherent(&pdev->dev,
		1024,
		nx_nc->dmabuf,
		nx_nc->dmabuf_phy);


#ifdef CONFIG_ARCH_APOLLO
	/* Fix the maximum OOB size to 128 for Apollo */
	if (mtd->oobsize > 128)
		mtd->oobsize = 128;
#endif

#if (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
	memset(&FlashConfig, 0, sizeof(FlashConfig));

	FlashConfig.dataWidth = pconfig.dataWidth; /* REVISIT */
	FlashConfig.enableCENDontCare = pconfig.enableCENDontCare;
	FlashConfig.enableWrProtect = pconfig.enableWrProtect;
	switch (mtd->writesize >> 9) {
		case 1:
			FlashConfig.pageSize = tmhwEfmc_512Byte;
			FlashConfig.subpageSize = tmhwEfmc_SubPage512Byte;
			break;
		case 2:
			FlashConfig.pageSize = tmhwEfmc_1024Byte;
			FlashConfig.subpageSize = tmhwEfmc_SubPage1024Byte;
			break;
		case 4:
			FlashConfig.pageSize = tmhwEfmc_2048Byte;
			FlashConfig.subpageSize = tmhwEfmc_SubPage1024Byte;
			break;
		case 8:
			FlashConfig.pageSize = tmhwEfmc_4096Byte;
			FlashConfig.subpageSize = tmhwEfmc_SubPage1024Byte;
			break;
		case 16:
			FlashConfig.pageSize = tmhwEfmc_8192Byte;
			FlashConfig.subpageSize = tmhwEfmc_SubPage1024Byte;
			break;
		default:
			BUG();
	}
	FlashConfig.oobSize =  mtd->oobsize;
	switch (mtd->oobsize) {
		case 16:
			/* Using 512B subpage,
			 * 16 bytes oob is divided into 4 bytes user + 12 bytes ecc 
			 * Max. ecc level that fits in 12 bytes =
			 * 			(12*8)/14 ~ 6 (rounded)
			 */
			FlashConfig.eccLevel = 6;
			break;
		case 32:
			/* Using 1K subpage,
			 * 32 bytes oob per subpage = 4 bytes user + 28 bytes ecc 
			 * Max. ecc level that fits in 28 bytes =
			 * 				 (28*8)/14 = 16 
			 * Thus, 32 bytes oob = 4 bytes user + 28 bytes ecc
			 */
			FlashConfig.eccLevel = 16;
			break;
		case 64:
			/* Using 1K subpage,
			 * 64 bytes oob spanning two supages = 8 bytes user + 56 bytes ecc 
			 */
			FlashConfig.eccLevel = 16;
			break;
		case 128:
			/* Using 1K subpage,
			 * 128 bytes oob spanning four supages = 
			 * 		16 bytes user + 112 bytes ecc 
			 */
			FlashConfig.eccLevel = 16;
			break;
		case 224:
		case 448:
			/* Using 1K subpage,
			 * 56 bytes oob per subpage = 4 bytes user + 52 bytes ecc 
			 * Max. ecc level that fits in 52 bytes =
			 * 	= (52*8)/14 ~ 28 (since ecc level 30 needs 52.5 
			 * 		i.e. 56 rounded to next 4 bytes)
			 */
			FlashConfig.eccLevel = 28;
			break;
		default:
			BUG();
	}
	FlashConfig.erasedPageThres = FlashConfig.eccLevel/2; /* FIXME */

	pdata = nx_nand_get_pdata(pdev);
	if (pdata) {
		ptmhwEfmc_TimingConfig_t pDevTiming = &FlashConfig.devTiming;

		pDevTiming->tAleHold  = ( ( pdata->dev_timing0 & TMVH_EFMC_DEV_TIMING0_TALH_MSK ) >> TMVH_EFMC_DEV_TIMING0_TALH_POS );
		pDevTiming->tAleSetup = ( ( pdata->dev_timing0 & TMVH_EFMC_DEV_TIMING0_TALS_MSK ) >> TMVH_EFMC_DEV_TIMING0_TALS_POS );
		pDevTiming->tCleHold  = ( ( pdata->dev_timing0 & TMVH_EFMC_DEV_TIMING0_TCLH_MSK ) >> TMVH_EFMC_DEV_TIMING0_TCLH_POS );
		pDevTiming->tCleSetup = ( ( pdata->dev_timing0 & TMVH_EFMC_DEV_TIMING0_TCLS_MSK ) >> TMVH_EFMC_DEV_TIMING0_TCLS_POS );
		pDevTiming->tCenHold  = ( ( pdata->dev_timing0 & TMVH_EFMC_DEV_TIMING0_TCPH_MSK ) >> TMVH_EFMC_DEV_TIMING0_TCPH_POS );
		pDevTiming->tCenSetup = ( ( pdata->dev_timing0 & TMVH_EFMC_DEV_TIMING0_TCPS_MSK ) >> TMVH_EFMC_DEV_TIMING0_TCPS_POS );

		pDevTiming->tRdDelay     = ( ( pdata->dev_timing1 & TMVH_EFMC_DEV_TIMING1_TDRD_MSK ) >> TMVH_EFMC_DEV_TIMING1_TDRD_POS );
		pDevTiming->tWaitForBusy = ( ( pdata->dev_timing1 & TMVH_EFMC_DEV_TIMING1_TWB_MSK ) >> TMVH_EFMC_DEV_TIMING1_TWB_POS );
		pDevTiming->tWenWidth    = ( ( pdata->dev_timing1 & TMVH_EFMC_DEV_TIMING1_TWP_MSK ) >> TMVH_EFMC_DEV_TIMING1_TWP_POS );
		pDevTiming->tWenHigh     = ( ( pdata->dev_timing1 & TMVH_EFMC_DEV_TIMING1_TWH_MSK ) >> TMVH_EFMC_DEV_TIMING1_TWH_POS );
		pDevTiming->tRenWidth    = ( ( pdata->dev_timing1 & TMVH_EFMC_DEV_TIMING1_TRP_MSK ) >> TMVH_EFMC_DEV_TIMING1_TRP_POS );
		pDevTiming->tRenHigh     = ( ( pdata->dev_timing1 & TMVH_EFMC_DEV_TIMING1_TRH_MSK ) >> TMVH_EFMC_DEV_TIMING1_TRH_POS );
	} else {
		memset(&FlashConfig.devTiming, 0xFF, sizeof(FlashConfig.devTiming));
	}

	tmhwEfmc_SetFlashConfig(nx_nc->unitid,nx_nc->slotid, &FlashConfig);
	nx_nc->flash_config = FlashConfig;

	nx_nc->blk_size = (SZ_512 << FlashConfig.subpageSize);
#endif

	/* Calculate number of 512 or 1024 byte blocks in a page */
	nx_nc->num_blks = mtd->writesize/nx_nc->blk_size;

#ifdef CONFIG_ARCH_APOLLO
	/* Allocate ECC status array */
	nx_nc->ecc_status = kzalloc(sizeof(int) * nx_nc->num_blks, GFP_KERNEL);
	if(!nx_nc->ecc_status) {
		printk(KERN_ERR "nx_nand: ECC status alloc \r\n");
		ret = -ENOMEM;
		goto out_free1;
	}
#endif

	/* Allocate internal driver buffer */
	nx_nc->dmabuf = dmam_alloc_coherent(&pdev->dev,
	mtd->writesize + mtd->oobsize, 
	&nx_nc->dmabuf_phy, GFP_DMA | GFP_KERNEL);
	if(!nx_nc->dmabuf) {
		printk(KERN_ERR "nx_nand: DMA buf alloc \r\n");
		ret = -ENOMEM;
		goto out_free1;
	}

	if(mtd->writesize > SZ_512) {
		chip->cmdfunc = nx_nand_command_lp;
		nx_nc->lb_chip = 1;
	}

	chip->options |= NAND_NO_SUBPAGE_WRITE;

	chip->ecc.read_page_raw = nx_nand_read_page_raw;
	chip->ecc.write_page_raw = nx_nand_write_page_raw;
	chip->ecc.read_oob = nx_nand_read_oob;
	chip->ecc.write_oob = nx_nand_write_oob;

#ifdef CONFIG_MTD_NX_NAND_HWECC
	nx_nc->hwecc = true;
	chip->ecc.read_page = nx_nand_read_page;
	chip->ecc.write_page = nx_nand_write_page;
	chip->ecc.mode = NAND_ECC_HW;
	chip->ecc.calculate = nx_nand_calculate_ecc;
	chip->ecc.correct = nx_nand_correct_data;
	chip->ecc.hwctl = nx_nand_hwctl;
	chip->ecc.size = nx_nc->blk_size;
#if (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
	chip->ecc.strength = FlashConfig.eccLevel;
	pOobInfo.parity_bytes = (FlashConfig.eccLevel * 14)/8;
	/* Is number of parity bytes multiple of 4 ? */
	pOobInfo.parity_bytes = (pOobInfo.parity_bytes%4)?
		(((pOobInfo.parity_bytes/4)*4)+4):
		(pOobInfo.parity_bytes);
	chip->ecc.bytes = pOobInfo.parity_bytes;
#else
	chip->ecc.strength = 5;
	chip->ecc.bytes = 12;
#endif
	chip->bbt_td = &nx_bbt_main;
	chip->bbt_md = &nx_bbt_mirror;
#endif

#ifdef CONFIG_MTD_NX_NAND_SWECC
	nx_nc->hwecc = false;
	chip->ecc.mode = NAND_ECC_SOFT;
#endif

#ifdef CONFIG_MTD_NX_NAND_NONEECC
	chip->ecc.read_page = nx_nand_read_page_raw;
	chip->ecc.write_page = nx_nand_write_page_raw;
	nx_nc->hwecc = false;
	chip->ecc.mode = NAND_ECC_NONE;
#endif

#ifdef CONFIG_MTD_NX_NAND_HWECC
	switch (mtd->oobsize) {
		case 16:
			chip->ecc.layout = &nx_nand_oob_16;
			break;
		case 64:
			chip->ecc.layout = &nx_nand_oob_64;
			break;
		case 128:
			chip->ecc.layout = &nx_nand_oob_128;
			break;
#if (defined (CONFIG_PLAT_STB) && !defined (CONFIG_ARCH_APOLLO))
		case 224:
			chip->ecc.layout = &nx_nand_oob_224;
			break;
		case 448:
			chip->ecc.layout = &nx_nand_oob_448;
			break;
#endif
		default:
			printk(KERN_WARNING "nx_nand: No oob scheme defined for "
				"oobsize %d\n", mtd->oobsize);
			BUG();
	}
#endif

	/* Enable the following for a flash based bad block table */
	chip->bbt_options = NAND_BBT_USE_FLASH;

	ret = nand_scan_tail(mtd);
	if(ret) {
		if(ret == -ENOSPC) {
			printk(KERN_ERR "nx_nand: No space to write BBT \n");
		} else {
			printk(KERN_ERR "nx_nand: NAND scan tail \r\n");
			ret = -ENXIO;
			goto out_free1;
		}
	}

	/* Scan for the partitions */
	mtd_device_parse_register(mtd, part_probes, NULL, NULL, 0);

	return 0;

out_free1:	
	kfree(nx_nc);

	return ret;
}

/**
 * nx_nand_remove - Module remove function
 * @pdev: Device structure
 *
 * Removes the device & do the deinitialisation
 */
static int nx_nand_remove(struct platform_device *pdev)
{
	struct nx_nand_ctrl *nc = dev_get_drvdata(&pdev->dev);

	/* Release resources */
	nand_release(&nc->mtd);

	/* Free DMA buf */
	kfree(nc);

	return 0;
}

#ifdef CONFIG_PM
static int nx_nand_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct nx_nand_ctrl *nc = dev_get_drvdata(&pdev->dev);
	int ret = 0;
	unsigned long val;

	if (nc) { 
		ret = mtd_suspend(&nc->mtd);

		/* Put the controller (IP2017) into power down mode */
		if(!ret) {
			val = readl((nc->ctrl_base + NX_NAND_POWER_DOWN_CFG_OFFSET));

			val &= ~NX_NAND_POWER_DOWN_MASK;
			val |= (NX_NAND_POWER_DOWN_ENABLE & NX_NAND_POWER_DOWN_MASK);

			writel(val, (nc->ctrl_base + NX_NAND_POWER_DOWN_CFG_OFFSET));
		}
	}

	return ret;
}

static int nx_nand_resume(struct platform_device *pdev)
{
	struct nx_nand_ctrl *nc = dev_get_drvdata(&pdev->dev);
	unsigned long val;

	if (nc) {
		/* Bring the controller out of power down mode */
		val = readl((nc->ctrl_base + NX_NAND_POWER_DOWN_CFG_OFFSET));

		val &= ~NX_NAND_POWER_DOWN_MASK;
		val |= (NX_NAND_POWER_DOWN_DISABLE & NX_NAND_POWER_DOWN_MASK);

		writel(val, (nc->ctrl_base + NX_NAND_POWER_DOWN_CFG_OFFSET));

		mtd_resume(&nc->mtd);
	}

	return 0;
}
#else

#define nx_nand_suspend NULL
#define nx_nand_resume	NULL

#endif


/**
* NAND device registration
*/
static struct platform_driver nx_nand_driver = {
	//.probe		= nx_nand_probe,
	.remove		= nx_nand_remove,
	.suspend 	= nx_nand_suspend,
	.resume		= nx_nand_resume,
	.driver		= {
		.name = "nx_2017",
		.owner	= THIS_MODULE,
		.of_match_table = nx_nand_dt_match,
	},
};

module_platform_driver_probe(nx_nand_driver, nx_nand_probe);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NAND Flash driver for IP_2070 NAND controller");
MODULE_ALIAS("platform:nx_2017");
