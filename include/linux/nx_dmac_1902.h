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

#ifndef _NX_DMAC_1902_H_
#define _NX_DMAC_1902_H_

#include <linux/dmaengine.h>

/*
 * There is no platform data present for DMA controller IP_1902
 */

/** burst size */
/*!
 * This enumeration specifies the burst size of a burst transfer used in DMA 
 * transfer
 */
typedef enum {
  nx_dmac_1902_burst_1 = 0x00, /*!< Burst size = 1 */
  nx_dmac_1902_burst_4,        /*!< Burst size = 4 */
  nx_dmac_1902_burst_8,        /*!< Burst size = 8 */
  nx_dmac_1902_burst_16,       /*!< Burst size = 16 */
  nx_dmac_1902_burst_32,       /*!< Burst size = 32 */
  nx_dmac_1902_burst_64,       /*!< Burst size = 64 */
  nx_dmac_1902_burst_128,      /*!< Burst size = 128 */
  nx_dmac_1902_burst_256       /*!< Burst size = 256 */
} nx_dmac_1902_burst;

/*!
 * This enumeration specifies the flow control type that is used to transfer
 * the data through DMA
 */
typedef enum {
  nx_dmac_1902_fcntl_DMA_M_to_M = 0x00, /*!< DMA being flow-controller and between memory to memory */
  nx_dmac_1902_fcntl_DMA_M_to_P,        /*!< DMA being flow-controller and from memory to peripheral direction */
  nx_dmac_1902_fcntl_DMA_P_to_M,        /*!< DMA being flow-controller and from peripheral to memory direction */
  nx_dmac_1902_fcntl_DMA_P_to_P,        /*!< DMA being flow-controller and from peripheral to peripheral direction */
  nx_dmac_1902_fcntl_dst_per_P_to_P,    /*!< Destination peripheral being flow-controller and from peripheral to peripheral direction */
  nx_dmac_1902_fcntl_per_M_to_P,        /*!< Peripheral being flow-controller and from memory to peripheral direction */
  nx_dmac_1902_fcntl_per_P_to_M,        /*!< Peripheral being flow-controller and from peripheral to memory direction */
  nx_dmac_1902_fcntl_src_per_P_to_P,    /*!< Source peripheral being flow-controller and from peripheral to peripheral direction */
} nx_dmac_1902_fcntl;

/*!
 * This enumeration specifies the AHB master that is selected for different
 * transfers (source/destination)
 */
typedef enum {
  nx_dmac_1902_ahb_master_1 = 0x00, /*!< AHB master 1 is selected for the transfer */
  nx_dmac_1902_ahb_master_2         /*!< AHB master 2 is selected for the transfer */
} nx_dmac_1902_ahb_master;

/**
 * enum dma_slave_width - DMA slave register access width.
 * @DMA_SLAVE_WIDTH_8BIT: Do 8-bit slave register accesses
 * @DMA_SLAVE_WIDTH_16BIT: Do 16-bit slave register accesses
 * @DMA_SLAVE_WIDTH_32BIT: Do 32-bit slave register accesses
 */
enum dma_slave_width {
	DMA_SLAVE_WIDTH_8BIT,
	DMA_SLAVE_WIDTH_16BIT,
	DMA_SLAVE_WIDTH_32BIT,
};

/*!
 * DMAC IP_1902 specific configuration to be performed by the DMA client. This
 * structure should be updated by the DMA client (other peripheral drivers
 * which makes use of the DMAC IP_1902 for data transfer before calling the 
 * \a dma_async_client_chan_request() in there driver module.
 *
 * This data should be part of the platform specific data of the DMA client.
 *
 * @dma_dev: required DMA master device
 * @src_per_num: Peripheral number that is assigned in the DMAC IP_1902 for source device
 * @dst_per_num: Peripheral number that is assigned in the DMAC IP_1902 for destination device
 * @src_burst: Source burst size for DMA transfer
 * @dst_burst: Destintation burst size for DMA transfer
 * @src_incr: To increment the source location (0->no increment, 1->increment)
 * @dst_incr: To increment the destination location (0->no increment, 1->increment)
 * @flow_cntrl: Flow controller type for the data transfer.
 * @src_select: AHB master selected for source transfer
 * @dst_select: AHB master selected for destination transfer
 * @src_width: Source peripheral bus width
 * @dst_width: Destination peripheral bus width
 */
struct nx_dmac_1902_slave {
  struct device           *dma_dev;
  /* client specific data to be programmed into the IP_1902 */
  u8                      src_per_num;
  u8                      dst_per_num;
  nx_dmac_1902_burst      src_burst;
  nx_dmac_1902_burst      dst_burst;
  u8                      src_incr;
  u8                      dst_incr;
  nx_dmac_1902_fcntl      flow_cntrl;
  nx_dmac_1902_ahb_master src_select;
  nx_dmac_1902_ahb_master dst_select;
  enum dma_slave_width    src_width;
  enum dma_slave_width    dst_width;
  enum dma_slave_width	  reg_width;
  dma_addr_t		  tx_reg;
  dma_addr_t		  rx_reg;
};

#endif /* _NX_DMAC_1902_H_ */

