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

#ifndef _MACH_GCS_H_
#define _MACH_GCS_H_

#define IP2070_GCS_CFG_BASE            (ARM_A9_HOST_MMIO_BASE + 0x36000)
#define IP2070_GCS_CFG_LEN             (0x1000)

/* GCS Mode Register */
#define IP2070_GCS_MODE_REG            (IP2070_GCS_CFG_BASE + 0x0)
#define IP2070_GCS_MODE_PCI_ISA_MODE   (1UL<<0)

/* GCS Clock Mux Register */
#define IP2070_GCS_CLOCK_MUX           (IP2070_GCS_CFG_BASE + 0x64)
#define IP2070_GCS_CLOCK_MUX_EN                (1UL<<0)


/* GCS Mode Register */
#define IP2070_GCS_MODE_REG              (IP2070_GCS_CFG_BASE + 0x0)
#define IP2070_GCS_MODE_PCI_ISA_MODE     (1UL<<0)
#define IP2070_GCS_MODE_SDIO_RGMII       (1UL<<1)
#define IP2070_GCS_MODE_SDIO_IO          (1UL<<2)

/* GCS SDIO Chip Select Route */
#define IP2070_GCS_SDIO_CS_ROUTE        (IP2070_GCS_CFG_BASE + 0x60)

/* GCS Clock Mux Register */
#define IP2070_GCS_CLOCK_MUX            (IP2070_GCS_CFG_BASE + 0x64)
#define IP2070_GCS_CLOCK_MUX_EN         (1UL<<0)

/* GCS SDIO Clock Mux Register */
#define IP2070_GCS_SDIO_CLOCK_MUX       (IP2070_GCS_CFG_BASE + 0x68)
#define IP2070_GCS_SDIO_CLOCK_MUX_EN    (1UL<<0)

#define IP2070_GCS_SDIO_HPROT           (IP2070_GCS_CFG_BASE + 0x6c)
#define IP2070_GCS_SDIO_CACHEABLE       (1UL<<3)
#define IP2070_GCS_SDIO_BUFFERABLE      (1UL<<2)

#endif /* _MACH_GCS_H_ */

