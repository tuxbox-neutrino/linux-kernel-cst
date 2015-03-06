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

#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

#include <mach/gpio.h>

/* ------------------------------------------------------------------------
 *  Interrupts - bit assignment (primary)
 *	Cortex A9 interrupt distributor can handle 224 interrupts.
 *	permitted values are 0, 32, 64, 96, 128, 160, 192, 224.
 *
 *  REMARK: for the real assignment, see bottom of file -> IRQ_...
 *
 * ------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------
 * Cortex A9 internal interrupts are not going via the Distributor
 * ------------------------------------------------------------------------
 */
#define INT_SGI_ID00		0   /* Cortex A9 SW generated IRQ id-0 */
#define INT_SGI_ID01		1   /* Cortex A9 SW generated IRQ id-1 */
#define INT_SGI_ID02		2   /* Cortex A9 SW generated IRQ id-2 */
#define INT_SGI_ID03		3   /* Cortex A9 SW generated IRQ id-3 */
#define INT_SGI_ID04		4   /* Cortex A9 SW generated IRQ id-4 */
#define INT_SGI_ID05		5   /* Cortex A9 SW generated IRQ id-5 */
#define INT_SGI_ID06		6   /* Cortex A9 SW generated IRQ id-6 */
#define INT_SGI_ID07		7   /* Cortex A9 SW generated IRQ id-7 */
#define INT_SGI_ID08		8   /* Cortex A9 SW generated IRQ id-8 */
#define INT_SGI_ID09		9   /* Cortex A9 SW generated IRQ id-9 */
#define INT_SGI_ID10		10  /* Cortex A9 SW generated IRQ id-10 */
#define INT_SGI_ID11		11  /* Cortex A9 SW generated IRQ id-11 */
#define INT_SGI_ID12		12  /* Cortex A9 SW generated IRQ id-12 */
#define INT_SGI_ID13		13  /* Cortex A9 SW generated IRQ id-13 */
#define INT_SGI_ID14		14  /* Cortex A9 SW generated IRQ id-14 */
#define INT_SGI_ID15		15  /* Cortex A9 SW generated IRQ id-15 */
#define INT_SOURCE16		16  /* reserved by by ARM */
#define INT_SOURCE17		17  /* reserved by by ARM */
#define INT_SOURCE18		18  /* reserved by by ARM */
#define INT_SOURCE19		19  /* reserved by by ARM */
#define INT_SOURCE20		20  /* reserved by by ARM */
#define INT_SOURCE21		21  /* reserved by by ARM */
#define INT_SOURCE22		22  /* reserved by by ARM */
#define INT_SOURCE23		23  /* reserved by by ARM */
#define INT_SOURCE24		24  /* reserved by by ARM */
#define INT_SOURCE25		25  /* reserved by by ARM */
#define INT_SOURCE26		26  /* reserved by by ARM */
#define INT_A9_GLOB_TIMER	27  /* Cortex A9 global timer        */
#define INT_A9_LEGACY_FIQ	28  /* Cortex A9 nFIQ in legacy mode */
#define INT_A9_PRIV_TIMER	29  /* Cortex A9 privat timer        */
#define INT_A9_WDG_TIMER	30  /* Cortex A9 watchdog timer      */
#define INT_A9_LEGACY_IRQ	31  /* Cortex A9 nIRQ in legacy mode */
/*
 * Interrupt  0 - 31 is internal Cortex A9 (local per core),
 *           32 - 223 are external interrupts (via Distributor).
 */
#define IRQ_DISTR_OFFSET	32  /* IRQ offset to non internal irq's */

#define P_IN_BEGIN_OF_LIST	0   /* GIC IRQ-PIN starts from here */
#define P_IN_GPIO_LEFT		0   /* gpio_out_gic_l */
#define P_IN_IPC_CORTEXM3	1   /* intrpt_ipc0 = intrpt_cortexm3 */
#define P_IN_IPC_CORTEXA9	2   /* intrpt_ipc1 = intrpt_cortexa9 */
#define P_IN_IPC_ARM926		3   /* intrpt_ipc2 = intrpt_arm926 */
#define P_IN_IPC_TM_VDSP	4   /* intrpt_ipc3 = intrpt_tm_vdsp */
#define P_IN_IPC_TM_ADSP	5   /* intrpt_ipc4 = intrpt_tm_adsp */
#define P_IN_CGU_CLOCK		6   /* intrpt_clock (CGU) */
#define P_IN_SATA		7   /* intrpt_sata */
#define P_IN_ETHR_MAC0		8   /* intrpt_ethernet_mac_0 */
#define P_IN_ETHR_PWR0		9   /* intrpt_ethernet_pwr_0 */
#define P_IN_NS			10  /* intrpt_ns */
#define P_IN_PT0		11  /* pt0_irq (Pulse Timer) */
#define P_IN_PT1		12  /* pt1_irq */
#define P_IN_CEC		13  /* cec_int_n */
#define P_IN_UART_A0		14  /* uart_intreq_a0 */
#define P_IN_UART_A1		15  /* uart_intreq_a2 */
#define P_IN_UART_A2		16  /* uart intreq_a1 */
#define P_IN_UART_A3		17  /* uart intreq_a3 */
#define P_IN_SPI_COMB		18  /* intrpt_spi_combined */
#define P_IN_SPI_DMAC		19  /* intrpt_spi_dmacintr */
#define P_IN_SM0		20  /* sm0_irq (SMartcard) */
#define P_IN_SM1		21  /* sm1_irq */
#define P_IN_IR_IO		22  /* ir_req (Irin/out) */
#define P_IN_I2C2		23  /* intrpt_iic2 */
#define P_IN_I2C3		24  /* intrpt_iic3 */
#define P_IN_I2C4		25  /* intrpt_iic4 */
#define P_IN_RTC0		26  /* timer_irq[23:0]->[49:26] (timers/RTC) */
/* ... gap in list ... */
#define P_IN_SDCN		50  /* intrpt_sdcn */
#define P_IN_TSA		51  /* tsa_irq */
#define P_IN_MCX		52  /* mcx_irq */
#define P_IN_TSP		53  /* tsp_irq */
#define P_IN_TSR		54  /* tsr_irq */
#define P_IN_TSX		55  /* tsx_irq */
#define P_IN_VD0		56  /* vd0_irq */
#define P_IN_VD1		57  /* vd1_irq */
#define P_IN_VD2		58  /* vd2_irq */
#define P_IN_VD3		59  /* vd3_irq */
#define P_IN_CAM		60  /* cam_irq */
#define P_IN_USB0		61  /* intrpt_usb0 */
#define P_IN_USB1		62  /* intrpt_usb1 */
#define P_IN_USB2		63  /* intrpt_usb2 */
#define P_IN_DMA_MON0		64  /* intrpt_mtlmon0 (DMA monitor) */
#define P_IN_DMA_MON1		65  /* intrpt_mtlmon1 */
#define P_IN_DMA_MON2		66  /* intrpt_mtlmon2 */
#define P_IN_DMA_MON3		67  /* intrpt_mtlmon3 */
#define P_IN_DMA_NETW		68  /* intreq_h0 (DMA netw hub) */
#define P_IN_DMA_HUB		69  /* intreq_h1 (DMA arb, con) */
#define P_IN_GPIO_RIGHT		70  /* gpio_out_gic_r */
#define P_IN_DRAW2D		71  /* intrpt_draw2d */
#define P_IN_GPPM		72  /* intrpt_gppm (PCI + IO/Flash Cntl+SDIO)(GCS) */
#define P_IN_PCI_DMA		73  /* intrpt_pci_dma */
#define P_IN_PCI		74  /* intrpt_pci */
#define P_IN_PCI_IN		75  /* pci_inta_in */
#define P_IN_GCS_DMA		76  /* dmacintr */
#define P_IN_TURING_CLK		77  /* ckl_irq	TURING + FUSE_CTRL */
#define P_IN_CRYPTO		78  /* crypto_irq */
#define P_IN_NEWT		79  /* newt_intreq */
#define P_IN_SSP_HOST0		80  /* ssp_host_int[1:0] */
#define P_IN_SSP_HOST1		81  /* */
#define P_IN_M_CARD		82  /* o_cc_irq	(M-Card CCIF) */
#define P_IN_ACP_DMA_1902	83  /* dmacintr	(DMAC_1902) */
#define P_IN_THALIA		84  /* thalia_irq (IMG SGX531) */
#define P_IN_APB0		85  /* vdec_apb_intr[2:0] = FMVD (malone, arm926 , Video Dsp , Video dsp gic) */
#define P_IN_APB1		86  /* */
#define P_IN_APB2		87  /* */
#define P_IN_VPIPE0		88  /* irq_vpipe[6:0] vsd_vpipe (mbvp_s2k9+cpipe_s2k9 + CPIPE_S2V0 ) */
#define P_IN_VPIPE1		89  /* */
#define P_IN_VPIPE2		90  /* */
#define P_IN_VPIPE3		91  /* */
#define P_IN_VPIPE4		92  /* */
#define P_IN_VPIPE5		93  /* */
#define P_IN_VPIPE6		94  /* */
#define P_IN_SPDO		95  /* intrpt_spdo */
#define P_IN_IPC5		96  /* intrpt_ipc5 */
#define P_IN_IPC6		97  /* intrpt_ipc6 */
#define P_IN_IPC7		98  /* intrpt_ipc7 */
#define P_IN_AO_HDMI		99  /* intrpt_ao_hdmi (AO to HDMI) */
#define P_IN_AO_BTSC		100 /* intrpt_ao_btsc (AO to BTSC) */
#define P_IN_AO			101 /* intrpt_ao (AO NS ADAC) */
#define P_IN_DENC		102 /* denc_dcs_cp_irq */
#define P_IN_ADCN		103 /* intrpt_adcn (Host ARM MMIO) */
#define P_IN_AVDSN		104 /* intrpt_avdsn (A/V Control MMIO BUS) */
#define P_IN_HDMITX		105 /* intrpt_hdmitx */
#define P_IN_ETHR_MAC1		106 /* intrpt_ethernet_mac_1 (GMAC) */
#define P_IN_ETHR_PWR1		107 /* intrpt_ethernet_pwr_1 */
#define P_IN_VIP		108 /* intrpt_vip*/
#define P_IN_IP_2017		109 /* intrpt_ip_2017_intr */
#define P_IN_SFC		110 /* intrpt_sfc */
#define P_IN_DMA_MON4		111 /* intrpt_mtlmon4 */
#define P_IN_DMA_MON5		112 /* intrpt_mtlmon5 */
#define P_IN_DMA_MON6		113 /* intrpt_mtlmon6 */
#define P_IN_DMA_MON7		114 /* intrpt_mtlmon7 */
#define P_IN_TYPHOON		115 /* typhoon_interrupt */
#define P_IN_AI			116 /* intrpt_ai */
#define P_IN_I2C5		117 /* intrpt_iic5 */
#define P_IN_AUX_CTI_TRIG7	118 /* aux_ctitrigout2[7] (CS-GDS) */
#define P_IN_AUX_CTI_TRIG6	119 /* aux_ctitrigout2[6] */
#define P_IN_PWM1		120 /* PWM1 */
#define P_IN_PWM2		121 /* PWM2 */
#define P_IN_PWM3		122 /* PWM3 */
#define P_IN_GMAC_PHY		123 /* intrpt_gmac_phy */
#define P_IN_SD_CARD		124 /* intrpt_sdcard */
#define P_IN_PMU		125 /* intrpt_pmuirq */
#define P_IN_A3CRC		126 /* intrpt_a3crc_done */
#define P_IN_127RESERVED	127 /* Reserved */
#define P_IN_L2_CCR		128 /* l2ccintr (L2 cache only on the A9 IRQ(128)) */
#define P_IN_COMM_RX		129 /* COMMRX */
#define P_IN_COMM_TX		130 /* COMMTX */
#define P_IN_A9_CTI		131 /* a9_ctiirq */
#define P_IN_132RESERVED	132 /* Reserved [255:164]->[223-132] */
/* ... gap in list ... */
#define P_IN_223RESERVED	223 /* */
#define P_IN_END_OF_LIST	224 /* END OF LIST (NOT USABLE IRQ-PIN)*/
/* End of supported interrupts in Apollo */

/* ------------------------------------------------------------------------
 *
 * DO THE REAL IRQ ASSIGNMENTS...
 *
 * ------------------------------------------------------------------------
 */
#define IRQ_LOWEST_NBR		INT_SGI_ID00
/* Cortex A9 core internal interrupts */
#define IRQ_A9_GLOB_TIMER	INT_A9_GLOB_TIMER
#define IRQ_A9_PRIV_TIMER	INT_A9_PRIV_TIMER
#define IRQ_A9WDTIM		INT_A9_WDG_TIMER
/* Cortex A9 external core provided interrupts */
#define IRQ_GPIO_LEFT		(P_IN_GPIO_LEFT		+ IRQ_DISTR_OFFSET)
#define IRQ_IPC_CORTEXM3	(P_IN_IPC_CORTEXM3	+ IRQ_DISTR_OFFSET)
#define IRQ_IPC_CORTEXA9	(P_IN_IPC_CORTEXA9	+ IRQ_DISTR_OFFSET)
#define IRQ_IPC_ARM926		(P_IN_IPC_ARM926	+ IRQ_DISTR_OFFSET)
#define IRQ_IPC_TM_VDSP		(P_IN_IPC_TM_VDSP	+ IRQ_DISTR_OFFSET)
#define IRQ_IPC_TM_ADSP		(P_IN_IPC_TM_ADSP	+ IRQ_DISTR_OFFSET)
#define IRQ_CGU_CLOCK		(P_IN_CGU_CLOCK		+ IRQ_DISTR_OFFSET)
#define IRQ_SATA		(P_IN_SATA		+ IRQ_DISTR_OFFSET)
#define IRQ_ETHR_MAC_0		(P_IN_ETHR_MAC0		+ IRQ_DISTR_OFFSET)
#define IRQ_ETHR_PWR_0		(P_IN_ETHR_PWR0		+ IRQ_DISTR_OFFSET)
#define IRQ_NS			(P_IN_NS		+ IRQ_DISTR_OFFSET)
#define IRQ_PT_0		(P_IN_PT0		+ IRQ_DISTR_OFFSET)
#define IRQ_PT_1		(P_IN_PT1		+ IRQ_DISTR_OFFSET)
#define IRQ_CEC			(P_IN_CEC		+ IRQ_DISTR_OFFSET)
#define IRQ_UART_0		(P_IN_UART_A0		+ IRQ_DISTR_OFFSET)
#define IRQ_UART_1		(P_IN_UART_A1		+ IRQ_DISTR_OFFSET)
#define IRQ_UART_2		(P_IN_UART_A2		+ IRQ_DISTR_OFFSET)
#define IRQ_UART_3		(P_IN_UART_A3		+ IRQ_DISTR_OFFSET)
#define IRQ_SPI_COMB		(P_IN_SPI_COMB		+ IRQ_DISTR_OFFSET)
#define IRQ_SPI_DMAC		(P_IN_SPI_DMAC		+ IRQ_DISTR_OFFSET)
#define IRQ_SM0			(P_IN_SM0		+ IRQ_DISTR_OFFSET)
#define IRQ_SM1			(P_IN_SM1		+ IRQ_DISTR_OFFSET)
#define IRQ_IR_IO		(P_IN_IR_IO		+ IRQ_DISTR_OFFSET)
#define IRQ_I2C_2		(P_IN_I2C2		+ IRQ_DISTR_OFFSET)
#define IRQ_I2C_3		(P_IN_I2C3		+ IRQ_DISTR_OFFSET)
#define IRQ_I2C_4		(P_IN_I2C4		+ IRQ_DISTR_OFFSET)
#define IRQ_RTC0		(P_IN_RTC0		+ IRQ_DISTR_OFFSET)
/* ... gap in list ... [23:0]->[49:26] (timers/RTC) ... */
#define IRQ_SDCN		(P_IN_SDCN		+ IRQ_DISTR_OFFSET)
#define IRQ_TSA			(P_IN_TSA		+ IRQ_DISTR_OFFSET)
#define IRQ_MCX			(P_IN_MCX		+ IRQ_DISTR_OFFSET)
#define IRQ_TSP			(P_IN_TSP		+ IRQ_DISTR_OFFSET)
#define IRQ_TSR			(P_IN_TSR		+ IRQ_DISTR_OFFSET)
#define IRQ_TSX			(P_IN_TSX		+ IRQ_DISTR_OFFSET)
#define IRQ_VD_0		(P_IN_VD0		+ IRQ_DISTR_OFFSET)
#define IRQ_VD_1		(P_IN_VD1		+ IRQ_DISTR_OFFSET)
#define IRQ_VD_2		(P_IN_VD2		+ IRQ_DISTR_OFFSET)
#define IRQ_VD_3		(P_IN_VD3		+ IRQ_DISTR_OFFSET)
#define IRQ_CAM			(P_IN_CAM		+ IRQ_DISTR_OFFSET)
#define IRQ_USB_0		(P_IN_USB0		+ IRQ_DISTR_OFFSET)
#define IRQ_USB_1		(P_IN_USB1		+ IRQ_DISTR_OFFSET)
#define IRQ_USB_2		(P_IN_USB2		+ IRQ_DISTR_OFFSET)
#define IRQ_DMA_MON_0		(P_IN_DMA_MON0		+ IRQ_DISTR_OFFSET)
#define IRQ_DMA_MON_1		(P_IN_DMA_MON1		+ IRQ_DISTR_OFFSET)
#define IRQ_DMA_MON_2		(P_IN_DMA_MON2		+ IRQ_DISTR_OFFSET)
#define IRQ_DMA_MON_3		(P_IN_DMA_MON3		+ IRQ_DISTR_OFFSET)
#define IRQ_DMA_NETW		(P_IN_DMA_NETW		+ IRQ_DISTR_OFFSET)
#define IRQ_DMA_HUB		(P_IN_DMA_HUB		+ IRQ_DISTR_OFFSET)
#define IRQ_GPIO_RIGHT		(P_IN_GPIO_RIGHT	+ IRQ_DISTR_OFFSET)
#define IRQ_DRAW2D		(P_IN_DRAW2D		+ IRQ_DISTR_OFFSET)
#define IRQ_GPPM		(P_IN_GPPM		+ IRQ_DISTR_OFFSET)
#define IRQ_PCI_DMA		(P_IN_PCI_DMA		+ IRQ_DISTR_OFFSET)
#define IRQ_PCI			(P_IN_PCI		+ IRQ_DISTR_OFFSET)
#define IRQ_PCI_IN		(P_IN_PCI_IN		+ IRQ_DISTR_OFFSET)
#define IRQ_GCS_DMA		(P_IN_GCS_DMA		+ IRQ_DISTR_OFFSET)
#define IRQ_TURING_CLK		(P_IN_TURING_CLK	+ IRQ_DISTR_OFFSET)
#define IRQ_CRYPTO		(P_IN_CRYPTO		+ IRQ_DISTR_OFFSET)
#define IRQ_NEWT		(P_IN_NEWT		+ IRQ_DISTR_OFFSET)
#define IRQ_SSP_HOST_0		(P_IN_SSP_HOST0		+ IRQ_DISTR_OFFSET)
#define IRQ_SSP_HOST_1		(P_IN_SSP_HOST1		+ IRQ_DISTR_OFFSET)
#define IRQ_M_CARD		(P_IN_M_CARD		+ IRQ_DISTR_OFFSET)
#define IRQ_ACP_DMA_1902	(P_IN_ACP_DMA_1902	+ IRQ_DISTR_OFFSET)
#define IRQ_THALIA		(P_IN_THALIA		+ IRQ_DISTR_OFFSET)
#define IRQ_APB_0		(P_IN_APB0		+ IRQ_DISTR_OFFSET)
#define IRQ_APB_1		(P_IN_APB1		+ IRQ_DISTR_OFFSET)
#define IRQ_APB_2		(P_IN_APB2		+ IRQ_DISTR_OFFSET)
#define IRQ_VPIPE_0		(P_IN_VPIPE0		+ IRQ_DISTR_OFFSET)
#define IRQ_VPIPE_1		(P_IN_VPIPE1		+ IRQ_DISTR_OFFSET)
#define IRQ_VPIPE_2		(P_IN_VPIPE2		+ IRQ_DISTR_OFFSET)
#define IRQ_VPIPE_3		(P_IN_VPIPE3		+ IRQ_DISTR_OFFSET)
#define IRQ_VPIPE_4		(P_IN_VPIPE4		+ IRQ_DISTR_OFFSET)
#define IRQ_SPDO		(P_IN_SPDO		+ IRQ_DISTR_OFFSET)
#define IRQ_IPC_5		(P_IN_IPC5		+ IRQ_DISTR_OFFSET)
#define IRQ_IPC_6		(P_IN_IPC6		+ IRQ_DISTR_OFFSET)
#define IRQ_IPC_7		(P_IN_IPC7		+ IRQ_DISTR_OFFSET)
#define IRQ_AO_HDMI		(P_IN_AO_HDMI		+ IRQ_DISTR_OFFSET)
#define IRQ_AO_BTSC		(P_IN_AO_BTSC		+ IRQ_DISTR_OFFSET)
#define IRQ_AO			(P_IN_AO		+ IRQ_DISTR_OFFSET)
#define IRQ_DENC		(P_IN_DENC		+ IRQ_DISTR_OFFSET)
#define IRQ_ADCN		(P_IN_ADCN		+ IRQ_DISTR_OFFSET)
#define IRQ_AVDSN		(P_IN_AVDSN		+ IRQ_DISTR_OFFSET)
#define IRQ_HDMITX		(P_IN_HDMITX		+ IRQ_DISTR_OFFSET)
#define IRQ_ETHR_MAC_1		(P_IN_ETHR_MAC1		+ IRQ_DISTR_OFFSET)
#define IRQ_ETHR_PWR_1		(P_IN_ETHR_PWR1		+ IRQ_DISTR_OFFSET)
#define IRQ_VIP			(P_IN_VIP		+ IRQ_DISTR_OFFSET)
#define IRQ_IP_2017		(P_IN_IP_2017		+ IRQ_DISTR_OFFSET)
#define IRQ_SFC			(P_IN_SFC		+ IRQ_DISTR_OFFSET)
#define IRQ_DMA_MON_4		(P_IN_DMA_MON4		+ IRQ_DISTR_OFFSET)
#define IRQ_DMA_MON_5		(P_IN_DMA_MON5		+ IRQ_DISTR_OFFSET)
#define IRQ_DMA_MON_6		(P_IN_DMA_MON6		+ IRQ_DISTR_OFFSET)
#define IRQ_DMA_MON_7		(P_IN_DMA_MON7		+ IRQ_DISTR_OFFSET)
#define IRQ_TYPHOON		(P_IN_TYPHOON		+ IRQ_DISTR_OFFSET)
#define IRQ_AI			(P_IN_AI		+ IRQ_DISTR_OFFSET)
#define IRQ_I2C_5		(P_IN_I2C5		+ IRQ_DISTR_OFFSET)
#define IRQ_AUX_CTI_TRIG_7	(P_IN_AUX_CTI_TRIG7	+ IRQ_DISTR_OFFSET)
#define IRQ_AUX_CTI_TRIG_6	(P_IN_AUX_CTI_TRIG6	+ IRQ_DISTR_OFFSET)
#define IRQ_PWM_1		(P_IN_PWM1		+ IRQ_DISTR_OFFSET)
#define IRQ_PWM_2		(P_IN_PWM2		+ IRQ_DISTR_OFFSET)
#define IRQ_PWM_3		(P_IN_PWM3		+ IRQ_DISTR_OFFSET)
#define IRQ_GMAC_PHY		(P_IN_GMAC_PHY		+ IRQ_DISTR_OFFSET)
#define IRQ_SD_CARD		(P_IN_SD_CARD		+ IRQ_DISTR_OFFSET)
#define IRQ_PMU			(P_IN_PMU		+ IRQ_DISTR_OFFSET)
#define IRQ_A3CRC		(P_IN_A3CRC		+ IRQ_DISTR_OFFSET)
#define IRQ_127_RESERVED	(P_IN_127RESERVED	+ IRQ_DISTR_OFFSET)
#define IRQ_L2_CCR		(P_IN_L2_CCR		+ IRQ_DISTR_OFFSET)
#define IRQ_COMM_RX		(P_IN_COMM_RX		+ IRQ_DISTR_OFFSET)
#define IRQ_COMM_TX		(P_IN_COMM_TX		+ IRQ_DISTR_OFFSET)
#define IRQ_A9_CTI		(P_IN_A9_CTI		+ IRQ_DISTR_OFFSET)
#define IRQ_132_RESERVED	(P_IN_132RESERVED	+ IRQ_DISTR_OFFSET)
/* ... gap in list ... [255:164]->[223-132] ... */
#define IRQ_223_RESERVED	(P_IN_223RESERVED	+ IRQ_DISTR_OFFSET)
#define IRQ_MAX_NBR		(P_IN_END_OF_LIST	+ IRQ_DISTR_OFFSET)

/* ------------------------------------------------------------------------
 *   Additional IRQ assignment i.f.o. generic kernel parts
 * ------------------------------------------------------------------------
 */
#define IRQ_GLOBALTIMER		IRQ_A9_GLOB_TIMER
#define IRQ_LOCALTIMER		IRQ_A9_PRIV_TIMER

#define IRQ_LOCALWDOG		IRQ_A9WDTIM
#define IRQ_GIC_START		(P_IN_BEGIN_OF_LIST + IRQ_DISTR_OFFSET)

#ifdef  CONFIG_GPIO_STB
#define NR_IRQS			(GPIO1_IRQ_BASE + GPIO1_MAX_NR)
#else
#define NR_IRQS			IRQ_MAX_NBR
#endif
#define DISTR_MAX_IRQS		NR_IRQS

#ifndef NR_IRQS
#error "NR_IRQS not defined by the board-specific files"
#endif

#endif /* __ASM_ARCH_IRQS_H */
