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

#ifndef __ASM_ARM_ARCH_HARDWARE_IP3106_H
#define __ASM_ARM_ARCH_HARDWARE_IP3106_H


/**
 * Registers overview
 *
 * The IP3106 UART has the following registers that are accessible at an offset
 * to the base address of the UART. The base address is dependend on the memory
 * layout of the board that uses the IP3106.
 *
 *	Register	Address	R/RW	Description
 *	Name		Offset
 *  ---------------------------------------------------------------------------
 *	RBR		0x000	R	Receive Buffer Register
 *	THR		0x000	W	Transmit Holding Register
 *	DLL		0x000	RW	Divisor Latch (LSB)
 *	IER		0x004	RW	Interrupt Enable Register
 *	DLM		0x004	RW	Divisor Latch MSB
 *	IIR		0x008	R	Interrupt Identification Register
 *	FCR		0x008	W	FIFO Control Register
 *	LCR		0x00C	RW	Line Control Register
 *	MCR		0x010	RW	Mocem Control Register
 *	LSR		0x014	R	Line Status Register
 *	MSR		0x018	R	Modem Status Register
 *	SCR		0x01C	RW	Scratch Register
 *	ACR		0x020	RW	Auto-baud Control Register
 *	ICR		0x024	RW	IrDA Control Register
 *	FDR		0x028	RW	Fractional Diveder Register
 *	POP		0x030	W	NHP Pop Register
 *	MODE		0x034	RW	NHP Mode Selection Register
 *	CFG		0xFD4	R	Configuration Register
 * 	INTCE		0xFD8	W	Interrupt Clear Enable Register
 *	INTSE		0xFDC	W	Interrupt Set Enable Register
 *	INTS		0xFE0	R	Interrupt Status Register
 * 	INTE		0xFE4	R	Interrupt Enable Register
 *	INTCS		0xFE8	W	Interrupt Clear Status Register
 * 	INTCE		0xFEC	W	Interrupt Set Status Register
 *	MID		0xFFC	R	Module Identification Register
 */

/* --------------------------------------------------------------------------
 *  Register offsets from base address
 *  -------------------------------------------------------------------------*/
#define IP3106_UART_RBR_REG         0x000        /* Receiver Buffer Register */
#define IP3106_UART_THR_REG         0x000        /* Transmit Holding Register*/
#define IP3106_UART_DLL_REG         0x000        /* Divisor Latch LSB */
#define IP3106_UART_DLM_REG         0x004        /* Divisor Latch MSB */
#define IP3106_UART_IER_REG         0x004        /* Interrupt Enable Register*/
#define IP3106_UART_IIR_REG         0x008        /* Interrupt ID Register */
#define IP3106_UART_FCR_REG         0x008        /* FIFO Control Register */
#define IP3106_UART_LCR_REG         0x00C        /* Line Control Register */
#define IP3106_UART_MCR_REG         0x010        /* Modem Control Register */
#define IP3106_UART_LSR_REG         0x014        /* Line Status Register */
#define IP3106_UART_MSR_REG         0x018        /* Modem Status Register */
#define IP3106_UART_SCR_REG         0x01C        /* Scratch Pad Register */
#define IP3106_UART_ACR_REG         0x020        /* Auto-baud Control Register */
#define IP3106_UART_ICR_REG         0x024        /* IrDA Control Register */
#define IP3106_UART_FDR_REG         0x028        /* Fractional Divider Register*/
#define IP3106_UART_OSR_REG         0x02C        /* Over Sampling Register*/
#define IP3106_UART_POP_REG         0x030        /* NHP Pop Register */
#define IP3106_UART_MODE_REG        0x034        /* NHP Mode Register */
#define IP3106_UART_CFG_REG         (0x3F5 << 2) /* Configuration Register */
#define IP3106_UART_INTCE_REG       (0x3F6 << 2) /* Int Clear Enable Register */
#define IP3106_UART_INTSE_REG       (0x3F7 << 2) /* Int Set Enable Register */
#define IP3106_UART_INTS_REG        (0x3F8 << 2) /* Int Status Register */
#define IP3106_UART_INTE_REG        (0x3F9 << 2) /* Int Enable Register */
#define IP3106_UART_INTCS_REG       (0x3FA << 2) /* Int Clear Status Register*/
#define IP3106_UART_INTSS_REG       (0x3FB << 2) /* Int Set Status Register*/
#define IP3106_UART_MID_REG         (0x3FF << 2) /* Module Id Register*/

#define IP3106_UART_REGS_NUMBER     13

/* --------------------------------------------------------------------------
 *  Receiver Fifo Buffer Register (RBR) (0x00) - Read Only
 *  -------------------------------------------------------------------------*/
#define IP3106_UART_RBR_FIFO_REG_MSK      0x000000FF /* RBR FIFO register mask */
#define IP3106_UART_RBR_FIFO_REG_RW_MSK   0x00000000 /* Read/Write bits*/

/* Data */
#define IP3106_UART_RBR_FIFO_REG_DATA_POS          0
#define IP3106_UART_RBR_FIFO_REG_DATA_LEN          8
#define IP3106_UART_RBR_FIFO_REG_DATA_MSK 0x000000FF

/* --------------------------------------------------------------------------
 *  Transmitter Fifo Holding Register (THR) (0x00) - Write Only
 *  -------------------------------------------------------------------------*/

#define IP3106_UART_THR_FIFO_REG_MSK      0x000000FF /* THR FIFO register mask */
#define IP3106_UART_THR_FIFO_REG_RW_MSK   0x00000000 /* Read/Write bits */

/* Data */
#define IP3106_UART_THR_FIFO_REG_DATA_POS          0
#define IP3106_UART_THR_FIFO_REG_DATA_LEN          8
#define IP3106_UART_THR_FIFO_REG_DATA_MSK 0x000000FF

/* --------------------------------------------------------------------------
 *  Divisor Latch LSB Register (DLL) (0x00) - Read/Write
 *  ------------------------------------------------------------------------*/
#define IP3106_UART_DLL_REG_MSK           0x000000FF /* DLL register mask */
#define IP3106_UART_DLL_REG_RW_MSK        0x000000FF /* Read/Write bits */

/* Data */
#define IP3106_UART_DLL_REG_DATA_POS               0
#define IP3106_UART_DLL_REG_DATA_LEN               8
#define IP3106_UART_DLL_REG_DATA_MSK      0x000000FF

/* --------------------------------------------------------------------------
 *  Interrupt Holding Register  (IER) (0X04) - Read/Write
 *  -------------------------------------------------------------------------*/
/* Interrupt register mask */
#define IP3106_UART_IER_REG_MSK           0x0000038F /* Bits 4, 5 and 6 are unused */
#define IP3106_UART_IER_REG_RW_MSK        0x0000038F /* Read/Write bits */

/* Receive Data Interrupt Enable */
#define IP3106_UART_IER_RDAI_E_POS                 0
#define IP3106_UART_IER_RDAI_E_LEN                 1
#define IP3106_UART_IER_RDAI_E_MSK        0x00000001
/* THRE Interrupt Enable */
#define IP3106_UART_IER_THREI_E_POS                1
#define IP3106_UART_IER_THREI_E_LEN                1
#define IP3106_UART_IER_THREI_E_MSK       0x00000002
/* Rx Line Status Interrupt Enable */
#define IP3106_UART_IER_RLSI_E_POS                 2
#define IP3106_UART_IER_RLSI_E_LEN                 1
#define IP3106_UART_IER_RLSI_E_MSK        0x00000004
/* Modem Status Interrupt Enable */
#define IP3106_UART_IER_MSI_E_POS                  3
#define IP3106_UART_IER_MSI_E_LEN                  1
#define IP3106_UART_IER_MSI_E_MSK         0x00000008
/* Auto CTS Interrupt Enable */
#define IP3106_UART_IER_CTSI_E_POS                 7
#define IP3106_UART_IER_CTSI_E_LEN                 1
#define IP3106_UART_IER_CTSI_E_MSK        0x00000080
/* End of Auto-baud Interrupt Enable */
#define IP3106_UART_IER_ABEOI_E_POS                8
#define IP3106_UART_IER_ABEOI_E_LEN                1
#define IP3106_UART_IER_ABEOI_E_MSK       0x00000100
/* Auto-baud TimeOut Interrupt Enable */
#define IP3106_UART_IER_ABTOI_E_POS                9
#define IP3106_UART_IER_ABTOI_E_LEN                1
#define IP3106_UART_IER_ABTOI_E_MSK       0x00000200

/* --------------------------------------------------------------------------
 *  Divisor Latch MSB Register (DLM) (0x04) - Read/Write
 *  -------------------------------------------------------------------------*/
#define IP3106_UART_DLM_REG_MSK           0x000000FF /* DLM register mask */
#define IP3106_UART_DLM_REG_RW_MSK        0x000000FF /* Read/Write bits */

/* Data */
#define IP3106_UART_DLM_REG_DATA_POS               0
#define IP3106_UART_DLM_REG_DATA_LEN               8
#define IP3106_UART_DLM_REG_DATA_MSK      0x000000FF

/* --------------------------------------------------------------------------
 *  Interrupt ID Register  (IIR) (0x08) - Read Only
 *  -------------------------------------------------------------------------*/
/* Interrupt ID mask */
#define IP3106_UART_IIR_REG_MSK           0x0000029F /* Bits 4 and 5 are unused */
#define IP3106_UART_IIR_REG_RW_MSK        0x00000000 /* Read/Write bits */

/* Interrupt Pending */
#define IP3106_UART_IIR_PENDING_POS                0
#define IP3106_UART_IIR_PENDING_LEN                1
#define IP3106_UART_IIR_PENDING_MSK       0x00000001
/* Interrupt Identification */
#define IP3106_UART_IIR_ID_POS                     1
#define IP3106_UART_IIR_ID_LEN                     3
#define IP3106_UART_IIR_ID_MSK            0x0000000E
/* FIFO Enable */
#define IP3106_UART_IIR_FIFOS_EN_POS               6
#define IP3106_UART_IIR_FIFOS_EN_LEN               2
#define IP3106_UART_IIR_FIFOS_EN_MSK      0x000000C0
/* End of Auto-baud Interrupt Enable */
#define IP3106_UART_IIR_ABEO_INT_POS               8
#define IP3106_UART_IIR_ABEO_INT_LEN               1
#define IP3106_UART_IIR_ABEO_INT_MSK      0x00000100
/* Auto-baud TimeOut Interrupt Enable */
#define IP3106_UART_IIR_ABTO_INT_POS               9
#define IP3106_UART_IIR_ABTO_INT_LEN               1
#define IP3106_UART_IIR_ABTO_INT_MSK      0x00000200

/* Valid values */
/* Interrupt Identification */
#define IP3106_UART_IIR_MSI_INT_ID        (0x00000000) /* Modem Int Id */
#define IP3106_UART_IIR_THRE_INT_ID       (0x00000001) /* THRE Int Id */
#define IP3106_UART_IIR_RDA_INT_ID        (0x00000002) /* RDA Int Id */
#define IP3106_UART_IIR_RLS_INT_ID        (0x00000003) /* Rx Line Status Int Id */
#define IP3106_UART_IIR_CTI_INT_ID        (0x00000006) /* Char TI Int Id */
#define IP3106_UART_IIR_INT_MASK          (0x0000000E) /* Mask */

/* --------------------------------------------------------------------------
 *  FIFO Control Register (FCR) (0x08) - Write Only
 *  ------------------------------------------------------------------------*/
/* FIFO Control mask */
#define IP3106_UART_FCR_REG_MSK           0x0000009F /* Bits 4 and 5 are unused */
#define IP3106_UART_FCR_REG_RW_MSK        0x00000000 /* Read/Write bits */

/* FIFO Enable */
#define IP3106_UART_FCR_FIF_ENA_POS                0
#define IP3106_UART_FCR_FIF_ENA_LEN                1
#define IP3106_UART_FCR_FIF_ENA_MSK       0x00000001
/* RX FIFO Reset */
#define IP3106_UART_FCR_RXF_RES_POS                1
#define IP3106_UART_FCR_RXF_RES_LEN                1
#define IP3106_UART_FCR_RXF_RES_MSK       0x00000002
/* TX FIFO Reset */
#define IP3106_UART_FCR_TXF_RES_POS                2
#define IP3106_UART_FCR_TXF_RES_LEN                1
#define IP3106_UART_FCR_TXF_RES_MSK       0x00000004
/* DMA Mode Select */
#define IP3106_UART_FCR_DMA_POS                    3
#define IP3106_UART_FCR_DMA_LEN                    1
#define IP3106_UART_FCR_DMA_MSK           0x00000008
/* RX Trigger Level Select */
#define IP3106_UART_FCR_RX_TRIGGER_POS             6
#define IP3106_UART_FCR_RX_TRIGGER_LEN             2
#define IP3106_UART_FCR_RX_TRIGGER_MSK    0x000000C0

/* Valid values */
#define IP3106_UART_FCR_INT_1_DEEP            (0x0)
#define IP3106_UART_FCR_INT_4_DEEP            (0x1)
#define IP3106_UART_FCR_INT_8_DEEP            (0x2)
#define IP3106_UART_FCR_INT_14_DEEP           (0x3)

#define IP3106_UART_FCR_RX_TRIG_LEVEL_0       (0x0)
#define IP3106_UART_FCR_RX_TRIG_LEVEL_1       (0x1)
#define IP3106_UART_FCR_RX_TRIG_LEVEL_2       (0x2)
#define IP3106_UART_FCR_RX_TRIG_LEVEL_3       (0x3)

/* --------------------------------------------------------------------------
 *  Line Control Register (LCR) (0x0C) - Read/Write
 *  -------------------------------------------------------------------------*/
#define IP3106_UART_LCR_REG_MSK           0x000000FF /* Line Control mask */
#define IP3106_UART_LCR_REG_RW_MSK        0x000000FF /* Read/Write bits */

/* Word Length Select */
#define IP3106_UART_LCR_WORD_LEN_POS               0
#define IP3106_UART_LCR_WORD_LEN_LEN               2
#define IP3106_UART_LCR_WORD_LEN_MSK      0x00000003
/* Stop Bit Select */
#define IP3106_UART_LCR_STOPB_POS                  2
#define IP3106_UART_LCR_STOPB_LEN                  1
#define IP3106_UART_LCR_STOPB_MSK         0x00000004
/* Parity Enable */
#define IP3106_UART_LCR_PAR_EN_POS                 3
#define IP3106_UART_LCR_PAR_EN_LEN                 1
#define IP3106_UART_LCR_PAR_EN_MSK        0x00000008
/* Parity Select */
#define IP3106_UART_LCR_PAR_SEL_POS                4
#define IP3106_UART_LCR_PAR_SEL_LEN                2
#define IP3106_UART_LCR_PAR_SEL_MSK       0x00000030
/* Break Control */
#define IP3106_UART_LCR_BRK_CTL_POS                6
#define IP3106_UART_LCR_BRK_CTL_LEN                1
#define IP3106_UART_LCR_BRK_CTL_MSK       0x00000040
/* Divisor Latch Access Bit */
#define IP3106_UART_LCR_DLAB_POS                   7
#define IP3106_UART_LCR_DLAB_LEN                   1
#define IP3106_UART_LCR_DLAB_MSK          0x00000080

/* Valid values */
/* Word Length Select */
#define IP3106_UART_LCR_WORDLEN_5BIT          (0x0)
#define IP3106_UART_LCR_WORDLEN_6BIT          (0x1)
#define IP3106_UART_LCR_WORDLEN_7BIT          (0x2)
#define IP3106_UART_LCR_WORDLEN_8BIT          (0x3)

/* Stop Bit Select */
#define IP3106_UART_LCR_1_STOP_BIT            (0x0)
#define IP3106_UART_LCR_2_STOP_BIT            (0x1)

/* Parity Select */
#define IP3106_UART_LCR_PARITY_ODD            (0x0)
#define IP3106_UART_LCR_PARITY_EVEN           (0x1)
#define IP3106_UART_LCR_PARITY_MARK           (0x2)
#define IP3106_UART_LCR_PARITY_SPACE          (0x3)

/* Parity Enable */
#define IP3106_UART_LCR_NO_PARITY             (0x0)
#define IP3106_UART_LCR_PARITY                (0x1)

/* Break Control */
#define IP3106_UART_LCR_NO_BREAK_TR           (0x0)
#define IP3106_UART_LCR_BREAK_TR              (0x1)

/* Divisor Latches */
#define IP3106_UART_LCR_NO_DIV_LAB            (0x0)
#define IP3106_UART_LCR_DIV_LAB               (0x1)

/* --------------------------------------------------------------------------
 *  Modem Control Register (MCR) (0x10) - Read/Write
 *  -------------------------------------------------------------------------*/
#define IP3106_UART_MCR_REG_MSK           0x000000FF /* Modem Control mask */
#define IP3106_UART_MCR_REG_RW_MSK        0x000000FF /* Read/Write bits */

/* NDTR Control */
#define IP3106_UART_MCR_DTR_POS                    0
#define IP3106_UART_MCR_DTR_LEN                    1
#define IP3106_UART_MCR_DTR_MSK           0x00000001
/* NRTS Control */
#define IP3106_UART_MCR_RTS_POS                    1
#define IP3106_UART_MCR_RTS_LEN                    1
#define IP3106_UART_MCR_RTS_MSK           0x00000002
/* NOUT Control */
#define IP3106_UART_MCR_OUT_POS                    2
#define IP3106_UART_MCR_OUT_LEN                    2
#define IP3106_UART_MCR_OUT_MSK           0x0000000C
/* LOOPBACK Control */
#define IP3106_UART_MCR_LOOP_POS                   4
#define IP3106_UART_MCR_LOOP_LEN                   1
#define IP3106_UART_MCR_LOOP_MSK          0x00000010
/* Auto RTS Flow Control Enable */
#define IP3106_UART_MCR_ARTS_EN_POS                6
#define IP3106_UART_MCR_ARTS_EN_LEN                1
#define IP3106_UART_MCR_ARTS_EN_MSK       0x00000040
/* Auto CTS Flow Control Enable */
#define IP3106_UART_MCR_ACTS_EN_POS                7
#define IP3106_UART_MCR_ACTS_EN_LEN                1
#define IP3106_UART_MCR_ACTS_EN_MSK       0x00000080

/* Valid values */
/* Loopback Mode Select */
#define IP3106_UART_MCR_NO_LOOPBACK           (0x0)
#define IP3106_UART_MCR_LOOPBACK              (0x1)

/* --------------------------------------------------------------------------
 *  Line Status Register (LSR) (0x14) - Read Only
 *  -------------------------------------------------------------------------*/
#define IP3106_UART_LSR_REG_MSK           0x000000FF /* Line Status mask */
#define IP3106_UART_LSR_REG_RW_MSK        0x00000000 /* Read/Write bits */

/* Receiver Data Ready */
#define IP3106_UART_LSR_DR_POS                     0
#define IP3106_UART_LSR_DR_LEN                     1
#define IP3106_UART_LSR_DR_MSK            0x00000001
/* Overrun Error */
#define IP3106_UART_LSR_OE_POS                     1
#define IP3106_UART_LSR_OE_LEN                     1
#define IP3106_UART_LSR_OE_MSK            0x00000002
/* Parity Error */
#define IP3106_UART_LSR_PE_POS                     2
#define IP3106_UART_LSR_PE_LEN                     1
#define IP3106_UART_LSR_PE_MSK            0x00000004
/* Framing Error */
#define IP3106_UART_LSR_FE_POS                     3
#define IP3106_UART_LSR_FE_LEN                     1
#define IP3106_UART_LSR_FE_MSK            0x00000008
/* Break Interrupt */
#define IP3106_UART_LSR_BI_POS                     4
#define IP3106_UART_LSR_BI_LEN                     1
#define IP3106_UART_LSR_BI_MSK            0x00000010
/* Transmitter Holding Register Empty */
#define IP3106_UART_LSR_THRE_POS                   5
#define IP3106_UART_LSR_THRE_LEN                   1
#define IP3106_UART_LSR_THRE_MSK          0x00000020
/* Transmitter Empty */
#define IP3106_UART_LSR_TEMT_POS                   6
#define IP3106_UART_LSR_TEMT_LEN                   1
#define IP3106_UART_LSR_TEMT_MSK          0x00000040
/* Error in RX FIFO */
#define IP3106_UART_LSR_RXERR_POS                  7
#define IP3106_UART_LSR_RXERR_LEN                  1
#define IP3106_UART_LSR_RXERR_MSK         0x00000080

/* --------------------------------------------------------------------------
 *  Modem Status Register (MSR) (0x18) - Read Only
 *  -------------------------------------------------------------------------*/
#define IP3106_UART_MSR_REG_MSK           0x000000FF /* Modem Status mask */
#define IP3106_UART_MSR_REG_RW_MSK        0x00000000 /* Read/Write bits */

/* Delta NCTS */
#define IP3106_UART_MSR_DCTS_POS                   0
#define IP3106_UART_MSR_DCTS_LEN                   1
#define IP3106_UART_MSR_DCTS_MSK          0x00000001
/* Delta NDSR */
#define IP3106_UART_MSR_DDSR_POS                   1
#define IP3106_UART_MSR_DDSR_LEN                   1
#define IP3106_UART_MSR_DDSR_MSK          0x00000002
/* Trailing Edge NRI */
#define IP3106_UART_MSR_TERI_POS                   2
#define IP3106_UART_MSR_TERI_LEN                   1
#define IP3106_UART_MSR_TERI_MSK          0x00000004
/* Delta NDCD */
#define IP3106_UART_MSR_DDCD_POS                   3
#define IP3106_UART_MSR_DDCD_LEN                   1
#define IP3106_UART_MSR_DDCD_MSK          0x00000008
/* Clear to Send State */
#define IP3106_UART_MSR_CTS_POS                    4
#define IP3106_UART_MSR_CTS_LEN                    1
#define IP3106_UART_MSR_CTS_MSK           0x00000010
/* Data Set Ready State */
#define IP3106_UART_MSR_DSR_POS                    5
#define IP3106_UART_MSR_DSR_LEN                    1
#define IP3106_UART_MSR_DSR_MSK           0x00000020
/* Ring Indicator State */
#define IP3106_UART_MSR_RI_POS                     6
#define IP3106_UART_MSR_RI_LEN                     1
#define IP3106_UART_MSR_RI_MSK            0x00000040
/* Data Carrier Detect State */
#define IP3106_UART_MSR_DCD_POS                    7
#define IP3106_UART_MSR_DCD_LEN                    1
#define IP3106_UART_MSR_DCD_MSK           0x00000080

/* --------------------------------------------------------------------------
 *  Scratch Pad Register (SCR) (0x1C) - Read/Write
 *  -------------------------------------------------------------------------*/
#define IP3106_UART_SCR_REG_MSK           0x000000FF /* SCR register mask */
#define IP3106_UART_SCR_REG_RW_MSK        0x000000FF /* Read/Write bits */

/* Data */
#define IP3106_UART_SCR_REG_DATA_POS               0
#define IP3106_UART_SCR_REG_DATA_LEN               8
#define IP3106_UART_SCR_REG_DATA_MSK      0x000000FF

/* --------------------------------------------------------------------------
 *  Auto-baud Control Register (ACR) (0x20) - Read/Write
 *  -------------------------------------------------------------------------*/
/* ACR register mask */
#define IP3106_UART_ACR_REG_MSK           0x00000307 /* Bits 3-7 are unused */
#define IP3106_UART_ACR_REG_RW_MSK        0x00000007 /* Read/Write bits */

/* Power-on/Reset mask & value */
#define IP3106_UART_ACR_REG_POR_MSK       0x00000307
#define IP3106_UART_ACR_REG_POR_VAL       0x00000000

/* Start */
#define IP3106_UART_ACR_START_POS                  0
#define IP3106_UART_ACR_START_LEN                  1
#define IP3106_UART_ACR_START_MSK         0x00000001

/* Mode */
#define IP3106_UART_ACR_MODE_POS                   1
#define IP3106_UART_ACR_MODE_LEN                   1
#define IP3106_UART_ACR_MODE_MSK          0x00000002

/* Auto Restart */
#define IP3106_UART_ACR_ARES_POS                   2
#define IP3106_UART_ACR_ARES_LEN                   1
#define IP3106_UART_ACR_ARES_MSK          0x00000004

/* End of Auto-baud Interrupt Clear */
#define IP3106_UART_ACR_ABEOIC_POS                 8
#define IP3106_UART_ACR_ABEOIC_LEN                 1
#define IP3106_UART_ACR_ABEOIC_MSK        0x00000100

/* Auto-baud TimeOut Interrupt Clear */
#define IP3106_UART_ACR_ABTOIC_POS                 9
#define IP3106_UART_ACR_ABTOIC_LEN                 1
#define IP3106_UART_ACR_ABTOIC_MSK        0x00000200

/* --------------------------------------------------------------------------
 *  IrDA Control Register (ICR) (0x24) - Read/Write
 *  -------------------------------------------------------------------------*/
#define IP3106_UART_ICR_REG_MSK           0x0000003F /* ICR register mask */
#define IP3106_UART_ICR_REG_RW_MSK        0x0000003F /* Read/Write bits */

/* Enable */
#define IP3106_UART_ICR_EN_POS                     0
#define IP3106_UART_ICR_EN_LEN                     1
#define IP3106_UART_ICR_EN_MSK            0x00000001

/* Invert */
#define IP3106_UART_ICR_INV_POS                    1
#define IP3106_UART_ICR_INV_LEN                    1
#define IP3106_UART_ICR_INV_MSK           0x00000002

/* Fixed Pulse Width */
#define IP3106_UART_ICR_FPW_POS                    2
#define IP3106_UART_ICR_FPW_LEN                    1
#define IP3106_UART_ICR_FPW_MSK           0x00000004

/* Pulse Divisor */
#define IP3106_UART_ICR_PD_POS                     3
#define IP3106_UART_ICR_PD_LEN                     3
#define IP3106_UART_ICR_PD_MSK            0x00000038

/* Valid Values */
/* Pulse Divisor */
#define IP3106_UART_ICR_VAL_PD2                  (0)
#define IP3106_UART_ICR_VAL_PD4                  (1)
#define IP3106_UART_ICR_VAL_PD8                  (2)
#define IP3106_UART_ICR_VAL_PD16                 (3)
#define IP3106_UART_ICR_VAL_PD32                 (4)
#define IP3106_UART_ICR_VAL_PD64                 (5)
#define IP3106_UART_ICR_VAL_PD128                (6)
#define IP3106_UART_ICR_VAL_PD256                (7)

/* --------------------------------------------------------------------------
 *  Fractional Divider Register (FDR) (0x28) - Read/Write
 *  -------------------------------------------------------------------------*/
#define IP3106_UART_FDR_REG_MSK           0x000000FF /* ICR register mask */
#define IP3106_UART_FDR_REG_RW_MSK        0x000000FF /* Read/Write bits */

/* Divisor */
#define IP3106_UART_FDR_DIV_POS                    0
#define IP3106_UART_FDR_DIV_LEN                    4
#define IP3106_UART_FDR_DIV_MSK           0x0000000F

/* Multiplier */
#define IP3106_UART_FDR_MUL_POS                    4
#define IP3106_UART_FDR_MUL_LEN                    4
#define IP3106_UART_FDR_MUL_MSK           0x000000F0

/* --------------------------------------------------------------------------
 *  Configuration Register (CFG) (0xFD4) - Read Only
 *  -------------------------------------------------------------------------*/
/* CFG register mask */
#define IP3106_UART_CFG_REG_MSK           0x00003333 /* Bits 2,3, 6,7, 10,11 are unused */
#define IP3106_UART_CFG_REG_RW_MSK        0x00000000 /* Read/Write bits */

/* Type */
#define IP3106_UART_CFG_TYPE_POS                   0
#define IP3106_UART_CFG_TYPE_LEN                   2
#define IP3106_UART_CFG_TYPE_MSK          0x00000003

/* Modem */
#define IP3106_UART_CFG_MODEM_POS                  4
#define IP3106_UART_CFG_MODEM_LEN                  2
#define IP3106_UART_CFG_MODEM_MSK         0x00000030

/* DMA */
#define IP3106_UART_CFG_DMA_POS                    8
#define IP3106_UART_CFG_DMA_LEN                    1
#define IP3106_UART_CFG_DMA_MSK           0x00000100

/* Level */
#define IP3106_UART_CFG_LEVEL_POS                  9
#define IP3106_UART_CFG_LEVEL_LEN                  1
#define IP3106_UART_CFG_LEVEL_MSK         0x00000200

/* IrDA */
#define IP3106_UART_CFG_IRDA_POS                  12
#define IP3106_UART_CFG_IRDA_LEN                   1
#define IP3106_UART_CFG_IRDA_MSK          0x00001000

/* NHP compliant */
#define IP3106_UART_CFG_NHP_POS                   13
#define IP3106_UART_CFG_NHP_LEN                    1
#define IP3106_UART_CFG_NHP_MSK           0x00002000

/* Valid Values */
/* Type */
#define IP3106_UART_CFG_VAL_450                (0x0)
#define IP3106_UART_CFG_VAL_550                (0x1)
#define IP3106_UART_CFG_VAL_650                (0x2)
#define IP3106_UART_CFG_VAL_750                (0x3)

/* Modem */
#define IP3106_UART_CFG_VAL_NO_MODEM           (0x0)
#define IP3106_UART_CFG_VAL_RTSCTS             (0x1)
#define IP3106_UART_CFG_VAL_FULL_MODEM         (0x2)

/* DMA */
#define IP3106_UART_CFG_VAL_NO_DMA             (0x0)
#define IP3106_UART_CFG_VAL_HAS_DMA            (0x1)

/* Level */
#define IP3106_UART_CFG_VAL_NO_LEVEL           (0x0)
#define IP3106_UART_CFG_VAL_HAS_LEVEL          (0x1)

/* IrDA */
#define IP3106_UART_CFG_VAL_NO_IRDA            (0x0)
#define IP3106_UART_CFG_VAL_HAS_IRDA           (0x1)

/* NHP */
#define IP3106_UART_CFG_VAL_NO_NHP             (0x0)
#define IP3106_UART_CFG_VAL_HAS_NHP            (0x1)

/* --------------------------------------------------------------------------
 *  Module Identification Register (MID) (0xFFC) - Read Only
 *  -------------------------------------------------------------------------*/
#define IP3106_UART_MID_REG_MSK           0xFFFFFFFF /* MID register mask */
#define IP3106_UART_MID_REG_RW_MSK        0x00000000 /* Read/Write bits */

/* Aperture */
#define IP3106_UART_MID_APERTURE_POS               0
#define IP3106_UART_MID_APERTURE_LEN               8
#define IP3106_UART_MID_APERTURE_MSK      0x000000FF

/* Minor Revision */
#define IP3106_UART_MID_MINOR_POS                  8
#define IP3106_UART_MID_MINOR_LEN                  4
#define IP3106_UART_MID_MINOR_MSK         0x00000F00

/* Major Revision */
#define IP3106_UART_MID_MAJOR_POS                 12
#define IP3106_UART_MID_MAJOR_LEN                  4
#define IP3106_UART_MID_MAJOR_MSK         0x0000F000

/* Unique Identification Number */
#define IP3106_UART_MID_ID_POS                    16
#define IP3106_UART_MID_ID_LEN                    16
#define IP3106_UART_MID_ID_MSK            0xFFFF0000

#endif /* __ASM_ARM_ARCH_HARDWARE_IP3106_H */
