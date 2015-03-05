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

#ifndef TMNXMODID_H
#define TMNXMODID_H


#ifdef __cplusplus
extern "C"
{
#endif

#define COLUMBUS_21_MOD_ID		0X00000021	/* 3D Comb and SWAN Filters */
#define VMPG_100_MOD_ID			0X00000100	/* High Level MPEG Decoder */
#define C1394_101_MOD_ID		0X00000101	/* 1394 */
#define FPBC_102_MOD_ID			0X00000102	/* PI-Bus Controller @ 150 MHz */
#define JTAG_103_MOD_ID			0X00000103	/* JTAG debug interface for TM  */
#define EJTAG_104_MOD_ID		0X00000104	/* EJTAG debug interface for MIPS */
#define IIC_105_MOD_ID			0X00000105	/* I2C */
#define SMCARD_106_MOD_ID		0X00000106	/* SmartCard */
#define UART_107_MOD_ID			0X00000107	/* UART, full DMA support */
#define CLOCKS_108_MOD_ID		0X00000108	/* Clock Generation */
#define USB_109_MOD_ID			0X00000109	/* USB */
#define BOOT_10A_MOD_ID			0X0000010A	/* Boot logic/Reset */
#define MPBC_10B_MOD_ID			0X0000010B	/* PI-Bus Controller @ 150 MHz */
#define SSI_10C_MOD_ID			0X0000010C	/* SSI Synchronous Serial Interface */
#define AI_10D_MOD_ID			0X0000010D	/* Audio In */
#define VMSP_10E_MOD_ID			0X0000010E	/* MPEG-2 System Part */
#define GPIO_10F_MOD_ID			0X0000010F	/* General Purpose I/O */
#define SPDI_110_MOD_ID			0X00000110	/* SPDIF-in */
#define AICP1_111_MOD_ID		0X00000111	/* Image Composition Processor (1st) */
#define TPBC_112_MOD_ID			0X00000112	/* TM PI-Bus Controller @ 75 MHz */
#define PCI_113_MOD_ID			0X00000113	/* PCI M/S Interface w/ XIO @ 33 MHz */
#define MMI_114_MOD_ID			0X00000114	/* Memory Interface SDRAM 143MHz */
#define ORCA3_115_MOD_ID		0X00000115	/* 3D engine */
#define DBG_116_MOD_ID			0X00000116	/* Hardware Debug (SPY) */
#define DE_117_MOD_ID			0X00000117	/*   */
#define AICP2_118_MOD_ID		0X00000118	/* Image Composition Processor (2nd) */
#define MBS_119_MOD_ID			0X00000119	/* Memory Based Scaler */
#define VIP_11A_MOD_ID			0X0000011A	/* Video Input Processor */
#define PIMI_11B_MOD_ID			0X0000011B	/* PI-DVP Memory Bus bridge (150-143) */
#define PIB_11C_MOD_ID			0X0000011C	/* PI-PI bridge 150-75 MHz */
#define PIC_11D_MOD_ID			0X0000011D	/* Interrupt Controller */
#define AO_120_MOD_ID			0X00000120	/* AO (Audio Out) */
#define SPDO_121_MOD_ID			0X00000121	/* SPDIF-out */
#define FPIMI_122_MOD_ID		0X00000122	/* PI-DVP Memory Bus bridge (75-143) */
#define RESET_123_MOD_ID		0X00000123	/*   */
#define NULL_124_MOD_ID			0X00000124	/*  the response from the buscontroller indicating there is no module */
#define TSDMA_125_MOD_ID		0X00000125	/*   */
#define GLBREG1_126_MOD_ID		0X00000126	/*   */
#define TMDBG_127_MOD_ID		0X00000127	/*   */
#define GLBREG2_128_MOD_ID		0X00000128	/*   */
#define DMA_130_MOD_ID			0X00000130	/*   */
#define IR_131_MOD_ID			0X00000131	/*   */
#define GFX2D_132_MOD_ID		0X00000132	/* Graphics Engine (2D) */
#define P1284_133_MOD_ID		0X00000133	/* Video decoder */
#define QNM_134_MOD_ID			0X00000134	/* Audio decoder */
#define VIDDEC_140_MOD_ID		0X00000140	/* Video decoder */
#define I2D_141_MOD_ID			0X00000141	/* Datalink Receiver (I2D Inter-Integrated Digital bus) */
#define FEF_142_MOD_ID			0X00000142	/* Front End Features */
#define MBF_143_MOD_ID			0X00000143	/* Memory Based Features */
#define BEF_144_MOD_ID			0X00000144	/* Back End Features */
#define DOP_145_MOD_ID			0X00000145	/* Digital Output Processor (Digital CRT driver) */
#define CGFX_146_MOD_ID			0X00000146	/* Character Based Display */
#define DCU_147_MOD_ID			0X00000147	/* Data Capture Unit */
#define SND_148_MOD_ID			0X00000148	/* Multi Channel TV Sound Processor */
#define FGPI_14B_MOD_ID			0X0000014B	/* Fast Generic Parallel Input DMA Block */
#define FGPO_14C_MOD_ID			0X0000014C	/* Fast Generic Parallel Output DMA Block */
#define VLD_14D_MOD_ID			0X0000014D	/* HD MPEG2 (and MPEG-1) VLD Decoder */
#define LAN100_14F_MOD_ID		0X0000014F	/* 10/100 ethernet interface */
#define PMARB_1010_MOD_ID		0X00001010	/* MTL memory bus arbiter */
#define INT_1101_MOD_ID			0X00001101	/* PI Interrupt Controller */
#define GIC_1106_MOD_ID			0X00001106	/* Generic interupt controller */
#define MCU_2031_MOD_ID			0X00002031	/* MTL based multi-port DDS SDRAM controller */
#define EFM_2111_MOD_ID			0X00002111	/* AHB/VPB Embedded Flash Module */
#define PR1910_2B03_MOD_ID		0X00002B03	/* MIPS PR1910 CPU */
#define PR33930_2B10_MOD_ID		0X00002B10	/* MIPS PR3930 */
#define PR3940_2B11_MOD_ID		0X00002B11	/* MIPS PR3940 CPU 16K/8K 150 MHz */
#define TM3218_2B80_MOD_ID		0X00002B80	/* TriMedia TM3218 CPU 200 MHz */
#define TM5250_2B90_MOD_ID		0X00002B90	/* TriMedia processor Spitfire/TM5250 */
#define LITE_2B91_MOD_ID		0X00002B91	/* TriMedia processor TM-Lite (TM2270) */
#define TM3270_2B92_MOD_ID		0X00002B92	/* TriMedia Processor TM3270 (TM-Video) */
#define TIMER_3002_MOD_ID		0X00003002	/* PI Timer */
#define TIMER_3003_MOD_ID		0X00003003	/* VPB Timer Template (to be replaced by IP 3004) */
#define TIMER_3004_MOD_ID		0X00003004	/* VPB Timer Template (to be replaced by IP 3012) */
#define TIMER_3012_MOD_ID		0X00003012	/* VPB Timer Template */
#define UART_3102_MOD_ID		0X00003102	/* Extended UART (DMA) */
#define UART_3103_MOD_ID		0X00003103	/* 16C UART HDL Template (to be replaced by IP 3106) */
#define UART_3106_MOD_ID		0X00003106	/* 16C UART HDL Template */
#define I2C_3203_MOD_ID			0X00003203	/* High Speed IIC controller with DMA */
#define I2C_3204_MOD_ID			0X00003204	/* I2C Template */
#define GPIO_4001_MOD_ID		0X00004001	/* Configurable Multi Purpose IO Port */
#define GPIO_4002_MOD_ID		0X00004002	/* VPB Flexible General Purpose I/O Template */
#define GPIO_4004_MOD_ID		0X00004004	/* VPB Flexible General Purpose I/O Template */
#define ACOMP_A000_MOD_ID		0X0000A000	/* Audio Encoder, e.g. MPEG2L1,DDC */
#define VF_A001_MOD_ID			0X0000A001	/* Video Frontend, Video In, Color Conv,Noise Filter, Scaler */
#define VCOMP_A002_MOD_ID		0X0000A002	/* MPEG2 Video Encoder */
#define OIF_A003_MOD_ID			0X0000A003	/* Output Interface, basically DTL to 8bit parallel out + clk  & sync) */
#define SCR_A004_MOD_ID			0X0000A004	/* System Clock Reference */
#define DVDD_A005_MOD_ID		0X0000A005	/* DTV-DVDD block adapted: register access via DTL-MMIO; data via DTL-DMA bus */
#define EDMA_A006_MOD_ID		0X0000A006	/* enhanced stb_dma: data connected via DTL-DMA bus */
#define DCU_NODMA_A007_MOD_ID		0X0000A007	/* PI-Bus VBI Data Capture Unit without DMA  */
#define TSIN_A00B_MOD_ID		0X0000A00B	/* Transport Stream input?! */
#define QVCP5L_A014_MOD_ID		0X0000A014	/* QVCP */
#define SFE_A016_MOD_ID			0X0000A016	/* Satellite Front End IP */
#define QNM_A017_MOD_ID			0X0000A017	/* Condor: Natural Motion Coprocessor */
#define QVCP2L_A019_MOD_ID		0X0000A019	/* QVCP */
#define D3D_A01A_MOD_ID			0X0000A01A	/* 3D Setup and Rendering Engine */
#define QTNR_A02A_MOD_ID		0X0000A02A	/* Quality Temporal Noise Reduction */
#define VMIBC_A02B_MOD_ID		0X0000A02B	/* MMIO Interconnect Bus Controller (Viper PVR) */
#define PATA_A02C_MOD_ID		0X0000A02C	/* Parallel ATA Host controller (all modes to UDMA 100). */
#define GDMA_A02D_MOD_ID		0X0000A02D	/* Generic DMA physical interface (master/slave, 8-16-bit, etc.) */
#define VIDDEC_SEC_A02E_MOD_ID		0X0000A02E	/* CVBS-only Video decoder (Stripped 0x0140) */
#define M4PP_A02F_MOD_ID		0X0000A02F	/* Mpeg4 Post-processing HW accelerator (M4PP) */
#define M4MC_A030_MOD_ID		0X0000A030	/* Mpeg4 Motion Compensation HW accelerator (M4MC) */
#define M4CSC_A031_MOD_ID		0X0000A031	/* Mpeg4 Colour Space Conversion HW accelerator (M4CSC) */
#define ATA_A032_MOD_ID			0X0000A032	/* ATA Host controller HW block */
#define UCGDMA_A033_MOD_ID		0X0000A033	/* uController & Generic Dma interface */
#define I2S_A034_MOD_ID			0X0000A034	/* i2s with integrated DMA & clock logic */
#define AHB2PCI_A035_MOD_ID		0X0000A035	/* AHB to PCI Bridge */
#define SDAC_A036_MOD_ID		0X0000A036	/* SDAC, serial DAC for audio out  */
#define CAN_A037_MOD_ID			0X0000A037	/* CAN-bus interface (PeliCan ) */
#define ADC_A038_MOD_ID			0X0000A038	/* ADC using SigmaDelta  'C18AD16B44K' from AMoS */
#define CAMERA_A039_MOD_ID		0X0000A039	/*  Camera Interface (Capture block) */
#define MJPEG_A03A_MOD_ID		0X0000A03A	/* motion jpeg codec */
#define CLOCK_A03B_MOD_ID		0X0000A03B	/* MPMP1 clock generator (using C18PL160M) */
#define WATCHDOG_A03C_MOD_ID		0X0000A03C	/* watchdog timer */
#define CHARLCD_A03D_MOD_ID		0X0000A03D	/* character lcd interface */
#define NF_A03E_MOD_ID			0X0000A03E	/* NAND flash interface */
#define TOUCHC_A03F_MOD_ID		0X0000A03F	/* touch screen controller */
#define KEYPAD_A040_MOD_ID		0X0000A040	/* keypad controller (36 keys) */
#define SPI_A041_MOD_ID			0X0000A041	/* BlueBerry/S3 master / slave SPI  */
#define PCM_A042_MOD_ID			0X0000A042	/* BlueBerry/S3 ipint PCM */
#define PWM_A043_MOD_ID			0X0000A043	/* PWM/PDM controller */
#define DAC_A044_MOD_ID			0X0000A044	/* DAC 10-bits 1-chan (AMoS) */
#define ADC_A045_MOD_ID			0X0000A045	/* ADC 10-bits 8-chan (AMoS) */
#define QMBS_A046_MOD_ID		0X0000A046	/* Quality Memory Based Scaler */
#define MDCTL_A047_MOD_ID		0X0000A047	/* DCS Controller for Viper 2 MIPS Bus Segment */
#define TDCTL_A048_MOD_ID		0X0000A048	/* DCS Controller for Viper 2 TM Bus Segment */
#define BLMP_A049_MOD_ID		0X0000A049	/* DCS Controller for BLMP PNX1315 Project */
#define TUNNEL_A04A_MOD_ID		0X0000A04A	/* A chip-to-chip interconnect supporting high bandwidth streaming data & low latency IO traffic */
#define SIF_A04B_MOD_ID			0X0000A04B	/* Low-cost, High performance 16 bits SDRAM interface */
#define MIU_A04C_MOD_ID			0X0000A04C	/* Low-cost Memory interface unit to connect 8 and 16 bits peripherals and memories */
#define DISPLAY_A04D_MOD_ID		0X0000A04D	/* Display unit with scaling/extracting features. 2 taps vertical filter, 10 taps horizontal */
#define VIEWCONTROL_A04E_MOD_ID		0X0000A04E	/* Cryptographic module to introduce a low-cost anti-piracy system in chips */
#define DENC_A04F_MOD_ID		0X0000A04F	/* CVBS/Y and C  encoder used to  drive the  DACs */
#define LCDC_A050_MOD_ID		0X0000A050	/* TFT LCD Power sequencing module */
#define PCI_A051_MOD_ID			0X0000A051	/*  PCI  w/ 16bit XIO and DTL initiator/DTL targets */
#define QVCP2L_A052_MOD_ID		0X0000A052	/* Two layer QVCP */
#define RTC_A053_MOD_ID			0X0000A053	/* Real Time Clock MPMP1 */
#define DOP_JAGUAR_A054_MOD_ID		0X0000A054	/* DOP Standalone Core for Jaguar Project */
#define DOP_TOP_GPRU_A055_MOD_ID	0X0000A055	/* DOP Standalone Top Level */
#define PMSEC_A056_MOD_ID		0X0000A056	/* Pipelined Memory Access Networks (PMAN) Security */
#define DMAMON_A057_MOD_ID		0X0000A057	/* Pipelined Memory Access Networks (PMAN) monitor */
#define MDSEC_A058_MOD_ID		0X0000A058	/* DCS controller Security module for the Viper2 MIPS network */
#define TDSEC_A059_MOD_ID		0X0000A059	/* DCS controller Security module for the Viper2 Trimedia network  */
#define GPIO_A05C_MOD_ID		0X0000A05C	/* General puprose IO 24 bit Bidirectional */
#define INTERLACER_A05D_MOD_ID		0X0000A05D	/* Converts progressive video flow into interlaced video */
#define ITU656F_A05F_MOD_ID		0X0000A05F	/* Psuedo ITU-656 text, video and colour burst data formatter */
#define ESP_A060_MOD_ID			0X0000A060	/* Embedded Security Processor */
#define STPP0_A061_MOD_ID		0X0000A061	/* HDLi template - streaming IP to IP connector */
#define EDMA3_A062_MOD_ID		0X0000A062	/* Enhanced dvi_edma with 3DES functionality and connected to DTL_DMA Bus */
#define CLOCKS_A063_MOD_ID		0X0000A063	/* clock block */
#define RESET_A064_MOD_ID		0X0000A064	/* Reset Block for the PNX1500 */
#define GPIO_A065_MOD_ID		0X0000A065	/* General Purpose I/O for PNX1500 */
#define MMON_A066_MOD_ID		0X0000A066	/* memory system monitor used to monitor a PMAN (Pipelined Memory Access Network) system */
#define SND_SYS_A067_MOD_ID		0X0000A067	/* TV Sound Core, containing Demodulator/Decoder and Audio DSPs */
#define TCTRL_A068_MOD_ID		0X0000A068	/* Global control register for Tiger chip  */
#define UPC_A069_MOD_ID			0X0000A069	/* Video frame up-converter on Tiger chip */
#define VPK_A06A_MOD_ID			0X0000A06A	/* Vertical Peaking module on Tiger chip */
#define TASK_A06B_MOD_ID		0X0000A06B	/* Task module on Tiger chip */
#define QTUN_OUT_A06C_MOD_ID		0X0000A06C	/* The counterpart of sat_qtun_in supporting high bandwidth chip-to-chip communication and data traffic */
#define DCSC_A06D_MOD_ID		0X0000A06D	/* DCS Controller Template (Configuration Aperture) */
#define DCSC_SECURITY_A06E_MOD_ID	0X0000A06E	/* DCS Controller Template (Security Aperture) */
#define QVCP2L_LITE_A06F_MOD_ID		0X0000A06F	/* QVCP-2L with no pool elements */
#define ME_A070_MOD_ID			0X0000A070	/* Motion Estimator */
#define TC_A071_MOD_ID			0X0000A071	/* Video Texture Codec for MPEG 2/4 */
#define BSG_A072_MOD_ID			0X0000A072	/* Bit-stream Generator for MPEG video encoding */
#define PAK_A073_MOD_ID			0X0000A073	/* Bit-stream packer */
#define EVENT_ROUTER_A075_MOD_ID	0X0000A075	/* asynchronous event/interrupt capture and routing */
#define PCCARD_A076_MOD_ID		0X0000A076	/* 16-bit PC-Card plus ATA-100 host.  (No Card-Bus.) */
#define MTL_MONITOR_A077_MOD_ID		0X0000A077	/* The MTL Monitor observes traffic at a given point and stores bandwidth/latency information */
#define RTC_A078_MOD_ID			0X0000A078	/* Real Time Clock */
#define MDCTL_A079_MOD_ID		0X0000A079	/* DCS Controller for IRIS (BLBA) MIPS Bus Segment */
#define EDMA_V2_A07A_MOD_ID		0X0000A07A	/* Enhanced DMA Engine */
#define VO_A07B_MOD_ID			0X0000A07B	/* Simple Video Output and an external Sync slave */
#define LVDS_LTX_A07C_MOD_ID		0X0000A07C	/* Low Voltage Differential Signaling(LVDS) Transmitter IP */
#define MPIP_A07D_MOD_ID		0X0000A07D	/* (Multi-) Picture in Picture with CSM */
#define HD_CLOCKS_A07E_MOD_ID		0X0000A07E	/* clock block for pnx2015 */
#define QTUN_OUT_A07F_MOD_ID		0X0000A07F	/* The south chip interconnect supporting high bandwidth streaming data & low latency IO traffic */
#define HOST_IF_A080_MOD_ID		0X0000A080	/* Host IF  */
#define CPIPE_MVP_A081_MOD_ID		0X0000A081	/* Video Output Pipe for Monarch */
#define EDMA_CA_A082_MOD_ID		0X0000A082	/* EDMA with conditional access */
#define CDU_MMU_A083_MOD_ID		0X0000A083	/* Central Data Unit with Memory management */
#define BMI_A084_MOD_ID			0X0000A084	/* Burst memory interface for connection to DVD+RW frontend */
#define BM_A085_MOD_ID			0X0000A085	/* Buffer manager for optimized frontend-backend connection */
#define UBAR_A086_MOD_ID		0X0000A086	/* Universal Block Artefact Removal (UBAR) */
#define MSVD_A087_MOD_ID		0X0000A087	/* Multi-Standard Video Decoder (MSVD) */
#define MSVD_CORE_A088_MOD_ID		0X0000A088	/* MSVD Core */
#define EDA_A089_MOD_ID			0X0000A089	/* Entropy Decoding Accelerator (EDA) */
#define TSU_A08A_MOD_ID			0X0000A08A	/* PNX2015 Time-Stamp Unit */
#define HD_GLBREG_A08B_MOD_ID		0X0000A08B	/* Miscellaneous control register block for PNX2015 */
#define AIO_A08C_MOD_ID			0X0000A08C	/* audio inout */
#define AXI_MONITOR_A08D_MOD_ID		0X0000A08D	/* The AXI Monitor observes traffic at a given point and stores bandwidth/latency information */
#define USB2_HS_OTG_A08E_MOD_ID		0X0000A08E	/* USB2 High speed OTG unit with DTL bus */
#define CPIPE_PROPIC_A08F_MOD_ID	0X0000A08F	/* Composition Pipe for the Propic Project */
#define MTLHUB_A090_MOD_ID		0X0000A090	/* configurable MTL hub template */
#define ADC_IF_A091_MOD_ID		0X0000A091	/* Interface between VPB and C18AD10b400k  */
#define MBVP_A092_MOD_ID		0X0000A092	/* Memory Based Video Processor */
#define PNX8520_CLOCKS_A093_MOD_ID	0X0000A093	/* Clock block for PNX8520 device */
#define TSU16_A094_MOD_ID		0X0000A094	/* Time stamp unit (16 time stamps) for PNX8520 */
#define PNX8520_CI_A095_MOD_ID		0X0000A095	/* Conditional access block used on pnx8520 */
#define PNX8520_GLBREG1_A096_MOD_ID	0X0000A096	/* Global registers for PNX8520 */
#define ITU656F_A097_MOD_ID		0X0000A097	/* ITU Formatter for PNX8520 */
#define RGU_A098_MOD_ID			0X0000A098	/* Reset Builder */
#define SATA_HOST_A099_MOD_ID		0X0000A099	/* Serial ATA Host controller IP */
#define DICFID_A09A_MOD_ID		0X0000A09A	/* Digital IVN Chip / Feature ID */
#define SCU_A09B_MOD_ID			0X0000A09B	/* Digital IVN System Control Unit */
#define PWM_A09C_MOD_ID			0X0000A09C	/* Digital IVN Pulse Width Modulator */
#define CPIPE_V1_A09D_MOD_ID		0X0000A09D	/* Composition Pipe for the VSD Program */
#define SVM_A09E_MOD_ID			0X0000A09E	/* scan velocity modulation */
#define PNX8520_MDCN_A09F_MOD_ID	0X0000A09F	/* DCS Network for PNX8520 MIPS */
#define PNX8520_MDCN_A0A0_MOD_ID	0X0000A0A0	/* DCS Network Security for PNX8520 MIPS */
#define PNX8520_TDCN_A0A1_MOD_ID	0X0000A0A1	/* DCS Network for PNX8520 Trimedia */
#define PNX8520_TDCN_A0A2_MOD_ID	0X0000A0A2	/* DCS Network Security for PNX8520 Trimedia */
#define SYSCREG_A0A3_MOD_ID		0X0000A0A3	/* System configuration register */
#define SND_SYS_A0A4_MOD_ID		0X0000A0A4	/* TV Sound Core Demodulator and Decoder, DemDec DSP */
#define SND_SYS_A0A5_MOD_ID		0X0000A0A5	/* TV back-end audio processing, Audio DSP */
#define OUT_A0A6_MOD_ID			0X0000A0A6	/* Generic Transport stream or byte output interface */
#define FR_DLC_A0A7_MOD_ID		0X0000A0A7	/* FlexRay Data Link Controller */
#define CGU_A0A8_MOD_ID			0X0000A0A8	/* Clock Builder */
#define AHBMON_A0A9_MOD_ID		0X0000A0A9	/* AHB Bus Monitor */
#define TVCLASSREG_A0AA_MOD_ID		0X0000A0AA	/* TV Tuner Class Register */
#define EXTINT_A0AB_MOD_ID		0X0000A0AB	/* External Interrupts */
#define IOCONF_A0AC_MOD_ID		0X0000A0AC	/* GPIO */
#define MEM_DGEN_A0AD_MOD_ID		0X0000A0AD	/* DTL data generator, receiver and checker */
#define CPIPE_L2_A0AE_MOD_ID		0X0000A0AE	/* Composition PIPE for Video Signals created for TV520 / PNX8535 */
#define S2D_AS2D_A0AF_MOD_ID		0X0000A0AF	/* Audio stream to dtl converter */
#define S2D_DS2D_A0B0_MOD_ID		0X0000A0B0	/* VBI stream to dtl converter */
#define S2D_TS2D_A0B1_MOD_ID		0X0000A0B1	/* Transport stream to dtl converter */
#define S2D_VS2D_A0B2_MOD_ID		0X0000A0B2	/* VTL stream to dtl converter */
#define DRM_A0B3_MOD_ID			0X0000A0B3	/* Digital Right Management Module */
#define LVDSTX10_A0B4_MOD_ID		0X0000A0B4	/* 10bit lvds transmitter interface */
#define INTERRUPT_VA_A0B5_MOD_ID	0X0000A0B5	/* Interrupt collector */
#define PMU_A0B6_MOD_ID			0X0000A0B6	/* Gallardo Power Management Unit */
#define PNX8535_GLBREG_A0B7_MOD_ID	0X0000A0B7	/* Global register module for PNX8535 */
#define RANDOM_GENERATOR_A0B8_MOD_ID	0X0000A0B8	/* True random generator based on an analog clock */
#define SND_TOP_A0B9_MOD_ID		0X0000A0B9	/* TV Sound Core Demodulator and Decoder, DemDec DSP ASdec */
#define SND_TOP_A0BA_MOD_ID		0X0000A0BA	/* TV back-end audio processing, Audio DSP APP */
#define LVDS_DRX_A0BB_MOD_ID		0X0000A0BB	/* Dual LVDS reciever */
#define CHAGALL_A0BD_MOD_ID		0X0000A0BD	/* Multi-standard video-compression IP */
#define BLOB_A0BE_MOD_ID		0X0000A0BE	/* Wasabi "Yellow Blob" Level 2 Cache */

/******************************************************************************
* Compatibility names for historic misnomers
* NOTE: This legacy hardware module ID name list addendum is not part of the
*       tmNxModId.h automatically generated by the MoReUse website.  It is
*       added by NDK to allow older components to build with the new header
*       file.  When a new version of tmNxModId.h is downloaded from the
*       MoReUse website, this section should be copied from this file into
*       the new file before checking it into the NDK project (until all
*       legacy module ID references have been removed from NDK).
*******************************************************************************
*/
#define AICP_111_MOD_ID           AICP1_111_MOD_ID
#define AICP_118_MOD_ID           AICP2_118_MOD_ID
#define GLBREG_126_MOD_ID         GLBREG1_126_MOD_ID
#define GLBREG_128_MOD_ID         GLBREG2_128_MOD_ID
#define GFX2D_131_MOD_ID          GFX2D_132_MOD_ID
#define PR3930_2B10_MOD_ID        PR33930_2B10_MOD_ID
#define TM3260_2B80_MOD_ID        TM3218_2B80_MOD_ID
#define TM64_2b81_MOD_ID          TM64_2B81_MOD_ID
#define IIC_3203_MOD_ID           I2C_3203_MOD_ID
#define DVDCSS_A005_MOD_ID        DVDD_A005_MOD_ID
#define QVCP_A014_MOD_ID          QVCP5L_A014_MOD_ID
#define QVCP_A019_MOD_ID          QVCP2L_A019_MOD_ID
#define LVDS_A07C_MOD_ID          LVDS_LTX_A07C_MOD_ID


#ifdef __cplusplus
}
#endif

#endif   /* TMNXMODID_H */
