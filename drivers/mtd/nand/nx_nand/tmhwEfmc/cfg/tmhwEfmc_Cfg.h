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

#ifndef  TMHWEFMC_CFG_H
#define  TMHWEFMC_CFG_H

/*----------------------------------------------------------------------------
* Standard include files:
*----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
* Project specific include files:
*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
* Component specific include files:
*-----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C"
{
#endif

/*-----------------------------------------------------------------------------
* Typedefs and Macros
*-----------------------------------------------------------------------------*/

/* Scalability Settings Start */

/*!
 * \def TMFL_SD_ALL
 * Select all the APIs for final image. If this flag is disabled then scalability settings as
 * defined below are used by preprocessor.
 * \n NOTE: The flags need to be modified by the user as per system requirements. Also, care
 * should be taken in case multiple units of this IP are available in Hw sybsystem.
 * In case of multiple units with different features User will need to enable superset of
 * features of individual unit. In such case it is Applications responsibility to keep track of which
 * features are supported by which IP.
 */

#define NUMBER_OF_MODULES (1)

#define TMFL_SD_ALL  (0)

//#if (TMFL_SD_ALL == 0)

#define TMFL_EFMCSD_POWER              1
/*!< Select APIs to handle Power management functions. */
#define TMFL_EFMCSD_EBI                0
/*!< Select APIs to handle EBI (Pin sharing) functionality of EFMC */
#define TMFL_EFMCSD_AHB                0
/*!< Select APIs to handle AHB related functions of EFMC */
#define TMFL_EFMCSD_APB                0
/*!< Select APIs to handle APB related functions of EFMC */
#define TMFL_EFMCSD_AES               1
/*!< Select APIs to handle the AES functions of EFMC */
#define TMFL_EFMCSD_DMA               1
/*!< Select APIs to handle the DMA functions of EFMC */
#define TMFL_EFMCSD_INT               1
/*!< Select APIs to handle the Interrupt functions of EFMC */
#define TMFL_EFMCSD_OTHERS             1
/*!< Select APIs to handle other functions. */

//#endif
/* Scalability Settings End */

/*-----------------------------------------------------------------------------
* Global data
*-----------------------------------------------------------------------------*/

typedef struct tmhwEfmc_Cfg
/*! \brief This structure is the HwAPI configuration structure. */
 {
   UInt32  baseAddress; /*!< Physical base address of EFMC IP.*/ 
   pUInt8  ahbMemAddress; /*!< AHB memory address for EFMC IP.*/ 
 } tmhwEfmc_Cfg_t, *ptmhwEfmc_Cfg_t ;

//extern const tmhwEfmc_Cfg_t              gktmhwEfmc_Config[NUMBER_OF_MODULES];
extern tmhwEfmc_Cfg_t              gktmhwEfmc_Config[NUMBER_OF_MODULES];
/*!< Config Structure definitions  */
#define TMHW_EFMC_GET_BASE(x)           (gktmhwEfmc_Config[x].baseAddress)
#define TMHW_EFMC_GET_AHB_ADDRESS(x)           (gktmhwEfmc_Config[x].ahbMemAddress)
/*!< Get Base definitions  */

/*-----------------------------------------------------------------------------
* Exported functions
*-----------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* TMHWEFMC_CFG_H */
