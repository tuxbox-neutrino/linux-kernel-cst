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


//-----------------------------------------------------------------------------
// FILE NAME:    tmSysCfg.h
//
// DESCRIPTION:  This is the platform specific system configuration header
//               file.  It's purpose is to provide a central place where 
//               platform specific settings can be changed by the customer
//               (or whoever is building the DVP platform software).  This 
//               file is included into DVP component configuration source
//               files (tmdl<Component>Cfg.c).  Values defined here will 
//               override the values defined in the local source files.
//
//               This should be the only file modified by the end user for
//               a specific platform (changing the individual DVP component 
//               configuration source files will have an impact on all target
//               platforms and operating systems using that component).  To
//               override the default values in the component configuration
//               file(s), just #define the new values in this file (refer to
//               the DVP component configuration files for the #define'able
//               settings):
//
//               #define TM[LAYER]_<COMP>_CFG_XXX   <NewDefault>
//
//               Values can be defined in tmSysCfg.h for specific CPU/OS 
//               build configurations using the tmFlags.h build flags such 
//               as this example for the tmbslV2Pci x86_nt host CPU debug 
//               memory buffer size:
//
//               // Reserve 1MB debug memory buffer for x86_nt target builds
//               #if   ((TMFL_CPU_IS_X86) && (TMFL_OS_IS_NT))
//               #define TMBSL_V2PCI_HOSTDBG_SIZE   0x100000
//               #endif // ((TMFL_CPU_IS_X86) && (TMFL_OS_IS_NT))
//
//               DVP component configuration file #define'd values should be
//               MoReUse compliant so that there are no conflicts between 
//               different components or between layers of the same component
//               type.  DVP component configuration files should define all
//               values surrounded by the #ifndef/#define/#endif preprocessor
//               commands so that if values with the same name are defined in
//               this file (tmSysCfg.h), they override the component header
//               file values without generating compiler redefinition warnings:
//
//               #ifndef TM[LAYER]_<COMP>_CFG_XXX  // If !defined in tmSysCfg.h
//               #define TM[LAYER]_<COMP>_CFG_XXX   <ComponentCfgDefault>
//               #endif
//
// DOCUMENT REF: DVP Software Coding Guidelines specification
//               MoReUse Naming Conventions specification
//               DVP Software Architecture specification
//               DVP Device Library specification
//
// NOTES:        System RAM region size defines for each board/OS combination
//               are located in the BSL target board/OS specific files:
//
//  /DS_ndk/boards/comps/tmbsl<BoardComp>/src/<targetOS>/tmbslBoardOsSpecific.c
//
//               Refer to the specific BSL board files for more details about
//               RAM configuration options in single and multiple processor
//               hardware configurations.
//-----------------------------------------------------------------------------
//
#ifndef TMSYSCFG_H 
#define TMSYSCFG_H

#include <tmFlags.h>                    // DVP target environment build flags

#ifdef __cplusplus
extern "C"
{
#endif


//-----------------------------------------------------------------------------
// Types and defines:
//-----------------------------------------------------------------------------
//
// TODO: Define any values here that will be used to used to override default
//       component configuration values (optionally using CPU/OS conditional
//       flags from tmFlags.h).
//


// Possibility for customers (projects) to have their own addition/overrides on
// values that are defined here.

#ifdef TMFL_TMCUSTOMSYSCFG_H
#include <tmCustomSysCfg.h>
#endif

#ifdef __cplusplus
}
#endif

#endif // TMSYSCFG_H //-----------------
