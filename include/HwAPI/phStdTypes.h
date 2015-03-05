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

#ifndef _PHSTDTYPES_H_
#define _PHSTDTYPES_H_

#if defined(__TCS__)			               /* Trimedia compilation System */
#include "phStdTypesTcs.h"

#elif defined(TORNADO_GNU) || defined(VIPER) || ( defined (TMFL_OS_IS_VXWORKS) && (TMFL_OS_IS_VXWORKS==1) )  /* Viper Mips VxWorks (SDE4: TORNADO_GNU, SDE1: VIPER) */
#include "phStdTypesGccMipsVxWorks.h"

#elif defined(STBUP_TASKING)                   /* 8051 Standby Controller */
#include "phStdTypesTasking8051.h"

#elif defined(_WIN32)                          /* Windows X86 simulation */
#include "phStdTypesMsvcX86.h"

#elif defined(LINUX) && defined(MONTAVISTA_GNU)/* Linux MIPS build */
#include "phStdTypesGccMipsLinux.h"

#elif defined(LINUX)
#include "phStdTypesGccX86Linux.h"             /* Linux X86 build */

#else

#error undefined toolchain, please add a new phStdTypes instance for this toolchain to osapi

#endif

#endif /* _PHSTDTYPES_H_ */

