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

#ifndef TMSTDLIB_H
#define TMSTDLIB_H
#include <linux/types.h>

// _DLL is defined if compiling for NT and -MDd
#if        !defined(__CRTIMP)
#if        defined(_DLL) || (defined(UNDER_CE) && !defined(COREDLL))
#define __CRTIMP __declspec(dllimport)
#else   // defined(_DLL) || (defined(UNDER_CE) && !defined(COREDLL))
#define __CRTIMP
#endif  // defined(_DLL) || (defined(UNDER_CE) && !defined(COREDLL))
#endif  // !defined(__CRTIMP)

#ifdef _MSC_VER
#define __CDECL __cdecl
#else   // TMFL_OS_IS_CE || TMFL_OS_IS_NT
#define __CDECL
#endif  // TMFL_OS_IS_CE || TMFL_OS_IS_NT

// overload the __CRTIMP and __CDECL defines as 'empty'
#if defined(__GNUC__) || defined(UNDER_CE)
#undef __CRTIMP
#define __CRTIMP
#undef __CDECL
#define __CDECL
#endif
#include <linux/string.h>

#endif // TMSTDLIB_H //------------------
