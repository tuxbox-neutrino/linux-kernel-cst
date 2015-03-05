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


#ifndef _PHEXTRATYPES_H_
#define _PHEXTRATYPES_H_

#ifndef TMOAPTMTYPESINCLUDED

#define	False		0
#define	Null		0
#define	True		1

typedef void            Void;
typedef char       *    Address;
typedef char const *	ConstAddress;
typedef void       *	Pointer;	/* pointer to anonymous object */
typedef void const *	ConstPointer;
typedef char const *	ConstString;

typedef unsigned char   Byte;		/* raw byte */
typedef int		Int;		/* machine-natural integer */
typedef unsigned int	UInt;		/* machine-natural unsigned integer */
typedef float		Float;		/* fast float */
typedef float		Float32;	/* single-precision float */
#if	!defined(__MWERKS__)
typedef double		Float64;	/* double-precision float */
#endif

#if defined(__KERNEL__) && defined(__linux__)
# include <linux/types.h>

typedef unsigned long   Flags;
#else
typedef UInt32          Flags;

#if	defined(_WIN32)
#define	LL_CONST(c)	(c##i64)
#define	ULL_CONST(c)	(c##ui64)
#define LL_MOD		"I64"
#else	/* !defined(_WIN32) */
#define	LL_CONST(c)	(c##LL)
#define	ULL_CONST(c)	(c##ULL)
#define LL_MOD		"ll"
#endif	/* !defined(_WIN32) */

#endif	/* !defined(KERNEL) */


#if defined (TMFL_DVP4_BUILD) || defined(SDE4_BUILD)
typedef Int		Endian;
#define	BigEndian	0
#define	LittleEndian	1  /* type clashes with winperf.h declaration */
#endif

typedef enum { TM32 = 0, TM3260, TM5250, TM2270, TM3270,
	TM64=100 } TMArch;
/* TM32 = 0 for compatibility!, allow many tm32 versions before TM64 */
extern char* TMArch_names[];
/* LTS 601673 - To determnie the TMArch value from the name, we need this
   array because TMArch values are not consecutive */
extern TMArch TMArch_values[];

typedef struct {
	UInt8  majorVersion;
	UInt8  minorVersion;
	UInt16 buildVersion;
} tmVersion_t, *ptmVersion_t;

extern char *get_TMArch_name(TMArch arch) ;

#endif

#endif /* _PHEXTRATYPES_H_ */
