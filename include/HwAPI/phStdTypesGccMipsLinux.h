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

#ifndef _PHSTDTYPESGCCMIPSLINUX_H_
#define _PHSTDTYPESGCCMIPSLINUX_H_


#ifndef TMOAPTMTYPESINCLUDED

typedef char                *String;
typedef unsigned int         Bool;
typedef char                 Char;

typedef signed char          Int8;
typedef short                Int16;
typedef long                 Int32;
typedef long long            Int64;

typedef unsigned char        UInt8;
typedef unsigned short       UInt16;
typedef unsigned long        UInt32;
typedef unsigned long long   UInt64;

#ifdef MIPSEL        /* SDE1 & SDE4 Build */
/* Default */
#elif TMFL_ENDIAN    /* SDE2 Build */
#ifdef TMFL_ENDIAN && (TMFL_ENDIAN == TMFL_ENDIAN_BIG)
#define MIPSEB
#else
#define MIPSEL
#endif
#else
#error Endianess unknown
#endif

typedef struct                          /* Int64: 64-bit signed integer */
{
	/* Get the correct endianness (this has no impact on any other part of
	   the system, but it may make memory dumps easier to understand). */
#ifdef MIPSEB
	Int32 hi; UInt32 lo;
#else
	UInt32 lo; Int32 hi;
#endif
}   _Int64;

typedef struct                          /* UInt64: 64-bit unsigned integer */
{
#ifdef MIPSEB
	UInt32 hi; UInt32 lo;
#else
	UInt32 lo; UInt32 hi;
#endif
}   _UInt64;


#endif /* TMOAPTMTYPESINCLUDED */

typedef UInt32               tmErrorCode_t;

typedef Int8               * pInt8;
typedef Int16              * pInt16;
typedef Int32              * pInt32;
typedef Int64              * pInt64;

typedef UInt8              * pUInt8;
typedef UInt16             * pUInt16;
typedef UInt32             * pUInt32;
typedef UInt64             * pUInt64;

typedef Bool               * pBool;
typedef Char               * pChar;
typedef String             * pString;

#ifdef  TRUE
#undef  TRUE
#endif
#define TRUE        1

#ifdef  FALSE
#undef  FALSE
#endif
#define FALSE       0

#undef  NULL
#define NULL        (0)  /* Not cast to void*, because it should also work for ROM pointers */

#define TM_OK       0

#endif /* _PHSTDTYPESGCCMIPSLINUX_H_ */
