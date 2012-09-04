/****************************************************************************
 * include/asm-arm/arch-nevis/startup.h
 *
 * Copyright (C) 2007 Conexant Systems Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 2as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02111-1307  USA

 ****************************************************************************/
/* $Id $
 ****************************************************************************/


/* GET RID OFF */

#ifndef _STARTUP_H
#define _STARTUP_H

/*****************/
/* Include Files */
/*****************/
#include <asm/arch/hardware.h>

/****************************************************************************/
/*                                                                          */
/* Hardware register access macros                                          */
/*                                                                          */
/****************************************************************************/

#define RMO(y)                   ( ((y) & 0x00000001) ?  0 : \
                                   ( ((y) & 0x00000002) ?  1 : \
                                     ( ((y) & 0x00000004) ?  2 : \
                                       ( ((y) & 0x00000008) ?  3 : \
                                         ( ((y) & 0x00000010) ?  4 : \
                                           ( ((y) & 0x00000020) ?  5 : \
                                             ( ((y) & 0x00000040) ?  6 : \
                                               ( ((y) & 0x00000080) ?  7 : \
                                                 ( ((y) & 0x00000100) ?  8 : \
                                                   ( ((y) & 0x00000200) ?  9 : \
                                                     ( ((y) & 0x00000400) ? 10 : \
                                                       ( ((y) & 0x00000800) ? 11 : \
                                                         ( ((y) & 0x00001000) ? 12 : \
                                                           ( ((y) & 0x00002000) ? 13 : \
                                                             ( ((y) & 0x00004000) ? 14 : \
                                                               ( ((y) & 0x00008000) ? 15 : \
                                                                 ( ((y) & 0x00010000) ? 16 : \
                                                                   ( ((y) & 0x00020000) ? 17 : \
                                                                     ( ((y) & 0x00040000) ? 18 : \
                                                                       ( ((y) & 0x00080000) ? 19 : \
                                                                         ( ((y) & 0x00100000) ? 20 : \
                                                                           ( ((y) & 0x00200000) ? 21 : \
                                                                             ( ((y) & 0x00400000) ? 22 : \
                                                                               ( ((y) & 0x00800000) ? 23 : \
                                                                                 ( ((y) & 0x01000000) ? 24 : \
                                                                                   ( ((y) & 0x02000000) ? 25 : \
                                                                                     ( ((y) & 0x04000000) ? 26 : \
                                                                                       ( ((y) & 0x08000000) ? 27 : \
                                                                                         ( ((y) & 0x10000000) ? 28 : \
                                                                                           ( ((y) & 0x20000000) ? 29 : \
                                                                                             ( ((y) & 0x40000000) ? 30 : \
                                                                                               ( ((y) & 0x80000000) ? 31 : 0 ))))))))))))))))))))))))))))))))

#endif

/****************************************************************************
 * Modifications:
 * $Log:
 *  3    Linux_SDK 1.2         6/14/07 6:25:34 PM IST Debashish Rath  Header
 *       file include which was not needed is removed
 *  2    Linux_SDK 1.1         3/27/07 5:43:24 PM IST Satpal Parmar   GPL
 *       header and Starteam footer addition.
 *  1    Linux_SDK 1.0         3/1/07 11:47:20 PM IST Vineet Seth     
 * $
 ****************************************************************************/
