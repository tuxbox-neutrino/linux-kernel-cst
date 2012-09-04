/**************************************************************************************
 *  linux/include/asm-arm/arch-nevis/param.h
 *
 *  Copyright (C) 2007 Conexant Systems Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 2 as published by
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
 ***************************************************************************************/

/* FIXME: can we get rid of this file ? */

/* tick rate */
/* FIXME: take this from kernel configuration. */
#define HZ	100

#if HZ != 100
#define hz_to_std(a) (((a)*HZ)/100)
#endif

/**************************************************************************************
 * Modifications:
 * $Log$
 *
 ***************************************************************************************/
