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

#ifndef __MACH_BARRIERS_H
#define __MACH_BARRIERS_H
 
extern void DoDummyWriteRead(void);

#define rmb()     dmb()
#define mb()      do { dsb(); outer_sync(); DoDummyWriteRead(); dsb(); } while (0)
#define wmb()     mb()
 
#endif  /* __MACH_BARRIERS_H */
