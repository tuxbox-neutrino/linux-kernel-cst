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

#ifndef __ASM_ARCH_LOCALTIMER_H
#define __ASM_ARCH_LOCALTIMER_H

#define LOCAL_TIMER_RELOAD		0x00
#define LOCAL_TIMER_COUNTER		0x04
#define LOCAL_TIMER_CONTROL		0x08
#define LOCAL_TIMER_STATUS		0x0C

#define LOCAL_TIMER_CTRL_IRQ_ENA	0x04
#define LOCAL_TIMER_CTRL_AUTORELOAD_ENA	0x02
#define LOCAL_TIMER_CTRL_TIMER_ENA	0x01

#define LOCAL_TIMER_STAT_EVENT		0x01

#endif
