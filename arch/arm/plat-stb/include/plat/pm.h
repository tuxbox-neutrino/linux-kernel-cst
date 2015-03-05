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

#ifndef __PLAT_PM_H
#define __PLAT_PM_H

struct stb_pm_stdby_ctrlr_ops
{
	int (*prepare)(void);
	int (*enter)(unsigned long);
	int (*exit)(void);
	int (*finish)(void);
	int (*recover)(void);
};
struct stb_pm_gic_dist_save {
	u32     cpu_ctrl;
	u32     cpu_primask;
	u32     cpu_binpoint;
	u32     cpu_intack;
	u32     cpu_eoi;
	u32     cpu_runningpri;
	u32     cpu_highpri;
	u32     dist_enable_set[8];
	u32     dist_enable_clear[8];
	u32     dist_pending_set[8];
	u32     dist_softint;
};

extern int stb_pm_register(struct stb_pm_stdby_ctrlr_ops *pOps);

#endif /* __PLAT_PM_H */
