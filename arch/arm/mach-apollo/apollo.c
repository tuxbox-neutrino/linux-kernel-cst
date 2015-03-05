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

#include <linux/of_platform.h>
#include <linux/irqchip.h>
#include <linux/clocksource.h>
#include <linux/dma-mapping.h>
#include <linux/phy.h>
#include <asm/mach/arch.h>
#include <mach/generic.h>
#include <mach/soc.h>
#include <mach/timex.h>
#include <mach/splash_logo.h>
#include <asm/setup.h>
#include <linux/stmmac.h>

#ifdef CONFIG_ARCH_HAS_BARRIERS
unsigned long *pDummyUncachedWrite0 = NULL;
unsigned long DummyUncachedRead0 = 0;
unsigned long *pDummyUncachedWrite1 = NULL;
unsigned long DummyUncachedRead1 = 0;

void DoDummyWriteRead(void)
{
	if(pDummyUncachedWrite0) {
		*pDummyUncachedWrite0 = 0;
		DummyUncachedRead0 = *pDummyUncachedWrite0;
	}
	if(pDummyUncachedWrite1) {
		*pDummyUncachedWrite1 = 0;
		DummyUncachedRead1 = *pDummyUncachedWrite1;
	}
}
EXPORT_SYMBOL(DoDummyWriteRead);
#endif

static struct stmmac_dma_cfg gmac0_dma_cfg = {
	.burst_len	= DMA_AXI_BLEN_256,
	.fixed_burst	= 1,
	.pbl		= 32,
};
static struct stmmac_dma_cfg gmac1_dma_cfg = {
	.burst_len	= DMA_AXI_BLEN_256,
	.fixed_burst	= 1,
	.pbl		= 32,
};

static struct plat_stmmacenet_data gmac0_plat_dat = {
	.dma_cfg	= &gmac0_dma_cfg,
	.clk_csr	= 1, /* Deac Fifo - 2K */
	.enh_desc		= 1,
	.bugged_jumbo		= 1,
	.force_sf_dma_mode	= 1,
};

static struct plat_stmmacenet_data gmac1_plat_dat = {
	.dma_cfg	= &gmac1_dma_cfg,
	.clk_csr	= 1, /* Deac Fifo - 2K */
	.enh_desc		= 1,
	.bugged_jumbo		= 1,
	.force_sf_dma_mode	= 1,
};

static int apollo_platform_notifier(struct notifier_block *nb,
				  unsigned long event, void *__dev)
{
	struct device *dev = __dev;

	if (event != BUS_NOTIFY_ADD_DEVICE)
		return NOTIFY_DONE;

	if (of_device_is_compatible(dev->of_node, "entr,gmac")) {
		void __iomem *reg = NULL;
		struct plat_stmmacenet_data *plat_dat = NULL;
		struct resource *res;
		u32 regval, id;

		res = platform_get_resource(to_platform_device(dev),
					    IORESOURCE_MEM, 0);

		if (res) {
			if (__IOMEM(res->start) == SOC_GMAC0_BASE) {
				reg = (void __iomem *)SOC_GMAC0_CTRL_REG;
				plat_dat = &gmac0_plat_dat;
				id = 0;
			} else if ( __IOMEM(res->start) == SOC_GMAC1_BASE) {
				reg = (void __iomem *)SOC_GMAC1_CTRL_REG;
				plat_dat = &gmac1_plat_dat;
				id = 1;
			}
		} else
			return NOTIFY_DONE;

		if (!reg || !plat_dat)
			return NOTIFY_DONE;

		if (!dev->platform_data)
			dev->platform_data = plat_dat;
		else
			plat_dat = dev->platform_data;

			
		regval = (readl(reg) >> 4) & 3;
		if (regval == 0) /* MII */
			plat_dat->interface = PHY_INTERFACE_MODE_MII;
		else if (regval == 1)
			plat_dat->interface = PHY_INTERFACE_MODE_RGMII;
		else if (regval == 2)
			plat_dat->interface = PHY_INTERFACE_MODE_RMII;
		else /* reserved */
			plat_dat->interface = PHY_INTERFACE_MODE_MII;

		pr_info("GMAC%u: active interface is %d\n", id, plat_dat->interface);
	}

	return NOTIFY_OK;
}

static struct notifier_block apollo_platform_nb = {
	.notifier_call = apollo_platform_notifier,
};

static void __init apollo_dt_init(void)
{
	int ret;

#ifdef CONFIG_STB_SPLASH
	stb_splash();
#endif

#ifdef CONFIG_ARCH_HAS_BARRIERS
	pDummyUncachedWrite0 = ioremap_nocache(meminfo.bank[0].start + (meminfo.bank[0].size - SZ_2M), PAGE_SIZE);
	if (!pDummyUncachedWrite0) {
		pr_err("failed to ioremap barrier pool\n");
		BUG();
	}

	if (meminfo.nr_banks > 1) {
		pDummyUncachedWrite1 = ioremap_nocache(meminfo.bank[1].start + (meminfo.bank[1].size - SZ_2M), PAGE_SIZE);
		if (!pDummyUncachedWrite1) {
			pr_err("failed to ioremap barrier pool\n");
			BUG();
		}
	}
#endif
	bus_register_notifier(&platform_bus_type, &apollo_platform_nb);

	ret = of_platform_populate(NULL, of_default_bus_match_table,
		NULL, NULL);
	if (ret) {
		pr_err("of_platform_populate failed: %d\n", ret);
		BUG();
	}

	stb_get_chip_rev_id();
}

static const char * const apollo_dt_board_compat[] = {
	"entr,apollo",
	NULL,
};

DT_MACHINE_START(APOLLO_DT, "Entropic Apollo STB Platform with FDT")
	.smp		= smp_ops(apollo_smp_ops),
	.map_io		= apollo_map_io,
	.init_irq	= irqchip_init,
	.init_time	= apollo_timer_init,
	.init_machine	= apollo_dt_init,
	.restart	= apollo_restart,
	.dt_compat	= apollo_dt_board_compat,
MACHINE_END
