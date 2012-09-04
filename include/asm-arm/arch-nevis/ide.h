/*
 * linux/include/asm-arm/arch-shark/ide.h
 *
 * derived from:
 * linux/include/asm-arm/arch-ebsa285/ide.h
 * Copyright (c) 1998 Russell King
 */

#include <asm/irq.h>


static __inline__ int ide_default_irq(ide_ioreg_t base)
{
	return 0;
}

static __inline__ ide_ioreg_t ide_default_io_base(int index)
{
	return 0;
}


static __inline__ void
ide_init_hwif_ports(hw_regs_t * hw, int data_port, int ctrl_port, int *irq)
{
}

static __inline__ void ide_init_default_hwifs(void)
{
}
