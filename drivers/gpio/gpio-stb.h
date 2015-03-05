#ifndef __GPIO_STB_H
#define __GPIO_STB_H

#include <linux/gpio.h>

struct stb_gpio_chip
{
	struct gpio_chip chip;
	int irq_base;
	int irq;
	struct mutex mutex;
	spinlock_t lock;
	spinlock_t irq_lock;
	u32 usage;
	void __iomem * base;
	unsigned int no_irq:1;
	struct irq_domain *irq_domain;
};

#define GET_OFFSET(off)      ((off) >> 4)
#define GET_OFFSET_WORD(off) ((off) >> 5)

#define GPIO_MODE_CTRL       0x000
#define GPIO_MASK_IODATA     0x038
#define GPIO_CTRL_REG        0x070
#define GPIO_INT_STATUS      0x0A8
#define GPIO_INT_ENABLE      0x0E0
#define GPIO_INT_CLEAR       0x0FC
#define GPIO_INT_SET         0x118
#define GPIO_POWER_DOWN      0xFF4
#define GPIO_MODULE_REG      0xFFC

#define GPIO_MODULE_ID       0xA0943000

#define GPIO_MODE_PRIMARY    0x01UL
#define GPIO_MODE_IO         0x02UL
#define GPIO_MODE_OPENDRAIN  0x03UL

#define GPIO_DIR_IN          0x01UL
#define GPIO_DIR_OUT_LO      0x02UL
#define GPIO_DIR_OUT_HI      0x03UL

static inline void gpset_mode(void __iomem * base, int offset, int mode)
{
	u32 regoffset = GPIO_MODE_CTRL + (GET_OFFSET(offset)*sizeof(u32*));
	u32 pinvalue = ((mode & 0x03) << ((offset & 0xF) * 2));

	writel(pinvalue, base + regoffset);
}

static inline int gpget_mode(void __iomem * base, int offset)
{
	return readl(base + GPIO_MODE_CTRL + (GET_OFFSET(offset)*sizeof(u32*)));
}


static inline void gpset_dir(void __iomem * base, int offset, int mode)
{
	u32 regoffset = GPIO_MASK_IODATA + (GET_OFFSET(offset)*sizeof(u32*));
	u32 off = offset & 0xF;
	u32 pinvalue = ((mode & 1) << off) | ((mode & 2) << (15 + off));

	writel(pinvalue, base + regoffset);
}

static inline int gpget_dir(void __iomem * base, int offset)
{
	return readl(base + GPIO_MASK_IODATA + (GET_OFFSET(offset)*sizeof(u32*)));
}


static inline int gpget_value(void __iomem * base, int offset)
{
	return (readl(base + GPIO_MASK_IODATA + (GET_OFFSET(offset)*sizeof(u32*))) 
			& BIT(offset & 0xF)) != 0;
}

static inline void gpset_type(void __iomem * base, int offset, int type)
{
	u32 regoffset = GPIO_CTRL_REG + (GET_OFFSET(offset)*sizeof(u32*));
	u32 val = readl(base + regoffset);
	int off = (offset & 0x0F) << 1;

	val = (val & ~(3UL << off)) | (type << off);

	writel(val, base + regoffset);
}

static inline int gpget_type(void __iomem * base, int offset)
{
	return readl(base + GPIO_CTRL_REG + GET_OFFSET(offset));
}

static inline u32 gpget_masked_int_status(void __iomem * base, int offset)
{
	u32 intstatusval;
	u32 regoffsetintenb = GPIO_INT_ENABLE + ((offset/2) * sizeof(u32*));
	u32 regoffsetintstatus = GPIO_INT_STATUS + ((offset) * sizeof(u32*));
	u32 intenbaleval = readl(base + regoffsetintenb);

	if (0 == (offset % 2)) /*1st 16 bit*/
	{
		intenbaleval &= 0xFFFFUL;
		intenbaleval |= (intenbaleval << 16);
	}
	else
	{
		intenbaleval &= 0xFFFF0000UL;
		intenbaleval |= (intenbaleval >> 16);
	}

	intstatusval = readl(base + regoffsetintstatus);

	/*printk("gpget_masked_int_status::intenbaleval=%x intstatusval=%x\n",intenbaleval,intstatusval);*/

	return (intenbaleval & intstatusval);
}

static inline struct stb_gpio_chip *to_ach(struct gpio_chip *chip)
{
	return container_of(chip, struct stb_gpio_chip, chip);
}

static inline void gpset_int(void __iomem * base, int offset)
{
	u32 val;
	u32 regoffsetintenb = GPIO_INT_SET + (GET_OFFSET_WORD(offset)*sizeof(u32));
	val = 1 << (offset & 0x1F);
	writel(val, base + regoffsetintenb);
}

static inline void gpset_int_enable(void __iomem * base, int offset, int en)
{
	u32 val;
	u32 regoffsetintenb = GPIO_INT_ENABLE + (GET_OFFSET_WORD(offset)*sizeof(u32));
	val = readl(base + regoffsetintenb);
	val &= ~BIT(offset & 0x1F);
	val |= en << (offset & 0x1F);
	writel(val, base + regoffsetintenb);
}

static inline u32 gpget_int_enable(void __iomem * base, int offset)
{
	u32 val;
	u32 regoffsetintenb = GPIO_INT_ENABLE + (GET_OFFSET_WORD(offset)*sizeof(u32));
	val = readl(base + regoffsetintenb);
	val &= BIT(offset & 0x1F);
	val = (val >> (offset & 0x1F));
	return val ;
}


static inline u32 gpget_powermode(void __iomem * base)
{
	u32 powermode = readl(base + GPIO_POWER_DOWN);
	powermode &= BIT(31); /*31th bit of power down register describes the power mode*/
	powermode = (powermode >> 31);
	return powermode;
}

static inline void gpset_powermode(void __iomem * base, int mode)
{
	u32 powermode = readl(base + GPIO_POWER_DOWN);
	powermode &= ~(BIT(31)); /*31th bit of power down register describes the power mode*/
	powermode |= (mode&0x1) << 31;  /*TODO*/

	writel(powermode, base + GPIO_POWER_DOWN);
}


static inline void gpclear_int(void __iomem * base, int offset)
{
	writel(BIT(offset & 0x1F), base + GPIO_INT_CLEAR + (GET_OFFSET_WORD(offset)*sizeof(u32)));
}

static inline int apgpio_check_moduleid(void __iomem * base)
{
	return (readl(base + GPIO_MODULE_REG)) == GPIO_MODULE_ID;
}

/* genirq interfaces are not available to modules */
#ifdef MODULE
#define is_module()	true
#else
#define is_module()	false
#endif

static inline void activate_irq(int irq)
{
#ifdef CONFIG_ARM
	/* ARM requires an extra step to clear IRQ_NOREQUEST, which it
	 * sets on behalf of every irq_chip.  Also sets IRQ_NOPROBE.
	 */
	set_irq_flags(irq, IRQF_VALID);
#else
	/* same effect on other architectures */
	set_irq_noprobe(irq);
#endif
}

#endif /* __GPIO_STB_H */
