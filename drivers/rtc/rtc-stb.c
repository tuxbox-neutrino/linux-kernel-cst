/*
 *  linux/drivers/rtc/rtc_stb.c
 *
 *  Copyright 2010 Trident Microsystems (Far East) Ltd. All rights reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#define TIMER_VALUE		(0x00)
#define TIMER_LIMIT		(0x04)
#define TIMER_MODE		(0x08)
#define TIMER_TIMEBASE		(0x0C)
#define TIMER_IRQ		(0x200)

struct stb_rtc_device {
	struct platform_device	*pdev;
	struct rtc_device	*rtc;
	void __iomem		*base;
	int			irq;
	u32			index;
	spinlock_t		lock;
	u32			old_timer_mode;
};

static int rtc_stb_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct stb_rtc_device *rtc = dev_get_drvdata(dev);
	int ret = 0;
	u32 read_reg;

	switch (cmd) {
	case RTC_UIE_ON:
		read_reg = readl(rtc->base + TIMER_MODE);
		writel(read_reg|0x8, rtc->base + TIMER_MODE);
		break;

	case RTC_UIE_OFF:
		read_reg = readl(rtc->base + TIMER_MODE);
		writel(read_reg&(~0x8), rtc->base + TIMER_MODE);
		break;

	default:
		 ret = -ENOIOCTLCMD;
		 break;
	}

	return ret;
}

static irqreturn_t rtc_stb_interrupt(int irq, void *dev_id)
{
	struct device *dev = dev_id;
	struct stb_rtc_device *rtc = dev_get_drvdata(dev);
	unsigned long events = RTC_IRQF | RTC_UF;
	u32 read_reg;

	/* Clear interrupt */
	read_reg = readl(rtc->base + TIMER_MODE);
	writel(read_reg, rtc->base + TIMER_MODE);

	rtc_update_irq(rtc->rtc, 1, events);

	return IRQ_HANDLED;
}

#if 0
static int rtc_stb_irq_enable(struct device *dev, unsigned int enabled)
{
	struct stb_rtc_device *rtc = dev_get_drvdata(dev);
	unsigned long flags;
	u32 read_reg;

	spin_lock_irqsave(&rtc->lock, flags);

	/* Clear interrupt */
	read_reg = readl(rtc->base + TIMER_MODE);
	if (enabled)
		read_reg |= (1 << 3);
	else
		read_reg &= ~(1 << 3);
	writel(read_reg, rtc->base + TIMER_MODE);

	spin_unlock_irqrestore(&rtc->lock, flags);

	return 0;
}
#endif

static int rtc_stb_read_time(struct device *dev, struct rtc_time *tm)
{
	struct stb_rtc_device *rtc = dev_get_drvdata(dev);

	rtc_time_to_tm(readl(rtc->base + TIMER_VALUE), tm);

	pr_debug("STB RTC Read Time done\n");

	return 0;
}

/*
 * Set the RTC time.  Unfortunately, we can't accurately set
 * the point at which the counter updates.
 *
 */
static int rtc_stb_set_time(struct device *dev, struct rtc_time *tm)
{
	struct stb_rtc_device *rtc = dev_get_drvdata(dev);
	unsigned long time;
	int ret;
	u32 read_reg;

	/* convert tm to seconds. */
	ret = rtc_valid_tm(tm);
	if (ret)
		return ret;

	/* Timer Counter Value Reg can be written only when counter is Enabled, so enable it*/
	read_reg = readl(rtc->base + TIMER_MODE);
	writel(read_reg | 0x01, rtc->base + TIMER_MODE);

	ret = rtc_tm_to_time(tm, &time);
	if (ret == 0)
		writel(time, rtc->base + TIMER_VALUE);

	pr_debug("STB RTC Set Time done\n");

	return ret;
}

static const struct rtc_class_ops rtc_stb_ops = {
	.ioctl		= rtc_stb_ioctl,
	.read_time	= rtc_stb_read_time,
	.set_time	= rtc_stb_set_time,
};

static const struct of_device_id rtc_stb_dt_match[] = {
	{ .compatible = "entr,stb-rtc", },
	{}
};
MODULE_DEVICE_TABLE(of, rtc_stb_dt_match);

static int rtc_stb_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct stb_rtc_device *rtc;
	int ret;

	rtc = devm_kzalloc(&pdev->dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rtc->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rtc->base))
		return PTR_ERR(rtc->base);

	rtc->irq = platform_get_irq(pdev, 0);
	if (rtc->irq <= 0)
		return -EBUSY;

	rtc->index = rtc->irq - IRQ_RTC0;

	rtc->rtc = devm_rtc_device_register(&pdev->dev,
					    dev_name(&pdev->dev),
					    &rtc_stb_ops, THIS_MODULE);
	if (IS_ERR(rtc->rtc)) {
		ret = PTR_ERR(rtc->rtc);
		dev_err(&pdev->dev, "Unable to register RTC device (err: %d)\n",
			ret);
		return ret;
	}

	/* set context info. */
	rtc->pdev = pdev;
	spin_lock_init(&rtc->lock);

	platform_set_drvdata(pdev, rtc);

	device_init_wakeup(&pdev->dev, 1);

	/* Set the Mode to 0, Limit to Max and Time base to 50Mhz to get 1sec resolution */
	writel(0, rtc->base + TIMER_MODE);
	writel(0xFFFFFFFF, rtc->base + TIMER_LIMIT);
	writel(0x2FAF080, rtc->base + TIMER_TIMEBASE);

	ret = devm_request_irq(&pdev->dev,
			       rtc->irq, rtc_stb_interrupt, IRQF_TRIGGER_HIGH,
			       dev_name(&pdev->dev), &pdev->dev);
	if (ret) {
		dev_err(&pdev->dev,
			"Unable to request interrupt for device (err=%d).\n",
			ret);
		return ret;
	}

	/* Enable the timer here */
	writel(1, rtc->base + TIMER_MODE);

	pr_debug("STB RTC Driver Probe complete\n");

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int rtc_stb_suspend(struct device *dev)
{
	struct stb_rtc_device *rtc = dev_get_drvdata(dev);

	rtc->old_timer_mode = readl(rtc->base + TIMER_MODE);
	writel(0, rtc->base + TIMER_MODE);

	/* leave the alarms on as a wake source. */
	if (device_may_wakeup(dev))
		enable_irq_wake(rtc->irq);

	return 0;
}

static int rtc_stb_resume(struct device *dev)
{
	struct stb_rtc_device *rtc = dev_get_drvdata(dev);

	writel(rtc->old_timer_mode, rtc->base + TIMER_MODE);

	/* alarms were left on as a wake source, turn them off. */
	if (device_may_wakeup(dev))
		disable_irq_wake(rtc->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(rtc_stb_pm_ops, rtc_stb_suspend, rtc_stb_resume);

static void rtc_stb_shutdown(struct platform_device *pdev)
{
	struct stb_rtc_device *rtc = dev_get_drvdata(&pdev->dev);
	u32 read_reg;

	dev_vdbg(&pdev->dev, "disabling interrupts.\n");

	read_reg = readl(rtc->base + TIMER_MODE);
	writel(read_reg | 0x08, rtc->base + TIMER_MODE);
}

MODULE_ALIAS("platform:rtc_stb");
static struct platform_driver rtc_stb_driver = {
	.shutdown	= rtc_stb_shutdown,
	.driver		= {
		.name	= "rtc_stb",
		.owner	= THIS_MODULE,
		.of_match_table = rtc_stb_dt_match,
		.pm	= &rtc_stb_pm_ops,
	},
};

module_platform_driver_probe(rtc_stb_driver, rtc_stb_probe)

MODULE_AUTHOR("Nitin Garg <nitin.garg@tridentmicro.com>");
MODULE_DESCRIPTION("STB RTC Driver");
MODULE_LICENSE("GPL");
