/*
 * EHCI HCD (Host Controller Driver) for USB.
 *
 * Bus Glue for Conexant CX2450X (Nevis) SoC
 * (C) Copyright 2008 CoolStream Ltd.
 *
 * Based on "ehci-fsl.c"
 * (C) Copyright 2005 MontaVista Software
 *
 * Based on "ehci-au1xxx.c" Bus Glue for AMD Alchemy Au1xxx
 * (C) Copyright 2000-2004 David Brownell <dbrownell@users.sourceforge.net>
 *
 * This file is licenced under the GPL.
 */

#include <linux/platform_device.h>
#include <linux/version.h>
#include <asm/arch/gpio.h>
#include <asm/arch/cx2450x.h>

#define PORTSC_REG_OFS	0x00000084

/*******************************************************************************/

/*
 * configure so an HC device and id are always called with process context; 
 * sleeping is OK
 *
 */

/* usb_ehci_cx2450x_probe - initialize cx2450x-based HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and then invokes the 
 * start() method for the HCD associated with it through the hotplug entry's 
 * driver_data.
 */

int usb_ehci_cx2450x_probe(const struct hc_driver *driver, struct platform_device *pdev)
{
    int retval;
    struct usb_hcd *hcd;
    struct resource *res;
    int irq;

    res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (!res) 
    {
	dev_err(&pdev->dev, "%s: found hostcontroller with no IRQ. Check your setup!\n", pdev->dev.bus_id);
	return -ENODEV;
    }
    irq = res->start;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) 
    {
	dev_err(&pdev->dev, "%s: found hostcontroller with no register address. Check your setup!\n", pdev->dev.bus_id);
	return -ENODEV;
    }

    hcd = usb_create_hcd(driver, &pdev->dev, pdev->dev.bus_id);

    if (!hcd)
	return -ENOMEM;

    hcd->rsrc_start = res->start;
    hcd->rsrc_len = res->end - res->start + 1;

    if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) 
    {
	usb_put_hcd(hcd);
	return -EBUSY;
    }

    hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);

    if (!hcd->regs) 
    {
	printk(KERN_ERR "failed to ioremap USB driver\n");
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
	return -ENOMEM;
    }

    retval = usb_add_hcd(hcd, irq, IRQF_SHARED | IRQF_DISABLED);
    if (!retval) 
	return retval;

    iounmap(hcd->regs);
    release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
    usb_put_hcd(hcd);

    return retval;
}

/*******************************************************************************/

/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/*
 * usb_ehci_hcd_cx2450x_remove - shutdown processing for cx2450x-based HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_ehci_hcd_cx2450x_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */

void usb_ehci_cx2450x_remove(struct usb_hcd *hcd, struct platform_device *pdev)
{
    usb_remove_hcd(hcd);
    iounmap(hcd->regs);
    release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
    usb_put_hcd(hcd);
}

/*******************************************************************************/

static int ehci_cx2450x_reinit(struct ehci_hcd *ehci)
{
    volatile u32 *usb_enable = (volatile u32*) SREG_USB_ENABLE_REG;

    *usb_enable |= 0x00000703;	/* datasheet is wrong. For "active low" power control bit 6 and 7 must be leave at 0 */
    msleep(200);

    ehci_port_power(ehci, 0);

    return 0;
}

/*******************************************************************************/

static int ehci_cx2450x_setup(struct usb_hcd *hcd)
{
    struct ehci_hcd *ehci = hcd_to_ehci(hcd);
    int retval;

    /* Conexant EHCI capability registers start at 0x100 */
    ehci->caps = hcd->regs;
    ehci->regs = hcd->regs + HC_LENGTH(readl(&ehci->caps->hc_capbase));

    /* Conexant USB controller has a TT. */
    hcd->has_tt = 1;

    dbg_hcs_params(ehci, "reset");
    dbg_hcc_params(ehci, "reset");
		
    /* cache this readonly data; minimize chip reads */
    ehci->hcs_params = readl(&ehci->caps->hcs_params);

    ehci_reset(ehci);
    retval = ehci_halt(ehci);
    if (retval)
	return retval;

    /* data structure init */
    retval = ehci_init(hcd);
    if (retval)
	return retval;

#if 0
    temp = HCS_N_CC(ehci->hcs_params) * HCS_N_PCC(ehci->hcs_params);
    temp &= 0x0F;
    if (temp && HCS_N_PORTS(ehci->hcs_params) > temp) 
    {
	printk("bogus port configuration: "
	       "cc=%d x pcc=%d < ports=%d\n",
	       HCS_N_CC(ehci->hcs_params),
	       HCS_N_PCC(ehci->hcs_params),
	       HCS_N_PORTS(ehci->hcs_params));
    }
#endif

    retval = ehci_cx2450x_reinit(ehci);
    return retval;
}

/*******************************************************************************/

static const struct hc_driver ehci_cx2450x_hc_driver = 
{
    .description   = hcd_name,
    .product_desc  = "Conexant CX2450X EHCI USB-Controller",
    .hcd_priv_size = sizeof(struct ehci_hcd),

    /* generic hardware linkage */
    .irq   = ehci_irq,
    .flags = HCD_MEMORY | HCD_USB2,

    /* basic lifecycle operations */
    .reset    = ehci_cx2450x_setup,
    .start    = ehci_run,
    .stop     = ehci_stop,
    .shutdown = ehci_shutdown,

    /* managing i/o requests and associated device resources */
    .urb_enqueue      = ehci_urb_enqueue,
    .urb_dequeue      = ehci_urb_dequeue,
    .endpoint_disable = ehci_endpoint_disable,

    /* scheduling support */
    .get_frame_number = ehci_get_frame,

    /* root hub support */
    .hub_status_data = ehci_hub_status_data,
    .hub_control     = ehci_hub_control,

    .relinquish_port  = ehci_relinquish_port,
    .port_handed_over = ehci_port_handed_over,

    .bus_suspend = ehci_bus_suspend,
    .bus_resume  = ehci_bus_resume

    #ifdef	CONFIG_PM
    .hub_suspend = NULL,
    .hub_resume  = NULL,
    #endif
};

/*******************************************************************************/

static int ehci_hcd_cx2450x_drv_probe(struct platform_device *pdev)
{
    if (usb_disabled())
	return -ENODEV;

    return usb_ehci_cx2450x_probe(&ehci_cx2450x_hc_driver, pdev);
}

/*******************************************************************************/

static int ehci_hcd_cx2450x_drv_remove(struct platform_device *pdev)
{
    struct usb_hcd *hcd = platform_get_drvdata(pdev);

    usb_ehci_cx2450x_remove(hcd, pdev);
    return 0;
}

/*******************************************************************************/

MODULE_ALIAS("platform:cx2450x-ehci");

static struct platform_driver ehci_cx2450x_driver = 
{
    .probe    = ehci_hcd_cx2450x_drv_probe,
    .remove   = ehci_hcd_cx2450x_drv_remove,
    .shutdown = usb_hcd_platform_shutdown,
    .driver   = 
    {
	.name = "cx2450x-ehci",
	.bus = &platform_bus_type
    }
};

/*******************************************************************************/
