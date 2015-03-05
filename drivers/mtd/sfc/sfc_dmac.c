#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/io.h>


#include <linux/dmaengine.h>
#include <linux/nx_dmac_1902.h>
#include <linux/nx_dmac.h>

#define NX_DMAC_1902_DMAC_ENABLE             (0x00000001)
#define NX_DMAC_1902_DMAC_DISABLE            (0x00000000)

#define NX_DMAC_1902_REG_INT_STATUS          (0x000)
#define NX_DMAC_1902_REG_INT_TC_STATUS       (0x004)
#define NX_DMAC_1902_REG_INT_TC_CLEAR        (0x008)
#define NX_DMAC_1902_REG_INT_ERR_STATUS      (0x00C)
#define NX_DMAC_1902_REG_INT_ERR_CLEAR       (0x010)
#define NX_DMAC_1902_REG_INT_RAW_TC_STATUS   (0x014)
#define NX_DMAC_1902_REG_INT_RAW_ERR_STATUS  (0x018)
#define NX_DMAC_1902_REG_ENABLED_CHANS       (0x01C)

/* channel specific register */
#define NX_DMAC_1902_REG_CHAN_BASE(x)        (0x100+((x)*0x20))
#define NX_DMAC_1902_REG_CHAN_SRC_ADDR       (0x000)
#define NX_DMAC_1902_REG_CHAN_DST_ADDR       (0x004)
#define NX_DMAC_1902_REG_CHAN_LLI_ADDR       (0x008)
#define NX_DMAC_1902_REG_CHAN_CONTROL        (0x00C)
#define NX_DMAC_1902_REG_CHAN_CONFIG         (0x010)

#define NX_DMAC_1902_CHAN_CNTRL_DST_INCR(x)  ((x)<<27)
#define NX_DMAC_1902_CHAN_CNTRL_SRC_INCR(x)  ((x)<<26)
#define NX_DMAC_1902_CHAN_CNTRL_DST_AHB(x)   ((x)<<25)
#define NX_DMAC_1902_CHAN_CNTRL_SRC_AHB(x)   ((x)<<24)
#define NX_DMAC_1902_CHAN_CNTRL_DST_WIDTH(x) ((x)<<21)
#define NX_DMAC_1902_CHAN_CNTRL_SRC_WIDTH(x) ((x)<<18)
#define NX_DMAC_1902_CHAN_CNTRL_DST_BURST(x) ((x)<<15)
#define NX_DMAC_1902_CHAN_CNTRL_SRC_BURST(x) ((x)<<12)

#define NX_DMAC_1902_DMAC_CHAN_ENABLE        (0x00000001)
#define NX_DMAC_1902_DMAC_CHAN_DISABLE       (0x00000000)

#define NX_DMAC_1902_INT_ENABLE              (0x80000000UL)

#define NX_DMAC_1902_UNMASK_INT              (0x0000C000UL)

#define NX_DMAC_1902_REG_DMAC_CONFIG         (0x030)

#define MAX_NX_DMAC_1902_CHANNELS            (8)

static void __iomem *dma_base;

extern volatile bool sfc_dma_done, sfc_dma_error;

static unsigned int phys_bounce_read_buffer;
static void  *virt_bounce_read_buffer;

int nx_sfc_dma_init(struct platform_device *pdev)
{
   struct resource *res_reg;
   res_reg = platform_get_resource(pdev, IORESOURCE_MEM, 1);
   if (res_reg == NULL) 
   {
      printk(KERN_ERR "SFC DMAC get resource failed \n");
      return -ENXIO;
   }
   dma_base = devm_ioremap(&pdev->dev, res_reg->start, (res_reg->end-res_reg->start+1));

   virt_bounce_read_buffer = dmam_alloc_coherent(&pdev->dev, SZ_4K, &phys_bounce_read_buffer, GFP_DMA | GFP_KERNEL);
   if(!virt_bounce_read_buffer) 
   {
      printk(KERN_ERR "nx_sfc: DMA read buf alloc fail\r\n");
      return -ENOMEM;
   }

   /* disable the DMA channel */
   writel(NX_DMAC_1902_DMAC_CHAN_DISABLE, 
      dma_base+NX_DMAC_1902_REG_CHAN_BASE(NX_DMAC_SFC_DMA_CHANNEL_ID)+
      NX_DMAC_1902_REG_CHAN_CONFIG);

   /* clear the error interrupt status */
   writel((1<<NX_DMAC_SFC_DMA_CHANNEL_ID), dma_base+NX_DMAC_1902_REG_INT_ERR_CLEAR);

   /* clear the tc interrupt status */
   writel((1<<NX_DMAC_SFC_DMA_CHANNEL_ID), dma_base+NX_DMAC_1902_REG_INT_TC_CLEAR);
   return 0;
}

void nx_sfc_dma_exit(struct platform_device *pdev)
{
   dmam_free_coherent(&pdev->dev, SZ_4K, virt_bounce_read_buffer, phys_bounce_read_buffer);
}

void nx_sfc_dma_start(
   u32 src_addr, 
   u32 dst_addr, 
   u32 xfer_count, 
   bool iswrite)
{
   u32 control = 0;
   u32 config  = 0;

   sfc_dma_done = false;
   sfc_dma_error = false;

   if (src_addr%4)
   {
      printk(KERN_INFO "nx_sfc_dma_start :: src_addr not aligned\n");
   }
   if (dst_addr%4)
   {
      printk(KERN_INFO "nx_sfc_dma_start :: dst_addr not aligned\n");
   }
   control |= NX_DMAC_1902_INT_ENABLE;
   control |= NX_DMAC_1902_CHAN_CNTRL_DST_INCR(1);
   control |= NX_DMAC_1902_CHAN_CNTRL_SRC_INCR(1);
   if(iswrite)
   {
      control |= NX_DMAC_1902_CHAN_CNTRL_DST_AHB(nx_dmac_1902_ahb_master_2);
      control |= NX_DMAC_1902_CHAN_CNTRL_SRC_AHB(nx_dmac_1902_ahb_master_1);
      control |= NX_DMAC_1902_CHAN_CNTRL_DST_WIDTH(0x2);
      control |= NX_DMAC_1902_CHAN_CNTRL_SRC_WIDTH(0x2);
   }
   else
   {
      control |= NX_DMAC_1902_CHAN_CNTRL_DST_AHB(nx_dmac_1902_ahb_master_1);
      control |= NX_DMAC_1902_CHAN_CNTRL_SRC_AHB(nx_dmac_1902_ahb_master_2);
      control |= NX_DMAC_1902_CHAN_CNTRL_DST_WIDTH(0x2);
      control |= NX_DMAC_1902_CHAN_CNTRL_SRC_WIDTH(0x2);
   }
   control |= NX_DMAC_1902_CHAN_CNTRL_DST_BURST(nx_dmac_1902_burst_256);
   control |= NX_DMAC_1902_CHAN_CNTRL_SRC_BURST(nx_dmac_1902_burst_256);
   control |= ((xfer_count)/4); //32 bit xfer width

   config = NX_DMAC_1902_UNMASK_INT;

   writel(src_addr, 
      dma_base+NX_DMAC_1902_REG_CHAN_BASE(NX_DMAC_SFC_DMA_CHANNEL_ID) +
      NX_DMAC_1902_REG_CHAN_SRC_ADDR);
   writel(dst_addr, 
      dma_base+NX_DMAC_1902_REG_CHAN_BASE(NX_DMAC_SFC_DMA_CHANNEL_ID) +
      NX_DMAC_1902_REG_CHAN_DST_ADDR);
   writel(0, 
      dma_base+NX_DMAC_1902_REG_CHAN_BASE(NX_DMAC_SFC_DMA_CHANNEL_ID) +
      NX_DMAC_1902_REG_CHAN_LLI_ADDR);
   writel(control, 
      dma_base+NX_DMAC_1902_REG_CHAN_BASE(NX_DMAC_SFC_DMA_CHANNEL_ID) +
      NX_DMAC_1902_REG_CHAN_CONTROL);
   
   writel(config | NX_DMAC_1902_DMAC_CHAN_ENABLE, 
      dma_base+NX_DMAC_1902_REG_CHAN_BASE(NX_DMAC_SFC_DMA_CHANNEL_ID) +
      NX_DMAC_1902_REG_CHAN_CONFIG);
}

int nx_sfc_dma_completion_wait(void)
{
   unsigned long dma_wait_timeout = jiffies + msecs_to_jiffies(10000);

   do
   {
      if (time_after_eq(jiffies, dma_wait_timeout)) 
      {
         printk(KERN_ERR "nx_sfc_dma_completion_wait: timeout!\n");
         return -EIO;
      }
   } while (sfc_dma_done == false);

   /* disable the DMA channel */
   writel(NX_DMAC_1902_DMAC_CHAN_DISABLE, 
      dma_base+NX_DMAC_1902_REG_CHAN_BASE(NX_DMAC_SFC_DMA_CHANNEL_ID)+
      NX_DMAC_1902_REG_CHAN_CONFIG);

   /* See if there is any error from GCS DMA */
   if(sfc_dma_error)
   {
       printk(KERN_ERR "nx_nor_dma_completion_wait: Error in NOR DMA!\n");
       return -EIO;
   }
   return 0;
}

void nx_sfc_map_copy_from(void *to, unsigned long from, ssize_t len)
{
   int status;
   unsigned long xfer_count= len;
   if(xfer_count%4)
   {
      xfer_count = xfer_count + (4-(xfer_count%4));
   }
   nx_sfc_dma_start(from, phys_bounce_read_buffer, xfer_count, 0);
   status = nx_sfc_dma_completion_wait();
   if (status)
   {
      printk(KERN_ERR "nx_sfc_map_copy_from: nx_sfc_dma_completion_wait failed.\n");
      return;
   }
   memcpy(to, virt_bounce_read_buffer, len);
   return;
}

void nx_sfc_map_copy_to (unsigned long to, const void *from, ssize_t len)
{
   
   dma_addr_t dma_src;
   int status;
   u32 src = (u32)from;
   dma_src = dma_map_single(NULL, (void *)src, len, DMA_TO_DEVICE);
   nx_sfc_dma_start(dma_src, to, len, 1);
   status = nx_sfc_dma_completion_wait();
   if (status)
   {
      printk(KERN_ERR "nx_sfc_map_copy_to: nx_sfc_dma_completion_wait failed.\n");
      return;
   }
   return;
}

