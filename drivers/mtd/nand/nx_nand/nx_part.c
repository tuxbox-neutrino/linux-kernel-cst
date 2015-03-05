/*
 * MTD style partition parser for BFFS tags
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/vmalloc.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/proc_fs.h>

#define BFFS_NAME_LEN		(32)
#define UBTM_START		(0x00000000)
#define	OFFSET_TDF_KEY		(0x0200)
#define	OFFSET_PARTITION_TABLE	(0x0400)
#define PUBLIC_KEY_LEN          (512)
#define	PART_INFO_NUM_PAGES	(2)
#define	BL_NUM_BLKS		(3)

static uint8_t  tdf_key[PUBLIC_KEY_LEN];

#define ATTRIBUTE_UNUSED __attribute__ ((__unused__))

/*
 * Make TDF public key available using the /proc filesystem
 */
static int read_tdf_key(char *buf, char **start, off_t fpos, int length, int *eof, void *data)
{
    printk("proc length = %d\n", length);

    memcpy(buf, tdf_key, PUBLIC_KEY_LEN);
    
    *eof = 1;
    
    return PUBLIC_KEY_LEN;
}

/*
 * Read ProcFS function
 */
void *nx_nand_read_key(void)
{
    return (void *)tdf_key;
}
EXPORT_SYMBOL(nx_nand_read_key);

/*
 * Parse BFFS paritition tables
 */
static int nx_part_parse(struct mtd_info *master,
				 struct mtd_partition **pparts,
				 unsigned long data ATTRIBUTE_UNUSED)
{
	struct mtd_partition *parts;
	struct mtd_oob_ops ops;
	int ret, blk_num = 0, blk_cnt=0, numparts = 0, realparts = 0;
	loff_t blk_offset;
	uint8_t *buf, *p8, i;
	char *names;
	struct proc_dir_entry *proc_sign;
	uint32_t ubtm_size, pages_per_blk;

	/* Part info is in 2 pages */	
	pages_per_blk = master->erasesize / master->writesize;
	ubtm_size = (PART_INFO_NUM_PAGES * master->writesize);	
	buf = vmalloc(ubtm_size);
	if (!buf) {
		printk(KERN_ERR "nx_part: mem alloc failure \r\n");
		return -ENOMEM;
	}	

	/* Find last block of bootloader */
	while(blk_num < BL_NUM_BLKS) {
		blk_offset = UBTM_START + (blk_cnt * master->erasesize);		
		ret = master->block_isbad(master, blk_offset);
		if(!ret) {
			blk_num++;
		}
		blk_cnt++;
	}

	/* Read from last 2 pages of last block */	
	blk_offset += ((pages_per_blk - PART_INFO_NUM_PAGES) * master->writesize);

	/* Read data */
	ops.mode = MTD_OOB_RAW;
	ops.datbuf = buf;
	ops.oobbuf = NULL;
	ops.len = ubtm_size;
	ret = master->read_oob(master, blk_offset, &ops);
	if (ret) {
		printk(KERN_ERR "nx_part: Read data failed 0x%x \r\n", ret);
		goto out;
	}

	if (ops.retlen != ubtm_size) {
		printk(KERN_ERR "nx_part: Return len wrong! 0x%x \r\n", ops.retlen);
		ret = -EIO;
		goto out;
	}

	ret = -EINVAL;

	/* Get a reference to the partition content */
	p8 = buf + OFFSET_PARTITION_TABLE;

	/* First count the number of partitions to create */
	for (i = 0; i < 20; i++) {
		uint32_t x;

		/* offset info */
		x  = *p8 * 16777216; p8++;
		x += *p8 * 65536;    p8++;
		x += *p8 * 256;      p8++;
		x += *p8;            p8++;

		/* Check if end of parts info */
		if (x == 0xFFFFFFFF)
		{
			break;
		}

		/* Check if offset is valid */
		if(x >= master->size) {
			printk(KERN_WARNING "nx_part:offset corrupted! 0x%x \r\n", x);
			break;
		}
		
		/* Size  info */
		x  = *p8 * 16777216; p8++;
		x += *p8 * 65536;    p8++;
		x += *p8 * 256;      p8++;
		x += *p8;            p8++;
		
		/* Check if size is valid */
		if(x >= master->size) {
			printk(KERN_WARNING "nx_part:size corrupted! 0x%x \r\n", x);
			break;
		}

		numparts++;

		//p8 += 4;
		p8 += 4;
		p8++;
		p8 += 4;

		printk("Found: %s\n", (char *)p8);

		p8 += 12;
	}

	realparts = numparts;
	printk("Found %d partitions\n", numparts);

	/*
	* Since the TVBL binary is read copy the TDF signature as well 
	*/
	memcpy(tdf_key, buf + OFFSET_TDF_KEY, PUBLIC_KEY_LEN);

	/* 
	* Allow the signature to be read from the outside through the /proc fs
	*/
	proc_sign = create_proc_read_entry("public_key", 0, NULL, read_tdf_key, NULL);

	/* Add one additionally for the uBTM partition */
	//realparts++;
	parts = kzalloc((BFFS_NAME_LEN + sizeof(*parts)) * realparts,
		GFP_KERNEL);
	if (!parts) {
		ret = -ENOMEM;
		goto out;
	}

	names = (char *) &parts[realparts];

	/* Create mtd1 (ubtm device) */
	//parts[0].size = blk_cnt * master->erasesize;
	//parts[0].offset = UBTM_START;
	//parts[0].name = names;
	//strcpy(names, "u-boot");
	//names += BFFS_NAME_LEN;

	numparts = 0;

	/* Lets start from the beginning again and walk the walk */
	p8 = buf + OFFSET_PARTITION_TABLE;

	/* Walk the table and fill in the mtd parts */
	while (numparts < realparts) {
		uint32_t x;

		/* Offset */
		x  = *p8 * 16777216; p8++;
  	x += *p8 * 65536;    p8++;
  	x += *p8 * 256;      p8++;
  	x += *p8;            p8++;
		parts[numparts].offset = x;

		/* Size */
		x  = *p8 * 16777216; p8++;
		x += *p8 * 65536;    p8++;
		x += *p8 * 256;      p8++;
		x += *p8;            p8++;
		parts[numparts].size = x;

		/* Skip info from partition table not needed by the kernel */
  	p8 += 4;

		/* R/W flag */
		if (*p8 == 0)
			parts[numparts].mask_flags = MTD_WRITEABLE;

		/* Name */
		p8++;
		p8 += 4;
		parts[numparts].name = names;
		strncpy(names, p8, 12 - 1);

		/* Next part info start */
		p8 += 12;

		names += BFFS_NAME_LEN - 1;
		*names++ = 0x0;

		numparts++;
	}

	ret = realparts;
	*pparts = parts;

out:
	vfree(buf);
	return ret;
}

/**
* Partition parser for BFFS
*/
static struct mtd_part_parser nx_parser = {
	.owner = THIS_MODULE,
	.parse_fn = nx_part_parse,
	.name = "nxpart",
};

static int __init nx_parse_init(void)
{
	return register_mtd_parser(&nx_parser);
}

static void __exit nx_parse_exit(void)
{
	deregister_mtd_parser(&nx_parser);
}

module_init(nx_parse_init);
module_exit(nx_parse_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Thomas Gleixner");
MODULE_DESCRIPTION("Parsing code for BFFS tables");
