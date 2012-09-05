#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/seqlock.h>

extern int get_idle_list(struct seq_file *m, void *v);

static int mipidle_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, get_idle_list, NULL);
}

static const struct file_operations mipidle_proc_fops = {
	.open		= mipidle_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_mipidle_init(void)
{
	proc_create("mipidle", 0, NULL, &mipidle_proc_fops);
	return 0;
}
module_init(proc_mipidle_init);
