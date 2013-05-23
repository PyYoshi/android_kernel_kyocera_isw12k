/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 */
/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include <linux/moduleparam.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/hdreg.h>
#include <linux/kdev_t.h>
#include <linux/miscdevice.h>
#include <linux/blkdev.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/string_helpers.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <linux/kc_fota_ui_driver.h>
#include "../mmc/card/queue.h"


#define WR_SZ 512
#define MAX_WR_SZ (1024*128)

struct mmc_blk_data {
	spinlock_t	lock;
	struct gendisk	*disk;
	struct mmc_queue queue;
	unsigned int	usage;
	unsigned int	read_only;
};

struct mmc_blk_data *g_mmcblk_ptr = NULL;

void kc_fota_set_blkptr(struct mmc_blk_data *in_ptr)
{
	if( g_mmcblk_ptr == NULL )
	{
		g_mmcblk_ptr = in_ptr;
	}
}
EXPORT_SYMBOL(kc_fota_set_blkptr);


static u32 kc_get_card_status(struct mmc_card *card)
{
	struct mmc_command cmd;
	int err;

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = MMC_SEND_STATUS;
	if (!mmc_host_is_spi(card->host))
		cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;
	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err)
		printk(KERN_ERR "fota_ui_emmc:error %d sending status comand", err);
	return cmd.resp[0];
}


static u32 kc_fota_mmc_sd_num_wr_blocks(struct mmc_card *card, unsigned long addr, int mode, unsigned long size, unsigned char *buff)
{
	u32 result= 0;
	struct mmc_request mrq;
	struct mmc_command cmd;
	struct mmc_command stop;
	
	struct mmc_data data;
	struct scatterlist sg;
	u32 status = 0;

	mmc_claim_host(card->host);
	
	memset(&mrq,  0, sizeof(struct mmc_request));
	memset(&cmd,  0, sizeof(struct mmc_command));
	memset(&stop, 0, sizeof(struct mmc_command));
	memset(&data, 0, sizeof(struct mmc_data));

	mrq.cmd = &cmd;
	mrq.data = &data;
	mrq.stop = &stop;
	stop.opcode = MMC_STOP_TRANSMISSION;
	stop.arg = 0;
	stop.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
	
	cmd.arg = addr;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = WR_SZ;
	data.blocks = size;

	if( data.blocks > 1 )
	{
		switch( mode )
		{
		case IOCTL_EMMC_FOTA_READ_CMD:
			cmd.opcode = MMC_READ_MULTIPLE_BLOCK;
			data.flags = MMC_DATA_READ;
			break;

		case IOCTL_EMMC_FOTA_WRITE_CMD:
			cmd.opcode = MMC_WRITE_MULTIPLE_BLOCK;
			data.flags = MMC_DATA_WRITE;
			break;

		default:
			printk(KERN_ERR "fota_emmc: error kc_fota_mmc_sd_num_wr_blocks");
			break;
		}
	}
	else
	{
		switch( mode )
		{
		case IOCTL_EMMC_FOTA_READ_CMD:
			cmd.opcode = MMC_READ_SINGLE_BLOCK;
			data.flags = MMC_DATA_READ;
			mrq.stop = NULL;
		break;
			case IOCTL_EMMC_FOTA_WRITE_CMD:
			cmd.opcode = MMC_WRITE_BLOCK;
			data.flags = MMC_DATA_WRITE;
			mrq.stop = NULL;
		break;

		default:
			printk(KERN_ERR "fota_emmc: error kc_fota_mmc_sd_num_wr_blocks");
			break;
		}

	}

	mmc_set_data_timeout(mrq.data, card);
	
	data.sg = &sg;
	sg_init_one( &sg,  buff,  size*WR_SZ );
	data.sg_len = 1;

	mmc_wait_for_req(card->host, &mrq);

	if (cmd.error || data.error || stop.error)
	{
		status = kc_get_card_status(card);

		if (cmd.error) {
			printk(KERN_ERR "fota_emmc: error %d sending read/write "
				   "command, response %#x, card status %#x\n",
				   cmd.error, cmd.resp[0], status);
		}

		if (data.error) {
			if ( data.error == -ETIMEDOUT &&  mrq.stop)
				status = mrq.stop->resp[0];
		}

		if (stop.error) {
			printk(KERN_ERR "fota_emmc: error %d sending stop command, "
				   "response %#x, card status %#x\n",
					stop.error,
					stop.resp[0], status);
		}
		result = -1;
		
	}

	if (!mmc_host_is_spi(card->host) && data.flags !=  MMC_DATA_READ )
	{
		do {
			int err;

			cmd.opcode = MMC_SEND_STATUS;
			cmd.arg = card->rca << 16;
			cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
			err = mmc_wait_for_cmd(card->host, &cmd, 5);
			if (err) {
				result = -1;
				printk(KERN_ERR "fota_emmc: error %d requesting status\n",err);
				break;
			}
		} while (!(cmd.resp[0] & R1_READY_FOR_DATA) || 
			(R1_CURRENT_STATE(cmd.resp[0]) == 7));
	}

	mmc_release_host(card->host);

	return result;
}


int kc_fota_ioctl(unsigned int cmd, unsigned long addr, unsigned long size, unsigned char *buff)
{
	struct mmc_blk_data *md;
	int sec_addr;
	int sec_size;

	sec_addr = addr / WR_SZ;
	sec_size = size / WR_SZ;

	if( g_mmcblk_ptr )
	{
		md = g_mmcblk_ptr;
		return kc_fota_mmc_sd_num_wr_blocks( md->queue.card, sec_addr, (int)cmd, sec_size, buff );
	}else{
		return -1;
	}
}
EXPORT_SYMBOL(kc_fota_ioctl);



/*static int fota_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)*/
static long fota_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long result = -EFAULT;
	int  ret = -EFAULT;
	struct emmc_fota_ioctl_info info;
	unsigned char *mid_buff = NULL;
	unsigned int the_phy_sec_num = 0;
	unsigned int the_phy_sec_sz = 0;

	memset( &info, 0x00, sizeof(struct emmc_fota_ioctl_info) );
	result = copy_from_user(&info, (struct emmc_fota_ioctl_info __user *)arg, sizeof(struct emmc_fota_ioctl_info) );

	result = info.size % WR_SZ;
	if( result )
	{
		printk(KERN_ERR "%s sz error\n", __func__);
		goto fail1;
	}

	if(info.addr)
	{
		result = info.addr % WR_SZ;
		if( result )
		{
			printk(KERN_ERR "%s ad error\n", __func__);
			goto fail1;
		}
	}

	if( MAX_WR_SZ < info.size )
				{
		printk(KERN_ERR "%s max sz error\n", __func__);
		goto fail1;
		}

	switch(cmd)
	{
	case IOCTL_EMMC_FOTA_READ_CMD:
	case IOCTL_EMMC_FOTA_WRITE_CMD:
		mid_buff = (char*)kmalloc(64*2048, GFP_KERNEL);
		if (!mid_buff)
		{
			printk(KERN_ERR "fota_workbuf : Out of memory\n");
			info.size = 0;
			info.result = -1;
		}else{
			if(((info.addr & (BOARD_EMMC_BLOCK_SIZE-1)) + info.size ) > BOARD_EMMC_BLOCK_SIZE)
			{
				printk(KERN_ERR "Fota_Block_over_write\n");
				info.size = 0;
				info.result = -1;

			}else{
				the_phy_sec_num = info.addr;
				the_phy_sec_sz	 = info.size;
				if( cmd == IOCTL_EMMC_FOTA_WRITE_CMD )
				{
					result = copy_from_user(mid_buff, info.buff, info.size); 
					info.result = kc_fota_ioctl( IOCTL_EMMC_FOTA_WRITE_CMD, the_phy_sec_num, the_phy_sec_sz, mid_buff );
				}else{
					info.result = kc_fota_ioctl( IOCTL_EMMC_FOTA_READ_CMD, the_phy_sec_num, the_phy_sec_sz, mid_buff );
					result = copy_to_user(info.buff, mid_buff, info.size);
				}
			}
			kfree(mid_buff);
		}
		break;

	default:
			info.result = -1;
		break;
	}

	result = copy_to_user((struct emmc_fota_ioctl_info __user *)arg, &info, sizeof(struct emmc_fota_ioctl_info) );
	if(info.result)
	{
		ret = -EFAULT;
		info.size = 0;
	}else{
		ret = 0;
	}

	return ret;

fail1:
	info.result = -1;
	result = copy_to_user( (struct emmc_fota_ioctl_info __user *)arg, &info, sizeof(struct emmc_fota_ioctl_info) );
	ret = -EFAULT;
	return ret;

}


static int fota_open(
			struct inode		*inode,
			struct file 		*file )
{
	printk(KERN_INFO "fota_driver_open\n");
	return 0;
}


static const struct file_operations fota_fops = {
	.owner = THIS_MODULE,
	.open = fota_open,
	/*.ioctl = fota_ioctl,*/
    .unlocked_ioctl = fota_ioctl,
};


static struct miscdevice kc_fota_device = {
	.fops = &fota_fops,
	.name = "fota_fl",
	.minor = MISC_DYNAMIC_MINOR,
};


static int __init kc_fota_driver_init(void)
{
	int ret;
	ret = misc_register(&kc_fota_device);
	return ret;
}


static void __exit kc_fota_driver_exit(void)
{
	misc_deregister(&kc_fota_device);
}

module_init(kc_fota_driver_init);
module_exit(kc_fota_driver_exit);
MODULE_AUTHOR("KYOCERA");
MODULE_DESCRIPTION("MMC MISC Driver");
MODULE_LICENSE("GPL v2");
