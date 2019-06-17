/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 and
 *  only version 2 as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#define pr_fmt(fmt) "DNANDDRV: " fmt

#include <linux/module.h>
#include <linux/types.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/bio.h>
#include <linux/genhd.h>
#include <linux/gfp.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/dnand_status.h>

#include "dnand_drv.h"

/* dnand always exists at first block of MMC */
static dev_t dnand_parent_dev = MKDEV(MMC_BLOCK_MAJOR, 0);
const static dev_t gpt_parent_dev = MKDEV(MMC_BLOCK_MAJOR, 0);
const static char dnand_partition_label[] = "dnand";
const static char sbl1_partition_label[] = "sbl1";
const static char aboot_partition_label[] = "aboot";
const static char rpm_partition_label[] = "rpm";
const static char tz_partition_label[] = "tz";
const static char recovery_partition_label[] = "recovery";
const static char misc_partition_label[] = "misc";

#define DNAND_DEV_INVALID (-1)
static dev_t dnand_dev_t = DNAND_DEV_INVALID;
#define MAX_READ_SECTOR_NUM (128)
#define LIMIT_READ_SECTOR_NUM (0x200)   /* 32[MB] (0x200*128*512) */
static dev_t sbl1_dev_t = DNAND_DEV_INVALID;
static uint32_t sbl1_sector_blk_num = DNAND_DEV_INVALID;
static dev_t aboot_dev_t = DNAND_DEV_INVALID;
static uint32_t aboot_sector_blk_num = DNAND_DEV_INVALID;
static dev_t rpm_dev_t = DNAND_DEV_INVALID;
static uint32_t rpm_sector_blk_num = DNAND_DEV_INVALID;
static dev_t tz_dev_t = DNAND_DEV_INVALID;
static uint32_t tz_sector_blk_num = DNAND_DEV_INVALID;
static dev_t recovery_dev_t = DNAND_DEV_INVALID;
static uint32_t recovery_sector_blk_num = DNAND_DEV_INVALID;
static dev_t misc_dev_t = DNAND_DEV_INVALID;
static uint32_t misc_sector_blk_num = DNAND_DEV_INVALID;

struct page *g_dnand_page = NULL;

#define DNAND_MAX_WAITCOUNT (30)
#define DNAND_MAX_WAIT_MS (100)

static void end_bio_dnand(struct bio *bio, int err)
{
	if (IS_ERR(&err)) {
		pr_err("bio error occured\n");
		return;
	}
	if (bio->bi_private) {
		complete((struct completion*)(bio->bi_private));
	}
	bio_put(bio);
}

int submit_dnand(struct block_device *dnand_bdev, int rw,
		 uint32_t sector, uint32_t size,
		 struct page *dnand_page)
{
	int ret = 0;
	struct bio *dnand_bio;
	DECLARE_COMPLETION_ONSTACK(waithandle);

	dnand_bio = bio_alloc(GFP_KERNEL, 1);

	if (IS_ERR(dnand_bio)) {
		pr_err("Could not alloc bio\n");
		return -ENOMEM;
	}

	/* setup bio params */
	dnand_bio->bi_sector = sector;
	dnand_bio->bi_bdev = dnand_bdev;

	if (bio_add_page(dnand_bio, dnand_page, size, 0) != size) {
		pr_err("Could not add page\n");
		ret = -ENOMEM;
		goto out;
	}

	dnand_bio->bi_vcnt = 1;
	dnand_bio->bi_idx = 0;
	dnand_bio->bi_size = size;
	dnand_bio->bi_end_io = end_bio_dnand;
	dnand_bio->bi_rw = rw;
	dnand_bio->bi_private = &waithandle;

	if (!(rw & REQ_WRITE)) {
		bio_set_pages_dirty(dnand_bio);
	}
	bio_get(dnand_bio);
	submit_bio(rw, dnand_bio);
	wait_for_completion(&waithandle);

out:
	bio_put(dnand_bio);

	return ret;
}

int32_t dnand_drv_read(uint32_t sector, uint32_t num_sector, uint8_t *pbuf)
{
	int ret = DNAND_NO_ERROR;
	struct block_device *dnand_bdev;
	struct page *dnand_page;
	int total;

	if (dnand_dev_t == DNAND_DEV_INVALID) {
		pr_err("dnand_drv not initialized\n");
		return DNAND_DEV_ERROR;
	}

	/* open block_device of dnand */
	dnand_bdev = blkdev_get_by_dev(dnand_dev_t, FMODE_READ, NULL);
	if (IS_ERR(dnand_bdev)) {
		pr_err("Could not get dnand_bdev\n");
		return DNAND_DEV_ERROR;
	}

	total = (num_sector * DNAND_DRV_SECTOR_BLK_SIZE);
	dnand_page = alloc_pages(GFP_KERNEL, get_order(total));
	if (!dnand_page) {
		pr_err("Could not alloc pages total=%d\n", total);
		ret = DNAND_DEV_ERROR;
		goto out_put;
	}

	ret = submit_dnand(dnand_bdev, 0,
			   sector, total, dnand_page);
	if (ret) {
		pr_err("Could not sumbmit dnand io @sector=%d, @num_sector=%d\n",
		       sector, num_sector);
		ret = DNAND_DEV_ERROR;
		goto out_free;
	}
	ret = DNAND_NO_ERROR;
	memcpy(pbuf, page_address(dnand_page), total);
out_free:
	__free_pages(dnand_page, get_order(total));
out_put:
	blkdev_put(dnand_bdev, FMODE_READ);
	return ret;
}

int32_t dnand_drv_write(uint32_t sector, uint32_t num_sector, uint8_t *pbuf)
{
	int ret;
	struct block_device *dnand_bdev;
	struct page *dnand_page;
	int total;

	if (dnand_dev_t == DNAND_DEV_INVALID) {
		pr_err("dnand_drv not initialized\n");
		return DNAND_DEV_ERROR;
	}

	/* open block_device of dnand */
	dnand_bdev = blkdev_get_by_dev(dnand_dev_t, FMODE_WRITE, NULL);
	if (IS_ERR(dnand_bdev)) {
		pr_err("Could not get dnand_bdev\n");
		ret = DNAND_DEV_ERROR;
		goto out_put;
	}

	/* setup page */
	total = (num_sector * DNAND_DRV_SECTOR_BLK_SIZE);
	dnand_page = alloc_pages(GFP_KERNEL, get_order(total));
	if (!dnand_page) {
		pr_err("Could not alloc pages total=%d\n", total);
		return DNAND_DEV_ERROR;
	}
	memcpy(page_address(dnand_page), pbuf, total);

	ret = submit_dnand(dnand_bdev, (REQ_WRITE | REQ_FLUSH),
			   sector, total, dnand_page);
	if (ret) {
		pr_err("Could not sumbmit dnand io @sector=%d, @num_sector=%d\n",
		       sector, num_sector);
		ret = DNAND_DEV_ERROR;
		goto out_free;
	}
	ret = DNAND_NO_ERROR;

out_free:
	__free_pages(dnand_page, get_order(total));
out_put:
	blkdev_put(dnand_bdev, FMODE_WRITE);
	return ret;
}

void dnand_drv_refresh(void)
{
	int ret;
	int loop = 0;
	struct block_device *emmc_bdev;
	int total, read_sector, total_blk;

    struct block_device *bdev;
    struct disk_part_iter piter;
    struct hd_struct *part;

    bdev = blkdev_get_by_dev(gpt_parent_dev, FMODE_READ, NULL);
    if (IS_ERR(bdev))
    {
        return;
    }
    disk_part_iter_init(&piter, bdev->bd_disk, DISK_PITER_INCL_EMPTY);
    while ((part = disk_part_iter_next(&piter)) && (loop < 39))
    {
        if (0 == strcmp(sbl1_partition_label, part->info->volname))
        {
            if ((sbl1_dev_t == DNAND_DEV_INVALID) || (sbl1_sector_blk_num != part->nr_sects))
            {
                disk_part_iter_exit(&piter);
                blkdev_put(bdev, FMODE_READ);
                pr_err("not found sbl1 partition\n");
                return;
            }
        }
        if (0 == strcmp(aboot_partition_label, part->info->volname))
        {
            if ((aboot_dev_t == DNAND_DEV_INVALID) || (aboot_sector_blk_num != part->nr_sects))
            {
                disk_part_iter_exit(&piter);
                blkdev_put(bdev, FMODE_READ);
                pr_err("not found aboot partition\n");
                return;
            }
        }
        if (0 == strcmp(rpm_partition_label, part->info->volname))
        {
            if ((rpm_dev_t == DNAND_DEV_INVALID) || (rpm_sector_blk_num != part->nr_sects))
            {
                disk_part_iter_exit(&piter);
                blkdev_put(bdev, FMODE_READ);
                pr_err("not found rpm partition\n");
                return;
            }
        }
        if (0 == strcmp(tz_partition_label, part->info->volname))
        {
            if ((tz_dev_t == DNAND_DEV_INVALID) || (tz_sector_blk_num != part->nr_sects))
            {
                disk_part_iter_exit(&piter);
                blkdev_put(bdev, FMODE_READ);
                pr_err("not found tz partition\n");
                return;
            }
        }
        if (0 == strcmp(recovery_partition_label, part->info->volname))
        {
            if ((recovery_dev_t == DNAND_DEV_INVALID) || (recovery_sector_blk_num != part->nr_sects))
            {
                disk_part_iter_exit(&piter);
                blkdev_put(bdev, FMODE_READ);
                pr_err("not found recovery partition\n");
                return;
            }
        }
        if (0 == strcmp(misc_partition_label, part->info->volname))
        {
            if ((misc_dev_t == DNAND_DEV_INVALID) || (misc_sector_blk_num != part->nr_sects))
            {
                disk_part_iter_exit(&piter);
                blkdev_put(bdev, FMODE_READ);
                pr_err("not found misc partition\n");
                return;
            }
        }
        loop++;
    }
    disk_part_iter_exit(&piter);
    blkdev_put(bdev, FMODE_READ);

	/* setup page */
	total = (MAX_READ_SECTOR_NUM * DNAND_DRV_SECTOR_BLK_SIZE);
	if (!g_dnand_page)
	{
	    pr_err("Re-challenge of memory allocation\n");
	    g_dnand_page = alloc_pages(GFP_KERNEL, get_order(total));
	    if (!g_dnand_page) {
		    pr_err("Could not alloc pages total=%d\n", total);
		    return;
	    }
	}

	pr_err("emmc refresh start\n");

	/* open block_device of dnand */
	if (sbl1_dev_t == DNAND_DEV_INVALID || sbl1_sector_blk_num == DNAND_DEV_INVALID) {
		pr_err("dnand_drv not initialized\n");
		goto out_free;
	}
	emmc_bdev = blkdev_get_by_dev(sbl1_dev_t, FMODE_READ, NULL);
	if (IS_ERR(emmc_bdev)) {
		pr_err("Could not get emmc_bdev\n");
		goto out_free;
	}
	total_blk = sbl1_sector_blk_num / MAX_READ_SECTOR_NUM;
	for (loop = 0; ((loop < total_blk) && (loop < LIMIT_READ_SECTOR_NUM)); loop++)
	{
		read_sector = loop * MAX_READ_SECTOR_NUM;
        ret = submit_dnand(emmc_bdev, 0, read_sector, total, g_dnand_page);
        if (ret) {
            pr_err("Could not sumbmit refresh io @sector=%d\n", read_sector);
            blkdev_put(emmc_bdev, FMODE_READ);
            goto out_free;
        }
	}
	blkdev_put(emmc_bdev, FMODE_READ);

	if (aboot_dev_t == DNAND_DEV_INVALID || aboot_sector_blk_num == DNAND_DEV_INVALID) {
		pr_err("dnand_drv not initialized\n");
		goto out_free;
	}
	emmc_bdev = blkdev_get_by_dev(aboot_dev_t, FMODE_READ, NULL);
	if (IS_ERR(emmc_bdev)) {
		pr_err("Could not get emmc_bdev\n");
		goto out_free;
	}
	total_blk = aboot_sector_blk_num / MAX_READ_SECTOR_NUM;
	for (loop = 0; ((loop < total_blk) && (loop < LIMIT_READ_SECTOR_NUM)); loop++)
	{
		read_sector = loop * MAX_READ_SECTOR_NUM;
        ret = submit_dnand(emmc_bdev, 0, read_sector, total, g_dnand_page);
        if (ret) {
            pr_err("Could not sumbmit refresh io @sector=%d\n", read_sector);
            blkdev_put(emmc_bdev, FMODE_READ);
            goto out_free;
        }
	}
	blkdev_put(emmc_bdev, FMODE_READ);

	if (rpm_dev_t == DNAND_DEV_INVALID || rpm_sector_blk_num == DNAND_DEV_INVALID) {
		pr_err("dnand_drv not initialized\n");
		goto out_free;
	}
	emmc_bdev = blkdev_get_by_dev(rpm_dev_t, FMODE_READ, NULL);
	if (IS_ERR(emmc_bdev)) {
		pr_err("Could not get emmc_bdev\n");
		goto out_free;
	}
	total_blk = rpm_sector_blk_num / MAX_READ_SECTOR_NUM;
	for (loop = 0; ((loop < total_blk) && (loop < LIMIT_READ_SECTOR_NUM)); loop++)
	{
		read_sector = loop * MAX_READ_SECTOR_NUM;
        ret = submit_dnand(emmc_bdev, 0, read_sector, total, g_dnand_page);
        if (ret) {
            pr_err("Could not sumbmit refresh io @sector=%d\n", read_sector);
            blkdev_put(emmc_bdev, FMODE_READ);
            goto out_free;
        }
	}
	blkdev_put(emmc_bdev, FMODE_READ);

	if (tz_dev_t == DNAND_DEV_INVALID || tz_sector_blk_num == DNAND_DEV_INVALID) {
		pr_err("dnand_drv not initialized\n");
		goto out_free;
	}
	emmc_bdev = blkdev_get_by_dev(tz_dev_t, FMODE_READ, NULL);
	if (IS_ERR(emmc_bdev)) {
		pr_err("Could not get emmc_bdev\n");
		goto out_free;
	}
	total_blk = tz_sector_blk_num / MAX_READ_SECTOR_NUM;
	for (loop = 0; ((loop < total_blk) && (loop < LIMIT_READ_SECTOR_NUM)); loop++)
	{
		read_sector = loop * MAX_READ_SECTOR_NUM;
        ret = submit_dnand(emmc_bdev, 0, read_sector, total, g_dnand_page);
        if (ret) {
            pr_err("Could not sumbmit refresh io @sector=%d\n", read_sector);
            blkdev_put(emmc_bdev, FMODE_READ);
            goto out_free;
        }
	}
	blkdev_put(emmc_bdev, FMODE_READ);

	if (recovery_dev_t == DNAND_DEV_INVALID || recovery_sector_blk_num == DNAND_DEV_INVALID) {
		pr_err("dnand_drv not initialized\n");
		goto out_free;
	}
	emmc_bdev = blkdev_get_by_dev(recovery_dev_t, FMODE_READ, NULL);
	if (IS_ERR(emmc_bdev)) {
		pr_err("Could not get emmc_bdev\n");
		goto out_free;
	}
	total_blk = recovery_sector_blk_num / MAX_READ_SECTOR_NUM;
	for (loop = 0; ((loop < total_blk) && (loop < LIMIT_READ_SECTOR_NUM)); loop++)
	{
		read_sector = loop * MAX_READ_SECTOR_NUM;
        ret = submit_dnand(emmc_bdev, 0, read_sector, total, g_dnand_page);
        if (ret) {
            pr_err("Could not sumbmit refresh io @sector=%d\n", read_sector);
            blkdev_put(emmc_bdev, FMODE_READ);
            goto out_free;
        }
	}
	blkdev_put(emmc_bdev, FMODE_READ);

	if (misc_dev_t == DNAND_DEV_INVALID || misc_sector_blk_num == DNAND_DEV_INVALID) {
		pr_err("dnand_drv not initialized\n");
		goto out_free;
	}
	emmc_bdev = blkdev_get_by_dev(misc_dev_t, FMODE_READ, NULL);
	if (IS_ERR(emmc_bdev)) {
		pr_err("Could not get emmc_bdev\n");
		goto out_free;
	}
	total_blk = misc_sector_blk_num / MAX_READ_SECTOR_NUM;
	for (loop = 0; ((loop < total_blk) && (loop < LIMIT_READ_SECTOR_NUM)); loop++)
	{
		read_sector = loop * MAX_READ_SECTOR_NUM;
        ret = submit_dnand(emmc_bdev, 0, read_sector, total, g_dnand_page);
        if (ret) {
            pr_err("Could not sumbmit refresh io @sector=%d\n", read_sector);
            blkdev_put(emmc_bdev, FMODE_READ);
            goto out_free;
        }
	}
	blkdev_put(emmc_bdev, FMODE_READ);

	pr_err("emmc refresh finish\n");

out_free:
	return;
}

int __init dnand_drv_init(void)
{
	int ret = DNAND_DEV_ERROR;
	struct block_device *bdev;
	struct disk_part_iter piter;
	struct hd_struct *part;
	int count;

	do {
		/* Although we configured dnand as late_initcall(),
		 * block device would not appear in some situation.
		 * We wait for it a bit here.
		 */
		bdev = blkdev_get_by_dev(dnand_parent_dev, FMODE_READ, NULL);
		if (!IS_ERR(bdev))
			break;
		if (count < DNAND_MAX_WAITCOUNT) {
			pr_warn("waiting for bdev initialized... count=%d\n", count);
			count++;
			msleep(DNAND_MAX_WAIT_MS);
		} else {
			/* give up */
			pr_err("Could not get bdev\n");
			ret = DNAND_DEV_ERROR;
			return ret;
		}
	} while (1);

	/* search partition for dnand */
	disk_part_iter_init(&piter, bdev->bd_disk, DISK_PITER_INCL_EMPTY);
	while ((part = disk_part_iter_next(&piter))) {
		if (0 == strcmp(sbl1_partition_label, part->info->volname)) {
			sbl1_dev_t = part_devt(part);
			sbl1_sector_blk_num = part->nr_sects;
			pr_info("found sbl1 partition, partno=%d, major=%d, minor=%d, start_sect=%llu, nr_sects=%llu\n",
				part->partno, MAJOR(sbl1_dev_t), MINOR(sbl1_dev_t), part->start_sect, part->nr_sects);
		}

		if (0 == strcmp(aboot_partition_label, part->info->volname)) {
			aboot_dev_t = part_devt(part);
			aboot_sector_blk_num = part->nr_sects;
			pr_info("found aboot partition, partno=%d, major=%d, minor=%d, start_sect=%llu, nr_sects=%llu\n",
				part->partno, MAJOR(aboot_dev_t), MINOR(aboot_dev_t), part->start_sect, part->nr_sects);
		}

		if (0 == strcmp(rpm_partition_label, part->info->volname)) {
			rpm_dev_t = part_devt(part);
			rpm_sector_blk_num = part->nr_sects;
			pr_info("found rpm partition, partno=%d, major=%d, minor=%d, start_sect=%llu, nr_sects=%llu\n",
				part->partno, MAJOR(rpm_dev_t), MINOR(rpm_dev_t), part->start_sect, part->nr_sects);
		}

		if (0 == strcmp(tz_partition_label, part->info->volname)) {
			tz_dev_t = part_devt(part);
			tz_sector_blk_num = part->nr_sects;
			pr_info("found tz partition, partno=%d, major=%d, minor=%d, start_sect=%llu, nr_sects=%llu\n",
				part->partno, MAJOR(tz_dev_t), MINOR(tz_dev_t), part->start_sect, part->nr_sects);
		}

		if (0 == strcmp(recovery_partition_label, part->info->volname)) {
			recovery_dev_t = part_devt(part);
			recovery_sector_blk_num = part->nr_sects;
			pr_info("found recovery partition, partno=%d, major=%d, minor=%d, start_sect=%llu, nr_sects=%llu\n",
				part->partno, MAJOR(recovery_dev_t), MINOR(recovery_dev_t), part->start_sect, part->nr_sects);
		}

		if (0 == strcmp(misc_partition_label, part->info->volname)) {
			misc_dev_t = part_devt(part);
			misc_sector_blk_num = part->nr_sects;
			pr_info("found misc partition, partno=%d, major=%d, minor=%d, start_sect=%llu, nr_sects=%llu\n",
				part->partno, MAJOR(misc_dev_t), MINOR(misc_dev_t), part->start_sect, part->nr_sects);
		}

		if (0 == strcmp(dnand_partition_label,
				part->info->volname)) {
			dnand_dev_t = part_devt(part);
			pr_info("found dnand partition, partno=%d, major=%d, minor=%d, start_sect=%llu, nr_sects=%llu\n",
				part->partno, MAJOR(dnand_dev_t), MINOR(dnand_dev_t), part->start_sect, part->nr_sects);
			ret = DNAND_NO_ERROR;
			break;
		}
	}
	disk_part_iter_exit(&piter);

	blkdev_put(bdev, FMODE_READ);

	/* setup page */
	g_dnand_page = alloc_pages(GFP_KERNEL, get_order(MAX_READ_SECTOR_NUM*DNAND_DRV_SECTOR_BLK_SIZE));
	if (!g_dnand_page) {
		pr_err("Could not alloc pages total=64[KB]\n");
	}

	return ret;
}
