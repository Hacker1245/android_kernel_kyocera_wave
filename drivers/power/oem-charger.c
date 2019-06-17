/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/module.h>
#include <linux/of.h>

int oem_chg_is_off_charge(void)
{
	static int status = -1;

	if (status == -1) {
		status = (strstr(saved_command_line, "androidboot.mode=kccharger") != NULL);
	}

	return status;
}
EXPORT_SYMBOL(oem_chg_is_off_charge);

int oem_chg_is_recoverymode(void)
{
	static int status = -1;

	if (status == -1) {
		status = (strstr(saved_command_line, "androidboot.mode=recoverymode") != NULL);
	}

	return status;
}
EXPORT_SYMBOL(oem_chg_is_recoverymode);

int oem_chg_is_fotamode(void)
{
       static int status = -1;

       if (status == -1) {
               status = (strstr(saved_command_line, "kcdroidboot.mode=f-ksg") != NULL);
       }

       return status;
}
EXPORT_SYMBOL(oem_chg_is_fotamode);
