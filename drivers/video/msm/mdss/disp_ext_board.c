/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kc_led.h>

#include "mdss_dsi.h"
#include "disp_ext.h"

static int panel_detection = 0;       /* -1:not panel 0:Not test 1:panel found */

static int disp_ext_pinctrl_set_state(
	struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	bool active)
{
	struct pinctrl_state *pin_state;
	int rc = -EFAULT;

	if (IS_ERR_OR_NULL(ctrl_pdata->pin_res.pinctrl))
		return PTR_ERR(ctrl_pdata->pin_res.pinctrl);

	pin_state = active ? ctrl_pdata->pin_res.gpio_state_det_active1
				: ctrl_pdata->pin_res.gpio_state_det_active2;

	if (!IS_ERR_OR_NULL(pin_state)) {
		rc = pinctrl_select_state(ctrl_pdata->pin_res.pinctrl,
				pin_state);
		if (rc)
			pr_err("%s: can not set %s pins\n", __func__,
			       active ? MDSS_PINCTRL_STATE_DET_ACTIVE1
			       : MDSS_PINCTRL_STATE_DET_ACTIVE2);
	} else {
		pr_err("%s: invalid '%s' pinstate\n", __func__,
		       active ? MDSS_PINCTRL_STATE_DET_ACTIVE1
		       : MDSS_PINCTRL_STATE_DET_ACTIVE2);
	}
	return rc;
}

int lcd_det_check( struct mdss_dsi_ctrl_pdata *ctrl )
{
	int disp_det_gpio;
	int det_check = 0;

	disp_det_gpio = ctrl->lcd_det_gpio;
	disp_ext_pinctrl_set_state(ctrl, 1);
	det_check = gpio_get_value(disp_det_gpio);

	if(det_check==0){
		det_check=1;
	}else{
		det_check=0;
	}
	disp_ext_pinctrl_set_state(ctrl, 0);

	return det_check;
}

int disp_ext_board_detect_board(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int det_check = 0;
	int panel_id = 0;

	if( panel_detection != 0 ) {
		pr_debug("%s: panel Checked\n", __func__);
		return panel_detection;
	}

	det_check = lcd_det_check(ctrl);
#ifdef CONFIG_DISP_EXT_PANEL_ID
	panel_id = disp_ext_get_panel_id();
#endif /* CONFIG_DISP_EXT_PANEL_ID */
	if (det_check != 1) {
		pr_err("%s: panel not found\n", __func__);
		light_led_disp_set_panel(LIGHT_MAIN_WLED_LCD_DIS, panel_id);
		panel_detection = -1;
		return panel_detection;
	}
	pr_info("%s: panel found\n", __func__);
	light_led_disp_set_panel(LIGHT_MAIN_WLED_LCD_EN, panel_id);
	panel_detection = 1;
	return panel_detection;
}

int disp_ext_board_get_panel_detect(void)
{
    return panel_detection;
}
