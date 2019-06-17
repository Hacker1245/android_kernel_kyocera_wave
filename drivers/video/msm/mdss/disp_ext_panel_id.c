/*
 * This software is contributed or developed by KYOCERA Corporation.
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
#include <linux/of_gpio.h>

#include "disp_ext.h"

static int disp_ext_pinctrl_set_state(
	struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	struct pinctrl_state *pin_state)
{
	int rc = -EFAULT;

	if (IS_ERR_OR_NULL(ctrl_pdata->pin_res.pinctrl))
		return PTR_ERR(ctrl_pdata->pin_res.pinctrl);

	if (!IS_ERR_OR_NULL(pin_state)) {
		rc = pinctrl_select_state(ctrl_pdata->pin_res.pinctrl,
				pin_state);
		if (rc)
			pr_err("%s: can not set pins\n", __func__);
	} else {
		pr_err("%s: invalid pinstate\n", __func__);
	}
	return rc;
}

static int panel_id = -1;

static int detect_panel_id(struct platform_device *pdev)
{
	int id = 0;
	struct mdss_dsi_ctrl_pdata *ctrl;

	if( panel_id != -1) {
		pr_notice("%s: panel id Checked\n", __func__);
		return panel_id;
	}

	ctrl = platform_get_drvdata(pdev);

	ctrl->lcd_id0_gpio = of_get_named_gpio(pdev->dev.of_node,
			 "qcom,platform-lcd_id0-gpio", 0);
	if (!gpio_is_valid(ctrl->lcd_id0_gpio)) {
		pr_err("%s:%d, lcd_id0 gpio not specified\n",
						__func__, __LINE__);
		goto error;
	}

	ctrl->lcd_id1_gpio = of_get_named_gpio(pdev->dev.of_node,
			 "qcom,platform-lcd_id1-gpio", 0);
	if (!gpio_is_valid(ctrl->lcd_id1_gpio)) {
		pr_err("%s:%d, lcd_id1 gpio not specified\n",
						__func__, __LINE__);
		goto error;
	}

	disp_ext_pinctrl_set_state(ctrl, ctrl->pin_res.gpio_state_id0_active1);
	disp_ext_pinctrl_set_state(ctrl, ctrl->pin_res.gpio_state_id1_active1);

	if (gpio_get_value(ctrl->lcd_id0_gpio)) {
		id |= BIT(1);
	} else {
		disp_ext_pinctrl_set_state(ctrl, ctrl->pin_res.gpio_state_id0_active2);
	}

	if (gpio_get_value(ctrl->lcd_id1_gpio)) {
		id |= BIT(0);
	} else {
		disp_ext_pinctrl_set_state(ctrl, ctrl->pin_res.gpio_state_id1_active2);
	}

	panel_id = id;
	pr_info("%s: panel id = %d\n", __func__, panel_id);

error:
	return panel_id;
}

struct device_node *disp_ext_find_panel_of_node(struct platform_device *pdev)
{
	struct mdss_dsi_ctrl_pdata *ctrl;
	struct device_node *dsi_pan_node = NULL, *mdss_node = NULL;

	ctrl = platform_get_drvdata(pdev);

	mdss_node = of_parse_phandle(pdev->dev.of_node,
		"qcom,mdss-mdp", 0);
	if (!mdss_node) {
		pr_err("%s: %d: mdss_node null\n",
			__func__, __LINE__);
		return NULL;
	}

	switch (detect_panel_id(pdev)) {
	case 0:
		dsi_pan_node = of_find_node_by_name(mdss_node,
				"qcom,mdss_dsi_kc_djn_qhd_video");
		break;
	default:
		dsi_pan_node = of_parse_phandle(pdev->dev.of_node,
				"qcom,dsi-pref-prim-pan", 0);
		break;
	};

	return dsi_pan_node;
}

int disp_ext_get_panel_id(void)
{
	return panel_id;
}
