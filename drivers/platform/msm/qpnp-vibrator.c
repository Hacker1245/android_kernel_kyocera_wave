/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 */
/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/of_device.h>
#include <linux/spmi.h>
#include <linux/qpnp/pwm.h>
#include <linux/err.h>
#include "../../staging/android/timed_output.h"

#define VIB_DEBUG		0
#if VIB_DEBUG
#define VIB_DEBUG_LOG( msg... )	printk(KERN_ERR "[VIB]:" msg)
#else
#define VIB_DEBUG_LOG( msg... )
#endif
#define VIB_ERR_LOG( msg... ) 	printk(KERN_ERR "[VIB]:" msg)
#define VIB_INFO_LOG( msg... ) 	printk(KERN_INFO "[VIB]:" msg)

#define QPNP_VIB_VTG_CTL(base)		(base + 0x41)
#define QPNP_VIB_EN_CTL(base)		(base + 0x46)

#define QPNP_VIB_MAX_LEVEL		31
#define QPNP_VIB_MIN_LEVEL		12

#define QPNP_VIB_DEFAULT_TIMEOUT	15000
#define QPNP_VIB_DEFAULT_VTG_LVL	3000

#define QPNP_VIB_EN			BIT(7)
#define QPNP_VIB_VTG_SET_MASK		0x1F
#define QPNP_VIB_LOGIC_SHIFT		4

enum qpnp_vib_mode {
	QPNP_VIB_MANUAL,
	QPNP_VIB_DTEST1,
	QPNP_VIB_DTEST2,
	QPNP_VIB_DTEST3,
};

struct qpnp_pwm_info {
	struct pwm_device *pwm_dev;
	u32 pwm_channel;
	u32 duty_us;
	u32 period_us;
};

struct qpnp_vib {
	struct spmi_device *spmi;
	struct hrtimer vib_timer;
	struct timed_output_dev timed_dev;
	struct work_struct work;
	struct qpnp_pwm_info pwm_info;
	enum   qpnp_vib_mode mode;

	u8  reg_vtg_ctl;
	u8  reg_en_ctl;
	u8  active_low;
	u16 base;
	int state;
	int vtg_level;
	int timeout;
	struct mutex lock;
};

static struct qpnp_vib *vib_dev;

#define VIB_TEST
#ifdef VIB_TEST
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/ioctl.h>

#define VIB_TEST_IOC_MAGIC 'v'
#define IOCTL_VIB_TEST_CTRL _IOWR(VIB_TEST_IOC_MAGIC, 1, vib_test_param)

#define VIB_TEST_SET_VOLTAGE 0x0001

#define VIB_TEST_STATUS_SUCCESS (0)
#define VIB_TEST_STATUS_FAIL    (-1)

typedef struct {
    u16 req_code;
	u8 data[4];
} vib_test_param;

typedef struct {
    u16 voltage;
    u8 reserved[2];
} vib_test_set_voltage_req_data;

typedef struct {
    u16 status;
    u8 reserved[2];
} vib_test_rsp_data;
#endif /* VIB_TEST */

static int qpnp_vib_read_u8(struct qpnp_vib *vib, u8 *data, u16 reg)
{
	int rc;

	rc = spmi_ext_register_readl(vib->spmi->ctrl, vib->spmi->sid,
							reg, data, 1);
	if (rc < 0)
		dev_err(&vib->spmi->dev,
			"Error reading address: %X - ret %X\n", reg, rc);

	return rc;
}

static int qpnp_vib_write_u8(struct qpnp_vib *vib, u8 *data, u16 reg)
{
	int rc;

	rc = spmi_ext_register_writel(vib->spmi->ctrl, vib->spmi->sid,
							reg, data, 1);
	if (rc < 0)
		dev_err(&vib->spmi->dev,
			"Error writing address: %X - ret %X\n", reg, rc);

	return rc;
}

static int qpnp_vibrator_config(struct qpnp_vib *vib)
{
	u8 reg = 0;
	int rc;

	/* Configure the VTG CTL regiser */
	rc = qpnp_vib_read_u8(vib, &reg, QPNP_VIB_VTG_CTL(vib->base));
	if (rc < 0)
		return rc;
	reg &= ~QPNP_VIB_VTG_SET_MASK;
	reg |= (vib->vtg_level & QPNP_VIB_VTG_SET_MASK);
	rc = qpnp_vib_write_u8(vib, &reg, QPNP_VIB_VTG_CTL(vib->base));
	if (rc)
		return rc;
	vib->reg_vtg_ctl = reg;

	/* Configure the VIB ENABLE regiser */
	rc = qpnp_vib_read_u8(vib, &reg, QPNP_VIB_EN_CTL(vib->base));
	if (rc < 0)
		return rc;
	reg |= (!!vib->active_low) << QPNP_VIB_LOGIC_SHIFT;
	if (vib->mode != QPNP_VIB_MANUAL) {
		vib->pwm_info.pwm_dev = pwm_request(vib->pwm_info.pwm_channel,
								 "qpnp-vib");
		if (IS_ERR_OR_NULL(vib->pwm_info.pwm_dev)) {
			dev_err(&vib->spmi->dev, "vib pwm request failed\n");
			return -ENODEV;
		}

		rc = pwm_config(vib->pwm_info.pwm_dev, vib->pwm_info.duty_us,
						vib->pwm_info.period_us);
		if (rc < 0) {
			dev_err(&vib->spmi->dev, "vib pwm config failed\n");
			pwm_free(vib->pwm_info.pwm_dev);
			return -ENODEV;
		}

		reg |= BIT(vib->mode - 1);
	}

	rc = qpnp_vib_write_u8(vib, &reg, QPNP_VIB_EN_CTL(vib->base));
	if (rc < 0)
		return rc;
	vib->reg_en_ctl = reg;

	return rc;
}

static int qpnp_vib_set(struct qpnp_vib *vib, int on)
{
	int rc;
	u8 val;

	if (on) {
		if (vib->mode != QPNP_VIB_MANUAL)
			pwm_enable(vib->pwm_info.pwm_dev);
		else {
			val = vib->reg_en_ctl;
			val |= QPNP_VIB_EN;
			rc = qpnp_vib_write_u8(vib, &val,
					QPNP_VIB_EN_CTL(vib->base));
			if (rc < 0)
				return rc;
			vib->reg_en_ctl = val;
		}
	} else {
		if (vib->mode != QPNP_VIB_MANUAL)
			pwm_disable(vib->pwm_info.pwm_dev);
		else {
			val = vib->reg_en_ctl;
			val &= ~QPNP_VIB_EN;
			rc = qpnp_vib_write_u8(vib, &val,
					QPNP_VIB_EN_CTL(vib->base));
			if (rc < 0)
				return rc;
			vib->reg_en_ctl = val;
		}
	}

	return 0;
}

static void qpnp_vib_enable(struct timed_output_dev *dev, int value)
{
	struct qpnp_vib *vib = container_of(dev, struct qpnp_vib,
					 timed_dev);

	VIB_DEBUG_LOG("%s() called. value=%d\n", __func__, value);

	mutex_lock(&vib->lock);
	hrtimer_cancel(&vib->vib_timer);

	if (value == 0)
		vib->state = 0;
	else {
		if(value < 50) {
			VIB_DEBUG_LOG("%s() set value to 55.\n", __func__);
			value = 55;
		} else if(value < 55) {
			VIB_DEBUG_LOG("%s() add value 5.\n", __func__);
			value += 5;
		} else if(value < 60) {
			VIB_DEBUG_LOG("%s() add value 3.\n", __func__);
			value += 3;
		}
		vib->state = 1;
		hrtimer_start(&vib->vib_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
	mutex_unlock(&vib->lock);
	schedule_work(&vib->work);

	VIB_DEBUG_LOG("%s() end.\n", __func__);
}

static void qpnp_vib_update(struct work_struct *work)
{
	struct qpnp_vib *vib = container_of(work, struct qpnp_vib,
					 work);
	qpnp_vib_set(vib, vib->state);
}

static int qpnp_vib_get_time(struct timed_output_dev *dev)
{
	struct qpnp_vib *vib = container_of(dev, struct qpnp_vib,
							 timed_dev);

	if (hrtimer_active(&vib->vib_timer)) {
		ktime_t r = hrtimer_get_remaining(&vib->vib_timer);
		return (int)ktime_to_us(r);
	} else
		return 0;
}

static enum hrtimer_restart qpnp_vib_timer_func(struct hrtimer *timer)
{
	struct qpnp_vib *vib = container_of(timer, struct qpnp_vib,
							 vib_timer);

	vib->state = 0;
	schedule_work(&vib->work);

	return HRTIMER_NORESTART;
}

#ifdef VIB_TEST
static int vibrator_test_open(struct inode *ip, struct file *fp)
{
	VIB_DEBUG_LOG("%s() called.\n", __func__);
	VIB_DEBUG_LOG("%s() end.\n", __func__);
	return 0;
}

static int vibrator_test_release(struct inode *ip, struct file *fp)
{
	VIB_DEBUG_LOG("%s() called.\n", __func__);
	VIB_DEBUG_LOG("%s() end.\n", __func__);
	return 0;
}

static int vibrator_test_set(int level)
{
	u8 val;
	int ret = 0;

	VIB_DEBUG_LOG("%s() called. leve=0x%x\n", __func__, level);
	if (level)
	{
		val = vib_dev->reg_vtg_ctl;
		val &= ~QPNP_VIB_VTG_SET_MASK;
		val |= (level & QPNP_VIB_VTG_SET_MASK);
		ret = qpnp_vib_write_u8(vib_dev, &val, QPNP_VIB_VTG_CTL(vib_dev->base));
		if (ret < 0)
			return ret;
		vib_dev->reg_vtg_ctl = val;
		val = vib_dev->reg_en_ctl;
		val |= QPNP_VIB_EN;
		ret = qpnp_vib_write_u8(vib_dev, &val, QPNP_VIB_EN_CTL(vib_dev->base));
		if (ret < 0)
			return ret;
		vib_dev->reg_en_ctl = val;
	}
	else
	{
		val = vib_dev->reg_en_ctl;
		val &= ~QPNP_VIB_EN;
		ret = qpnp_vib_write_u8(vib_dev, &val, QPNP_VIB_EN_CTL(vib_dev->base));
		if (ret < 0)
			return ret;
		vib_dev->reg_en_ctl = val;
	}
	VIB_DEBUG_LOG("%s() end. ret=0x%x\n", __func__, ret);
	return ret;
}
static int vibrator_test_set_voltage(u8 *data)
{
	int ret = 0;
	int level=0;
	vib_test_set_voltage_req_data *req_data =
	(vib_test_set_voltage_req_data *)data;
	vib_test_rsp_data *rsp_data = (vib_test_rsp_data *)data;
	s16 status = VIB_TEST_STATUS_SUCCESS;

	VIB_DEBUG_LOG("%s() called.\n", __func__);

	level = req_data->voltage /= 100;

	if (level) {
		if ((level < QPNP_VIB_MIN_LEVEL) ||
		    (level > QPNP_VIB_MAX_LEVEL)) {
			VIB_ERR_LOG("%s() Invalid voltage level\n", __func__);
			status = VIB_TEST_STATUS_FAIL;
		}
	}

	if (status == VIB_TEST_STATUS_SUCCESS)
	{
		ret = vibrator_test_set(level);
	}

	if (ret < 0) {
		VIB_ERR_LOG("%s() vibrator_test_set error.ret=%d\n", __func__, ret);
		status = VIB_TEST_STATUS_FAIL;
	}

	memset(rsp_data, 0x00, sizeof(vib_test_rsp_data));
	rsp_data->status = (u32)status;

	VIB_DEBUG_LOG("%s() end. ret=%d\n", __func__, ret);
	return ret;
}

static long vibrator_test_ioctl
                        (struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	int ret = 0;
	u64 ret2 = 0;
	vib_test_param test_param;
	VIB_DEBUG_LOG("%s() called. cmd=0x%08X\n", __func__, cmd);

	switch (cmd) {
	case IOCTL_VIB_TEST_CTRL:
		VIB_DEBUG_LOG("%s() cmd=IOCTL_VIB_TEST_CTRL\n", __func__);
		ret2 = copy_from_user(&test_param, (void *)arg, sizeof(test_param));
		VIB_DEBUG_LOG("%s() copy_from_user() called. ret2=%lu\n", __func__,
			     (long unsigned int)ret2);
		VIB_DEBUG_LOG("%s() copy_from_user() req_code=0x%04X,data=0x%02X%02X%02X%02X\n", __func__,
		              test_param.req_code, test_param.data[0], test_param.data[1],
			      test_param.data[2], test_param.data[3]);
	if (ret2) {
		VIB_ERR_LOG("%s() copy_from_user() error. ret2=%lu\n", __func__,
		       (long unsigned int)ret2);
	rc = -EINVAL;
		break;
	}
	VIB_DEBUG_LOG("%s() req_code=0x%04X\n", __func__, test_param.req_code);
	switch (test_param.req_code) {
	case VIB_TEST_SET_VOLTAGE:
		ret = vibrator_test_set_voltage(&test_param.data[0]);
		if (ret < 0) VIB_ERR_LOG("%s() vibrator_test_set_voltage() error. ret=%d\n", __func__, ret);
		ret2 = copy_to_user((void *)arg, &test_param, sizeof(vib_test_param));
		VIB_DEBUG_LOG("%s() copy_to_user() called. ret2=%lu\n", __func__,
			      (long unsigned int)ret2);
		VIB_DEBUG_LOG("%s() copy_to_user() req_code=0x%04X,data=0x%02X%02X%02X%02X\n", __func__,
		              test_param.req_code, test_param.data[0], test_param.data[1],
			      test_param.data[2], test_param.data[3]);
		if (ret2) {
			VIB_ERR_LOG("%s() copy_to_user() error. ret2=%lu\n", __func__,
			       (long unsigned int)ret2);
			rc = -EINVAL;
		}
		break;
	default:
		VIB_ERR_LOG("%s() req_code error. req_code=0x%04X\n", __func__,
		        test_param.req_code);
		rc = -EINVAL;
		break;
	}
	break;
    default:
		VIB_ERR_LOG("%s() cmd error. cmd=0x%08X\n", __func__, cmd);
		rc = -EINVAL;
		break;
	}

	VIB_DEBUG_LOG("%s() end. rc=%d\n", __func__, rc);
	return rc;
}

static const struct file_operations vibrator_test_fops = {
	.owner          = THIS_MODULE,
	.open           = vibrator_test_open,
	.release        = vibrator_test_release,
	.unlocked_ioctl = vibrator_test_ioctl,
};

static struct miscdevice vibrator_test_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "kc_vibrator_test",
	.fops = &vibrator_test_fops,
};

void vibrator_test_init(void)
{
	misc_register(&vibrator_test_dev);
}
#endif /* VIB_TEST */


#ifdef CONFIG_PM
static int qpnp_vibrator_suspend(struct device *dev)
{
	struct qpnp_vib *vib = dev_get_drvdata(dev);

	hrtimer_cancel(&vib->vib_timer);
	cancel_work_sync(&vib->work);
	/* turn-off vibrator */
	qpnp_vib_set(vib, 0);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(qpnp_vibrator_pm_ops, qpnp_vibrator_suspend, NULL);

static int qpnp_vib_parse_dt(struct qpnp_vib *vib)
{
	struct spmi_device *spmi = vib->spmi;
	int rc;
	const char *mode;
	u32 temp_val;

	vib->timeout = QPNP_VIB_DEFAULT_TIMEOUT;
	rc = of_property_read_u32(spmi->dev.of_node,
			"qcom,vib-timeout-ms", &temp_val);
	if (!rc) {
		vib->timeout = temp_val;
	} else if (rc != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read vib timeout\n");
		return rc;
	}

	vib->vtg_level = QPNP_VIB_DEFAULT_VTG_LVL;
	rc = of_property_read_u32(spmi->dev.of_node,
			"qcom,vib-vtg-level-mV", &temp_val);
	if (!rc) {
		vib->vtg_level = temp_val;
	} else if (rc != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read vtg level\n");
		return rc;
	}

	vib->vtg_level /= 100;
	if (vib->vtg_level < QPNP_VIB_MIN_LEVEL)
		vib->vtg_level = QPNP_VIB_MIN_LEVEL;
	else if (vib->vtg_level > QPNP_VIB_MAX_LEVEL)
		vib->vtg_level = QPNP_VIB_MAX_LEVEL;

	vib->mode = QPNP_VIB_MANUAL;
	rc = of_property_read_string(spmi->dev.of_node, "qcom,mode", &mode);
	if (!rc) {
		if (strcmp(mode, "manual") == 0)
			vib->mode = QPNP_VIB_MANUAL;
		else if (strcmp(mode, "dtest1") == 0)
			vib->mode = QPNP_VIB_DTEST1;
		else if (strcmp(mode, "dtest2") == 0)
			vib->mode = QPNP_VIB_DTEST2;
		else if (strcmp(mode, "dtest3") == 0)
			vib->mode = QPNP_VIB_DTEST3;
		else {
			dev_err(&spmi->dev, "Invalid mode\n");
			return -EINVAL;
		}
	} else if (rc != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read mode\n");
		return rc;
	}

	if (vib->mode != QPNP_VIB_MANUAL) {
		rc = of_property_read_u32(spmi->dev.of_node,
				"qcom,pwm-channel", &temp_val);
		if (!rc)
			vib->pwm_info.pwm_channel = temp_val;
		else
			return rc;

		rc = of_property_read_u32(spmi->dev.of_node,
				"qcom,period-us", &temp_val);
		if (!rc)
			vib->pwm_info.period_us = temp_val;
		else
			return rc;

		rc = of_property_read_u32(spmi->dev.of_node,
				"qcom,duty-us", &temp_val);
		if (!rc)
			vib->pwm_info.duty_us = temp_val;
		else
			return rc;
	}

	vib->active_low = of_property_read_bool(spmi->dev.of_node,
				"qcom,active-low");

	return 0;
}

static int qpnp_vibrator_probe(struct spmi_device *spmi)
{
	struct qpnp_vib *vib;
	struct resource *vib_resource;
	int rc;

	vib = devm_kzalloc(&spmi->dev, sizeof(*vib), GFP_KERNEL);
	if (!vib)
		return -ENOMEM;

	vib->spmi = spmi;

	vib_resource = spmi_get_resource(spmi, 0, IORESOURCE_MEM, 0);
	if (!vib_resource) {
		dev_err(&spmi->dev, "Unable to get vibrator base address\n");
		return -EINVAL;
	}
	vib->base = vib_resource->start;

	rc = qpnp_vib_parse_dt(vib);
	if (rc) {
		dev_err(&spmi->dev, "DT parsing failed\n");
		return rc;
	}

	rc = qpnp_vibrator_config(vib);
	if (rc) {
		dev_err(&spmi->dev, "vib config failed\n");
		return rc;
	}

	mutex_init(&vib->lock);
	INIT_WORK(&vib->work, qpnp_vib_update);

	hrtimer_init(&vib->vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib->vib_timer.function = qpnp_vib_timer_func;

	vib->timed_dev.name = "vibrator";
	vib->timed_dev.get_time = qpnp_vib_get_time;
	vib->timed_dev.enable = qpnp_vib_enable;

	dev_set_drvdata(&spmi->dev, vib);

	rc = timed_output_dev_register(&vib->timed_dev);
	if (rc < 0)
		return rc;

	vib_dev = vib;

	return rc;
}

static int qpnp_vibrator_remove(struct spmi_device *spmi)
{
	struct qpnp_vib *vib = dev_get_drvdata(&spmi->dev);

	cancel_work_sync(&vib->work);
	hrtimer_cancel(&vib->vib_timer);
	timed_output_dev_unregister(&vib->timed_dev);
	mutex_destroy(&vib->lock);

	return 0;
}

static struct of_device_id spmi_match_table[] = {
	{	.compatible = "qcom,qpnp-vibrator",
	},
	{}
};

static struct spmi_driver qpnp_vibrator_driver = {
	.driver		= {
		.name	= "qcom,qpnp-vibrator",
		.of_match_table = spmi_match_table,
		.pm	= &qpnp_vibrator_pm_ops,
	},
	.probe		= qpnp_vibrator_probe,
	.remove		= qpnp_vibrator_remove,
};

static int __init qpnp_vibrator_init(void)
{
#ifdef VIB_TEST
	vibrator_test_init();
#endif /* VIB_TEST */
	return spmi_driver_register(&qpnp_vibrator_driver);
}
module_init(qpnp_vibrator_init);

static void __exit qpnp_vibrator_exit(void)
{
	return spmi_driver_unregister(&qpnp_vibrator_driver);
}
module_exit(qpnp_vibrator_exit);

MODULE_DESCRIPTION("qpnp vibrator driver");
MODULE_LICENSE("GPL v2");
