/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 */
/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
//#define DISABLE_DISP_DETECT
#define ENABLE_CABC_PWM

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/wakelock.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <mach/vreg.h>
#include <linux/regulator/consumer.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/leds.h>
#include <linux/kc_led.h>
#include "oem_wled.h"

#undef FORCE_WLED_ON

#define BD65B60GWL_DEBUG        0
#if BD65B60GWL_DEBUG
#define BD65B60GWL_DEBUG_LOG( msg... )	printk(KERN_ERR "[WLED]:" msg)
#else
#define BD65B60GWL_DEBUG_LOG( msg... )
#endif

#define BD65B60GWL_ERR_LOG( msg... ) 	printk(KERN_ERR "[WLED]:" msg)
#define BD65B60GWL_INFO_LOG( msg... ) 	printk(KERN_INFO "[WLED]:" msg)


#define BD65B60GWL_DRV_NAME "BD65B60GWL"

#define I2C_RETRIES_NUM					(5)
#define BD65B60GWL_RESET_GPIO			(98)
#define BD65B60GWL_I2C_WRITE_MSG_NUM	(1)
#define BD65B60GWL_I2C_READ_MSG_NUM		(2)
#define BD65B60GWL_WRITE_BUF_LEN		(2)

#define BD65B60GWL_LED_ENABLE			0x01
#define BD65B60GWL_LED_DISABLE			0x00
#define BD65B60GWL_LED_BRIGHT_MASK		0xff

#define BACKLIGHT_ON					0x01
#define BACKLIGHT_OFF					0x00
#define BACKLIGHT_THRESHOLD_TEMPVAL		0x3C
#define BACKLIGHT_THRESHOLD_CURVAL		0xFF

#define GPIO_HIGH_VAL					1
#define GPIO_LOW_VAL					0

struct bd65b60gwl_data {
	struct led_classdev	st_cdev;
	struct i2c_client	*client;
	uint32_t			ul_value;
	struct work_struct	work;
	struct mutex		lock;
	struct wake_lock	work_wake_lock;
	int					wled_reset_gpio;
};

struct magsns_regulator_data {
	struct regulator* vdd_reg;
	uint32_t min_uV;
	uint32_t max_uV;
	uint32_t on_load_uA;
	uint32_t off_load_uA;
};

static struct magsns_regulator_data reg_data;

#define BACKLIGHT_INFO			"backlightinfo"

static struct i2c_client *client_bd65b60gwl = NULL;
static int32_t backlight_status = BACKLIGHT_OFF;
static unsigned int *g_wled_brightness = NULL;

#ifndef DISABLE_DISP_DETECT
static int do_disp_lock = 0;
static atomic_t g_disp_status = ATOMIC_INIT(0);
static atomic_t g_display_detect = ATOMIC_INIT(0);
static struct mutex led_disp_lock;
static atomic_t g_detect_lcd_en = ATOMIC_INIT(0);
#endif

static int bd65b60gwl_i2c_read(struct i2c_client *client, uint8_t uc_reg, uint8_t *rbuf, int len);
static int bd65b60gwl_i2c_write(struct i2c_client *client, uint8_t uc_reg, uint8_t uc_val);
static void bd65b60gwl_work_handler(struct work_struct *work);
#ifdef CONFIG_PM
static int bd65b60gwl_suspend(struct i2c_client *client, pm_message_t mesg);
static int bd65b60gwl_resume(struct i2c_client *client);
#endif
static int bd65b60gwl_remove(struct i2c_client *client);
static int bd65b60gwl_init_client(struct i2c_client *client);
static int bd65b60gwl_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int32_t __init bd65b60gwl_wled_init(void);
static void __exit bd65b60gwl_wled_exit(void);

static int bd65b60gw_activate_gpios(struct bd65b60gwl_data *data)
{
	int err = 0;

	err = regulator_set_optimum_mode(reg_data.vdd_reg, reg_data.on_load_uA);
	if( err < 0 ) {
		BD65B60GWL_ERR_LOG("%s() regulator_set_optimum_mode fail. err=%d\n", __func__, err);
		return -1;
	}
	usleep(1000);

	BD65B60GWL_DEBUG_LOG("%s() set gpio98 = %d\n", __func__, GPIO_HIGH_VAL);
	gpio_set_value_cansleep(data->wled_reset_gpio, GPIO_HIGH_VAL);
	usleep(200);

	return 0;
}

static int bd65b60gw_suspend_gpios(struct bd65b60gwl_data *data)
{
	int err = 0;

	BD65B60GWL_DEBUG_LOG("%s() set gpio98 = %d\n", __func__, GPIO_LOW_VAL);
	gpio_set_value_cansleep(data->wled_reset_gpio, GPIO_LOW_VAL);
	usleep(200);

	err = regulator_set_optimum_mode(reg_data.vdd_reg, reg_data.off_load_uA);
	if( err < 0 ) {
		BD65B60GWL_ERR_LOG("%s() regulator_set_optimum_mode fail. err=%d\n", __func__, err);
		return -1;
	}
	usleep(1000);

	return 0;
}

static int bd65b60gwl_i2c_read(struct i2c_client *client, uint8_t uc_reg, uint8_t *rbuf, int len)
{
	int ret = 0;
	int retry = 0;
	struct i2c_msg i2cMsg[BD65B60GWL_I2C_READ_MSG_NUM];
	u8 reg = 0;

	BD65B60GWL_DEBUG_LOG("%s() [IN] client=0x%p reg=0x%02X len=%d\n", __func__, client, uc_reg, len);

	if (client == NULL)
	{
		BD65B60GWL_ERR_LOG("%s() fail client=0x%p\n",__func__, client);
		return -ENODEV;
	}

	reg = uc_reg;

	i2cMsg[0].addr = client->addr;
	i2cMsg[0].flags = 0;
	i2cMsg[0].len = 1;
	i2cMsg[0].buf = &reg;

	i2cMsg[1].addr = client->addr;
	i2cMsg[1].flags = I2C_M_RD;
	i2cMsg[1].len = len;
	i2cMsg[1].buf = rbuf;

	do {
		ret = i2c_transfer(client->adapter, &i2cMsg[0], BD65B60GWL_I2C_READ_MSG_NUM);
		BD65B60GWL_DEBUG_LOG("%s(): i2c_transfer() call end ret=%d\n", __func__, ret);
	} while ((ret != BD65B60GWL_I2C_READ_MSG_NUM) && (++retry < I2C_RETRIES_NUM));

	if (ret != BD65B60GWL_I2C_READ_MSG_NUM) {
		BD65B60GWL_ERR_LOG("%s(): fail uc_reg=0x%02x, rbuf=0x%02x ret=%d\n", __func__, uc_reg, *rbuf, ret);
		ret = -1;
	}else{
		ret = 0;
	}
	BD65B60GWL_DEBUG_LOG("%s(): [OUT] rbuf=0x%02x\n", __func__, *rbuf);

	return ret;
}

static int bd65b60gwl_i2c_write(struct i2c_client *client, uint8_t uc_reg, uint8_t uc_val)
{
	int ret = 0;
	int retry = 0;
	struct i2c_msg i2cMsg;
	u8 ucwritebuf[BD65B60GWL_WRITE_BUF_LEN];

	BD65B60GWL_DEBUG_LOG("%s() [IN] client=0x%p reg=0x%02X val=0x%02X\n", __func__, client, uc_reg, uc_val);

	if (client == NULL)
	{
		BD65B60GWL_ERR_LOG("%s() fail client=0x%p\n",__func__, client);
		return -ENODEV;
	}

	ucwritebuf[0] = uc_reg;
	ucwritebuf[1] = uc_val;
	i2cMsg.addr  = client->addr;
	i2cMsg.flags = 0;
	i2cMsg.len   =  sizeof(ucwritebuf);
	i2cMsg.buf   =  &ucwritebuf[0];

	do {
		ret = i2c_transfer(client->adapter, &i2cMsg, BD65B60GWL_I2C_WRITE_MSG_NUM);
		BD65B60GWL_DEBUG_LOG("%s() i2c_transfer() call end ret=%d\n", __func__, ret);
	} while ((ret != BD65B60GWL_I2C_WRITE_MSG_NUM) && (++retry < I2C_RETRIES_NUM));

	if (ret != BD65B60GWL_I2C_WRITE_MSG_NUM) {
		BD65B60GWL_ERR_LOG("%s(): fail uc_reg 0x%02x, uc_val 0x%02x ret %d\n", __func__, ucwritebuf[0], ucwritebuf[1], ret);
		ret = -1;
	}else{
		ret = 0;
	}
	BD65B60GWL_DEBUG_LOG("%s(): [OUT]\n", __func__);

	return ret;
}

static void wled_led_set(struct led_classdev *pst_cdev, enum led_brightness value)
{
	struct bd65b60gwl_data *data;
	data = container_of(pst_cdev, struct bd65b60gwl_data, st_cdev);

	BD65B60GWL_DEBUG_LOG("%s() [IN] name=%s value=0x%08x\n", __func__, pst_cdev->name, value);

	data->ul_value = value;
	schedule_work(&data->work);
	BD65B60GWL_DEBUG_LOG("%s() [OUT]\n", __func__);
}

static enum led_brightness wled_led_get(struct led_classdev *pst_cdev)
{
	int32_t lret = 0;
	struct bd65b60gwl_data *data;
	BD65B60GWL_DEBUG_LOG("%s() [IN]\n", __func__);

	data = container_of(pst_cdev, struct bd65b60gwl_data, st_cdev);
	lret = data->ul_value;
	BD65B60GWL_DEBUG_LOG("%s() [OUT] ret=0x%02x\n", __func__, lret);

	return lret;
}

static void bd65b60gwl_work_handler(struct work_struct *work)
{
	struct bd65b60gwl_data *data = container_of(work, struct bd65b60gwl_data, work);
	int level;
	struct i2c_client *client = data->client;
	uint8_t reg = 0;
	int err = 0;
	int val;
	int gval;
#ifndef DISABLE_DISP_DETECT
	uint32_t lret = 0;
	int32_t display_detect;
	int32_t detect_lcd_en;
#endif

	level = data->st_cdev.brightness;
	BD65B60GWL_DEBUG_LOG("%s() [IN] level=0x%x\n", __func__, level);
	gval = gpio_get_value_cansleep(data->wled_reset_gpio);
	BD65B60GWL_DEBUG_LOG("%s() gpio98 = %d \n", __func__, gval);

	mutex_lock(&data->lock);

	if( BACKLIGHT_OFF == backlight_status ){
		bd65b60gw_activate_gpios( data );

		err = bd65b60gwl_init_client(client);
		if (err)
		{
			BD65B60GWL_ERR_LOG("%s() Failed bd65b60gwl_init_client\n", __func__);
		}
	}

#ifdef DISABLE_DISP_DETECT
	BD65B60GWL_DEBUG_LOG("%s(): DISABLE_DISP_DETECT\n", __func__);
#else
	display_detect = atomic_read(&g_display_detect);
	detect_lcd_en = atomic_read(&g_detect_lcd_en);
#endif

	if( g_wled_brightness ) {
		val = g_wled_brightness[level & BD65B60GWL_LED_BRIGHT_MASK];
	} else {
		val = level & BD65B60GWL_LED_BRIGHT_MASK;
	}
	BD65B60GWL_DEBUG_LOG("%s() level=0x%x rlevel=0x%x\n", __func__, level, val);

	bd65b60gwl_i2c_write(client, 0x05, val);

	if (level == 0) {
	    BD65B60GWL_DEBUG_LOG("%s(): OFF\n", __func__);
#ifndef DISABLE_DISP_DETECT
		if(display_detect == 0 && !detect_lcd_en){
			BD65B60GWL_DEBUG_LOG("%s() unknown display detect(off)\n", __func__);
			lret = light_led_disp_set(LIGHT_MAIN_WLED_LED_DIS);
			if (lret != 0){
				BD65B60GWL_ERR_LOG("%s() failed light_led_disp_set()\n", __func__);
			}
		}
		else if(display_detect == 1 || detect_lcd_en == 1)
#endif
		{
			bd65b60gwl_i2c_read(client, 0x05, &reg, 1);
			bd65b60gwl_i2c_write(client, 0x0E, BD65B60GWL_LED_DISABLE);
		}

		bd65b60gw_suspend_gpios( data );

		backlight_status = BACKLIGHT_OFF;
	} else {
	    BD65B60GWL_DEBUG_LOG("%s(): ON (0x%0x)\n", __func__, level);
#ifndef DISABLE_DISP_DETECT
		if(display_detect == 0){
			BD65B60GWL_DEBUG_LOG("%s() unknown display detect(on)\n", __func__);
			lret = light_led_disp_set(LIGHT_MAIN_WLED_LED_EN);
			if (lret != 0){
				BD65B60GWL_ERR_LOG("%s() failed light_led_disp_set()\n", __func__);
			}
		}
		else if(display_detect == 1)
#endif
		{
			bd65b60gwl_i2c_write(client, 0x0E, BD65B60GWL_LED_ENABLE);
		}
#ifndef DISABLE_DISP_DETECT
		else {
			BD65B60GWL_ERR_LOG("%s(): No set display display_detect=[%x] \n", __func__, (int32_t)display_detect);

			bd65b60gw_suspend_gpios( data );
        }
#endif
		backlight_status = BACKLIGHT_ON;
	}

	mutex_unlock(&data->lock);

	BD65B60GWL_DEBUG_LOG("%s() [OUT]\n", __func__);

	return;
}

#ifdef CONFIG_PM
static int bd65b60gwl_suspend(struct i2c_client *client, pm_message_t mesg)
{
	BD65B60GWL_DEBUG_LOG("%s() [IN]\n", __func__);
	BD65B60GWL_DEBUG_LOG("%s() [OUT]\n", __func__);
	return 0;
}

static int bd65b60gwl_resume(struct i2c_client *client)
{
	BD65B60GWL_DEBUG_LOG("%s() [IN]\n", __func__);
	BD65B60GWL_DEBUG_LOG("%s() [OUT]\n", __func__);
	return 0;
}
#else
#define bd65b60gwl_suspend    NULL
#define bd65b60gwl_resume     NULL
#endif

static int bd65b60gwl_remove(struct i2c_client *client)
{
	struct bd65b60gwl_data *data = i2c_get_clientdata(client);

	BD65B60GWL_DEBUG_LOG("%s() [IN]\n", __func__);

	kfree(data);

	BD65B60GWL_DEBUG_LOG("%s() [OUT]\n", __func__);

	return 0;
}

static int bd65b60gwl_init_client(struct i2c_client *client)
{
	struct bd65b60gwl_data *data = i2c_get_clientdata(client);
	int gval;

	BD65B60GWL_DEBUG_LOG("%s() [IN] client=0x%p\n", __func__, client);

	gval = gpio_get_value_cansleep(data->wled_reset_gpio);
	BD65B60GWL_DEBUG_LOG("%s() gpio98 = %d \n", __func__, gval);
	if( !gval ){
		bd65b60gw_activate_gpios( data );
	}
	bd65b60gwl_i2c_write(client, 0x01, 0x01);
	bd65b60gwl_i2c_write(client, 0x02, 0x02);
	bd65b60gwl_i2c_write(client, 0x03, 0x05);
	bd65b60gwl_i2c_write(client, 0x05, 0x60);
#ifdef ENABLE_CABC_PWM
	bd65b60gwl_i2c_write(client, 0x07, 0x26);
#else
	bd65b60gwl_i2c_write(client, 0x07, 0x06);
#endif
	bd65b60gwl_i2c_write(client, 0x08, 0x00);

#ifdef FORCE_WLED_ON
	BD65B60GWL_DEBUG_LOG("%s() force WLED ON\n", __func__);
	bd65b60gwl_i2c_write(client, 0x0E, 0x01);
#endif
	BD65B60GWL_DEBUG_LOG("%s() [OUT]\n", __func__);

    return 0;
}

static int bd65b60gwl_probe(struct i2c_client *client,
                   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct bd65b60gwl_data *data;
	int err = 0;
	int gval;

	BD65B60GWL_DEBUG_LOG("%s() [IN] client=0x%p\n", __func__, client);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	data = kzalloc(sizeof(struct bd65b60gwl_data), GFP_KERNEL);
	if (!data) {
		printk("%s: Failed kzalloc\n",__func__);
		err = -ENOMEM;
		goto exit;
	}

	data->st_cdev.brightness_set    = wled_led_set;
	data->st_cdev.brightness_get    = wled_led_get;

	mutex_init(&data->lock);

	data->client = client;
	i2c_set_clientdata(client, data);

	INIT_WORK(&data->work, bd65b60gwl_work_handler);

	data->st_cdev.name = BACKLIGHT_INFO;

	BD65B60GWL_DEBUG_LOG("%s(): led_classdev_register cdev.name=%s\n", __func__, data->st_cdev.name);
	err = led_classdev_register(&data->client->dev, &data->st_cdev);
	if (err) {
		BD65B60GWL_ERR_LOG("%s() unable to register led %s\n", __func__, data->st_cdev.name);
		goto fail_id_check;
	}

	data->wled_reset_gpio = of_get_named_gpio(client->dev.of_node, "kc,reset-gpio", 0);
	if (!gpio_is_valid(data->wled_reset_gpio)) {
		BD65B60GWL_ERR_LOG("%s() No valid RESET GPIO specified %d\n", __func__, data->wled_reset_gpio);
		goto fail_id_check;
	}

	err = gpio_request(data->wled_reset_gpio, BD65B60GWL_DRV_NAME);
	BD65B60GWL_DEBUG_LOG("%s() gpio_request GPIO=%d err=%d\n", __func__, data->wled_reset_gpio, err);
	if (err < 0) {
		BD65B60GWL_ERR_LOG("%s() failed to request GPIO=%d, ret=%d\n",
				__func__,
				data->wled_reset_gpio,
				err);
		goto fail_id_check;
	}
	gval = gpio_get_value_cansleep(data->wled_reset_gpio);
	BD65B60GWL_DEBUG_LOG("%s() gpio98 = %d \n", __func__, gval);

	g_wled_brightness = wled_brightness_panel0;

#ifdef DISABLE_DISP_DETECT
	err = bd65b60gwl_init_client(client);
	if (err)
	{
		BD65B60GWL_ERR_LOG("%s() Failed bd65b60gwl_init_client\n", __func__);
		goto fail_id_check;
	}
#endif
	client_bd65b60gwl = client;

	of_property_read_u32(client->dev.of_node, "wled-vdd-min-voltage", &reg_data.min_uV);
	of_property_read_u32(client->dev.of_node, "wled-vdd-max-voltage", &reg_data.max_uV);
	of_property_read_u32(client->dev.of_node, "wled-vdd-on-load-current", &reg_data.on_load_uA);
	of_property_read_u32(client->dev.of_node, "wled-vdd-off-load-current", &reg_data.off_load_uA);
	BD65B60GWL_DEBUG_LOG("%s() regulator min_uV = %d, max_uV = %d, on_load_uA = %d, off_load_uA = %d\n", __func__,
		reg_data.min_uV, reg_data.max_uV, reg_data.on_load_uA, reg_data.off_load_uA);

	reg_data.vdd_reg = regulator_get(&client->dev, "wled-vdd");
	if( IS_ERR(reg_data.vdd_reg) ) {
		BD65B60GWL_ERR_LOG("%s() failed regulator_get \n", __func__);
		goto fail_id_check;
	}

	err = regulator_set_voltage(reg_data.vdd_reg, reg_data.min_uV, reg_data.max_uV);
	if( err ) {
		BD65B60GWL_ERR_LOG("%s() regulator_set_voltage fail. err=%d\n", __func__, err);
		goto fail_id_check;
	}

	err = regulator_enable(reg_data.vdd_reg);
	if( err ) {
		BD65B60GWL_ERR_LOG("%s() regulator_enable fail. err=%d\n", __func__, err);
		goto fail_id_check;
	}

	BD65B60GWL_DEBUG_LOG("%s() [OUT]\n", __func__);

	return 0;

fail_id_check:
	mutex_destroy(&data->lock);
	led_classdev_unregister(&data->st_cdev);
	kfree(data);
exit:
	BD65B60GWL_DEBUG_LOG("%s() [OUT] err=%d\n", __func__, err);
	return err;
}


static const struct i2c_device_id bd65b60gwl_id[] = {
	{ BD65B60GWL_DRV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bd65b60gwl_id);

static struct of_device_id bd65b60gwl_match_table[] = {
	{ .compatible = BD65B60GWL_DRV_NAME,},
	{ },
};

static struct i2c_driver bd65b60gwl_driver = {
	.driver = {
		.name   = BD65B60GWL_DRV_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = bd65b60gwl_match_table,
	},
	.suspend = bd65b60gwl_suspend,
	.resume = bd65b60gwl_resume,
	.probe  = bd65b60gwl_probe,
	.remove = bd65b60gwl_remove,
	.id_table = bd65b60gwl_id,
};

static int32_t __init bd65b60gwl_wled_init(void)
{
	int32_t rc;

	 BD65B60GWL_DEBUG_LOG("%s() [IN]\n", __func__);

	rc = i2c_add_driver(&bd65b60gwl_driver);
	if (rc != 0) {
		BD65B60GWL_ERR_LOG("%s() can't add i2c driver\n", __func__);
		rc = -ENOTSUPP;
		return rc;
	}

	BD65B60GWL_DEBUG_LOG("%s() [OUT]\n", __func__);

	return 0;

}

static void __exit bd65b60gwl_wled_exit(void)
{
	BD65B60GWL_DEBUG_LOG("%s() [IN]\n", __func__);

	i2c_del_driver(&bd65b60gwl_driver);
	i2c_unregister_device(client_bd65b60gwl);
	client_bd65b60gwl = NULL;

	BD65B60GWL_DEBUG_LOG("%s() [OUT]\n", __func__);
}

int32_t light_led_disp_set_panel(e_light_main_wled_disp disp_status, e_light_lcd_panel panel_class)
{
	BD65B60GWL_DEBUG_LOG("%s() [IN] panel_class=0x%x\n", __func__, panel_class);
	switch( panel_class ){
		case LIGHT_LCD_PANEL0:
			BD65B60GWL_DEBUG_LOG("%s() panel class = LIGHT_LCD_PANEL0\n", __func__);
			g_wled_brightness = wled_brightness_panel0;
			break;
		case LIGHT_LCD_PANEL1:
			BD65B60GWL_DEBUG_LOG("%s() panel class = LIGHT_LCD_PANEL1\n", __func__);
			g_wled_brightness = wled_brightness_panel1;
			break;
		case LIGHT_LCD_PANEL2:
			BD65B60GWL_DEBUG_LOG("%s() panel class = LIGHT_LCD_PANEL2\n", __func__);
			g_wled_brightness = wled_brightness_panel2;
			break;
		case LIGHT_LCD_PANEL3:
			BD65B60GWL_DEBUG_LOG("%s() panel class = LIGHT_LCD_PANEL3\n", __func__);
			g_wled_brightness = wled_brightness_panel3;
			break;
		default:
			BD65B60GWL_ERR_LOG("%s() unknown panel class\n", __func__);
			break;
	}

	return light_led_disp_set(disp_status);
}
EXPORT_SYMBOL(light_led_disp_set_panel);

int32_t light_led_disp_set(e_light_main_wled_disp disp_status)
{
	int32_t ret=0;
#ifndef DISABLE_DISP_DETECT
	e_light_main_wled_disp status;

	if( !do_disp_lock ){
		BD65B60GWL_DEBUG_LOG("mutex_init() called\n");
		mutex_init(&led_disp_lock);
		do_disp_lock = 1;
	}

	mutex_lock(&led_disp_lock);
	BD65B60GWL_DEBUG_LOG("%s(): mutex_lock \n", __func__);
	status = (e_light_main_wled_disp)atomic_read(&g_disp_status);
	BD65B60GWL_DEBUG_LOG("%s(): start status=[0x%x] disp_status=[0x%x]\n", __func__, (uint32_t)status, (uint32_t)disp_status);

	if((atomic_read(&g_display_detect)) != 0){
		mutex_unlock(&led_disp_lock);
		BD65B60GWL_DEBUG_LOG("%s(): mutex_unlock \n", __func__);
		BD65B60GWL_DEBUG_LOG("%s(): [end]Already set g_display_detect=[%d]\n", __func__, (int32_t)atomic_read(&g_display_detect));
		return ret;
	}

	switch(disp_status) {
		case LIGHT_MAIN_WLED_LCD_EN:
			BD65B60GWL_DEBUG_LOG("%s(): LIGHT_MAIN_WLED_LCD_EN\n", __func__);
			atomic_set(&g_detect_lcd_en,1);
		case LIGHT_MAIN_WLED_LED_EN:
			status |= disp_status;
			if(LIGHT_MAIN_WLED_EN == status) {
				ret = bd65b60gwl_i2c_write(client_bd65b60gwl, 0x0E, BD65B60GWL_LED_ENABLE);
				atomic_set(&g_display_detect,1);
				BD65B60GWL_DEBUG_LOG("%s(): Set display detect status=[0x%x]\n", __func__, (uint32_t)status);
			}else{
				BD65B60GWL_ERR_LOG("%s(): discard status=[0x%x] disp_status=[0x%x]\n", __func__, (uint32_t)status, (uint32_t)disp_status);
			}
			break;
		case LIGHT_MAIN_WLED_LCD_DIS:
			atomic_set(&g_display_detect,-1);
			BD65B60GWL_ERR_LOG("%s(): No set display disp_status=[0x%x]\n", __func__, (uint32_t)disp_status);
		case LIGHT_MAIN_WLED_LED_DIS:
			status &= ~(disp_status>>4);
			BD65B60GWL_ERR_LOG("%s(): LIGHT_MAIN_WLED_LED_DIS status=[0x%x]\n", __func__, (uint32_t)status);
			break;
		default:
			break;
	}
	BD65B60GWL_DEBUG_LOG("%s(): status=[0x%x] g_display_detect=[%d]\n", __func__, (uint32_t)status, (int32_t)atomic_read(&g_display_detect));
	atomic_set(&g_disp_status,(uint32_t)status);
	mutex_unlock(&led_disp_lock);
	BD65B60GWL_DEBUG_LOG("%s(): mutex_unlock\n", __func__);
	BD65B60GWL_DEBUG_LOG("%s(): end ret=[%d]\n", __func__, ret);
#else
	BD65B60GWL_INFO_LOG("%s(): DISABLE_DISP_DETECT\n", __func__);
#endif
	return ret;
}
EXPORT_SYMBOL(light_led_disp_set);

module_init(bd65b60gwl_wled_init);
module_exit(bd65b60gwl_wled_exit);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("WLED");
MODULE_LICENSE("GPL");
