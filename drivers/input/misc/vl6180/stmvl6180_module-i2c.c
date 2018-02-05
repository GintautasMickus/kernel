/*
 *  stmvl6180.c - Linux kernel modules for STM VL6180 FlightSense TOF sensor
 *
 *  Copyright (c) 2017, Fuzhou Rockchip Electronics Co., Ltd
 *  Copyright (c) 2014 STMicroelectronics Imaging Division.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */
 
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
/*
 * power specific includes
 */
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include <linux/of.h>

/*
 * API includes
 */
#include "vl6180x_api.h"
#include "vl6180x_def.h"
#include "vl6180x_platform.h"
#include "vl6180x_i2c.h"
#include "stmvl6180-i2c.h"
#include "stmvl6180.h"

static int stmvl6180_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int rc = 0;
	struct stmvl6180_data *vl6180_data = NULL;
	struct i2c_data *data = NULL;
	struct device_node *np = client->dev.of_node;
	unsigned long irq_flags;

	vl6180_dbgmsg("Enter\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		rc = -EIO;
		vl6180_errmsg("i2c_check_functionality error!\n");
		return rc;
	}

	vl6180_data = stmvl6180_getobject();
	if (vl6180_data)
		data = &vl6180_data->client_object;
	data->client = client;

	vl6180_data->power_pin = of_get_named_gpio(np, "power-gpio", 0);

	if (gpio_is_valid(vl6180_data->power_pin)) {
		vl6180_dbgmsg("set power-pin = %d\n", vl6180_data->power_pin);
		if (devm_gpio_request(&client->dev, vl6180_data->power_pin, "power-gpio")) {
			vl6180_errmsg("failed to request GPIO%d for otg_drv\n", vl6180_data->power_pin);
			return -EINVAL;
		}
		gpio_direction_output(vl6180_data->power_pin, 0);
	}
	if (gpio_is_valid(vl6180_data->power_pin))
		gpio_set_value(vl6180_data->power_pin, 1);

	vl6180_data->irq_pin = of_get_named_gpio_flags(np, "irq-gpio", 0, (enum of_gpio_flags *)&irq_flags);
	vl6180_data->irq_flags = irq_flags;
	of_property_read_u32(np, "irq_enable", &vl6180_data->irq_enable);
	vl6180_dbgmsg("irq_enable = %d irq_pin=%d\n", vl6180_data->irq_enable, vl6180_data->irq_pin);

	/* setup client data */
	i2c_set_clientdata(client, vl6180_data);

	/* setup platform i2c client */
	i2c_setclient((void *)client);

	vl6180_dbgmsg("End\n");
	return rc;
}

static int stmvl6180_remove(struct i2c_client *client)
{
	struct stmvl6180_data *data = stmvl6180_getobject();
	vl6180_dbgmsg("Enter\n");

	/* Power down the device */
	stmvl6180_power_down_i2c((void *)&data->client_object);

	vl6180_dbgmsg("End\n");
	return 0;
}

#ifdef CONFIG_PM
static int stmvl6180_set_enable(struct i2c_client *client, unsigned int enable)
{
	return 0;
}

static int stmvl6180_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return stmvl6180_set_enable(client, 0);
}

static int stmvl6180_resume(struct i2c_client *client)
{
	return stmvl6180_set_enable(client, 1);
}

#else

#define stmvl6180_suspend	NULL
#define stmvl6180_resume	NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id stmvl6180_id[] = {
	{ STMVL6180_DRV_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, stmvl6180_id);

static const struct of_device_id st_stmv16180_dt_match[] = {
	{ .compatible = "st,stmvl6180", },
	{ },
};

static struct i2c_driver stmvl6180_driver = {
	.driver = {
		.name	= STMVL6180_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = st_stmv16180_dt_match,
	},
	.suspend = stmvl6180_suspend,
	.resume	= stmvl6180_resume,
	.probe	= stmvl6180_probe,
	.remove	= stmvl6180_remove,
	.id_table = stmvl6180_id,

};

int stmvl6180_power_up_i2c(void *i2c_object, unsigned int *preset_flag)
{
	int ret = 0;
	struct i2c_data *data = (struct i2c_data *)i2c_object;

	vl6180_dbgmsg("Enter\n");

	data->power_up = 1;
	*preset_flag = 1;

	vl6180_dbgmsg("End\n");
	return ret;
}

int stmvl6180_power_down_i2c(void *i2c_object)
{
	int ret = 0;
	struct i2c_data *data = (struct i2c_data *)i2c_object;

	vl6180_dbgmsg("Enter\n");

	data->power_up = 0;

	vl6180_dbgmsg("End\n");
	return ret;
}

int stmvl6180_init_i2c(void)
{
	int ret = 0;

	vl6180_dbgmsg("Enter\n");

	/* register as a i2c client device */
	ret = i2c_add_driver(&stmvl6180_driver);
	if (ret)
		vl6180_errmsg("%d erro ret:%d\n", __LINE__, ret);

	vl6180_dbgmsg("End with rc:%d\n", ret);

	return ret;
}

void stmvl6180_exit_i2c(void *i2c_object)
{
	vl6180_dbgmsg("Enter\n");
	i2c_del_driver(&stmvl6180_driver);

	vl6180_dbgmsg("End\n");
}

