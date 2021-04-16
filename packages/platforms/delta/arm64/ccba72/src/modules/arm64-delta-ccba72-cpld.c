// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * arm64-delta-ccba72-cpld.c - Read/Write ccba72 CPLD registers
 *
 * Copyright (C) 2021 Delta network Technology Corporation.
 * Chenglin Tsai <chenglin.tsai@deltaww.com>
 *
 * Based on:
 *	pca954x.c from Kumar Gala <galak@kernel.crashing.org>
 * Copyright (C) 2006
 *
 * Based on:
 *	pca954x.c from Ken Harrenstien
 * Copyright (C) 2004 Google, Inc. (Ken Harrenstien)
 *
 * Based on:
 *	i2c-virtual_cb.c from Brian Kuschak <bkuschak@yahoo.com>
 * and
 *	pca9540.c from Jean Delvare <khali@linux-fr.org>.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon-sysfs.h>
#include <linux/delay.h>

/* CPLD Register Define */
#define CCBA72_CPLD_SLAVE_ADDR		0x42

#define CCBA72_CPLD_REG_HW_VER		0x00 /* RO */
#define CCBA72_CPLD_REG_CPLD_VER	0x02 /* RO */
#define CCBA72_CPLD_REG_LED_STATUS	0x03 /* RW */
#define CCBA72_CPLD_REG_PWR_STATUS	0x04 /* RO */
#define CCBA72_CPLD_REG_PWR_CONTROL	0x05 /* RW */
#define CCBA72_CPLD_REG_INTR_INPUT	0x06 /* RC */
#define CCBA72_CPLD_REG_INTR_OUTPUT	0x07 /* RW */
#define CCBA72_CPLD_REG_RESET_OUTPUT	0x08 /* RW */
#define CCBA72_CPLD_REG_RESET_INPUT	0x09 /* RO */
#define CCBA72_CPLD_REG_OTHER_INPUT  	0x0A /* RO */
#define CCBA72_CPLD_REG_OTHER_OUTPUT  	0x0B /* RW */
#define CCBA72_CPLD_REG_WDT_FUNC	0x10 /* RW */
#define CCBA72_CPLD_REG_WDT_TIMER	0x11 /* RW */

#define CPLD_REG_BIT_LED_RED		0
#define CPLD_REG_BIT_LED_GREEN		1
#define CPLD_REG_BIT_LED_BLUE		2

#define CPLD_REG_BIT_PG_SYS		0
#define CPLD_REG_BIT_PG_ADAPTER		1
#define CPLD_REG_BIT_PG_B2B		2
#define CPLD_REG_BIT_PG_ALL		3

#define CPLD_REG_BIT_PWR_CTL_B2B	0

#define CPLD_REG_BIT_INTR_PM		0
#define CPLD_REG_BIT_INTR_TMP1075	1
#define CPLD_REG_BIT_INTR_USB		2
#define CPLD_REG_BIT_INTR_B2B_ACC_1	3
#define CPLD_REG_BIT_INTR_B2B_ACC_2	4
#define CPLD_REG_BIT_INTR_CPU		5

#define CPLD_REG_BIT_INTR_OUTPUT	0

#define CPLD_REG_BIT_RESET_CPU_88F7040	0
#define CPLD_REG_BIT_RESET_PHY_88E1512	2
#define CPLD_REG_BIT_RESET_SPI		3
#define CPLD_REG_BIT_RESET_ALL		4

#define CPLD_REG_BIT_RESET_FROM_B2B	0
#define CPLD_REG_BIT_RESET_FROM_CPU	1

#define CPLD_REG_BIT_ACS_STATUS		0
#define CPLD_REG_BIT_POE_MODE		1

#define CPLD_REG_BIT_SPI_WP		0

#define CPLD_REG_BIT_WDT_ENABLE		0
#define CPLD_REG_BIT_WDT_CLEAR		1

#define I2C_RW_RETRY_COUNT		10
#define I2C_RW_RETRY_INTERVAL		60 /* ms */

static LIST_HEAD(cpld_client_list);
static struct mutex list_lock;

struct cpld_client_node {
	struct i2c_client *client;
	struct list_head list;
};

enum cpld_type {
	ccba72_cpld
};

struct ccba72_cpld_data {
	char valid;
	struct device *hwmon_dev;
	struct mutex  update_lock;
	unsigned long last_updated; /* in jiffies */

	u8 reg_offset;
	u8 reg_data;

	/* register values */
	u8 hw_ver;		/* Reg: 0x00 RO */
	u8 cpld_ver;		/* Reg: 0x02 RO */
	u8 led_status;		/* Reg: 0x03 RW */
	u8 pwr_status;		/* Reg: 0x04 RO */
	u8 pwr_ctl;		/* Reg: 0x05 RW */
	u8 intr_input;		/* Reg: 0x06 RC */
	u8 intr_output;		/* Reg: 0x07 RW */
	u8 rst_output;		/* Reg: 0x08 RW */
	u8 rst_input;		/* Reg: 0x09 RO */
	u8 other_input;		/* Reg: 0x0A RO */
	u8 other_output;	/* Reg: 0x0B RW */
	u8 wdt_func;		/* Reg: 0x10 RW */
	u8 wdt_timer;		/* Reg: 0x11 RW */
};

static struct ccba72_cpld_data *ccba72_cpld_update_device(struct device *dev);

enum ccba72_cpld_attributes {
	HW_VERSION,
	CPLD_VERSION,
	LED_STATUS,
	WDT_TIMER
};

/*
 * Driver Data
 */
static const struct i2c_device_id ccba72_cpld_id[] = {
	{ "ccba72_cpld", ccba72_cpld },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ccba72_cpld_id);

/*
 * CPLD read/write functions
 */
static int ccba72_cpld_read_internal(struct i2c_client *client, u8 reg)
{
	int status = 0, retry = I2C_RW_RETRY_COUNT;

	while (retry) {
		status = i2c_smbus_read_byte_data(client, reg);
		if (unlikely(status < 0)) {
			msleep(I2C_RW_RETRY_INTERVAL);
			retry--;
			continue;
		}
		break;
	}

	return status;
}

static int ccba72_cpld_write_internal(struct i2c_client *client,
				      u8 reg, u8 value)
{
	int status = 0, retry = I2C_RW_RETRY_COUNT;

	while (retry) {
		status = i2c_smbus_write_byte_data(client, reg, value);
		if (unlikely(status < 0)) {
			msleep(I2C_RW_RETRY_INTERVAL);
			retry--;
			continue;
		}
		break;
	}

	return status;
}

int ccba72_cpld_read(unsigned short cpld_addr, u8 reg)
{
	struct list_head *list_node = NULL;
	struct cpld_client_node *cpld_node = NULL;
	struct ccba72_cpld_data *data;
	int ret = -EPERM;

	mutex_lock(&list_lock);

	list_for_each(list_node, &cpld_client_list)
	{
		cpld_node = list_entry(list_node,
				       struct cpld_client_node, list);

		if (cpld_node->client->addr == cpld_addr) {
			data = i2c_get_clientdata(cpld_node->client);
			mutex_lock(&data->update_lock);
			ret = ccba72_cpld_read_internal(cpld_node->client,
							reg);
			mutex_unlock(&data->update_lock);
			break;
		}
	}

	mutex_unlock(&list_lock);

	return ret;
}
EXPORT_SYMBOL(ccba72_cpld_read);

int ccba72_cpld_write(unsigned short cpld_addr, u8 reg, u8 value)
{
	struct list_head *list_node = NULL;
	struct cpld_client_node *cpld_node = NULL;
	struct ccba72_cpld_data *data;
	int ret = -EIO;

	mutex_lock(&list_lock);

	list_for_each(list_node, &cpld_client_list)
	{
		cpld_node = list_entry(list_node,
				       struct cpld_client_node, list);

		if (cpld_node->client->addr == cpld_addr) {
			data = i2c_get_clientdata(cpld_node->client);
			mutex_lock(&data->update_lock);
			ret = ccba72_cpld_write_internal(cpld_node->client,
							 reg, value);
			mutex_unlock(&data->update_lock);
			break;
		}
	}

	mutex_unlock(&list_lock);

	return ret;
}
EXPORT_SYMBOL(ccba72_cpld_write);

static inline u8 BIT_FROM_REG(int n, u8 reg)
{
	return !!(reg & BIT(n));
}

static inline u8 BIT_TO_REG(int n, u8 val, u8 reg)
{
	val = clamp_val(val, 0, 1);
	reg &= ~(BIT(n));
	reg |= (val << n);
	return reg;
}

static ssize_t offset_show(struct device *dev,
			   struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ccba72_cpld_data *data = i2c_get_clientdata(client);
	return sprintf(buf, "0x%x\n", data->reg_offset);
}

static ssize_t offset_store(struct device *dev,
			    struct device_attribute *devattr,
			    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ccba72_cpld_data *data = i2c_get_clientdata(client);

	u8 reg_offset = simple_strtoul(buf, NULL, 16);

	mutex_lock(&data->update_lock);
	data->reg_offset = reg_offset;
	mutex_unlock(&data->update_lock);

	return count;
}

static ssize_t data_show(struct device *dev,
			 struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ccba72_cpld_data *data = i2c_get_clientdata(client);
	u8 reg_data;

	mutex_lock(&data->update_lock);
	reg_data = ccba72_cpld_read_internal(client, data->reg_offset);
	mutex_unlock(&data->update_lock);

	return sprintf(buf, "0x%x\n", reg_data);
}

static ssize_t data_store(struct device *dev,
			  struct device_attribute *devattr,
			  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ccba72_cpld_data *data = i2c_get_clientdata(client);
	u8 reg_data;
	int err;

	reg_data = simple_strtoul(buf, NULL, 16);

	mutex_lock(&data->update_lock);
	err = ccba72_cpld_write_internal(client, data->reg_offset, reg_data);
	mutex_unlock(&data->update_lock);

	return err < 0 ? err : count;
}

static SENSOR_DEVICE_ATTR_RW(reg_offset, offset, 0);
static SENSOR_DEVICE_ATTR_RW(reg_data, data, 0);

static ssize_t reg_show(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct sensor_device_attribute *sda = to_sensor_dev_attr(devattr);
	struct ccba72_cpld_data *data = ccba72_cpld_update_device(dev);
	u8 reg_data;
	int res;

	switch (sda->index) {
	case HW_VERSION:
		reg_data = data->hw_ver & 0x0f;
		res = sprintf(buf, "0x%x\n", reg_data);
		break;
	case CPLD_VERSION:
		reg_data = data->cpld_ver & 0xff;
		res = sprintf(buf, "0x%x\n", reg_data);
		break;
	case LED_STATUS:
		reg_data = data->led_status & 0x07;
		res = sprintf(buf, "%d\n", reg_data);
		break;
	case WDT_TIMER:
		reg_data = data->wdt_timer & 0xff;
		res = sprintf(buf, "0x%x\n", reg_data);
		break;
	default:
		dev_err(dev, "Unknown case %d in reg_show\n", sda->index);
		return -EINVAL;
	}

	return res;
}

static ssize_t reg_store(struct device *dev,
			 struct device_attribute *devattr,
			 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_device_attribute *sda = to_sensor_dev_attr(devattr);
	struct ccba72_cpld_data *data = ccba72_cpld_update_device(dev);
	u8 reg_offset;
	u8 reg_data;
	int err;

	err = kstrtou8(buf, 0, &reg_data);
	if (err < 0)
		return err;

	switch (sda->index) {
	case LED_STATUS:
		reg_offset = CCBA72_CPLD_REG_LED_STATUS;
		break;
	case WDT_TIMER:
		reg_offset = CCBA72_CPLD_REG_WDT_TIMER;
		break;
	default:
		dev_err(dev, "Unknown case %d in reg_store\n", sda->index);
		return -EINVAL;
	}

	mutex_lock(&data->update_lock);
	err = ccba72_cpld_write_internal(client, reg_offset, reg_data);
	mutex_unlock(&data->update_lock);

	return err < 0 ? err : count;
}

/* HW Version, Register 0x00, Read Only */
static SENSOR_DEVICE_ATTR_RO(hw_version, reg, HW_VERSION);
/* CPLD Version, Register 0x02, Read Only */
static SENSOR_DEVICE_ATTR_RO(cpld_version, reg, CPLD_VERSION);
/* LED Status, Register 0x03, Read and Write */
static SENSOR_DEVICE_ATTR_RW(led_status, reg, LED_STATUS);
/* WDT Timer, Register 0x11, Read and Write */
static SENSOR_DEVICE_ATTR_RW(wdt_timer, reg, WDT_TIMER);

static ssize_t bit_show(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct sensor_device_attribute_2 *sda = to_sensor_dev_attr_2(devattr);
	struct ccba72_cpld_data *data = ccba72_cpld_update_device(dev);
	u8 reg_offset = sda->nr;
	u8 bit_num = sda->index;
	u8 reg_data;

	switch (reg_offset) {
	case CCBA72_CPLD_REG_PWR_STATUS:
		reg_data = data->pwr_status;
		break;
	case CCBA72_CPLD_REG_PWR_CONTROL:
		reg_data = data->pwr_ctl;
		break;
	case CCBA72_CPLD_REG_INTR_INPUT:
		reg_data = data->intr_input;
		break;
	case CCBA72_CPLD_REG_INTR_OUTPUT:
		reg_data = data->intr_output;
		break;
	case CCBA72_CPLD_REG_RESET_OUTPUT:
		reg_data = data->rst_output;
		break;
	case CCBA72_CPLD_REG_RESET_INPUT:
		reg_data = data->rst_input;
		break;
	case CCBA72_CPLD_REG_OTHER_INPUT:
		reg_data = data->other_input;
		break;
	case CCBA72_CPLD_REG_OTHER_OUTPUT:
		reg_data = data->other_output;
		break;
	case CCBA72_CPLD_REG_WDT_FUNC:
		reg_data = data->wdt_func;
		break;
	case CCBA72_CPLD_REG_WDT_TIMER:
		reg_data = data->wdt_timer;
		break;
	default:
		dev_err(dev, "Unknown offset 0x%x in bit_show\n", reg_offset);
		return -EINVAL;
	}
	return sprintf(buf, "%d\n", BIT_FROM_REG(bit_num, reg_data));
}

static ssize_t bit_store(struct device *dev,
			 struct device_attribute *devattr,
			 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_device_attribute_2 *sda = to_sensor_dev_attr_2(devattr);
	struct ccba72_cpld_data *data = ccba72_cpld_update_device(dev);
	u8 reg_offset = sda->nr;
	u8 bit_num = sda->index;
	u8 reg_data;
	u8 val;
	int err;

	err = kstrtou8(buf, 0, &val);
	if (err < 0)
		return err;

	switch (reg_offset) {
	case CCBA72_CPLD_REG_PWR_STATUS:
		reg_data = data->pwr_status;
		break;
	case CCBA72_CPLD_REG_PWR_CONTROL:
		reg_data = data->pwr_ctl;
		break;
	case CCBA72_CPLD_REG_INTR_INPUT:
		reg_data = data->intr_input;
		break;
	case CCBA72_CPLD_REG_INTR_OUTPUT:
		reg_data = data->intr_output;
		break;
	case CCBA72_CPLD_REG_RESET_OUTPUT:
		reg_data = data->rst_output;
		break;
	case CCBA72_CPLD_REG_RESET_INPUT:
		reg_data = data->rst_input;
		break;
	case CCBA72_CPLD_REG_OTHER_INPUT:
		reg_data = data->other_input;
		break;
	case CCBA72_CPLD_REG_OTHER_OUTPUT:
		reg_data = data->other_output;
		break;
	case CCBA72_CPLD_REG_WDT_FUNC:
		reg_data = data->wdt_func;
		break;
	case CCBA72_CPLD_REG_WDT_TIMER:
		reg_data = data->wdt_timer;
		break;
	default:
		dev_err(dev, "Unknown offset 0x%x in bit_store\n", reg_offset);
		return -EINVAL;
	}

	reg_data = BIT_TO_REG(bit_num, val, reg_data);

	mutex_lock(&data->update_lock);
	err = ccba72_cpld_write_internal(client, reg_offset, reg_data);
	mutex_unlock(&data->update_lock);

	return err < 0 ? err : count;
}

/* Power Status, Register 0x04, Read Only */
static SENSOR_DEVICE_ATTR_2_RO(powergood_sys, bit,
			       CCBA72_CPLD_REG_PWR_STATUS,
			       CPLD_REG_BIT_PG_SYS);
static SENSOR_DEVICE_ATTR_2_RO(powergood_adapter, bit,
			       CCBA72_CPLD_REG_PWR_STATUS,
			       CPLD_REG_BIT_PG_ADAPTER);
static SENSOR_DEVICE_ATTR_2_RO(powergood_b2b, bit,
			       CCBA72_CPLD_REG_PWR_STATUS,
			       CPLD_REG_BIT_PG_B2B);
static SENSOR_DEVICE_ATTR_2_RO(powergood_all, bit,
			       CCBA72_CPLD_REG_PWR_STATUS,
			       CPLD_REG_BIT_PG_ALL);

/* Power Control, Register 0x05, Read and Write */
static SENSOR_DEVICE_ATTR_2_RW(power_control_b2b, bit,
			       CCBA72_CPLD_REG_PWR_CONTROL,
			       CPLD_REG_BIT_PWR_CTL_B2B);

/* Interrupt Input, Register 0x06, Read Clear */
static SENSOR_DEVICE_ATTR_2_RO(intr_pm, bit,
			       CCBA72_CPLD_REG_INTR_INPUT,
			       CPLD_REG_BIT_INTR_PM);
static SENSOR_DEVICE_ATTR_2_RO(intr_tmp1075, bit,
			       CCBA72_CPLD_REG_INTR_INPUT,
			       CPLD_REG_BIT_INTR_TMP1075);
static SENSOR_DEVICE_ATTR_2_RO(intr_usb, bit,
			       CCBA72_CPLD_REG_INTR_INPUT,
			       CPLD_REG_BIT_INTR_USB);
static SENSOR_DEVICE_ATTR_2_RO(intr_b2b_acc_1, bit,
			       CCBA72_CPLD_REG_INTR_INPUT,
			       CPLD_REG_BIT_INTR_B2B_ACC_1);
static SENSOR_DEVICE_ATTR_2_RO(intr_b2b_acc_2, bit,
			       CCBA72_CPLD_REG_INTR_INPUT,
			       CPLD_REG_BIT_INTR_B2B_ACC_2);
static SENSOR_DEVICE_ATTR_2_RO(intr_cpu, bit,
			       CCBA72_CPLD_REG_INTR_INPUT,
			       CPLD_REG_BIT_INTR_CPU);

/* Interrupt Output, Register 0x07, Read and Write */
static SENSOR_DEVICE_ATTR_2_RW(intr_output, bit,
			       CCBA72_CPLD_REG_INTR_OUTPUT,
			       CPLD_REG_BIT_INTR_OUTPUT);

/* Reset Output, Register 0x08, Read and Write */
static SENSOR_DEVICE_ATTR_2_RW(rst_cpu, bit,
			       CCBA72_CPLD_REG_RESET_OUTPUT,
			       CPLD_REG_BIT_RESET_CPU_88F7040);
static SENSOR_DEVICE_ATTR_2_RW(rst_phy, bit,
			       CCBA72_CPLD_REG_RESET_OUTPUT,
			       CPLD_REG_BIT_RESET_PHY_88E1512);
static SENSOR_DEVICE_ATTR_2_RW(rst_spi, bit,
			       CCBA72_CPLD_REG_RESET_OUTPUT,
			       CPLD_REG_BIT_RESET_SPI);
static SENSOR_DEVICE_ATTR_2_RW(rst_all, bit,
			       CCBA72_CPLD_REG_RESET_OUTPUT,
			       CPLD_REG_BIT_RESET_ALL);

/* Reset Input, Register 0x09, Read Only */
static SENSOR_DEVICE_ATTR_2_RO(rst_from_b2b, bit,
			       CCBA72_CPLD_REG_RESET_INPUT,
			       CPLD_REG_BIT_RESET_FROM_B2B);
static SENSOR_DEVICE_ATTR_2_RO(rst_from_cpu, bit,
			       CCBA72_CPLD_REG_RESET_INPUT,
			       CPLD_REG_BIT_RESET_FROM_CPU);

/* Other Input, Register 0x0A, Read Only */
static SENSOR_DEVICE_ATTR_2_RO(acs_status, bit,
			       CCBA72_CPLD_REG_OTHER_INPUT,
			       CPLD_REG_BIT_ACS_STATUS);
static SENSOR_DEVICE_ATTR_2_RO(poe_mode, bit,
			       CCBA72_CPLD_REG_OTHER_INPUT,
			       CPLD_REG_BIT_POE_MODE);

/* Other Output, Register 0x0B, Read and Write */
static SENSOR_DEVICE_ATTR_2_RW(spi_wp, bit,
			       CCBA72_CPLD_REG_OTHER_OUTPUT,
			       CPLD_REG_BIT_SPI_WP);

/* WDT Function, Register 0x10, Read and Write */
static SENSOR_DEVICE_ATTR_2_RW(wdt_disable, bit,
			       CCBA72_CPLD_REG_WDT_FUNC,
			       CPLD_REG_BIT_WDT_ENABLE);
static SENSOR_DEVICE_ATTR_2_RW(wdt_clear, bit,
			       CCBA72_CPLD_REG_WDT_FUNC,
			       CPLD_REG_BIT_WDT_CLEAR);

static struct attribute *ccba72_cpld_attributes[] = {
	&sensor_dev_attr_reg_offset.dev_attr.attr,
	&sensor_dev_attr_reg_data.dev_attr.attr,
	&sensor_dev_attr_hw_version.dev_attr.attr,
	&sensor_dev_attr_cpld_version.dev_attr.attr,
	&sensor_dev_attr_led_status.dev_attr.attr,
	&sensor_dev_attr_powergood_sys.dev_attr.attr,
	&sensor_dev_attr_powergood_adapter.dev_attr.attr,
	&sensor_dev_attr_powergood_b2b.dev_attr.attr,
	&sensor_dev_attr_powergood_all.dev_attr.attr,
	&sensor_dev_attr_power_control_b2b.dev_attr.attr,
	&sensor_dev_attr_intr_pm.dev_attr.attr,
	&sensor_dev_attr_intr_tmp1075.dev_attr.attr,
	&sensor_dev_attr_intr_usb.dev_attr.attr,
	&sensor_dev_attr_intr_b2b_acc_1.dev_attr.attr,
	&sensor_dev_attr_intr_b2b_acc_2.dev_attr.attr,
	&sensor_dev_attr_intr_cpu.dev_attr.attr,
	&sensor_dev_attr_intr_output.dev_attr.attr,
	&sensor_dev_attr_rst_cpu.dev_attr.attr,
	&sensor_dev_attr_rst_phy.dev_attr.attr,
	&sensor_dev_attr_rst_spi.dev_attr.attr,
	&sensor_dev_attr_rst_all.dev_attr.attr,
	&sensor_dev_attr_rst_from_b2b.dev_attr.attr,
	&sensor_dev_attr_rst_from_cpu.dev_attr.attr,
	&sensor_dev_attr_acs_status.dev_attr.attr,
	&sensor_dev_attr_poe_mode.dev_attr.attr,
	&sensor_dev_attr_spi_wp.dev_attr.attr,
	&sensor_dev_attr_wdt_disable.dev_attr.attr,
	&sensor_dev_attr_wdt_clear.dev_attr.attr,
	&sensor_dev_attr_wdt_timer.dev_attr.attr,
	NULL
};

static const struct attribute_group ccba72_cpld_group = {
	.attrs = ccba72_cpld_attributes,
};

static void ccba72_cpld_add_client(struct i2c_client *client)
{
	struct cpld_client_node *node =
			kzalloc(sizeof(struct cpld_client_node), GFP_KERNEL);

	if (!node) {
		dev_dbg(&client->dev,
			"Can't allocate cpld_client_node (0x%x)\n",
			client->addr);
		return;
	}

	node->client = client;

	mutex_lock(&list_lock);
	list_add(&node->list, &cpld_client_list);
	mutex_unlock(&list_lock);
}

static void ccba72_cpld_remove_client(struct i2c_client *client)
{
	struct list_head *list_node = NULL;
	struct cpld_client_node *cpld_node = NULL;
	int found = 0;

	mutex_lock(&list_lock);

	list_for_each(list_node, &cpld_client_list)
	{
		cpld_node = list_entry(list_node,
				       struct cpld_client_node, list);

		if (cpld_node->client == client) {
			found = 1;
			break;
		}
	}

	if (found) {
		list_del(list_node);
		kfree(cpld_node);
	}

	mutex_unlock(&list_lock);
}

/* I2C Probe/Remove functions */
static int ccba72_cpld_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct i2c_adapter *adap = to_i2c_adapter(client->dev.parent);
	struct ccba72_cpld_data *data;
	int ret = -ENODEV;

	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_BYTE))
		goto exit;

	data = kzalloc(sizeof(struct ccba72_cpld_data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);
	mutex_init(&data->update_lock);
	data->reg_offset = 0x00;
	data->reg_data   = 0x00;

	ret = sysfs_create_group(&client->dev.kobj, &ccba72_cpld_group);
	if (ret)
		goto exit_free;

	ccba72_cpld_add_client(client);
	return 0;

exit_free:
	kfree(data);
exit:
	return ret;
}

static int ccba72_cpld_remove(struct i2c_client *client)
{
	struct ccba72_cpld_data *data = i2c_get_clientdata(client);

	ccba72_cpld_remove_client(client);

	/* Remove sysfs hooks */
	sysfs_remove_group(&client->dev.kobj, &ccba72_cpld_group);

	kfree(data);

	return 0;
}

static struct i2c_driver ccba72_cpld_driver = {
	.driver = {
		.name  = "arm64_delta_ccba72_cpld",
		.owner = THIS_MODULE,
	},
	.probe = ccba72_cpld_probe,
	.remove	= ccba72_cpld_remove,
	.id_table = ccba72_cpld_id,
};

static struct ccba72_cpld_data *ccba72_cpld_update_device(
						struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ccba72_cpld_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->update_lock);

	if(time_after(jiffies, data->last_updated + HZ) || !data->valid) {
		data->hw_ver = ccba72_cpld_read_internal(client,
					CCBA72_CPLD_REG_HW_VER);
		data->cpld_ver = ccba72_cpld_read_internal(client,
					CCBA72_CPLD_REG_CPLD_VER);
		data->led_status = ccba72_cpld_read_internal(client,
					CCBA72_CPLD_REG_LED_STATUS);
		data->pwr_status = ccba72_cpld_read_internal(client,
					CCBA72_CPLD_REG_PWR_STATUS);
		data->pwr_ctl = ccba72_cpld_read_internal(client,
					CCBA72_CPLD_REG_PWR_CONTROL);
		data->intr_input = ccba72_cpld_read_internal(client,
					CCBA72_CPLD_REG_INTR_INPUT);
		data->intr_output = ccba72_cpld_read_internal(client,
					CCBA72_CPLD_REG_INTR_OUTPUT);
		data->rst_output = ccba72_cpld_read_internal(client,
					CCBA72_CPLD_REG_RESET_OUTPUT);
		data->rst_input = ccba72_cpld_read_internal(client,
					CCBA72_CPLD_REG_RESET_INPUT);
		data->other_input = ccba72_cpld_read_internal(client,
					CCBA72_CPLD_REG_OTHER_INPUT);
		data->other_output = ccba72_cpld_read_internal(client,
					CCBA72_CPLD_REG_OTHER_OUTPUT);
		data->wdt_func = ccba72_cpld_read_internal(client,
					CCBA72_CPLD_REG_WDT_FUNC);
		data->wdt_timer = ccba72_cpld_read_internal(client,
					CCBA72_CPLD_REG_WDT_TIMER);
		data->last_updated = jiffies;
		data->valid = 1;
	}

	mutex_unlock(&data->update_lock);

	return data;
}

/* I2C init/exit functions */
static int __init ccba72_cpld_init(void)
{
	mutex_init(&list_lock);
	return i2c_add_driver(&ccba72_cpld_driver);
}

static void __exit ccba72_cpld_exit(void)
{
	i2c_del_driver(&ccba72_cpld_driver);
}

MODULE_AUTHOR("Chenglin Tsai <chenglin.tsai@deltaww.com>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("CCBA72 I2C CPLD driver");

module_init(ccba72_cpld_init);
module_exit(ccba72_cpld_exit);
