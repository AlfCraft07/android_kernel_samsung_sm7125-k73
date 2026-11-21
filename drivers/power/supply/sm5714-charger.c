// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024, map220v <map220v300@gmail.com>
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define SM5714_CHG_REG_STATUS1		0x0D
#define SM5714_CHG_REG_STATUS2		0x0E
#define SM5714_CHG_REG_STATUS3		0x0F
#define SM5714_CHG_REG_STATUS4		0x10
#define SM5714_CHG_REG_STATUS5		0x11

#define SM5714_CHG_REG_CNTL1		0x13
#define SM5714_CHG_REG_VBUSCNTL		0x15
#define SM5714_CHG_REG_CHGCNTL2		0x18
#define SM5714_CHG_REG_CHGCNTL4		0x1A
#define SM5714_CHG_REG_CHGCNTL5		0x1B

#define SM5714_CHG_POLL_MS		5000

struct sm5714_charger {
	struct power_supply *psy;
	struct regmap *regmap;
	struct delayed_work poll_work;
	bool use_autostop;
	int last_online;
	int last_status;
	int last_health;
};

static enum power_supply_property sm5714_charger_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
};

static int chg_set_input_current_limit(struct sm5714_charger *charger, int mA)
{
	u8 offset;

	if (mA < 100)
		offset = 0x00;
	else
		offset = ((mA - 100) / 25) & 0x7F;

	return regmap_update_bits(charger->regmap, SM5714_CHG_REG_VBUSCNTL, 0x7F, offset);
}

static int chg_set_charging_current(struct sm5714_charger *charger, int mA)
{
	u8 offset;
	int uA;

	uA = mA * 1000;

	if (uA < 109375)		/* 109.375 mA */
		offset = 0x07;
	else if (uA > 3500000)		/* 3500.000 mA */
		offset = 0xE0;
	else
		offset = (7 + ((uA - 109375) / 15625)) & 0xFF;

	return regmap_update_bits(charger->regmap, SM5714_CHG_REG_CHGCNTL2, 0xFF, offset);
}

static int chg_set_topoff_current(struct sm5714_charger *charger, int mA)
{
	u8 offset;

	if (mA < 100)
		offset = 0x0;		/* Topoff = 100mA */
	else if (mA < 800)
		offset = (mA - 100) / 25;
	else
		offset = 0x1C;		/* Topoff = 800mA */

	return regmap_update_bits(charger->regmap, SM5714_CHG_REG_CHGCNTL5, 0x1F, offset);
}

static int sm5714_charger_read_state(struct sm5714_charger *drv,
				     int *online, int *status, int *health)
{
	unsigned int reg_st1, reg_st2;
	int error;

	error = regmap_read(drv->regmap, SM5714_CHG_REG_STATUS1, &reg_st1);
	if (error)
		return error;

	error = regmap_read(drv->regmap, SM5714_CHG_REG_STATUS2, &reg_st2);
	if (error)
		return error;

	if (online)
		*online = (reg_st1 & BIT(0)) ? 1 : 0;

	if (status) {
		if (reg_st2 & BIT(5))
			*status = POWER_SUPPLY_STATUS_FULL;
		else if (reg_st2 & BIT(3))
			*status = POWER_SUPPLY_STATUS_CHARGING;
		else if (reg_st1 & BIT(0))
			*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			*status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	if (health) {
		if (reg_st1 & BIT(0))
			*health = POWER_SUPPLY_HEALTH_GOOD;
		else if (reg_st1 & BIT(2))
			*health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else if (reg_st1 & BIT(1))
			*health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		else
			*health = POWER_SUPPLY_HEALTH_GOOD;
	}

	return 0;
}

static int sm5714_charger_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct sm5714_charger *drv = power_supply_get_drvdata(psy);
	unsigned int value;
	int online, status, health;
	int error;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		error = regmap_read(drv->regmap, SM5714_CHG_REG_STATUS2, &value);
		if (error)
			return error;
		val->intval = (value & BIT(2)) ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		error = sm5714_charger_read_state(drv, &online, NULL, NULL);
		if (error)
			return error;
		val->intval = online;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		error = sm5714_charger_read_state(drv, NULL, &status, NULL);
		if (error)
			return error;
		val->intval = status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		error = sm5714_charger_read_state(drv, NULL, NULL, &health);
		if (error)
			return error;
		val->intval = health;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void sm5714_charger_poll_work(struct work_struct *work)
{
	struct sm5714_charger *drv = container_of(work, struct sm5714_charger, poll_work.work);
	int online, status, health;

	if (sm5714_charger_read_state(drv, &online, &status, &health) == 0) {
		if (online != drv->last_online ||
		    status != drv->last_status ||
		    health != drv->last_health) {
			struct power_supply *fg_psy;

			drv->last_online = online;
			drv->last_status = status;
			drv->last_health = health;
			power_supply_changed(drv->psy);

			fg_psy = power_supply_get_by_name("sm5714_fg");
			if (fg_psy) {
				power_supply_changed(fg_psy);
				power_supply_put(fg_psy);
			}
		}
	}

	schedule_delayed_work(&drv->poll_work, msecs_to_jiffies(SM5714_CHG_POLL_MS));
}

static const struct power_supply_desc sm5714_charger_desc = {
	.name			= "sm5714_charger",
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= sm5714_charger_props,
	.num_properties		= ARRAY_SIZE(sm5714_charger_props),
	.get_property		= sm5714_charger_get_property,
};

static const struct regmap_config sm5714_charger_regmap = {
	.reg_bits	= 8,
	.val_bits	= 8,
};

static int sm5714_charger_probe(struct i2c_client *i2c)
{
	int error;
	struct device *dev = &i2c->dev;
	struct power_supply_config charger_cfg = {};
	struct sm5714_charger *drv;
	u32 input_current_limit = 500, charging_current = 500, topoff_current = 100;

	drv = devm_kzalloc(dev, sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;

	drv->last_online = -1;
	drv->last_status = -1;
	drv->last_health = -1;
	charger_cfg.drv_data = drv;
	i2c_set_clientdata(i2c, drv);

	drv->regmap = devm_regmap_init_i2c(i2c, &sm5714_charger_regmap);
	if (IS_ERR(drv->regmap))
		return PTR_ERR(drv->regmap);

	drv->use_autostop = device_property_read_bool(dev, "siliconmitus,enable-autostop");

	error = regmap_update_bits(drv->regmap, SM5714_CHG_REG_CHGCNTL4, BIT(6),
				   drv->use_autostop ? BIT(6) : 0);
	if (error)
		return dev_err_probe(dev, error, "Unable to set autostop register\n");

	device_property_read_u32(dev, "siliconmitus,input-current-limit", &input_current_limit);

	error = chg_set_input_current_limit(drv, input_current_limit);
	if (error)
		return dev_err_probe(dev, error, "Unable to set default input current limit\n");

	device_property_read_u32(dev, "siliconmitus,charging-current", &charging_current);

	error = chg_set_charging_current(drv, charging_current);
	if (error)
		return dev_err_probe(dev, error, "Unable to set default charging current\n");

	device_property_read_u32(dev, "siliconmitus,topoff-current", &topoff_current);

	error = chg_set_topoff_current(drv, topoff_current);
	if (error)
		return dev_err_probe(dev, error, "Unable to set topoff current\n");

	error = regmap_update_bits(drv->regmap, SM5714_CHG_REG_CNTL1, BIT(3), BIT(3));
	if (error)
		return dev_err_probe(dev, error, "Unable to enable charging\n");

	drv->psy = devm_power_supply_register(dev, &sm5714_charger_desc, &charger_cfg);
	if (IS_ERR(drv->psy)) {
		dev_err(dev, "failed to register power supply\n");
		return PTR_ERR(drv->psy);
	}

	INIT_DELAYED_WORK(&drv->poll_work, sm5714_charger_poll_work);
	schedule_delayed_work(&drv->poll_work, msecs_to_jiffies(1000));

	return 0;
}

static void sm5714_charger_remove(struct i2c_client *i2c)
{
	struct sm5714_charger *drv = i2c_get_clientdata(i2c);

	cancel_delayed_work_sync(&drv->poll_work);
}

static int sm5714_charger_suspend(struct device *dev)
{
	struct sm5714_charger *drv = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&drv->poll_work);
	return 0;
}

static int sm5714_charger_resume(struct device *dev)
{
	struct sm5714_charger *drv = dev_get_drvdata(dev);

	schedule_delayed_work(&drv->poll_work, msecs_to_jiffies(200));
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(sm5714_charger_pm_ops,
				sm5714_charger_suspend,
				sm5714_charger_resume);

static const struct i2c_device_id sm5714_i2c_ids[] = {
	{ "sm5714-charger", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sm5714_i2c_ids);

static const struct of_device_id sm5714_of_match_table[] = {
	{ .compatible = "siliconmitus,sm5714-charger", },
	{ },
};
MODULE_DEVICE_TABLE(of, sm5714_of_match_table);

static struct i2c_driver sm5714_charger_driver = {
	.driver = {
		.name = "sm5714-charger",
		.of_match_table = sm5714_of_match_table,
		.pm = pm_sleep_ptr(&sm5714_charger_pm_ops),
	},
	.probe		= sm5714_charger_probe,
	.remove		= sm5714_charger_remove,
	.id_table	= sm5714_i2c_ids,
};

module_i2c_driver(sm5714_charger_driver);

MODULE_DESCRIPTION("Samsung SM5714-CHARGER");
MODULE_AUTHOR("map220v <map220v300@gmail.com>");
MODULE_LICENSE("GPL");
