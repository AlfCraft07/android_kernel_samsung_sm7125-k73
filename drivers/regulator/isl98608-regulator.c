// SPDX-License-Identifier: GPL-2.0-only
/*
 * Regulator driver for Intersil ISL98608/ISL98609 dual-rail display bias PMIC.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define ISL98608_REG_VBST	0x06
#define ISL98608_REG_VP		0x08
#define ISL98608_REG_VN		0x09

#define ISL98608_VAL_5P5V	0x08

enum isl98608_regulators {
	ISL98608_AVDD = 0,
	ISL98608_AVEE,
	ISL98608_NUM_REGULATORS,
};

struct isl98608_data {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *enable_gpio;
	struct mutex lock;
	unsigned int enable_mask;
};

static int isl98608_program_rails(struct isl98608_data *data)
{
	int ret;

	ret = regmap_write(data->regmap, ISL98608_REG_VBST, ISL98608_VAL_5P5V);
	if (ret)
		return ret;

	ret = regmap_write(data->regmap, ISL98608_REG_VP, ISL98608_VAL_5P5V);
	if (ret)
		return ret;

	return regmap_write(data->regmap, ISL98608_REG_VN, ISL98608_VAL_5P5V);
}

static int isl98608_enable(struct regulator_dev *rdev)
{
	struct isl98608_data *data = rdev_get_drvdata(rdev);
	unsigned int bit = BIT(rdev_get_id(rdev));
	int ret = 0;

	mutex_lock(&data->lock);
	if (!data->enable_mask) {
		if (data->enable_gpio) {
			gpiod_set_value_cansleep(data->enable_gpio, 1);
			usleep_range(2000, 3000);
		}
		ret = isl98608_program_rails(data);
		if (ret) {
			if (data->enable_gpio)
				gpiod_set_value_cansleep(data->enable_gpio, 0);
			mutex_unlock(&data->lock);
			return ret;
		}
	}
	data->enable_mask |= bit;
	mutex_unlock(&data->lock);

	return 0;
}

static int isl98608_disable(struct regulator_dev *rdev)
{
	struct isl98608_data *data = rdev_get_drvdata(rdev);
	unsigned int bit = BIT(rdev_get_id(rdev));

	mutex_lock(&data->lock);
	data->enable_mask &= ~bit;
	if (!data->enable_mask && data->enable_gpio)
		gpiod_set_value_cansleep(data->enable_gpio, 0);
	mutex_unlock(&data->lock);

	return 0;
}

static int isl98608_is_enabled(struct regulator_dev *rdev)
{
	struct isl98608_data *data = rdev_get_drvdata(rdev);

	return !!(data->enable_mask & BIT(rdev_get_id(rdev)));
}

static const struct regulator_ops isl98608_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.enable = isl98608_enable,
	.disable = isl98608_disable,
	.is_enabled = isl98608_is_enabled,
};

static const struct regulator_desc isl98608_descs[ISL98608_NUM_REGULATORS] = {
	[ISL98608_AVDD] = {
		.name = "avdd",
		.of_match = of_match_ptr("avdd"),
		.regulators_node = of_match_ptr("regulators"),
		.id = ISL98608_AVDD,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.ops = &isl98608_ops,
		.min_uV = 5500000,
		.n_voltages = 1,
	},
	[ISL98608_AVEE] = {
		.name = "avee",
		.of_match = of_match_ptr("avee"),
		.regulators_node = of_match_ptr("regulators"),
		.id = ISL98608_AVEE,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.ops = &isl98608_ops,
		.min_uV = 5500000,
		.n_voltages = 1,
	},
};

static const struct regmap_config isl98608_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = ISL98608_REG_VN,
};

static int isl98608_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config config = {};
	struct isl98608_data *data;
	int i;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	mutex_init(&data->lock);

	data->regmap = devm_regmap_init_i2c(client, &isl98608_regmap_config);
	if (IS_ERR(data->regmap))
		return dev_err_probe(dev, PTR_ERR(data->regmap),
				     "Failed to initialize regmap\n");

	data->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(data->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(data->enable_gpio),
				     "Failed to get enable GPIO\n");

	config.dev = dev;
	config.driver_data = data;
	config.regmap = data->regmap;

	for (i = 0; i < ISL98608_NUM_REGULATORS; i++) {
		struct regulator_dev *rdev;

		rdev = devm_regulator_register(dev, &isl98608_descs[i], &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "Failed to register regulator %s\n",
					     isl98608_descs[i].name);
	}

	return 0;
}

static const struct of_device_id isl98608_of_match[] = {
	{ .compatible = "intersil,isl98608" },
	{ .compatible = "intersil,isl98609" },
	{ }
};
MODULE_DEVICE_TABLE(of, isl98608_of_match);

static const struct i2c_device_id isl98608_id[] = {
	{ "isl98608" },
	{ "isl98609" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, isl98608_id);

static struct i2c_driver isl98608_driver = {
	.driver = {
		.name = "isl98608-regulator",
		.of_match_table = isl98608_of_match,
	},
	.probe = isl98608_probe,
	.id_table = isl98608_id,
};
module_i2c_driver(isl98608_driver);

MODULE_DESCRIPTION("Intersil ISL98608/ISL98609 display bias regulator driver");
MODULE_LICENSE("GPL");
