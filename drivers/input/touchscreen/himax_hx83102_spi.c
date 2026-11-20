// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPI touchscreen driver for Himax HX83102E (in-cell TDDI with on-chip flash)
 *
 * Modeled after upstream drivers/input/touchscreen/himax_hx83112b.c with
 * Himax SPI bus framing (0xF2 write / 0xF3 read) and HX83102E safe-mode
 * identification and 80-byte Samsung event-stack support.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define HX83102_MAX_POINTS			10
#define HX83102_INVALID_COORD			0xffff

/* Himax SPI transport framing */
#define HX83102_SPI_CMD_WRITE			0xf2
#define HX83102_SPI_CMD_READ			0xf3
#define HX83102_SPI_DUMMY_BYTE			0x00

/* Himax AHB bridge registers */
#define HX83102_AHB_ADDR_BYTE_0			0x00
#define HX83102_AHB_ADDR_RDATA_BYTE_0		0x08
#define HX83102_AHB_ADDR_ACCESS_DIR		0x0c
#define HX83102_AHB_ADDR_INCR4			0x0d
#define HX83102_AHB_ADDR_CONTI			0x13
#define HX83102_AHB_ADDR_EVENT_STACK		0x30
#define HX83102_AHB_ADDR_PSW_LB			0x31
#define HX83102_AHB_ADDR_PSW_UB			0x32

#define HX83102_AHB_CMD_ACCESS_DIR_READ		0x00
#define HX83102_AHB_CMD_INCR4_BASE		0x10
#define HX83102_AHB_CMD_CONTI			0x31
#define HX83102_SAFE_MODE_PSW_LB		0x27
#define HX83102_SAFE_MODE_PSW_UB		0x95

/* HX83102E 32-bit internal registers */
#define HX83102_REG_FW_CTRL_ISR			0x9000005c
#define HX83102_REG_CS_CENTRAL_STATE		0x900000a8
#define HX83102_REG_ICID			0x900000d0
#define HX83102_REG_TCON_RST			0x80020020
#define HX83102E_REG_RAW_OUT_SEL		0x100072ec
#define HX83102_REG_SORTING_MODE_EN		0x10007f04

#define HX83102_FW_STOP_CMD			0x000000a5
#define HX83102_FW_STOP_ACK			0x87
#define HX83102_STATE_ACTIVE			0x05
#define HX83102_STATE_SAFE_MODE			0x0c

#define HX83102_ICID_HX83102A			0x83102a
#define HX83102_ICID_HX83102B			0x83102b
#define HX83102_ICID_HX83102D			0x83102d
#define HX83102_ICID_HX83102E			0x83102e

/*
 * Standard 10-point Himax coordinate stack is 56 bytes:
 *   [0..39]  10 * (__be16 x, __be16 y)
 *   [40..49] 10 * u8 width
 *   [50..51] pad / ignore counter
 *   [52]     finger_num (0xff = all released, else & 0x0f)
 *   [53..55] state / checksum
 * Samsung HX83102E firmware appends 24 bytes (SEC_FINGER_INFO_SZ):
 *   [56..75] 10 * (u8 major, u8 minor)
 *   [76..79] extra / checksum
 */
#define HX83102_COORD_INFO_LEN			56
#define HX83102_SEC_EXTRA_LEN			24
#define HX83102_EVENT_LEN			(HX83102_COORD_INFO_LEN + HX83102_SEC_EXTRA_LEN)
#define HX83102_MAX_XFER_LEN			128

struct hx83102_spi_ts {
	struct spi_device *spi;
	struct input_dev *input_dev;
	struct gpio_desc *gpiod_rst;
	struct touchscreen_properties props;
	struct mutex spi_lock;
	u8 tx_buf[16] ____cacheline_aligned;
	u8 rx_buf[HX83102_MAX_XFER_LEN] ____cacheline_aligned;
};

static int hx83102_bus_write(struct hx83102_spi_ts *ts, u8 reg,
			     const void *data, size_t len)
{
	struct spi_transfer xfer = { };
	struct spi_message msg;
	int ret;

	if (len + 2 > sizeof(ts->tx_buf))
		return -EINVAL;

	mutex_lock(&ts->spi_lock);

	ts->tx_buf[0] = HX83102_SPI_CMD_WRITE;
	ts->tx_buf[1] = reg;
	if (len && data)
		memcpy(&ts->tx_buf[2], data, len);

	xfer.tx_buf = ts->tx_buf;
	xfer.len = len + 2;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync(ts->spi, &msg);

	mutex_unlock(&ts->spi_lock);
	return ret;
}

static int hx83102_bus_read(struct hx83102_spi_ts *ts, u8 reg,
			    void *dst, size_t len)
{
	struct spi_transfer xfer[2] = { };
	struct spi_message msg;
	int ret;

	if (len > sizeof(ts->rx_buf))
		return -EINVAL;

	mutex_lock(&ts->spi_lock);

	ts->tx_buf[0] = HX83102_SPI_CMD_READ;
	ts->tx_buf[1] = reg;
	ts->tx_buf[2] = HX83102_SPI_DUMMY_BYTE;

	xfer[0].tx_buf = ts->tx_buf;
	xfer[0].len = 3;
	xfer[1].rx_buf = ts->rx_buf;
	xfer[1].len = len;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer[0], &msg);
	spi_message_add_tail(&xfer[1], &msg);
	ret = spi_sync(ts->spi, &msg);
	if (!ret)
		memcpy(dst, ts->rx_buf, len);

	mutex_unlock(&ts->spi_lock);
	return ret;
}

static int hx83102_burst_enable(struct hx83102_spi_ts *ts, bool auto_inc)
{
	u8 val;
	int ret;

	val = HX83102_AHB_CMD_CONTI;
	ret = hx83102_bus_write(ts, HX83102_AHB_ADDR_CONTI, &val, 1);
	if (ret)
		return ret;

	val = HX83102_AHB_CMD_INCR4_BASE | (auto_inc ? 1 : 0);
	return hx83102_bus_write(ts, HX83102_AHB_ADDR_INCR4, &val, 1);
}

static int hx83102_ahb_write32(struct hx83102_spi_ts *ts, u32 addr, u32 val)
{
	u8 buf[8];

	put_unaligned_le32(addr, &buf[0]);
	put_unaligned_le32(val, &buf[4]);

	return hx83102_bus_write(ts, HX83102_AHB_ADDR_BYTE_0, buf, sizeof(buf));
}

static int hx83102_ahb_read(struct hx83102_spi_ts *ts, u32 addr,
			    void *dst, size_t len)
{
	u8 addr_buf[4];
	u8 dir = HX83102_AHB_CMD_ACCESS_DIR_READ;
	int ret;

	ret = hx83102_burst_enable(ts, len > 4);
	if (ret)
		return ret;

	put_unaligned_le32(addr, addr_buf);
	ret = hx83102_bus_write(ts, HX83102_AHB_ADDR_BYTE_0,
				addr_buf, sizeof(addr_buf));
	if (ret)
		return ret;

	ret = hx83102_bus_write(ts, HX83102_AHB_ADDR_ACCESS_DIR, &dir, 1);
	if (ret)
		return ret;

	ret = hx83102_bus_read(ts, HX83102_AHB_ADDR_RDATA_BYTE_0, dst, len);
	if (ret)
		return ret;

	if (len > 4)
		ret = hx83102_burst_enable(ts, false);

	return ret;
}

static void hx83102_reset(struct hx83102_spi_ts *ts)
{
	if (!ts->gpiod_rst)
		return;

	gpiod_set_value_cansleep(ts->gpiod_rst, 1);
	msleep(20);
	gpiod_set_value_cansleep(ts->gpiod_rst, 0);
	msleep(50);
}

static int hx83102_enter_safe_mode(struct hx83102_spi_ts *ts)
{
	u8 data[4] = { };
	u8 psw;
	int i, ret;

	for (i = 0; i < 10; i++) {
		if (i == 0 || (data[0] != 0xa5 && data[0] != 0x00 &&
			       data[0] != HX83102_FW_STOP_ACK)) {
			ret = hx83102_ahb_write32(ts, HX83102_REG_FW_CTRL_ISR,
						  HX83102_FW_STOP_CMD);
			if (ret)
				return ret;
		}
		usleep_range(10000, 11000);

		ret = hx83102_ahb_read(ts, HX83102_REG_CS_CENTRAL_STATE,
				       data, sizeof(data));
		if (ret)
			return ret;

		if (data[0] != HX83102_STATE_ACTIVE)
			break;

		ret = hx83102_ahb_read(ts, HX83102_REG_FW_CTRL_ISR,
				       data, sizeof(data));
		if (ret)
			return ret;

		if (data[0] == HX83102_FW_STOP_ACK)
			break;
	}

	for (i = 0; i < 5; i++) {
		psw = HX83102_SAFE_MODE_PSW_LB;
		ret = hx83102_bus_write(ts, HX83102_AHB_ADDR_PSW_LB, &psw, 1);
		if (ret)
			return ret;

		psw = HX83102_SAFE_MODE_PSW_UB;
		ret = hx83102_bus_write(ts, HX83102_AHB_ADDR_PSW_UB, &psw, 1);
		if (ret)
			return ret;

		ret = hx83102_ahb_read(ts, HX83102_REG_CS_CENTRAL_STATE,
				       data, sizeof(data));
		if (ret)
			return ret;

		if (data[0] == HX83102_STATE_SAFE_MODE) {
			/* Reset TCON */
			hx83102_ahb_write32(ts, HX83102_REG_TCON_RST, 0x00000000);
			usleep_range(1000, 1500);
			return 0;
		}

		usleep_range(5000, 6000);
		hx83102_reset(ts);
	}

	dev_warn(&ts->spi->dev,
		 "Failed to enter safe mode (central state=0x%02x)\n", data[0]);
	return -ETIMEDOUT;
}

static int hx83102_power_on_init(struct hx83102_spi_ts *ts)
{
	u8 dummy[4];
	u8 zero = 0;
	int ret;

	/* Clear raw-data output & sorting mode so normal coordinates are reported */
	ret = hx83102_ahb_write32(ts, HX83102E_REG_RAW_OUT_SEL, 0);
	if (ret)
		return ret;

	ret = hx83102_ahb_write32(ts, HX83102_REG_SORTING_MODE_EN, 0);
	if (ret)
		return ret;

	/* Wake AHB bridge and clear FW ISR control register */
	hx83102_bus_read(ts, HX83102_AHB_ADDR_RDATA_BYTE_0, dummy, sizeof(dummy));
	hx83102_burst_enable(ts, false);

	ret = hx83102_ahb_write32(ts, HX83102_REG_FW_CTRL_ISR, 0);
	if (ret)
		return ret;

	usleep_range(10000, 11000);

	/* Release safe mode and boot touch MCU from on-chip flash */
	if (ts->gpiod_rst) {
		hx83102_reset(ts);
	} else {
		hx83102_bus_write(ts, HX83102_AHB_ADDR_PSW_LB, &zero, 1);
		hx83102_bus_write(ts, HX83102_AHB_ADDR_PSW_UB, &zero, 1);
	}

	return 0;
}

static int hx83102_detect_chip(struct hx83102_spi_ts *ts)
{
	u8 dummy;
	u8 id_buf[4];
	u32 ic_id = 0;
	int i, ret;

	hx83102_reset(ts);

	ret = hx83102_bus_read(ts, HX83102_AHB_ADDR_CONTI, &dummy, 1);
	if (ret)
		return ret;

	/* Safe mode allows reliable IC ID read; continue even if already stopped */
	hx83102_enter_safe_mode(ts);

	for (i = 0; i < 5; i++) {
		ret = hx83102_ahb_read(ts, HX83102_REG_ICID,
				       id_buf, sizeof(id_buf));
		if (ret)
			continue;

		ic_id = ((u32)id_buf[3] << 16) | ((u32)id_buf[2] << 8) | id_buf[1];
		if (ic_id == HX83102_ICID_HX83102E ||
		    ic_id == HX83102_ICID_HX83102D ||
		    ic_id == HX83102_ICID_HX83102B ||
		    ic_id == HX83102_ICID_HX83102A) {
			dev_info(&ts->spi->dev,
				 "Detected Himax TDDI IC ID: 0x%06x\n", ic_id);
			return 0;
		}
		usleep_range(5000, 6000);
	}

	dev_err(&ts->spi->dev, "Unknown Himax IC ID: 0x%06x (raw %*ph)\n",
		ic_id, 4, id_buf);
	return -ENODEV;
}

static bool hx83102_is_esd_or_empty(const u8 *buf, size_t len)
{
	u8 first = buf[0];
	size_t i;

	if (first != 0x00 && first != 0xeb && first != 0xec && first != 0xed)
		return false;

	for (i = 1; i < len; i++)
		if (buf[i] != first)
			return false;

	return true;
}

static bool hx83102_verify_checksum(const u8 *buf, bool *has_sec_extra)
{
	u32 sum56 = 0, sum80 = 0;
	int i;

	for (i = 0; i < HX83102_COORD_INFO_LEN; i++)
		sum56 += buf[i];

	sum80 = sum56;
	for (; i < HX83102_EVENT_LEN; i++)
		sum80 += buf[i];

	if ((sum80 & 0xff) == 0) {
		*has_sec_extra = true;
		return true;
	}

	if ((sum56 & 0xff) == 0) {
		*has_sec_extra = false;
		return true;
	}

	return false;
}

static irqreturn_t hx83102_spi_irq_handler(int irq, void *dev_id)
{
	struct hx83102_spi_ts *ts = dev_id;
	u8 buf[HX83102_EVENT_LEN];
	bool has_sec_extra = false;
	int num_points, i, ret;

	ret = hx83102_bus_read(ts, HX83102_AHB_ADDR_EVENT_STACK,
			       buf, sizeof(buf));
	if (ret) {
		dev_err_ratelimited(&ts->spi->dev,
				    "Failed to read event stack: %d\n", ret);
		return IRQ_HANDLED;
	}

	if (hx83102_is_esd_or_empty(buf, HX83102_COORD_INFO_LEN))
		return IRQ_HANDLED;

	if (!hx83102_verify_checksum(buf, &has_sec_extra)) {
		dev_dbg_ratelimited(&ts->spi->dev,
				    "Event checksum mismatch: %*ph\n", 16, buf);
		return IRQ_HANDLED;
	}

	num_points = (buf[HX83102_COORD_INFO_LEN - 4] == 0xff) ?
		     0 : (buf[HX83102_COORD_INFO_LEN - 4] & 0x0f);

	for (i = 0; i < HX83102_MAX_POINTS && num_points > 0; i++) {
		u16 x = get_unaligned_be16(&buf[i * 4]);
		u16 y = get_unaligned_be16(&buf[i * 4 + 2]);
		u8 w = buf[HX83102_MAX_POINTS * 4 + i];
		u8 maj = w;
		u8 min = w;

		if (x == HX83102_INVALID_COORD || y == HX83102_INVALID_COORD)
			continue;

		if (x > ts->props.max_x || y > ts->props.max_y)
			continue;

		if (has_sec_extra) {
			u8 sec_maj = buf[HX83102_COORD_INFO_LEN + i * 2];
			u8 sec_min = buf[HX83102_COORD_INFO_LEN + i * 2 + 1];

			if (sec_maj)
				maj = sec_maj;
			if (sec_min)
				min = sec_min;
		}

		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
		touchscreen_report_pos(ts->input_dev, &ts->props, x, y, true);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, maj);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MINOR, min);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
		num_points--;
	}

	input_mt_sync_frame(ts->input_dev);
	input_sync(ts->input_dev);

	return IRQ_HANDLED;
}

static int hx83102_register_input(struct hx83102_spi_ts *ts)
{
	struct device *dev = &ts->spi->dev;
	struct input_dev *input;
	int ret;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	input->name = "Himax HX83102E Touchscreen";
	input->id.bustype = BUS_SPI;
	input->dev.parent = dev;

	input_set_capability(input, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(input, EV_ABS, ABS_MT_POSITION_Y);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	touchscreen_parse_properties(input, true, &ts->props);
	if (!ts->props.max_x || !ts->props.max_y) {
		dev_err(dev, "Missing touchscreen-size-x/y properties\n");
		return -EINVAL;
	}

	ret = input_mt_init_slots(input, HX83102_MAX_POINTS,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	ret = input_register_device(input);
	if (ret)
		return ret;

	ts->input_dev = input;
	return 0;
}

static int hx83102_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct hx83102_spi_ts *ts;
	int ret;

	if (spi->irq <= 0)
		return dev_err_probe(dev, -EINVAL, "Missing touch IRQ\n");

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->spi = spi;
	mutex_init(&ts->spi_lock);
	spi_set_drvdata(spi, ts);

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_3;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "spi_setup failed\n");

	ret = devm_regulator_get_enable_optional(dev, "vddi");
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret, "Failed to enable vddi supply\n");

	ts->gpiod_rst = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->gpiod_rst))
		return dev_err_probe(dev, PTR_ERR(ts->gpiod_rst),
				     "Failed to acquire reset GPIO\n");

	ret = hx83102_detect_chip(ts);
	if (ret)
		return ret;

	ret = hx83102_power_on_init(ts);
	if (ret)
		return dev_err_probe(dev, ret, "Power-on init failed\n");

	ret = hx83102_register_input(ts);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register input device\n");

	ret = devm_request_threaded_irq(dev, spi->irq, NULL,
					hx83102_spi_irq_handler,
					IRQF_ONESHOT, dev_name(dev), ts);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request IRQ %d\n",
				     spi->irq);

	return 0;
}

static int hx83102_spi_suspend(struct device *dev)
{
	struct hx83102_spi_ts *ts = dev_get_drvdata(dev);

	disable_irq(ts->spi->irq);
	return 0;
}

static int hx83102_spi_resume(struct device *dev)
{
	struct hx83102_spi_ts *ts = dev_get_drvdata(dev);

	enable_irq(ts->spi->irq);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(hx83102_spi_pm_ops,
				hx83102_spi_suspend, hx83102_spi_resume);

static const struct spi_device_id hx83102_spi_id[] = {
	{ "hx83102e-spi", 0 },
	{ "hx83102-spi", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, hx83102_spi_id);

static const struct of_device_id hx83102_spi_of_match[] = {
	{ .compatible = "himax,hx83102e-spi" },
	{ .compatible = "himax,hx83102-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, hx83102_spi_of_match);

static struct spi_driver hx83102_spi_driver = {
	.probe = hx83102_spi_probe,
	.id_table = hx83102_spi_id,
	.driver = {
		.name = "himax-hx83102-spi",
		.of_match_table = hx83102_spi_of_match,
		.pm = pm_sleep_ptr(&hx83102_spi_pm_ops),
	},
};
module_spi_driver(hx83102_spi_driver);

MODULE_DESCRIPTION("Himax HX83102E SPI touchscreen driver");
MODULE_LICENSE("GPL");
