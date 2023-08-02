// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices Generic AXI ADC IP core
 * Link: https://wiki.analog.com/resources/fpga/docs/axi_adc_ip
 *
 * Copyright 2012-2023 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/limits.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <linux/fpga/adi-axi-common.h>
#include <linux/iio/addc/converter.h>

/*
 * Register definitions:
 *   https://wiki.analog.com/resources/fpga/docs/axi_adc_ip#register_map
 */

/* ADC systhesis configuration*/
#define AXI_ADC_REG_CONFIG			0x000c
#define   AXI_ADC_DATAFORMAT_DISABLE_MASK	BIT(2)
/* ADC controls */
#define AXI_ADC_REG_RSTN			0x0040
#define   AXI_ADC_RSTN_RESET_MASK		GENMASK(1, 0)
#define   AXI_ADC_RSTN_MMCM_RSTN		BIT(1)
#define   AXI_ADC_RSTN_RSTN			BIT(0)

/* ADC Channel controls */
#define AXI_ADC_REG_CHAN_CTRL(c)		(0x0400 + (c) * 0x40)
#define   AXI_ADC_CHAN_CTRL_FMT_MASK		GENMASK(6, 4)
#define   AXI_ADC_CHAN_CTRL_FMT_EN		BIT(0)
#define   AXI_ADC_CHAN_CTRL_FMT_BIN_OFF		BIT(1)
#define   AXI_ADC_CHAN_CTRL_FMT_SIGEXT		BIT(2)
#define   AXI_ADC_CHAN_CTRL_EN_MASK		BIT(0)

#define AXI_ADC_REG_CHAN_CTRL_3(c)		(0x0418 + (c) * 0x40)
#define   AXI_ADC_CHAN_PN_SEL_MASK		GENMASK(19, 16)

/* IO Delays */
#define AXI_ADC_REG_DELAY(l)			(0x0800 + (l) * 0x4)
#define   AXI_ADC_DELAY_CTRL_MASK		GENMASK(4, 0)

enum {
	AXI_ADC_PN9A,
	AXI_ADC_PN23A,
	AXI_ADC_PN7 = 0x4,
	AXI_ADC_PN15,
	AXI_ADC_PN23,
	AXI_ADC_PN31,
	AXI_ADC_PNX = 0x9,
	AXI_ADC_RAMP_NIBBLE,
	AXI_ADC_RAMP_16,
};

struct axi_adc_state {
	struct regmap *regmap;
	/* Protect against concurrent access to the device registers */
	struct mutex lock;
	struct device *dev;
	u32 capabilities;
};

static int axi_adc_iodelay_set(struct converter_backend *conv,
			       unsigned int num_lanes, unsigned int delay)
{
	struct axi_adc_state *st = converter_get_drvdata(conv);
	unsigned int l, val;
	int ret;

	if (delay > FIELD_MAX(AXI_ADC_DELAY_CTRL_MASK))
		return -EINVAL;

	mutex_lock(&st->lock);
	for (l = 0; l < num_lanes; l++) {
		ret = regmap_update_bits(st->regmap, AXI_ADC_REG_DELAY(l),
					 AXI_ADC_DELAY_CTRL_MASK, delay);
		if (ret)
			break;
		/*
		 * If a readback is ~0, that means there are issues with the
		 * delay_clk
		 */
		ret = regmap_read(st->regmap, AXI_ADC_REG_DELAY(l), &val);
		if (val == U32_MAX) {
			ret = -EIO;
			break;
		}
	}
	mutex_unlock(&st->lock);

	return ret;
}

static const struct converter_test_pattern_xlate axi_adc_test_pattern[] = {
	{CONVERTER_PRBS_7, AXI_ADC_PN7},
	{CONVERTER_PRBS_15, AXI_ADC_PN15},
	{CONVERTER_PRBS_15, AXI_ADC_PN15},
	{CONVERTER_PRBS_23, AXI_ADC_PN23},
	{CONVERTER_PRBS_31, AXI_ADC_PN31},
	{CONVERTER_ADI_PRBS_9A, AXI_ADC_PN9A},
	{CONVERTER_ADI_PRBS_23A, AXI_ADC_PN23A},
	{CONVERTER_ADI_PRBS_X, AXI_ADC_PNX},
	{CONVERTER_RAMP_NIBBLE, AXI_ADC_RAMP_NIBBLE},
	{CONVERTER_RAMP_16, AXI_ADC_RAMP_16},
};

static int axi_adc_test_pattern_set(struct converter_backend *conv,
				    unsigned int chan,
				    enum converter_test_pattern pattern)
{
	struct axi_adc_state *st = converter_get_drvdata(conv);
	int val;

	val = converter_test_pattern_xlate(pattern, axi_adc_test_pattern);
	if (val < 0)
		return val;

	return regmap_update_bits(st->regmap, AXI_ADC_REG_CHAN_CTRL_3(chan),
				  AXI_ADC_CHAN_PN_SEL_MASK,
				  FIELD_PREP(AXI_ADC_CHAN_PN_SEL_MASK, val));
}

static int axi_adc_chan_enable(struct converter_backend *conv,
			       unsigned int chan)
{
	struct axi_adc_state *st = converter_get_drvdata(conv);

	return regmap_set_bits(st->regmap, AXI_ADC_REG_CHAN_CTRL(chan),
			       AXI_ADC_CHAN_CTRL_EN_MASK);
}

static int axi_adc_chan_disable(struct converter_backend *conv,
				unsigned int chan)
{
	struct axi_adc_state *st = converter_get_drvdata(conv);

	return regmap_clear_bits(st->regmap, AXI_ADC_REG_CHAN_CTRL(chan),
				 AXI_ADC_CHAN_CTRL_EN_MASK);
}

static int axi_adc_data_format_set(struct converter_backend *conv,
				   unsigned int chan,
				   const struct converter_data_fmt *data)
{
	struct axi_adc_state *st = converter_get_drvdata(conv);
	u32 val = 0;

	if (FIELD_GET(AXI_ADC_DATAFORMAT_DISABLE_MASK, st->capabilities))
		/* data format not available */
		return -ENOTSUPP;

	if (!data->enable)
		return regmap_clear_bits(st->regmap,
					 AXI_ADC_REG_CHAN_CTRL(chan),
					 AXI_ADC_CHAN_CTRL_FMT_MASK);

	val = FIELD_PREP(AXI_ADC_CHAN_CTRL_FMT_MASK, AXI_ADC_CHAN_CTRL_FMT_EN);
	if (data->sign_extend)
		val |= FIELD_PREP(AXI_ADC_CHAN_CTRL_FMT_MASK,
				  AXI_ADC_CHAN_CTRL_FMT_SIGEXT);

	if (data->type == CONVERTER_OFFSET_BINARY)
		val |= FIELD_PREP(AXI_ADC_CHAN_CTRL_FMT_MASK,
				  AXI_ADC_CHAN_CTRL_FMT_BIN_OFF);

	return regmap_update_bits(st->regmap, AXI_ADC_REG_CHAN_CTRL(chan),
				  AXI_ADC_CHAN_CTRL_FMT_MASK, val);
}

static void __axi_adc_disable(const struct axi_adc_state *st)
{
	regmap_clear_bits(st->regmap, AXI_ADC_REG_RSTN,
			  AXI_ADC_RSTN_RESET_MASK);
}

static int __axi_adc_enable(const struct axi_adc_state *st)
{
	/*
	 * TODO: Ask Michael (or Adrian) about sleeping 10ms in between and after?
	 * and if we can't just enable both at the same time.
	 * Also ask about drp loop as in the dds driver
	 */
	//axi_adc_write(st, AXI_ADC_REG_RSTN, AXI_ADC_REG_RSTN_MMCM_RSTN);
	//axi_adc_write(st, AXI_ADC_REG_RSTN,
	//	      AXI_ADC_REG_RSTN_RSTN | AXI_ADC_REG_RSTN_MMCM_RSTN);
	return regmap_set_bits(st->regmap, AXI_ADC_REG_RSTN,
			       AXI_ADC_RSTN_RESET_MASK);
}

static int axi_adc_enable(struct converter_backend *conv)
{
	return __axi_adc_enable(converter_get_drvdata(conv));
}

static void axi_adc_disable(struct converter_backend *conv)
{
	__axi_adc_disable(converter_get_drvdata(conv));
}

static int axi_adc_reset(struct axi_adc_state *st)
{
	int ret;

	__axi_adc_disable(st);
	fsleep(10);
	ret = __axi_adc_enable(st);
	if (ret)
		return ret;

	fsleep(10);
	return 0;
}

static const struct regmap_config axi_adc_regmap_config = {
	.val_bits = 32,
	.reg_bits = 32,
	.reg_stride = 4,
	.max_register = 0x0800,
};

static int axi_adc_generic_init(struct converter_backend *conv,
				struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	unsigned int ver, *expected_ver, ret;
	struct axi_adc_state *st;
	void __iomem *base;
	struct clk *clk;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->dev = dev;

	expected_ver = (unsigned int *)device_get_match_data(dev);
	if (!expected_ver)
		return -ENODEV;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	st->regmap = devm_regmap_init_mmio(dev, base, &axi_adc_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	converter_set_drvdata(conv, st);
	converter_set_regmap(conv, st->regmap);

	clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "clk_get failed with %ld\n", PTR_ERR(clk));
		return PTR_ERR(clk);
	}

	ret = axi_adc_reset(st);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADI_AXI_REG_VERSION, &ver);
	if (ret)
		return ret;

	if (*expected_ver > ver) {
		dev_err(&pdev->dev,
			"IP core version is too old. Expected %d.%.2d.%c, Reported %d.%.2d.%c\n",
			ADI_AXI_PCORE_VER_MAJOR(*expected_ver),
			ADI_AXI_PCORE_VER_MINOR(*expected_ver),
			ADI_AXI_PCORE_VER_PATCH(*expected_ver),
			ADI_AXI_PCORE_VER_MAJOR(ver),
			ADI_AXI_PCORE_VER_MINOR(ver),
			ADI_AXI_PCORE_VER_PATCH(ver));
		return -ENODEV;
	}

	/* fetch synthesis capabilities */
	ret = regmap_read(st->regmap, AXI_ADC_REG_CONFIG, &st->capabilities);
	if (ret)
		return ret;

	dev_dbg(&pdev->dev, "AXI ADC IP core (%d.%.2d.%c) up\n",
		ADI_AXI_PCORE_VER_MAJOR(ver),
		ADI_AXI_PCORE_VER_MINOR(ver),
		ADI_AXI_PCORE_VER_PATCH(ver));

	/* up to the frontend to explicitly enable us */
	__axi_adc_disable(st);
	mutex_init(&st->lock);
	return 0;
}

static const struct converter_ops adi_axi_adc_generic = {
	.backend_init = axi_adc_generic_init,
	.enable = axi_adc_enable,
	.disable = axi_adc_disable,
	.data_format_set = axi_adc_data_format_set,
	.test_pattern_set = axi_adc_test_pattern_set,
	.chan_enable = axi_adc_chan_enable,
	.chan_disable = axi_adc_chan_disable,
	.iodelay_set = axi_adc_iodelay_set,
};

static int axi_adc_probe(struct platform_device *pdev)
{
	return converter_add(&pdev->dev, &adi_axi_adc_generic);
}

/*
 * It actually matters to remove the converter in the .remove() hook. This means
 * that the all the converters (an the frontend) will be tear down before running
 * any specific devres cleanup (at the driver core level). What this all means is
 * that we can use devm_ apis in .backend_init() and being sure those resources
 * will be released before the frontend resources and before any devm_* used
 * in .probe().
 */
static int axi_adc_remove(struct platform_device *pdev)
{
	converter_del(&pdev->dev);
	return 0;
}

static unsigned int axi_adc_10_0_a = ADI_AXI_PCORE_VER(10, 0, 'a');

/* Match table for of_platform binding */
static const struct of_device_id axi_adc_of_match[] = {
	{ .compatible = "adi,axi-adc-10.0.a-new", .data = &axi_adc_10_0_a },
	{ /* end of list */ }
};
MODULE_DEVICE_TABLE(of, axi_adc_of_match);

static struct platform_driver axi_adc_driver = {
	.driver = {
		.name = "axi-adc",
		.of_match_table = axi_adc_of_match,
	},
	.probe = axi_adc_probe,
	.remove = axi_adc_remove,
};
module_platform_driver(axi_adc_driver);

MODULE_AUTHOR("Nuno Sa <nuno.sa@analog.com>");
MODULE_DESCRIPTION("Analog Devices Generic AXI ADC IP core driver");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_CONVERTER);
