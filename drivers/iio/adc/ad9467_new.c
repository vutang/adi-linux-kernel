// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD9467 SPI ADC driver
 *
 * Copyright 2012-2023 Analog Devices Inc.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#include <linux/iio/addc/converter.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>

/*
 * ADI High-Speed ADC common spi interface registers
 * See Application-Note AN-877:
 *   https://www.analog.com/media/en/technical-documentation/application-notes/AN-877.pdf
 */

#define AN877_ADC_REG_CHIP_ID		0x01
#define AN877_ADC_REG_CHAN_INDEX        0x05
 #define AN877_ADC_REG_TEST_IO		0x0D
#define AN877_ADC_REG_OUTPUT_MODE	0x14
#define AN877_ADC_REG_OUTPUT_PHASE	0x16
#define AN877_ADC_REG_OUTPUT_DELAY	0x17
#define AN877_ADC_REG_VREF		0x18
#define AN877_ADC_REG_TRANSFER		0xFF

/* AN877_ADC_REG_TRANSFER */
#define AN877_ADC_TRANSFER_SYNC		0x1

/* AN877_ADC_REG_OUTPUT_MODE */
#define AN877_ADC_OUTPUT_MODE_OFFSET_BINARY	0x0
#define AN877_ADC_OUTPUT_MODE_TWOS_COMPLEMENT	0x1

/* AN877_ADC_REG_OUTPUT_PHASE */
#define AN877_ADC_OUTPUT_EVEN_ODD_MODE_EN	0x20
#define AN877_ADC_INVERT_DCO_CLK		0x80

/* AN877_ADC_REG_TEST_IO */
#define AN877_ADC_TESTMODE_OFF			0x0
#define AN877_ADC_TESTMODE_PN23_SEQ             0x5
#define AN877_ADC_TESTMODE_PN9_SEQ              0x6

/*
 * Analog Devices AD9265 16-Bit, 125/105/80 MSPS ADC
 */

#define CHIPID_AD9265			0x64
#define AD9265_DEF_OUTPUT_MODE		0x40
#define AD9265_REG_VREF_MASK		0xC0

/*
 * Analog Devices AD9434 12-Bit, 370/500 MSPS ADC
 */

#define CHIPID_AD9434			0x6A
#define AD9434_DEF_OUTPUT_MODE		0x00
#define AD9434_REG_VREF_MASK		0xC0

/*
 * Analog Devices AD9467 16-Bit, 200/250 MSPS ADC
 */

#define CHIPID_AD9467			0x50
#define AD9467_DEF_OUTPUT_MODE		0x08
#define AD9467_REG_VREF_MASK		0x0F

struct ad9467_chip_info {
	const char *name;
	const struct iio_chan_spec *channels;
	const unsigned int (*scale_table)[2];
	unsigned int id;
	int num_scales;
	unsigned long max_rate;
	unsigned int default_output_mode;
	unsigned int vref_mask;
	unsigned int num_channels;
	unsigned int num_lanes;
	unsigned int dco_en;
	bool has_dco;
};

struct ad9467_state {
	const struct ad9467_chip_info *info;
	struct converter_backend *conv;
	struct spi_device *spi;
	struct clk *clk;
	unsigned int output_mode;
	unsigned long adc_clk;
};

/*
 * !TODO: Infer about moving to regmap... Moreover we need to make this DMA safe
 */
static int ad9467_spi_read(struct spi_device *spi, unsigned int reg)
{
	unsigned char tbuf[2], rbuf[1];
	int ret;

	tbuf[0] = 0x80 | (reg >> 8);
	tbuf[1] = reg & 0xFF;

	ret = spi_write_then_read(spi,
				  tbuf, ARRAY_SIZE(tbuf),
				  rbuf, ARRAY_SIZE(rbuf));

	if (ret < 0)
		return ret;

	return rbuf[0];
}

static int ad9467_spi_write(struct spi_device *spi, unsigned int reg,
			    unsigned int val)
{
	unsigned char buf[3];

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;
	buf[2] = val;

	return spi_write(spi, buf, ARRAY_SIZE(buf));
}

static void __ad9467_get_scale(struct ad9467_state *st, int index,
			       unsigned int *val, unsigned int *val2)
{
	const struct iio_chan_spec *chan = &st->info->channels[0];
	unsigned int tmp;

	tmp = (st->info->scale_table[index][0] * 1000000ULL) >> chan->scan_type.realbits;
	*val = tmp / 1000000;
	*val2 = tmp % 1000000;
}

/* needs to check for ret codes */
static int ad9467_get_scale(struct ad9467_state *st, int *val, int *val2)
{
	unsigned int i, vref_val;

	vref_val = ad9467_spi_read(st->spi, AN877_ADC_REG_VREF);

	vref_val &= st->info->vref_mask;

	for (i = 0; i < st->info->num_scales; i++) {
		if (vref_val == st->info->scale_table[i][1])
			break;
	}

	if (i == st->info->num_scales)
		return -ERANGE;

	__ad9467_get_scale(st, i, val, val2);

	return IIO_VAL_INT_PLUS_MICRO;
}

/* Needs mutex and check for ret codes */
static int ad9467_set_scale(struct ad9467_state *st, int val, int val2)
{
	unsigned int scale_val[2];
	unsigned int i;

	if (val != 0)
		return -EINVAL;

	for (i = 0; i < st->info->num_scales; i++) {
		__ad9467_get_scale(st, i, &scale_val[0], &scale_val[1]);
		if (scale_val[0] != val || scale_val[1] != val2)
			continue;

		ad9467_spi_write(st->spi, AN877_ADC_REG_VREF,
				 st->info->scale_table[i][1]);
		ad9467_spi_write(st->spi, AN877_ADC_REG_TRANSFER,
				 AN877_ADC_TRANSFER_SYNC);
		return 0;
	}

	return -EINVAL;
}

static int ad9467_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct ad9467_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		return ad9467_get_scale(st, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = clk_get_rate(st->clk);

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad9467_testmode_set(const struct ad9467_state *st,
			       unsigned int chan, unsigned int mode)
{
	int ret;

	ret = ad9467_spi_write(st->spi, AN877_ADC_REG_CHAN_INDEX, BIT(chan));
	if (ret)
		return ret;

	ret = ad9467_spi_write(st->spi, AN877_ADC_REG_TEST_IO, mode);
	if (ret)
		return ret;

	ret = ad9467_spi_write(st->spi, AN877_ADC_REG_CHAN_INDEX, 0x3);
	if (ret)
		return ret;

	return ad9467_spi_write(st->spi, AN877_ADC_REG_TRANSFER,
				AN877_ADC_TRANSFER_SYNC);
}

static int ad9467_calibrate_apply(const struct ad9467_state *st, bool dco,
				  unsigned int val)
{
	if (dco) {
		unsigned int out_delay;
		int ret;

		if (!val)
			out_delay = 0;
		else
			out_delay = (val - 1) | st->info->dco_en;

		ret = ad9467_spi_write(st->spi, AN877_ADC_REG_OUTPUT_DELAY,
				       out_delay);
		if (ret)
			return ret;

		return ad9467_spi_write(st->spi, AN877_ADC_REG_TRANSFER,
					AN877_ADC_TRANSFER_SYNC);
	}

	return converter_iodelay_set(st->conv, st->info->num_lanes, val);
}

static void ad9467_dump_table(const unsigned char *err_field,
			      unsigned int size, unsigned int val)
{
	unsigned int cnt;

	for (cnt = 0; cnt < size; cnt++) {
		if (cnt == val)
			pr_debug("|");
	else
		pr_debug("%c", err_field[cnt] ? '-' : 'o');
		if (cnt == size / 2)
			pr_debug("\n");
	}
}

static int ad9467_find_optimal_point(const unsigned char *err_field,
				     unsigned int size)
{
	unsigned int val, cnt = 0, max_cnt = 0, max_start = 0;
	int start = -1;

	for (val = 0; val < size; val++) {
		if (!err_field[val]) {
			if (start == -1)
				start = val;
			cnt++;
		} else {
			if (cnt > max_cnt) {
				max_cnt = cnt;
				max_start = start;
			}

			start = -1;
			cnt = 0;
		}
	}

	if (cnt > max_cnt) {
		max_cnt = cnt;
		max_start = start;
	}

	if (!max_cnt)
		return -EIO;

	val = max_start + max_cnt / 2;
	ad9467_dump_table(err_field, size, val);

	return val;
}

static int ad9467_do_calibrate(const struct ad9467_state *st, bool dco)
{
	unsigned int max_val = dco ? 32 : 31, c, val;
	unsigned int conv_pattern, pattern;
	unsigned char err_field[66] = {0};
	bool status, inv_range = false;
	int ret;

	for (c = 0; c < st->info->num_channels; c++) {
		if (c > 0) {
			pattern = AN877_ADC_TESTMODE_PN23_SEQ;
			conv_pattern = CONVERTER_ADI_PRBS_23A;
		} else {
			pattern = AN877_ADC_TESTMODE_PN9_SEQ;
			conv_pattern = CONVERTER_ADI_PRBS_9A;
		}

		ret = ad9467_testmode_set(st, c, pattern);
		if (ret)
			return ret;

		ret = converter_test_pattern_set(st->conv, c, conv_pattern);
		if (ret)
			return ret;

		ret = converter_chan_enable(st->conv, c);
		if (ret)
			return ret;
	}

retune:
	if (dco) {
		unsigned int phase = AN877_ADC_OUTPUT_EVEN_ODD_MODE_EN;

		if (inv_range)
			phase |= AN877_ADC_INVERT_DCO_CLK;

		ret = ad9467_spi_write(st->spi, AN877_ADC_REG_OUTPUT_PHASE,
				       phase);
		if (ret)
			return ret;
	} else {
		if (inv_range)
			ret = converter_set_falling_edge(st->conv);
		else
			ret = converter_set_rising_edge(st->conv);

		if (ret)
			return ret;
	}

	for (val = 0; val <= max_val; val++) {
		ret = ad9467_calibrate_apply(st, dco, val);
		if (ret)
			return ret;

		status = converter_test_pattern_status_all(st->conv);
		err_field[val + (inv_range * (max_val + 1))] = status;
	}

	/*
	 * Ask Michael about this. AFAICT, we should just try both variants
	 * and get the best result. No idea we invert the range in case
	 * err_field[max_val] == 0 (as in the original driver)
	 */
	if (!inv_range) {
		inv_range = true;
		goto retune;
	}

	val = ad9467_find_optimal_point(err_field, (max_val + 1) * 2);
	if (val < 0)
		return val;

	if (val < max_val) {
		if (dco)
			ret = ad9467_spi_write(st->spi,
					       AN877_ADC_REG_OUTPUT_PHASE,
					       AN877_ADC_OUTPUT_EVEN_ODD_MODE_EN);
		else
			ret = converter_set_rising_edge(st->conv);
	} else {
		val -= max_val + 1;
		/*
		 * inv_range = true is the last test to run. Hence, there's no
		 * need to re-do any configuration
		 */
		inv_range = false;
	}

	if (dco)
		dev_dbg(&st->spi->dev,
			" %s DCO 0x%X CLK %lu Hz\n", inv_range ? "INVERT" : "",
			 val > 0 ? ((val - 1) | st->info->dco_en) : 0,
			 st->adc_clk);
	else
		dev_dbg(&st->spi->dev,
			" %s IDELAY 0x%x\n", inv_range ? "INVERT" : "", val);

	for (c = 0; c < st->info->num_channels; c++) {
		ret = ad9467_testmode_set(st, c, AN877_ADC_TESTMODE_OFF);
		if (ret)
			return ret;

		ret = converter_chan_disable(st->conv, c);
		if (ret)
			return ret;
	}

	/* finally apply the optimal value */
	return ad9467_calibrate_apply(st, dco, val);
}

static int ad9467_dco_calibrate(const struct ad9467_state *st)
{
	if (!st->info->has_dco)
		return 0;

	return ad9467_do_calibrate(st, true);
}

static int ad9467_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct ad9467_state *st = iio_priv(indio_dev);
	long r_clk;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		return ad9467_set_scale(st, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		r_clk = clk_round_rate(st->clk, val);
		if (r_clk < 0 || r_clk > st->info->max_rate) {
			dev_warn(&st->spi->dev,
				 "Error setting ADC sample rate %ld", r_clk);
			iio_device_release_direct_mode(indio_dev);
			return -EINVAL;
		}

		ret = clk_set_rate(st->clk, r_clk);
		iio_device_release_direct_mode(indio_dev);
		return ret;
	default:
		return -EINVAL;
	}
}

static int ad9467_read_available(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 const int **vals, int *type, int *length,
				 long mask)
{
	struct ad9467_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		*vals = (const int *)st->info->scale_table;
		*type = IIO_VAL_INT_PLUS_MICRO;
		/* Values are stored in a 2D matrix */
		*length = st->info->num_scales * 2;
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

/*
 * TODO: Let's see how the axi_dac shapes up. We might be able to have
 * a generic @converter_update_scan_mode()
 */
static int ad9467_update_scan_mode(struct iio_dev *indio_dev,
				   const unsigned long *scan_mask)
{
	struct ad9467_state *st = iio_priv(indio_dev);
	unsigned int c;
	int ret;

	for (c = 0; c < st->info->num_channels; c++) {
		if (test_bit(c, scan_mask))
			ret = converter_chan_enable(st->conv, c);
		else
			ret = converter_chan_disable(st->conv, c);

		if (ret)
			return ret;
	}

	return 0;
}

static int ad9467_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad9467_state *st = iio_priv(indio_dev);
	struct spi_device *spi = st->spi;
	int ret;

	if (!readval) {
		ret = ad9467_spi_write(spi, reg, writeval);
		if (ret)
			return ret;

		return ad9467_spi_write(spi, AN877_ADC_REG_TRANSFER,
					AN877_ADC_TRANSFER_SYNC);
	}

	ret = ad9467_spi_read(spi, reg);
	if (ret < 0)
		return ret;

	*readval = ret;

	return 0;
}

/* missing available scales... */
#define AD9467_CHAN(_chan, _si, _bits, _sign)				\
{									\
	.type = IIO_VOLTAGE,						\
	.indexed = 1,							\
	.channel = _chan,						\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |		\
		BIT(IIO_CHAN_INFO_SAMP_FREQ),				\
	.info_mask_shared_by_type_available = BIT(IIO_CHAN_INFO_SCALE),	\
	.scan_index = _si,						\
	.scan_type = {							\
		.sign = _sign,						\
		.realbits = _bits,					\
		.storagebits = 16,					\
	},								\
}

static const struct iio_chan_spec ad9434_channels[] = {
	AD9467_CHAN(0, 0, 12, 'S'),
};

static const struct iio_chan_spec ad9467_channels[] = {
	AD9467_CHAN(0, 0, 16, 'S'),
};

static const unsigned int ad9265_scale_table[][2] = {
	{1250, 0x00}, {1500, 0x40}, {1750, 0x80}, {2000, 0xC0},
};

static const unsigned int ad9434_scale_table[][2] = {
	{1600, 0x1C}, {1580, 0x1D}, {1550, 0x1E}, {1520, 0x1F}, {1500, 0x00},
	{1470, 0x01}, {1440, 0x02}, {1420, 0x03}, {1390, 0x04}, {1360, 0x05},
	{1340, 0x06}, {1310, 0x07}, {1280, 0x08}, {1260, 0x09}, {1230, 0x0A},
	{1200, 0x0B}, {1180, 0x0C},
};

static const unsigned int ad9467_scale_table[][2] = {
	{2000, 0}, {2100, 6}, {2200, 7},
	{2300, 8}, {2400, 9}, {2500, 10},
};

static const struct ad9467_chip_info ad9467_chip_tbl = {
	.name = "ad9467",
	.id = CHIPID_AD9467,
	.max_rate = 250000000UL,
	.scale_table = ad9467_scale_table,
	.num_scales = ARRAY_SIZE(ad9467_scale_table),
	.channels = ad9467_channels,
	.num_channels = ARRAY_SIZE(ad9467_channels),
	.default_output_mode = AD9467_DEF_OUTPUT_MODE,
	.vref_mask = AD9467_REG_VREF_MASK,
	.num_lanes = 8,
};

static const struct ad9467_chip_info ad9265_chip_tbl = {
	.name = "ad9265",
	.id = CHIPID_AD9265,
	.max_rate = 125000000UL,
	.scale_table = ad9265_scale_table,
	.num_scales = ARRAY_SIZE(ad9265_scale_table),
	.channels = ad9467_channels,
	.num_channels = ARRAY_SIZE(ad9467_channels),
	.default_output_mode = AD9265_DEF_OUTPUT_MODE,
	.vref_mask = AD9265_REG_VREF_MASK,
	.has_dco = true,
};

static const struct ad9467_chip_info ad9434_chip_tbl = {
	.name = "ad9434",
	.id = CHIPID_AD9434,
	.max_rate = 500000000UL,
	.scale_table = ad9434_scale_table,
	.num_scales = ARRAY_SIZE(ad9434_scale_table),
	.channels = ad9434_channels,
	.num_channels = ARRAY_SIZE(ad9434_channels),
	.default_output_mode = AD9434_DEF_OUTPUT_MODE,
	.vref_mask = AD9434_REG_VREF_MASK,
	.num_lanes = 6,
};

static const struct iio_info ad9467_info = {
	.read_raw = ad9467_read_raw,
	.write_raw = ad9467_write_raw,
	.update_scan_mode = ad9467_update_scan_mode,
	.debugfs_reg_access = ad9467_reg_access,
	.read_avail = ad9467_read_available,
};

static int ad9467_reset(struct device *dev)
{
	struct gpio_desc *gpio;

	gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(gpio))
		return PTR_ERR(gpio);
	if (!gpio)
		return 0;

	fsleep(1);
	gpiod_set_value_cansleep(gpio, 0);
	fsleep(10);

	return 0;
}

/*
 * Also candidate for a generic helper...
 *
 * This is something that I don't like much because, hardwarewise, the dma is
 * connected to the backend device so it would make sense for the dma
 * properties to be in the platform device rather than the frontend. However,
 * detaching the IIO DMA buffer like that from the place where the IIO
 * device is handled would feel equally odd and, while doable, it would
 * require some hacking and new converter ops to make sure that resources
 * lifetime feel right (so also export the non devm_ @iio_dmaengine_buffer_alloc()).
 */
static int ad9467_buffer_get(struct iio_dev *indio_dev)
{
	struct device *dev = indio_dev->dev.parent;
	const char *dma_name;

	if (!device_property_present(dev, "dmas"))
		return 0;

	if (device_property_read_string(dev, "dma-names", &dma_name))
		dma_name = "rx";

	return devm_iio_dmaengine_buffer_setup(dev, indio_dev, dma_name);
}

static int ad9467_outputmode_set(struct spi_device *spi, unsigned int mode)
{
	int ret;

	ret = ad9467_spi_write(spi, AN877_ADC_REG_OUTPUT_MODE, mode);
	if (ret < 0)
		return ret;

	return ad9467_spi_write(spi, AN877_ADC_REG_TRANSFER,
				AN877_ADC_TRANSFER_SYNC);
}

static int ad9467_channels_setup(const struct ad9467_state *st, bool test_mode)
{
	struct converter_data_fmt data;
	unsigned int c, mode;
	int ret;

	if (test_mode) {
		data.enable = false;
		mode = st->info->default_output_mode;
	} else {
		mode = st->info->default_output_mode |
				AN877_ADC_OUTPUT_MODE_TWOS_COMPLEMENT;
		data.type = CONVERTER_TWOS_COMPLEMENT;
		data.sign_extend = true;
		data.enable = true;
	}

	ret = ad9467_outputmode_set(st->spi, mode);
	if (ret)
		return ret;

	for (c = 0; c < st->info->num_channels; c++) {
		ret = converter_data_format_set(st->conv, c, &data);
		if (ret)
			return ret;
	}

	return 0;
}

static int ad9467_idelay_calibrate(const struct ad9467_state *st)
{
	if (!st->info->num_lanes)
		return 0;

	return ad9467_do_calibrate(st, false);
}

static int ad9467_calibrate(const struct ad9467_state *st)
{
	int ret;

	ret = ad9467_channels_setup(st, true);
	if (ret)
		return ret;

	ret = ad9467_idelay_calibrate(st);
	if (ret)
		return ret;

	ret = ad9467_dco_calibrate(st);
	if (ret)
		return ret;

	return ad9467_channels_setup(st, false);
}

static int ad9467_init(struct converter_frontend *frontend, struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct iio_dev *indio_dev;
	struct ad9467_state *st;
	unsigned int id;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	st->info = spi_get_device_match_data(spi);
	if (!st->info)
		return -EINVAL;

	st->conv = converter_get(frontend, NULL);
	if (IS_ERR(st->conv))
		return PTR_ERR(st->conv);

	st->clk = devm_clk_get_enabled(dev, "adc-clk");
	if (IS_ERR(st->clk))
		return PTR_ERR(st->clk);

	st->adc_clk = clk_get_rate(st->clk);

	ret = ad9467_reset(dev);
	if (ret)
		return ret;

	id = ad9467_spi_read(spi, AN877_ADC_REG_CHIP_ID);
	if (id != st->info->id) {
		dev_err(dev, "Mismatch CHIP_ID, got 0x%X, expected 0x%X\n",
			id, st->info->id);
		return -ENODEV;
	}

	indio_dev->name = st->info->name;
	indio_dev->channels = st->info->channels;
	indio_dev->num_channels = st->info->num_channels;
	indio_dev->info = &ad9467_info;

	ret = ad9467_buffer_get(indio_dev);
	if (ret)
		return ret;

	ret = converter_enable(st->conv);
	if (ret)
		return ret;

	ret = ad9467_calibrate(st);
	if (ret)
		return ret;
	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return ret;

	converter_add_direct_reg_access(st->conv, indio_dev);

	return 0;
}

static const struct frontend_ops ad9467_ops = {
	.frontend_init = ad9467_init,
};

static int ad9467_probe(struct spi_device *spi)
{
	return converter_frontend_add(&spi->dev, &ad9467_ops);
}

/*
 * It actually matters to remove the frontend in the .remove() hook. This means
 * that all the converters (and the frontend) will be tear down before running
 * any specific devres cleanup (at the driver core level). What this all means is
 * that we can use devm_ apis in .frontend_init() and being sure those resources
 * will be released after the backend resources and before any devm_* used
 * in .probe().
 */
static void ad9467_remove(struct spi_device *spi)
{
	converter_del(&spi->dev);
}

static const struct of_device_id ad9467_of_match[] = {
	{ .compatible = "adi,ad9265", .data = &ad9265_chip_tbl, },
	{ .compatible = "adi,ad9434", .data = &ad9434_chip_tbl, },
	{ .compatible = "adi,ad9467-new", .data = &ad9467_chip_tbl, },
	{}
};
MODULE_DEVICE_TABLE(of, ad9467_of_match);

static const struct spi_device_id ad9467_ids[] = {
	{ "ad9265", (kernel_ulong_t)&ad9265_chip_tbl },
	{ "ad9434", (kernel_ulong_t)&ad9434_chip_tbl },
	{ "ad9467-new", (kernel_ulong_t)&ad9467_chip_tbl },
	{}
};
MODULE_DEVICE_TABLE(spi, ad9467_ids);

static struct spi_driver ad9467_driver = {
	.driver = {
		.name = "ad9467",
		.of_match_table = ad9467_of_match,
	},
	.probe = ad9467_probe,
	.remove = ad9467_remove,
	.id_table = ad9467_ids,
};
module_spi_driver(ad9467_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9467 ADC driver");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_CONVERTER);
