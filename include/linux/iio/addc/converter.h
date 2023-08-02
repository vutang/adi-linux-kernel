/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _CONVERTER_H
#define _CONVERTER_H

struct converter_frontend;
struct converter_backend;
struct iio_dev;
struct device;
struct regmap;

enum converter_test_pattern {
	CONVERTER_PRBS_7,
	CONVERTER_PRBS_15,
	CONVERTER_PRBS_23,
	CONVERTER_PRBS_31,
	CONVERTER_RAMP_NIBBLE,
	CONVERTER_RAMP_16,
	/* vendor specific from 32 */
	CONVERTER_ADI_PRBS_9A = 32,
	CONVERTER_ADI_PRBS_23A,
	CONVERTER_ADI_PRBS_X,
	CONVERTER_TEST_PATTERN_MAX
};

enum converter_data_type {
	CONVERTER_TWOS_COMPLEMENT,
	CONVERTER_OFFSET_BINARY,
	CONVERTER_DATA_TYPE_MAX
};

enum converter_edge {
	CONVERTER_RISING_EDGE,
	CONVERTER_FALLING_EDGE,
	CONVERTER_EDGE_MAX
}

/**
 * struct converter_data_fmt - Backend data format
 * @type:		Data type.
 * @sign_extend:	Bool to tell if the data is sign extended.
 * @enable:		Enable/Disable the data format module. If disabled,
 *			not formatting will happen.
 */
struct converter_data_fmt {
	enum converter_data_type type;
	bool sign_extend;
	bool enable;
};

/**
 * struct converter_test_pattern_xlate - Helper struct for test pattern handling
 * @pattern:	Pattern to configure.
 * @reg_val:	Register value for the pattern to configure.
 */
struct converter_test_pattern_xlate {
	enum converter_test_pattern pattern;
	unsigned int reg_val;
};

/**
 * struct converter_ops - Backend supported operations
 * @backend_init:	Mandatory function to initialize the backend device. It
 *			should be a replacement for .probe() where the latest
 *			should only have to care about doing @converter_add().
 * @backend_close:	Optional function to tear down the device.
 * @enable:		Enable the backend device.
 * @disable:		Disable the backend device.
 * @data_format_set:	Configure the data format for a specific channel.
 * @chan_enable:	Enable one channel.
 * @chan_disable:	Disable one channel.
 * @iodelay_set:	Controls the IO delay for all the lanes at the interface
 *			(where data is actually transferred between frontend and
			backend) level.
 * @test_pattern_set:	Set's a test pattern to be transmitted/received by the
 *			backend. Typically useful for debug or interface
 *			purposes calibration.
 */
struct converter_ops {
	int (*backend_init)(struct converter_backend *conv, struct device *dev);
	void (*backend_close)(struct converter_backend *conv);
	int (*enable)(struct converter_backend *conv);
	void (*disable)(struct converter_backend *conv);
	int (*data_format_set)(struct converter_backend *conv,
			       unsigned int chan,
			       const struct converter_data_fmt *data);
	int (*chan_enable)(struct converter_backend *conv, unsigned int chan);
	int (*chan_disable)(struct converter_backend *conv, unsigned int chan);
	int (*iodelay_set)(struct converter_backend *conv,
			   unsigned int num_lanes, unsigned int delay);
	int (*test_pattern_set)(struct converter_backend *conv,
				unsigned int chan,
				enum converter_test_pattern pattern);
};

/**
 * struct frontend_ops - Frontend supported operations
 * @frontend_init:	Mandatory function to initialize the frontend device. It
 *			should be a replacement for .probe() where the latest
 *			should only have to care about doing @frontend_add().
 * @frontend_close:	Optional function to tear down the device.
 */
struct frontend_ops {
	int (*frontend_init)(struct converter_frontend *frontend,
			     struct device *dev);
	void (*frontend_close)(struct converter_frontend *frontend);
};

/**
 * converter_test_pattern_xlate() - Helper macro for translatting test patterns
 * @pattern:	Pattern to translate.
 * @xlate:	List of @struct converter_test_pattern_xlate pairs.
 *
 * Simple helper to match a supported pattern and get the register value. Should
 * only be called by backend devices. Automatically computes the number of
 * @xlate entries.
 */
#define converter_test_pattern_xlate(pattern, xlate)	\
	__converter_test_pattern_xlate(pattern, xlate, ARRAY_SIZE(xlate));

#if defined(CONFIG_IIO_CONVERTER)

/**
 * converter_get_drvdata - Get driver private data
 * @conv:	Converter device.
 */
void *converter_get_drvdata(const struct converter_backend *conv);

/**
 * converter_set_drvdata - Set driver private data
 * @conv:	Converter device.
 * @drvdata:	Driver private data.
 */
void converter_set_drvdata(struct converter_backend *conv, void *drvdata);

/**
 * converter_set_regmap - Add a regmap object to a converter
 * @conv:	Converter device.
 * @regmap:	Regmap object.
 */
void converter_set_regmap(struct converter_backend *conv,
			  struct regmap *regmap);

/**
 * __converter_test_pattern_xlate - Helper macro for translatting test patterns
 * @pattern:	Pattern to translate.
 * @xlate:	List of @struct converter_test_pattern_xlate pairs.
 * @n_matches:	Number of entries in @xlate.
 *
 * Simple helper to match a supported pattern and get the register value. Should
 * only be called by backend devices.
 */
int __converter_test_pattern_xlate(unsigned int pattern,
				   const struct converter_test_pattern_xlate *xlate,
				   int n_matches);

/**
 *
 */
int converter_add(struct device *dev, const struct converter_ops *ops);

/**
 * converter_del - Remove the converter device
 * @dev:	device to remove from the aggregate
 *
 * Removes the converter from the aggregate device. This tears down the frontend
 * and all the converters.
 *
 * Ideally, this should be called from the backend driver .remove() callback.
 * This means that all the converters (and the frontend) will be tear down before
 * running any specific devres cleanup (at the driver core level). What this all
 * means is that we can use devm_ apis in @backend_init() and being sure those
 * resources will be released after the backend resources and before any devm_*
 * used in @probe(). If that is not the case, one should likely not use any
 * devm_ API in @backend_init(). That means .backend_close() should be
 * provided to do all the necessary cleanups.
 */
void converter_del(struct device *dev);

/**
 * converter_enable - Enable the device
 * @conv:	Converter device.
 *
 * Enables the backend device.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int converter_enable(struct converter_backend *conv);

/**
 * converter_disable - Disable the device
 * @conv:	Converter device.
 *
 * Disables the backend device.
 */
void converter_disable(struct converter_backend *conv);

/**
 * converter_test_pattern_set - Set a test pattern
 * @conv:	Converter device.
 * @chan:	Channel number.
 * @pattern:	Pattern to set.
 *
 * Set's a test pattern to be transmitted/received by the backend. Typically
 * useful for debug or interface calibration purposes. A backend driver can
 * call the @converter_test_pattern_xlate() helper to validate the pattern
 * (given an array of @struct converter_test_pattern_xlate).
 *
 * Note that some patterns might be frontend specific. I.e, as far as the
 * backend is concerned the pattern is valid (from a register point of view) but
 * the actual support for the pattern is not implemented in the device for this
 * specific frontend. It's up to the frontend to ask for a proper pattern
 * (as it should know better).
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int converter_test_pattern_set(struct converter_backend *conv,
			       unsigned int chan,
			       enum converter_test_pattern pattern);

int converter_chan_status_all(struct converter_backend *conv,
			      unsigned int num_chans);

/**
 * converter_data_format_set - Configure the data format
 * @conv:	Converter device.
 * @chan:	Channel number.
 * @data:	Data format.
 *
 * Properly configure a channel with respect to the expected data format. A
 * @struct converter_data_fmt must be passed with the settings.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int converter_data_format_set(struct converter_backend *conv,
			      unsigned int chan,
			      const struct converter_data_fmt *data);

int converter_edge_select(struct converter_backend *conv,
			  enum converter_edge edge);

static inline int converter_set_rising_edge(struct converter_backend *conv)
{
	return converter_edge_select(conv, CONVERTER_RISING_EDGE);
}

static inline int converter_set_falling_edge(struct converter_backend *conv)
{
	return converter_edge_select(conv, CONVERTER_FALLING_EDGE);
}

/**
 * converter_chan_enable - Enable a backend channel
 * @conv:	Converter device.
 * @chan:	Channel number.
 *
 * Enables a channel on the backend device.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int converter_chan_enable(struct converter_backend *conv, unsigned int chan);

/**
 * converter_chan_disable - Disable a backend channel
 * @conv:	Converter device.
 * @chan:	Channel number.
 *
 * Disables a channel on the backend device.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int converter_chan_disable(struct converter_backend *conv, unsigned int chan);

/**
 * converter_iodelay_set - Set's the backend data interface IO delay
 * @conv:	Converter device.
 * @num_lanes:	Number of lanes in the data interface.
 * @delay:	Delay to set.
 *
 * Controls the IO delay for all the lanes at the data interface (where data is
 * actually transferred between frontend and backend) level.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int converter_iodelay_set(struct converter_backend *conv,
			  unsigned int num_lanes, unsigned int delay);

/**
 * converter_frontend_del - Remove the frontend device
 * @dev:	Device to remove from the aggregate
 *
 * Removes the frontend from the aggregate device. This tears down the frontend
 * and all the converters.
 *
 * Ideally, this should be called from the frontend driver .remove() callback.
 * This means that all the converters (and the frontend) will be tear down before
 * running any specific devres cleanup (at the driver core level). What this all
 * means is that we can use devm_ apis in .frontend_init() and being sure those
 * resources will be released after the backend resources and before any devm_*
 * used in .probe(). If that is not the case, one should likely not use any
 * devm_ API in .frontend_init(). That means .frontend_close() should be
 * provided to do all the necessary cleanups.
 */
void converter_frontend_del(struct device *dev);

/**
 * converter_frontend_add - Allocate and add a frontend device
 * @dev:	Device to allocate frontend for.
 * @ops:	Frontend callbacks.
 *
 * This allocates the frontend device and looks for all converters needed
 * so that, when they are available, all of the devices in the aggregate can be
 * initialized.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int converter_frontend_add(struct device *dev, const struct frontend_ops *ops);

/**
 * converter_get - Get a converter object
 * @frontend:	Frontend device.
 * @name:	Converter name.
 *
 * Get's a pointer to a converter device. If name is NULL, then it is assumed
 * that only one backend device is bond with the frontend and the first element
 * in the list is retrieved.
 *
 * RETURNS:
 * A converter pointer, negative error pointer otherwise.
 */
struct converter_backend *__must_check
converter_get(const struct converter_frontend *frontend, const char *name);

/**
 * converter_add_direct_reg_access - Add debugfs direct register access
 * @conv: Coverter device
 * @indio_dev: IIO device
 *
 * This is analogous to the typical IIO direct register access in debugfs. The
 * extra converter file will be added in the same debugs dir as @indio_dev.
 * Moreover, if @conv->name is NULL, the file will be called
 * converter_direct_reg_access. Otherwise, will be
 * @conv->name_converter_direct_reg_access.
 */
void converter_add_direct_reg_access(struct converter_backend *conv,
				     struct iio_dev *indio_dev);

#else

static inline void *converter_get_drvdata(const struct converter_backend *conv)
{
	WARN_ONCE(1, "converter API is disabled");
}

static inline void converter_set_drvdata(struct converter_backend *conv,
					 void *drvdata)
{
	WARN_ONCE(1, "converter API is disabled");
}

static inline void converter_set_regmap(struct converter_backend *conv,
					struct regmap *regmap)
{
	WARN_ONCE(1, "converter API is disabled");
}

static inline int
__converter_test_pattern_xlate(unsigned int pattern,
			       const struct converter_test_pattern_xlate *xlate,
			       int n_matches)
{
	WARN_ONCE(1, "converter API is disabled");
	return -ENOTSUPP;
}

static inline struct converter_backend *__must_check
converter_get(const struct converter_frontend *frontend, const char *name)
{
	WARN_ONCE(1, "converter API is disabled");
	return ERR_PTR(-ENOTSUPP);
}

static inline int converter_add(struct device *dev,
				const struct converter_ops *ops)
{
	WARN_ONCE(1, "converter API is disabled");
	return -ENOTSUPP;
}

static inline void converter_del(struct device *dev)
{
	WARN_ONCE(1, "converter API is disabled");
}

static inline int converter_enable(struct converter_backend *conv)
{
	WARN_ONCE(1, "converter API is disabled");
	return -ENOTSUPP;
}

static inline void converter_disable(struct converter_backend *conv)
{
	WARN_ONCE(1, "converter API is disabled");
}

static inline int
converter_test_pattern_set(struct converter_backend *conv,
			   unsigned int chan,
			   enum converter_test_pattern pattern)
{
	WARN_ONCE(1, "converter API is disabled");
	return -ENOTSUPP;
}

static inline int
converter_data_format_set(struct converter_backend *conv,
			  unsigned int chan,
			  const struct converter_data_fmt *data)
{
	WARN_ONCE(1, "converter API is disabled");
	return -ENOTSUPP;
}

static inline int converter_chan_enable(struct converter_backend *conv,
					unsigned int chan)
{
	WARN_ONCE(1, "converter API is disabled");
	return -ENOTSUPP;
}

static inline int converter_chan_disable(struct converter_backend *conv,
					 unsigned int chan)
{
	WARN_ONCE(1, "converter API is disabled");
	return -ENOTSUPP;
}

static inline int converter_iodelay_set(struct converter_backend *conv,
					unsigned int num_lanes,
					unsigned int val)
{
	WARN_ONCE(1, "converter API is disabled");
	return -ENOTSUPP;
}

static inline void
converter_add_direct_reg_access(struct converter_backend *conv,
				struct iio_dev *indio_dev)
{
	WARN_ONCE(1, "converter API is disabled");
}

static inline int converter_frontend_add(struct device *dev,
					 const struct frontend_ops *ops)
{
	WARN_ONCE(1, "converter API is disabled");
	return -ENOTSUPP;
}

static inline void converter_frontend_del(struct device *dev)
{
	WARN_ONCE(1, "converter API is disabled");
}

#endif
#endif
