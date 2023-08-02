// SPDX-License-Identifier: GPL-2.0-only
/*
 * Framework to handle complex IIO aggregate devices
 *
 * A note on some of the design expectations with regards to lifetimes and
 * devices bringup/removal.
 *
 *
 *
 * Copyright (C) 2023 Analog Devices Inc.
 */

#define dev_fmt(fmt) "Converter - " fmt

#include <linux/component.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/iio/addc/converter.h>
#include <linux/iio/iio.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>

struct converter_backend {
	struct list_head entry;
	struct device *dev;
	const struct converter_ops *ops;
	const char *name;
	void *drvdata;

	struct regmap *regmap;
	unsigned int cached_reg_addr;

};

struct converter_frontend {
	struct list_head list;
	const struct frontend_ops *ops;
	struct device *dev;
};

static ssize_t converter_debugfs_read_reg(struct file *file,
					  char __user *userbuf,
					  size_t count, loff_t *ppos)
{
	struct converter_backend *conv = file->private_data;
	unsigned int val = 0;
	char read_buf[20];
	int ret, len;

	ret = regmap_read(conv->regmap, conv->cached_reg_addr, &val);
	if (ret) {
		dev_err(conv->dev, "%s: read failed\n", __func__);
		return ret;
	}

	len = scnprintf(read_buf, sizeof(read_buf), "0x%X\n", val);

	return simple_read_from_buffer(userbuf, count, ppos, read_buf, len);
}

static ssize_t converter_debugfs_write_reg(struct file *file,
					   const char __user *userbuf,
					   size_t count, loff_t *ppos)
{
	struct converter_backend *conv = file->private_data;
	unsigned int val;
	char buf[80];
	ssize_t rc;
	int ret;

	rc = simple_write_to_buffer(buf, sizeof(buf), ppos, userbuf, count);
	if (rc < 0)
		return rc;

	ret = sscanf(buf, "%i %i", &conv->cached_reg_addr, &val);

	switch (ret) {
	case 1:
		break;
	case 2:
		ret = regmap_write(conv->regmap, conv->cached_reg_addr, val);
		if (ret) {
			dev_err(conv->dev, "%s: write failed\n", __func__);
			return ret;
		}
		break;
	default:
		return -EINVAL;
	}

	return count;
}

static const struct file_operations converter_debugfs_reg_fops = {
	.open = simple_open,
	.read = converter_debugfs_read_reg,
	.write = converter_debugfs_write_reg,
};

static void __converter_add_direct_reg_access(struct converter_backend *conv,
					      struct iio_dev *indio_dev)
{
	struct dentry *d = iio_get_debugfs_dentry(indio_dev);
	const char *name = conv->name;
	char file_name[64];

	if (!conv->regmap)
		return;
	if (!d)
		return;

	if (!conv->name)
		name = "converter";

	snprintf(file_name, sizeof(file_name), "%s_direct_reg_access", name);

	debugfs_create_file(file_name, 0644, d, conv,
			    &converter_debugfs_reg_fops);
}

void converter_add_direct_reg_access(struct converter_backend *conv,
				     struct iio_dev *indio_dev)
{
	if (IS_ENABLED(CONFIG_DEBUG_FS))
		__converter_add_direct_reg_access(conv, indio_dev);
}
EXPORT_SYMBOL_NS_GPL(converter_add_direct_reg_access, IIO_CONVERTER);

static int converter_bind(struct device *dev, struct device *aggregate,
			  void *data)
{
	struct converter_frontend *frontend = dev_get_drvdata(aggregate);
	struct converter_backend *conv = dev_get_drvdata(dev);
	int ret;

	ret = conv->ops->backend_init(conv, dev);
	if (ret)
		return ret;

	list_add_tail(&conv->entry, &frontend->list);

	return 0;
}

static void converter_unbind(struct device *dev, struct device *aggregate,
			     void *data)
{
	struct converter_backend *conv = dev_get_drvdata(dev);

	if (conv->ops->backend_close)
		conv->ops->backend_close(conv);

	/* after this point the converter should not be used anymore */
	converter_set_drvdata(conv, NULL);
}

static const struct component_ops converter_component_ops = {
	.bind = converter_bind,
	.unbind = converter_unbind,
};

static int converter_frontend_bind(struct device *dev)
{
	struct converter_frontend *frontend = dev_get_drvdata(dev);
	int ret;

	ret = component_bind_all(dev, NULL);
	if (ret)
		return ret;
	/*
	 * Add comment about the expectations between bind and unbind ordering
	 * with respect to resources;
	 */
	if (!devres_open_group(dev, frontend, GFP_KERNEL))
		return -ENOMEM;

	ret = frontend->ops->frontend_init(frontend, dev);
	if (ret) {
		devres_release_group(dev, frontend);
		return ret;
	}

	devres_close_group(dev, NULL);
	return 0;
}

static void converter_frontend_unbind(struct device *dev)
{
	struct converter_frontend *frontend = dev_get_drvdata(dev);

	if (frontend->ops->frontend_close)
		frontend->ops->frontend_close(frontend);

	devres_release_group(dev, frontend);
	component_unbind_all(dev, NULL);
	list_del_init(&frontend->list);
}

static const struct component_master_ops frontend_component_ops = {
	.bind = converter_frontend_bind,
	.unbind = converter_frontend_unbind,
};

struct converter_backend *converter_get(const struct converter_frontend *frontend,
					const char *name)
{
	struct converter_backend *iter, *conv = NULL;
	struct device *dev = frontend->dev;
	struct fwnode_handle *fwnode;
	int index = 0;

	if (list_empty(&frontend->list)) {
		dev_err(dev, "Backend list is empty! They need to add themselves...\n");
		return ERR_PTR(-ENODEV);
	}

	/* if no name given, we assume only one converter_backend exists */
	if (!name)
		return list_first_entry(&frontend->list,
					struct converter_backend, entry);

	index = device_property_match_string(frontend->dev, "converter-names",
					     name);
	if (index < 0)
		return ERR_PTR(index);

	fwnode = fwnode_find_reference(dev_fwnode(dev), "converters", index);
	if (IS_ERR(fwnode))
		return ERR_CAST(fwnode);

	list_for_each_entry(iter, &frontend->list, entry) {
		if (device_match_fwnode(iter->dev, fwnode)) {
			conv = iter;
			break;
		}
	}

	fwnode_handle_put(fwnode);

	if (!conv) {
		dev_err(dev, "Converter (%s) not found in the list\n", name);
		return ERR_PTR(-ENODEV);
	}

	/* See if we can add device_property_string_read_index() */
	conv->name = kstrdup_const(name, GFP_KERNEL);
	if (!conv->name)
		return ERR_PTR(-ENOMEM);

	return conv;
}
EXPORT_SYMBOL_NS_GPL(converter_get, IIO_CONVERTER);

static int converter_frontend_add_matches(struct converter_frontend *frontend,
					  struct component_match **match)
{
	struct device *dev = frontend->dev;
	struct fwnode_handle *fwnode;
	int index = 0;

	do {
		fwnode = fwnode_find_reference(dev_fwnode(dev), "converters",
					       index);
		if (IS_ERR(fwnode))
			break;

		component_match_add_release(dev, match,
					    component_release_fwnode,
					    component_compare_fwnode, fwnode);
		index++;
	} while (true);

	/* no devices?! */
	if (!index) {
		dev_err(dev, "No converters. Make sure the \"converters\" property is given!\n");
		return -ENODEV;
	}

	if (PTR_ERR(fwnode) != -ENOENT)
		return PTR_ERR(fwnode);

	return 0;
}

int converter_test_pattern_set(struct converter_backend *conv,
			       unsigned int chan,
			       enum converter_test_pattern pattern)
{
	if (pattern >= CONVERTER_TEST_PATTERN_MAX)
		return -EINVAL;
	if (!conv->ops->test_pattern_set)
		return -ENOTSUPP;

	return conv->ops->test_pattern_set(conv, chan, pattern);
}
EXPORT_SYMBOL_NS_GPL(converter_test_pattern_set, IIO_CONVERTER);

int converter_chan_status_all(struct converter_backend *conv,
			      unsigned int num_chans)
{
	if (!numb_chan)
		return -EINVAL;
	if (!conv->ops->chans_status)
		return -ENOTSUPP;

	return conv->ops->chans_status(conv, numb_chans);
}
EXPORT_SYMBOL_NS_GPL(converter_test_pattern_status_all, IIO_CONVERTER);

int converter_iodelay_set(struct converter_backend *conv,
			  unsigned int num_lanes, unsigned int delay)
{
	if (!num_lanes)
		return -EINVAL;
	if (!conv->ops->iodelay_set)
		return -ENOTSUPP;

	return conv->ops->iodelay_set(conv, num_lanes, delay);
}
EXPORT_SYMBOL_NS_GPL(converter_iodelay_set, IIO_CONVERTER);

int converter_data_format_set(struct converter_backend *conv,
			      unsigned int chan,
			      const struct converter_data_fmt *data)
{
	if (data->type >= CONVERTER_DATA_TYPE_MAX)
		return -EINVAL;
	if (!conv->ops->data_format_set)
		return -ENOTSUPP;

	return conv->ops->data_format_set(conv, chan, data);
}
EXPORT_SYMBOL_NS_GPL(converter_data_format_set, IIO_CONVERTER);

int converter_edge_select(struct converter_backend *conv,
			  enum converter_edge edge)
{
	if (edge >= CONVERTER_EDGE_MAX)
		return -EINVAL;
	if (conv->ops->edge_select)
		return -ENOTSUPP;

	return conv->ops->edge_select(conv, edge);
}
EXPORT_SYMBOL_NS_GPL(converter_edge_select, IIO_CONVERTER);

int converter_chan_enable(struct converter_backend *conv, unsigned int chan)
{
	if (!conv->ops->chan_enable)
		return -ENOTSUPP;

	return conv->ops->chan_enable(conv, chan);
}
EXPORT_SYMBOL_NS_GPL(converter_chan_enable, IIO_CONVERTER);

int converter_chan_disable(struct converter_backend *conv, unsigned int chan)
{
	if (!conv->ops->disable)
		return -ENOTSUPP;

	return conv->ops->chan_disable(conv, chan);
}
EXPORT_SYMBOL_NS_GPL(converter_chan_disable, IIO_CONVERTER);

int converter_enable(struct converter_backend *conv)
{
	if (!conv->ops->enable)
		return -ENOTSUPP;

	return conv->ops->enable(conv);
}
EXPORT_SYMBOL_NS_GPL(converter_enable, IIO_CONVERTER);

void converter_disable(struct converter_backend *conv)
{
	if (!conv->ops->disable)
		return;

	conv->ops->disable(conv);
}
EXPORT_SYMBOL_NS_GPL(converter_disable, IIO_CONVERTER);

int __converter_test_pattern_xlate(unsigned int pattern,
				   const struct converter_test_pattern_xlate *xlate,
				   int n_matches)
{
	unsigned int p = n_matches;

	while (p--) {
		if (pattern == xlate[p].pattern)
			return xlate[p].reg_val;
	}

	return -EINVAL;
}
EXPORT_SYMBOL_NS_GPL(__converter_test_pattern_xlate, IIO_CONVERTER);

void converter_set_regmap(struct converter_backend *conv,
			  struct regmap *regmap)
{
	conv->regmap = regmap;
}
EXPORT_SYMBOL_NS_GPL(converter_set_regmap, IIO_CONVERTER);

void converter_set_drvdata(struct converter_backend *conv, void *drvdata)
{
	conv->drvdata = drvdata;
}
EXPORT_SYMBOL_NS_GPL(converter_set_drvdata, IIO_CONVERTER);

void *converter_get_drvdata(const struct converter_backend *conv)
{
	WARN_ON(!conv->drvdata);
	return conv->drvdata;
}
EXPORT_SYMBOL_NS_GPL(converter_get_drvdata, IIO_CONVERTER);

void converter_del(struct device *dev)
{
	component_del(dev, &converter_component_ops);
}
EXPORT_SYMBOL_NS_GPL(converter_del, IIO_CONVERTER);

static void converter_free(void *conv)
{
	struct converter_backend *__conv = conv;

	if (__conv->name)
		kfree_const(__conv->name);

	kfree(__conv);
}

int converter_add(struct device *dev, const struct converter_ops *ops)
{
	struct converter_backend *conv;
	int ret;

	if (!ops || !ops->backend_init)
		return -EINVAL;

	conv = kzalloc(sizeof(*conv), GFP_KERNEL);
	if (!conv)
		return -ENOMEM;

	ret = devm_add_action_or_reset(dev, converter_free, conv);
	if (ret)
		return ret;

	conv->ops = ops;
	dev_set_drvdata(dev, conv);
	conv->dev = dev;

	return component_add(dev, &converter_component_ops);
}
EXPORT_SYMBOL_NS_GPL(converter_add, IIO_CONVERTER);

void converter_frontend_del(struct device *dev)
{
	component_master_del(dev, &frontend_component_ops);
}
EXPORT_SYMBOL_NS_GPL(converter_frontend_del, IIO_CONVERTER);

int converter_frontend_add(struct device *dev, const struct frontend_ops *ops)
{
	struct converter_frontend *frontend;
	struct component_match *match;
	int ret;

	if (!ops || !ops->frontend_init) {
		dev_err(dev, "Mandatory ops missing\n");
		return -EINVAL;
	}

	frontend = devm_kzalloc(dev, sizeof(*frontend), GFP_KERNEL);
	if (!frontend)
		return -ENOMEM;

	frontend->ops = ops;
	frontend->dev = dev;
	INIT_LIST_HEAD(&frontend->list);
	dev_set_drvdata(dev, frontend);

	ret = converter_frontend_add_matches(frontend, &match);
	if (ret)
		return ret;

	return component_master_add_with_match(dev, &frontend_component_ops,
					       match);
}
EXPORT_SYMBOL_NS_GPL(converter_frontend_add, IIO_CONVERTER);

MODULE_AUTHOR("Nuno Sa <nuno.sa@analog.com>");
MODULE_DESCRIPTION("");
MODULE_LICENSE("GPL v2");
