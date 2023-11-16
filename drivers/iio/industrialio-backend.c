// SPDX-License-Identifier: GPL-2.0-only
/*
 * Framework to handle complex IIO aggregate devices
 *
 * Copyright (C) 2023 Analog Devices Inc.
 */
#define pr_fmt(fmt) "iio-backend: " fmt

#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/slab.h>

#include <linux/iio/backend.h>

struct iio_backend {
	struct list_head entry;
	const struct iio_backend_ops *ops;
	struct device *dev;
	struct module *owner;
	void *drvdata;
	/*
	 *
	 */
	struct mutex lock;
	struct kref ref;
};

static LIST_HEAD(iio_back_list);
static DEFINE_MUTEX(iio_back_lock);

#define iio_backend_op_call(back, op, args...) ({ \
	struct iio_backend *__back = back; \
	int __ret; \
			\
	guard(mutex)(&__back->lock); \
	if (WARN_ON_ONCE(!back->ops)) { \
		__ret = -ENODEV; \
	} else if (!__back->ops->op) { \
		__ret = -ENOTSUPP; \
	} else { \
		__ret = __back->ops->op(__back, ##args); \
	} \
		\
	__ret; \
})

#define iio_backend_void_op_call(back, op, args...) { \
	struct iio_backend *__back = back; \
						\
	guard(mutex)(&__back->lock); \
	if (!__back->ops) \
		WARN_ON_ONCE(1); \
	else if (!__back->ops->op) \
		dev_err(__back->dev, "Operation(%s) not supported\n", #op); \
	else \
		__back->ops->op(__back, ##args); \
}

int iio_backend_chan_enable(struct iio_backend *back, unsigned int chan)
{
	return iio_backend_op_call(back, chan_enable, chan);
}
EXPORT_SYMBOL_GPL(iio_backend_chan_enable);

int iio_backend_chan_disable(struct iio_backend *back, unsigned int chan)
{
	return iio_backend_op_call(back, chan_disable, chan);
}
EXPORT_SYMBOL_GPL(iio_backend_chan_disable);

int iio_backend_enable(struct iio_backend *back)
{
	return iio_backend_op_call(back, enable);
}
EXPORT_SYMBOL_GPL(iio_backend_enable);

int iio_backend_data_format_set(struct iio_backend *back, unsigned int chan,
				const struct iio_backend_data_fmt *data)
{
	if (data->type >= IIO_BACKEND_DATA_TYPE_MAX)
		return -EINVAL;

	return iio_backend_op_call(back, data_format_set, chan, data);

}
EXPORT_SYMBOL_GPL(iio_backend_data_format_set);

void iio_backend_disable(struct iio_backend *back)
{
	iio_backend_void_op_call(back, disable);
}
EXPORT_SYMBOL_GPL(iio_backend_disable);

static void iio_backend_free(struct kref *ref)
{
	struct iio_backend *back = container_of(ref, struct iio_backend, ref);

	pr_info("Freeing backend...\n");
	kfree(back);
}

static void iio_backend_release(void *arg)
{
	struct iio_backend *back = arg;

	pr_info("Releasing backend...\n");
	module_put(back->owner);
	kref_put(&back->ref, iio_backend_free);
}

struct iio_backend *devm_iio_backend_get(struct device *dev, const char *name)
{
	struct fwnode_handle *fwnode;
	struct iio_backend *back;
	int index = 0, ret;

	if (name) {
		index = device_property_match_string(dev, "backend-names",
						     name);
		if (index < 0)
			return ERR_PTR(index);
	}

	fwnode = fwnode_find_reference(dev_fwnode(dev), "backends", index);
	if (IS_ERR(fwnode)) {
		dev_err(dev, "Cannot get FW ref\n");
		return ERR_CAST(fwnode);
	}

	guard(mutex)(&iio_back_lock);
	list_for_each_entry(back, &iio_back_list, entry) {
		if (!device_match_fwnode(back->dev, fwnode))
			continue;

		fwnode_handle_put(fwnode);
		kref_get(&back->ref);
		if (!try_module_get(back->owner)) {
			dev_err(dev, "Cannot get module ref\n");
			return ERR_PTR(-ENODEV);
		}

		ret = devm_add_action_or_reset(dev, iio_backend_release, back);
		if (ret)
			return ERR_PTR(ret);

		return back;
	}

	fwnode_handle_put(fwnode);
	return ERR_PTR(-EPROBE_DEFER);
}
EXPORT_SYMBOL_GPL(devm_iio_backend_get);

void *iio_backend_get_drvdata(const struct iio_backend *back)
{
	return back->drvdata;
}
EXPORT_SYMBOL_GPL(iio_backend_get_drvdata);

static void iio_backend_unregister(void *arg)
{
	struct iio_backend *back = arg;

	mutex_lock(&iio_back_lock);
	list_del(&back->entry);
	mutex_unlock(&iio_back_lock);

	dev_info(back->dev, "Unregistering backend...\n");
	mutex_lock(&back->lock);
	back->ops = NULL;
	mutex_unlock(&back->lock);
	kref_put(&back->ref, iio_backend_free);
}

int devm_iio_backend_register(struct device *dev,
			      const struct iio_backend_ops *ops, void *drvdata)
{
	struct iio_backend *back;

	if (!ops || !drvdata) {
		dev_err(dev, "No backend ops or drvdata given\n");
		return -EINVAL;
	}

	back = kzalloc(sizeof(*back), GFP_KERNEL);
	if (!back)
		return -ENOMEM;

	kref_init(&back->ref);
	mutex_init(&back->lock);
	back->ops = ops;
	back->owner = dev->driver->owner;
	back->dev = dev;
	back->drvdata = drvdata;
	mutex_lock(&iio_back_lock);
	list_add(&back->entry, &iio_back_list);
	mutex_unlock(&iio_back_lock);

	return devm_add_action_or_reset(dev, iio_backend_unregister, back);
}
EXPORT_SYMBOL_GPL(devm_iio_backend_register);

MODULE_AUTHOR("Nuno Sa <nuno.sa@analog.com>");
MODULE_DESCRIPTION("Framework to handle complex IIO aggregate devices");
MODULE_LICENSE("GPL v2");
