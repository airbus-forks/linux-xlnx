// SPDX-License-Identifier: GPL-2.0+
/*
 * Xilinx Inter Processor Interrupt(IPI) Mailbox Client Driver
 *
 * Copyright (c) 2025 Airbus Defence and Space.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox/zynqmp-ipi-message.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>

#define DRIVER_NAME "zynqmp-ipi-mbox-client"
#define DRIVER_VERSION "1.0"

#define ZYNQMP_IPI_CLIENT_MSG_SIZE 32
#define ZYNQMP_IPI_CLIENT_FIFO_MSGS 16
#define ZYNQMP_IPI_CLIENT_FIFO_SIZE (ZYNQMP_IPI_CLIENT_FIFO_MSGS * ZYNQMP_IPI_CLIENT_MSG_SIZE)

struct zynqmp_ipi_client_counters {
	atomic_t rx;
	atomic_t tx;
	atomic_t dropped;
};

struct zynqmp_ipi_client_message {
	size_t len;
	u8 data[ZYNQMP_IPI_CLIENT_MSG_SIZE];
};

struct zynqmp_ipi_client_info {
	const char *name;
	struct device *dev;
	struct miscdevice miscdev;
	struct mbox_client client;
	struct mbox_chan *rx_chan;
	struct mbox_chan *tx_chan;
	struct zynqmp_ipi_client_counters counters;
	struct kfifo fifo;
	wait_queue_head_t wq;
	spinlock_t fifo_lock;
};

static atomic_t devn;

/**
 * Display device name
 *
 * @dev: device
 * @attr: attribute
 * @buf: input buffer
 */
static ssize_t name_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct zynqmp_ipi_client_info *info = (struct zynqmp_ipi_client_info *) dev_get_drvdata(dev->parent);

	return sysfs_emit(buf, "%s.\n", info->name);
}
/**
 * Display driver version
 *
 * @dev: device
 * @attr: attribute
 * @buf: input buffer
 */
static ssize_t version_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "version: %s.\n", DRIVER_VERSION);
}

/**
 * Display counters
 *
 * @dev: device
 * @attr: attribute
 * @buf: input buffer
 */
static ssize_t counters_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct zynqmp_ipi_client_info *info = (struct zynqmp_ipi_client_info *) dev_get_drvdata(dev->parent);

	return sysfs_emit(buf, "rx: %d, tx: %d, dropped: %d.\n", atomic_read(&info->counters.rx),
								 atomic_read(&info->counters.tx),
								 atomic_read(&info->counters.dropped));
}

/**
 * Reset Rx fifo
 *
 * @dev: device
 * @attr: attribute
 * @buf: input buffer
 * @count: input size
 */
static ssize_t reset_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct zynqmp_ipi_client_info *info = (struct zynqmp_ipi_client_info *)dev_get_drvdata(dev->parent);

	spin_lock(&info->fifo_lock);
	/* reset rx fifo */
	kfifo_reset(&info->fifo);
	spin_unlock(&info->fifo_lock);

	return count;
}

static DEVICE_ATTR_RO(name);
static DEVICE_ATTR_RO(version);
static DEVICE_ATTR_RO(counters);
static DEVICE_ATTR_WO(reset);

static struct attribute *zynqmp_ipi_client_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_version.attr,
	&dev_attr_counters.attr,
	&dev_attr_reset.attr,
	NULL,
};

/*
 * Device interface
 */
ATTRIBUTE_GROUPS(zynqmp_ipi_client);

/**
 * Read operation
 *
 * Used to display descriptor table content and indexes
 *
 * @file: file informations
 * @user_buffer: user space buffer to use
 * @size: user space buffer size
 * @offset: in our case the driver use it in order to progressively
 * return the descriptor table content.
 */
static ssize_t zynqmp_ipi_client_read(struct file *file, char __user *user_buffer,
		size_t size, loff_t *offset)
{
	struct miscdevice *dev = (struct miscdevice *) file->private_data;
	struct zynqmp_ipi_client_info *info = (struct zynqmp_ipi_client_info *) dev_get_drvdata(dev->parent);
	unsigned int copied;
	int rc, len;

	spin_lock(&info->fifo_lock);
	len = kfifo_len(&info->fifo);
	spin_unlock(&info->fifo_lock);
	if (len == 0) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
	}

	/* sleep if kfifo is empty */
	rc = wait_event_interruptible(info->wq, kfifo_len(&info->fifo) != 0);
	if (rc)
		return rc;

	spin_lock(&info->fifo_lock);
	rc = kfifo_to_user(&info->fifo, user_buffer, size, &copied);
	spin_unlock(&info->fifo_lock);
	if (!rc)
		rc = copied;

	return rc;
}

/**
 * Write operation
 *
 * Send a message on a specific channel
 *
 * @file: file informations
 * @user_buffer: user space buffer to use
 * @size: user space buffer size
 * @offset: user offset
 */
static ssize_t zynqmp_ipi_client_write(struct file *file, const char __user *user_buffer,
		size_t size, loff_t *offset)
{
	struct miscdevice *dev = (struct miscdevice *) file->private_data;
	struct zynqmp_ipi_client_info *info = (struct zynqmp_ipi_client_info *) dev_get_drvdata(dev->parent);
	struct zynqmp_ipi_client_message msg = {0};
	int rc;

	if (!size)
		return size;
	if (size > ZYNQMP_IPI_CLIENT_MSG_SIZE)
		size = ZYNQMP_IPI_CLIENT_MSG_SIZE;

	if (copy_from_user(&msg.data, user_buffer, size))
		return -EFAULT;

	dev_dbg(info->dev, "send %zu bytes.\n", size);

	/* force length to max supported */
	msg.len = ZYNQMP_IPI_CLIENT_MSG_SIZE;

	rc = mbox_send_message(info->tx_chan, &msg);
	if (rc < 0) {
		dev_err(info->dev, "mbox_send_message failed (%d).\n", rc);
		return rc;
	}

	atomic_fetch_inc(&info->counters.tx);

	return size;
}

/**
 * Char device operations
 */
static const struct file_operations zynqmp_ipi_client_fops = {
	.owner = THIS_MODULE,
	.open = nonseekable_open,
	.read = zynqmp_ipi_client_read,
	.write = zynqmp_ipi_client_write,
	.llseek = noop_llseek,
};

/**
 * Rx callback function
 *
 * Display message received
 *
 * @cl: mailbox client
 * @mssg: pointer to the message received
 */
static void zynqmp_ipi_client_rx_callback(struct mbox_client *cl, void *mssg)
{
	struct zynqmp_ipi_client_info *info = (struct zynqmp_ipi_client_info *)dev_get_drvdata(cl->dev);
	struct zynqmp_ipi_message *msg = (struct zynqmp_ipi_message *)mssg;
	int rc;

	if (msg->len) {
		dev_dbg(info->dev, "%ld bytes received.\n", msg->len);

		/* put data in fifo */
		rc = kfifo_in_spinlocked(&info->fifo, msg->data, msg->len, &info->fifo_lock);
		if (!rc) {
			dev_warn(info->dev, "rx fifo full, message is dropped.\n");
			atomic_fetch_inc(&info->counters.dropped);
		} else
			atomic_fetch_inc(&info->counters.rx);

		/* wake up reader(s) if any */
		wake_up_interruptible(&info->wq);

		/* send ack in order to enable IRQ */
		mbox_send_message(info->rx_chan, NULL);
	}
}

/**
 * Tx done callback function
 *
 * None
 *
 * @cl: mailbox client
 * @mssg: pointer to the message which has been sent
 * @r: status of last TX - OK or error
 */
static void zynqmp_ipi_client_tx_done_callback(struct mbox_client *cl, void *mssg, int r)
{
	struct zynqmp_ipi_client_info *info = (struct zynqmp_ipi_client_info *)dev_get_drvdata(cl->dev);

	dev_dbg(info->dev, "message sent.\n");
}

/**
 * Probe function
 *
 * Get informations from device tree, and initialize mailbox client
 *
 * @pdev: platform device informations
 *
 * return 0 if no error, else the error code
 */
static int zynqmp_ipi_client_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct zynqmp_ipi_client_info *info;
	int rc = 0;

	if (!node) {
		dev_err(&pdev->dev, "invalid driver entry in device tree.\n");
		return -ENODEV;
	}

	dev_dbg(&pdev->dev, "probing.\n");

	/* allocate info struct */
	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (unlikely(!info)) {
		dev_err(&pdev->dev, "unable to allocate memory.\n");
		return -ENOMEM;
	}

	/* fifo allocation */
	rc = kfifo_alloc(&info->fifo, ZYNQMP_IPI_CLIENT_FIFO_SIZE, GFP_KERNEL);
	if (rc) {
		dev_err(&pdev->dev, "unable to allocate memory.\n");
		rc = -ENOMEM;
		goto info_alloc_free;
	}

	info->dev = &pdev->dev;
	info->name = node->full_name;
	atomic_set(&info->counters.rx, 0);
	atomic_set(&info->counters.tx, 0);
	atomic_set(&info->counters.dropped, 0);
	spin_lock_init(&info->fifo_lock);

	if (of_find_property(pdev->dev.of_node, "mboxes", NULL)) {
		/* setup mailbox channel client (non-blocking mode) */
		info->client.dev = &pdev->dev;
		info->client.rx_callback = zynqmp_ipi_client_rx_callback;
		info->client.tx_block = false;
		info->client.tx_tout = 0; /* doesn't matter here */
		info->client.knows_txdone = false;
		info->client.tx_done = zynqmp_ipi_client_tx_done_callback;

		info->tx_chan = mbox_request_channel_byname(&info->client, "tx");
		if (IS_ERR(info->tx_chan)) {
			dev_err(&pdev->dev, "failed to request tx channel.\n");
			rc = -EPROBE_DEFER;
			goto fifo_alloc_free;
		}

		info->rx_chan = mbox_request_channel_byname(&info->client, "rx");
		if (IS_ERR(info->rx_chan)) {
			dev_err(&pdev->dev, "failed to request rx channel.\n");
			rc = -EPROBE_DEFER;
			goto tx_channel_release;
		}
	} else {
		dev_err(&pdev->dev, "mboxes property not found in device tree node.\n");
		rc = -ENOENT;
		goto fifo_alloc_free;
	}

	/* set driver data */
	platform_set_drvdata(pdev, info);

	/* misc device */
	info->miscdev.minor = MISC_DYNAMIC_MINOR;
	info->miscdev.name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "mbox%d", atomic_fetch_inc(&devn));
	info->miscdev.fops = &zynqmp_ipi_client_fops;
	info->miscdev.parent = info->dev;
	info->miscdev.groups = zynqmp_ipi_client_groups;

	rc = misc_register(&info->miscdev);
	if (rc) {
		dev_err(&pdev->dev, "unable to register misc device.\n");
		goto rx_channel_release;
	}

	/* wait queue */
	init_waitqueue_head(&info->wq);

	dev_info(&pdev->dev, "probed.\n");

	return 0;

rx_channel_release:
	if (info->rx_chan)
		mbox_free_channel(info->rx_chan);
tx_channel_release:
	if (info->tx_chan)
		mbox_free_channel(info->tx_chan);
fifo_alloc_free:
	kfifo_free(&info->fifo);
info_alloc_free:
	devm_kfree(&pdev->dev, info);
	info = NULL;

	return rc;
}

/**
 * Remove function
 *
 * @pdev: platform device informations
 *
 * return always 0
 */
static void zynqmp_ipi_client_remove(struct platform_device *pdev)
{
	struct zynqmp_ipi_client_info *info = (struct zynqmp_ipi_client_info *)platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "remove.\n");

	if (!info)
		goto removed;
	if (info->tx_chan)
		mbox_free_channel(info->tx_chan);
	if (info->rx_chan)
		mbox_free_channel(info->rx_chan);

	/* unregister misc device */
	misc_deregister(&info->miscdev);

	kfifo_free(&info->fifo);
	devm_kfree(&pdev->dev, info);

removed:
	dev_info(&pdev->dev, "removed.\n");
}

static const struct of_device_id zynqmp_ipi_client_driver_of_ids[] = {
	{ .compatible = "xlnx,zynqmp-ipi-mailbox-client", },
	{ .compatible = "ads,xilinx-mailbox-client", },
	{}
};

static struct platform_driver zynqmp_ipi_client_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = zynqmp_ipi_client_driver_of_ids,
	},
	.probe = zynqmp_ipi_client_probe,
	.remove = zynqmp_ipi_client_remove,
};

/**
 * Init function
 *
 * Register the driver and the filesystem
 */
static int __init zynqmp_ipi_client_init(void)
{
	/* clear device number */
	atomic_set(&devn, 0);

	return platform_driver_register(&zynqmp_ipi_client_driver);
}
/**
 * Exit function
 *
 * Unregister the driver and the filesystem
 *
 * Note: filesystem should be unmounted first
 */
static void __exit zynqmp_ipi_client_exit(void)
{
	platform_driver_unregister(&zynqmp_ipi_client_driver);
}

module_init(zynqmp_ipi_client_init)
module_exit(zynqmp_ipi_client_exit)

MODULE_AUTHOR("Rémi Debord <remi.debord@airbus.com>");
MODULE_DESCRIPTION("Xilinx ZynqMP IPI Mailbox client driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
