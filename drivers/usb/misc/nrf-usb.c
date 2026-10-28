// SPDX-License-Identifier: GPL-2.0
/*
 * nRF52833 USB driver — telemetry + commands
 *
 * Devices (CyberTech Labs):
 *   VID 0x2FE9  PID 0x0100
 *
 * The device exposes:
 *   ISO  IN  0x88  — isochronous telemetry stream (poll + read)
 *   INT  OUT 0x02  — low-latency command channel (write)
 *   BULK OUT       — bulk command channel (ioctl NRF_IO_BULK)
 *   BULK IN        — telemetry fallback when no ISO endpoint is present
 *
 * The kernel <-> userspace ABI (structs and ioctls) is defined in
 * <linux/nrf-usb.h> (include/uapi/linux/nrf-usb.h) — the same header is
 * shared with the userspace stack.
 *
 * Copyright (C) 2026 CyberTech Labs Ltd.
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/usb.h>

#include <linux/nrf-usb.h> /* UAPI: structs, ioctls, constants */

/* USB iProduct string of the TRIK periphery firmware (runs the app). */
#define NRF_PRODUCT		"periphery-module"

#define NRF_MAX_PACKETS_PER_URB	64
#define NRF_MAX_POOL_SIZE	64
#define NRF_MAX_RINGBUF_DEPTH	256
#define NRF_IO_TIMEOUT		5000	/* ms, synchronizing transfers */

/* Defaults, tunable via module parameters (taken at probe time). */
#define NRF_ISO_PACKETS_DEFAULT		4
#define NRF_POOL_SIZE_DEFAULT		4
#define NRF_RINGBUF_DEPTH_DEFAULT	8

static int nrf_packets_per_urb = NRF_ISO_PACKETS_DEFAULT;
module_param(nrf_packets_per_urb, int, 0644);
MODULE_PARM_DESC(nrf_packets_per_urb, "packets per ISO/telemetry URB");

static int nrf_pool_size = NRF_POOL_SIZE_DEFAULT;
module_param(nrf_pool_size, int, 0644);
MODULE_PARM_DESC(nrf_pool_size, "size of the receive URB pool");

static int nrf_ringbuf_depth = NRF_RINGBUF_DEPTH_DEFAULT;
module_param(nrf_ringbuf_depth, int, 0644);
MODULE_PARM_DESC(nrf_ringbuf_depth, "telemetry ring buffer depth (frames)");

static struct usb_driver nrf_driver;

/*
 * Per-device state.  The device owns the USB device/interface references;
 * every open file holds one extra reference (kref), so the structures stay
 * alive until the last handle is closed.
 */
struct nrf_device {
	struct usb_device	*udev;
	struct usb_interface	*interface;

	/* Pipes.  Zero means the endpoint was not present. */
	unsigned int		iso_pipe;
	unsigned int		int_pipe;
	unsigned int		bulk_in_pipe;
	unsigned int		bulk_out_pipe;

	/* Maximum packet size of the receive endpoint. */
	unsigned int		rx_maxp;

	/* Number of packets embedded in each receive URB. */
	unsigned int		packets_per_urb;
	unsigned int		pool_size;
	unsigned int		ringbuf_depth;

	/* Receive (telemetry) URB pool. */
	unsigned int		rx_pool_size;
	struct urb		**rx_pool;
	void			**rx_bufs;

	/* Fixed-slot ring buffer of telemetry frames. */
	uint8_t			*ringbuf;
	uint8_t			*ringbuf_len;
	unsigned int		ringbuf_head;
	unsigned int		ringbuf_tail;
	spinlock_t		ringbuf_lock;
	wait_queue_head_t	ringbuf_wait;

	/*
	 * Serializes outgoing transfers (interrupt OUT used by write() and
	 * the bulk OUT used by the ioctl) against each other and against
	 * disconnect.
	 */
	struct mutex		io_mutex;
	unsigned long		disconnected:1;
	struct kref		kref;
};

#define to_nrf_dev(d) container_of(d, struct nrf_device, kref)

static void nrf_kill_rx_pool(struct nrf_device *dev)
{
	unsigned int i;

	for (i = 0; i < dev->rx_pool_size; i++)
		usb_kill_urb(dev->rx_pool[i]);
}

static void nrf_delete(struct kref *kref)
{
	struct nrf_device *dev = to_nrf_dev(kref);
	unsigned int i;

	nrf_kill_rx_pool(dev);

	for (i = 0; i < dev->rx_pool_size; i++) {
		usb_free_urb(dev->rx_pool[i]);
		kfree(dev->rx_bufs[i]);
	}

	kfree(dev->rx_pool);
	kfree(dev->rx_bufs);
	kfree(dev->ringbuf);
	kfree(dev->ringbuf_len);
	usb_put_intf(dev->interface);
	usb_put_dev(dev->udev);
	kfree(dev);
}

/* ------------------------------------------------------------------ */
/* Telemetry ring buffer                                              */
/* ------------------------------------------------------------------ */

static void nrf_ringbuf_put(struct nrf_device *dev, const uint8_t *data, size_t len)
{
	unsigned long flags;
	unsigned int slot;

	spin_lock_irqsave(&dev->ringbuf_lock, flags);

	slot = dev->ringbuf_head * NRF_TELEMETRY_SIZE;
	len = min_t(size_t, len, NRF_TELEMETRY_SIZE);
	memcpy(dev->ringbuf + slot, data, len);
	dev->ringbuf_len[dev->ringbuf_head] = len;

	dev->ringbuf_head = (dev->ringbuf_head + 1) % dev->ringbuf_depth;
	if (dev->ringbuf_head == dev->ringbuf_tail)
		dev->ringbuf_tail = (dev->ringbuf_tail + 1) % dev->ringbuf_depth;

	spin_unlock_irqrestore(&dev->ringbuf_lock, flags);

	wake_up_interruptible(&dev->ringbuf_wait);
}

static bool nrf_ringbuf_avail(struct nrf_device *dev)
{
	unsigned long flags;
	bool avail;

	spin_lock_irqsave(&dev->ringbuf_lock, flags);
	avail = dev->ringbuf_tail != dev->ringbuf_head;
	spin_unlock_irqrestore(&dev->ringbuf_lock, flags);

	return avail;
}

/* Returns the length of the dequeued frame, 0 if the buffer is empty. */
static size_t nrf_ringbuf_get(struct nrf_device *dev, uint8_t *out)
{
	unsigned long flags;
	size_t len = 0;

	spin_lock_irqsave(&dev->ringbuf_lock, flags);

	if (dev->ringbuf_tail != dev->ringbuf_head) {
		unsigned int slot = dev->ringbuf_tail * NRF_TELEMETRY_SIZE;

		len = dev->ringbuf_len[dev->ringbuf_tail];
		memcpy(out, dev->ringbuf + slot, len);
		dev->ringbuf_tail = (dev->ringbuf_tail + 1) % dev->ringbuf_depth;
	}

	spin_unlock_irqrestore(&dev->ringbuf_lock, flags);

	return len;
}

/* ------------------------------------------------------------------ */
/* Receive path (ISO IN, or bulk IN as a fallback)                    */
/* ------------------------------------------------------------------ */

static void nrf_rx_complete(struct urb *urb)
{
	struct nrf_device *dev = urb->context;

	if (urb->status == 0 && urb->actual_length > 0)
		nrf_ringbuf_put(dev, urb->transfer_buffer, urb->actual_length);

	/* Disconnect path kills all URBs; do not resubmit after that. */
	if (urb->status == -ENOENT || urb->status == -ECONNRESET ||
	    urb->status == -ESHUTDOWN)
		return;

	usb_submit_urb(urb, GFP_ATOMIC);
}

static int nrf_submit_rx_pool(struct nrf_device *dev)
{
	unsigned int i;
	int ret;

	for (i = 0; i < dev->rx_pool_size; i++) {
		ret = usb_submit_urb(dev->rx_pool[i], GFP_KERNEL);
		if (ret && ret != -EPERM && ret != -ENODEV)
			return ret;
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* Char device file operations                                        */
/* ------------------------------------------------------------------ */

static int nrf_open(struct inode *inode, struct file *file)
{
	struct usb_interface *interface;
	struct nrf_device *dev;

	interface = usb_find_interface(&nrf_driver, iminor(inode));
	if (!interface)
		return -ENODEV;

	dev = usb_get_intfdata(interface);
	if (!dev)
		return -ENODEV;

	if (usb_autopm_get_interface(interface))
		return -ENODEV;

	kref_get(&dev->kref);
	file->private_data = dev;
	return 0;
}

static int nrf_release(struct inode *inode, struct file *file)
{
	struct nrf_device *dev = file->private_data;

	if (!dev)
		return -ENODEV;

	usb_autopm_put_interface(dev->interface);
	kref_put(&dev->kref, nrf_delete);
	return 0;
}

static ssize_t nrf_read(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	struct nrf_device *dev = file->private_data;
	uint8_t frame[NRF_TELEMETRY_SIZE];
	size_t len;
	int ret;

	if (!dev || count == 0)
		return 0;

	/*
	 * Deliver already received data without blocking, even if the device
	 * is being unplugged right now — no reason to drop telemetry that is
	 * already in the ring buffer.
	 */
	if (!nrf_ringbuf_avail(dev)) {
		if (dev->disconnected)
			return -ENODEV;

		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(dev->ringbuf_wait,
				dev->disconnected || nrf_ringbuf_avail(dev));
		if (ret < 0)
			return ret;

		if (dev->disconnected && !nrf_ringbuf_avail(dev))
			return -ENODEV;
	}

	len = nrf_ringbuf_get(dev, frame);
	len = min(len, count);
	if (copy_to_user(buf, frame, len))
		return -EFAULT;

	return len;
}

static __poll_t nrf_poll(struct file *file, struct poll_table_struct *wait)
{
	struct nrf_device *dev = file->private_data;
	__poll_t mask = 0;

	if (!dev)
		return EPOLLERR;

	poll_wait(file, &dev->ringbuf_wait, wait);

	if (nrf_ringbuf_avail(dev))
		mask |= EPOLLIN | EPOLLRDNORM;

	/* Writes are always allowed from the driver's point of view. */
	mask |= EPOLLOUT | EPOLLWRNORM;

	if (dev->disconnected)
		mask |= EPOLLERR | EPOLLHUP;

	return mask;
}

/* Send one frame over the interrupt OUT endpoint (used by write()). */
static int nrf_int_send(struct nrf_device *dev, const uint8_t __user *data, size_t len)
{
	void *buf;
	int ret;

	buf = memdup_user(data, len);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	mutex_lock(&dev->io_mutex);

	if (dev->disconnected || !dev->int_pipe) {
		ret = -ENODEV;
		goto out;
	}

	ret = usb_interrupt_msg(dev->udev, dev->int_pipe,
				buf, len, NULL, NRF_IO_TIMEOUT);
out:
	mutex_unlock(&dev->io_mutex);
	kfree(buf);

	return ret;
}

/* Send one frame over the bulk OUT endpoint (used by ioctl NRF_IO_BULK). */
static int nrf_bulk_send(struct nrf_device *dev, const void __user *data, size_t len)
{
	void *buf;
	int ret;

	buf = memdup_user(data, len);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	mutex_lock(&dev->io_mutex);

	if (dev->disconnected || !dev->bulk_out_pipe) {
		ret = -ENODEV;
		goto out;
	}

	ret = usb_bulk_msg(dev->udev, dev->bulk_out_pipe,
			   buf, len, NULL, NRF_IO_TIMEOUT);
out:
	mutex_unlock(&dev->io_mutex);
	kfree(buf);

	return ret;
}

static ssize_t nrf_write(struct file *file, const char __user *buf,
			 size_t count, loff_t *ppos)
{
	struct nrf_device *dev = file->private_data;

	if (!dev || count == 0)
		return 0;

	if (nrf_int_send(dev, buf, count) < 0)
		return -EIO;

	return count;
}

static long nrf_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct nrf_device *dev = file->private_data;
	void __user *uarg = (void __user *)arg;
	int ret;

	if (!dev)
		return -ENODEV;

	switch (cmd) {
	case NRF_IO_CTRL: {
		struct nrf_ctrl ctrl;
		void *buf = NULL;
		bool in;

		if (copy_from_user(&ctrl, uarg, sizeof(ctrl)))
			return -EFAULT;

		if (ctrl.wLength > sizeof(ctrl.data)) {
			ret = -EINVAL;
			break;
		}

		/*
		 * The control buffer must not live on the kernel stack: with
		 * CONFIG_VMAP_STACK the USB core refuses to DMA-map it and
		 * fails the transfer with -EAGAIN.  Use a DMA-safe kmalloc'd
		 * bounce buffer instead.
		 */
		in = ctrl.bmRequestType & USB_DIR_IN;
		if (ctrl.wLength) {
			buf = kmalloc(ctrl.wLength, GFP_KERNEL);
			if (!buf)
				return -ENOMEM;

			if (!in && copy_from_user(buf, ctrl.data, ctrl.wLength)) {
				kfree(buf);
				return -EFAULT;
			}
		}

		mutex_lock(&dev->io_mutex);

		if (dev->disconnected) {
			mutex_unlock(&dev->io_mutex);
			kfree(buf);
			return -ENODEV;
		}

		/*
		 * Control endpoint 0 is bidirectional; the pipe direction
		 * must match bmRequestType or usb_submit_urb() warns and the
		 * HCD gets a bogus control direction.
		 */
		ret = usb_control_msg(dev->udev,
				      (in && ctrl.wLength) ?
					usb_rcvctrlpipe(dev->udev, 0) :
					usb_sndctrlpipe(dev->udev, 0),
				      ctrl.bRequest, ctrl.bmRequestType,
				      ctrl.wValue, ctrl.wIndex,
				      buf, ctrl.wLength, ctrl.timeout);

		mutex_unlock(&dev->io_mutex);

		/*
		 * The transfer is bidirectional: hand the response buffer
		 * (and status) back to userspace.
		 */
		if (ret > 0 && in)
			memcpy(ctrl.data, buf, min_t(size_t, ret, sizeof(ctrl.data)));

		kfree(buf);

		if (copy_to_user(uarg, &ctrl, sizeof(ctrl)))
			ret = -EFAULT;

		break;
	}
	case NRF_IO_BULK: {
		struct nrf_bulk bulk;

		if (copy_from_user(&bulk, uarg, sizeof(bulk)))
			return -EFAULT;

		if (!bulk.length) {
			ret = 0;
			break;
		}

		ret = nrf_bulk_send(dev, bulk.data, bulk.length);
		break;
	}
	default:
		return -ENOTTY;
	}

	/* Positive values are byte counts — normalize them to zero. */
	return ret < 0 ? ret : 0;
}

static const struct file_operations nrf_fops = {
	.owner		= THIS_MODULE,
	.read		= nrf_read,
	.write		= nrf_write,
	.open		= nrf_open,
	.release	= nrf_release,
	.poll		= nrf_poll,
	.unlocked_ioctl	= nrf_ioctl,
	.llseek		= noop_llseek,
};

static struct usb_class_driver nrf_class = {
	.name		= "nrf%d",
	.fops		= &nrf_fops,
	.minor_base	= 0,	/* dynamic minor allocation */
};

/* ------------------------------------------------------------------ */
/* USB plumbing                                                       */
/* ------------------------------------------------------------------ */

static const struct usb_device_id nrf_id_table[] = {
	{ USB_DEVICE(NRF_VID, NRF_PID) },
	{ }
};
MODULE_DEVICE_TABLE(usb, nrf_id_table);

/* Finds endpoints and fills in the pipe fields. */
static int nrf_find_endpoints(struct nrf_device *dev)
{
	struct usb_interface *interface = dev->interface;
	struct usb_host_interface *alt = interface->cur_altsetting;
	struct usb_endpoint_descriptor *ep_iso_in = NULL;
	struct usb_endpoint_descriptor *ep_int_out = NULL;
	struct usb_endpoint_descriptor *ep_bulk_in = NULL;
	struct usb_endpoint_descriptor *ep_bulk_out = NULL;
	int i;

	for (i = 0; i < alt->desc.bNumEndpoints; i++) {
		struct usb_endpoint_descriptor *ep = &alt->endpoint[i].desc;

		if (usb_endpoint_is_isoc_in(ep))
			ep_iso_in = ep;
		else if (usb_endpoint_is_int_out(ep))
			ep_int_out = ep;
		else if (usb_endpoint_is_bulk_in(ep))
			ep_bulk_in = ep;
		else if (usb_endpoint_is_bulk_out(ep))
			ep_bulk_out = ep;
	}

	if (ep_iso_in) {
		dev->iso_pipe = usb_rcvisocpipe(dev->udev,
						ep_iso_in->bEndpointAddress);
		dev->rx_maxp = usb_endpoint_maxp(ep_iso_in);
	} else if (ep_bulk_in) {
		dev->bulk_in_pipe = usb_rcvbulkpipe(dev->udev,
						    ep_bulk_in->bEndpointAddress);
		dev->rx_maxp = usb_endpoint_maxp(ep_bulk_in);
	} else {
		dev_err(&interface->dev,
			"no ISO IN or bulk IN telemetry endpoint found\n");
		return -ENODEV;
	}

	if (ep_int_out)
		dev->int_pipe = usb_sndintpipe(dev->udev,
					       ep_int_out->bEndpointAddress);
	if (ep_bulk_out)
		dev->bulk_out_pipe = usb_sndbulkpipe(dev->udev,
						     ep_bulk_out->bEndpointAddress);

	if (!dev->int_pipe && !dev->bulk_out_pipe) {
		dev_err(&interface->dev,
			"no interrupt OUT or bulk OUT command endpoint found\n");
		return -ENODEV;
	}

	return 0;
}

/* Allocates and submits the receive URB pool (ISO, or bulk-IN fallback). */
static int nrf_setup_rx_pool(struct nrf_device *dev)
{
	unsigned int i;
	int ret;

	dev->packets_per_urb = clamp(nrf_packets_per_urb, 1, NRF_MAX_PACKETS_PER_URB);
	dev->pool_size = clamp(nrf_pool_size, 1, NRF_MAX_POOL_SIZE);
	dev->rx_pool_size = dev->pool_size;

	dev->rx_pool = kcalloc(dev->rx_pool_size, sizeof(*dev->rx_pool),
			       GFP_KERNEL);
	if (!dev->rx_pool)
		return -ENOMEM;

	dev->rx_bufs = kcalloc(dev->rx_pool_size, sizeof(*dev->rx_bufs),
			       GFP_KERNEL);
	if (!dev->rx_bufs)
		return -ENOMEM;

	for (i = 0; i < dev->rx_pool_size; i++) {
		size_t buf_size = (size_t)dev->rx_maxp * dev->packets_per_urb;

		dev->rx_bufs[i] = kzalloc(buf_size, GFP_KERNEL);
		if (!dev->rx_bufs[i])
			return -ENOMEM;

		if (dev->iso_pipe) {
			int p;

			dev->rx_pool[i] = usb_alloc_urb(dev->packets_per_urb,
							GFP_KERNEL);
			if (!dev->rx_pool[i])
				return -ENOMEM;

			usb_fill_bulk_urb(dev->rx_pool[i], dev->udev,
					  dev->iso_pipe,
					  dev->rx_bufs[i], buf_size,
					  nrf_rx_complete, dev);
			dev->rx_pool[i]->number_of_packets =
				dev->packets_per_urb;
			dev->rx_pool[i]->transfer_flags |= URB_ISO_ASAP;
			dev->rx_pool[i]->interval = 1;
			for (p = 0; p < dev->packets_per_urb; p++) {
				dev->rx_pool[i]->iso_frame_desc[p].offset =
					p * dev->rx_maxp;
				dev->rx_pool[i]->iso_frame_desc[p].length =
					dev->rx_maxp;
			}
		} else {
			dev->rx_pool[i] = usb_alloc_urb(0, GFP_KERNEL);
			if (!dev->rx_pool[i])
				return -ENOMEM;

			usb_fill_bulk_urb(dev->rx_pool[i], dev->udev,
					  dev->bulk_in_pipe,
					  dev->rx_bufs[i], buf_size,
					  nrf_rx_complete, dev);
		}
	}

	ret = nrf_submit_rx_pool(dev);
	if (ret)
		return ret;

	return 0;
}

static int nrf_probe(struct usb_interface *interface,
		     const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	struct nrf_device *dev;
	char product[64];
	int ret;

	/*
	 * Only claim the TRIK periphery app interface (iProduct
	 * "periphery-module", interface class 0xFF vendor).  The same chip
	 * can expose other USB functions that must stay with their own
	 * Linux drivers:
	 *   - BT HCI   (class 0xE0/01/01) -> btusb
	 *   - MCUboot  (CDC ACM 0x02/0x0a) -> cdc_acm
	 * Grabbing one of those interfaces would break it for the rightful
	 * driver (e.g. /dev/ttyACM* for firmware updates via
	 * /dev/nrf52833-bl).
	 */
	if (interface->cur_altsetting->desc.bInterfaceClass !=
							USB_CLASS_VENDOR_SPEC)
		return -ENODEV;
	if (usb_string(udev, udev->descriptor.iProduct,
		       product, sizeof(product)) < 0 ||
	    strcmp(product, NRF_PRODUCT) != 0)
		return -ENODEV;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	kref_init(&dev->kref);
	mutex_init(&dev->io_mutex);
	spin_lock_init(&dev->ringbuf_lock);
	init_waitqueue_head(&dev->ringbuf_wait);

	dev->udev = usb_get_dev(udev);
	dev->interface = usb_get_intf(interface);
	dev->ringbuf_depth = clamp(nrf_ringbuf_depth, 2, NRF_MAX_RINGBUF_DEPTH);

	dev->ringbuf = kmalloc_array(dev->ringbuf_depth, NRF_TELEMETRY_SIZE,
				     GFP_KERNEL);
	if (!dev->ringbuf) {
		ret = -ENOMEM;
		goto error;
	}

	dev->ringbuf_len = kcalloc(dev->ringbuf_depth, sizeof(*dev->ringbuf_len),
				   GFP_KERNEL);
	if (!dev->ringbuf_len) {
		ret = -ENOMEM;
		goto error;
	}

	ret = nrf_find_endpoints(dev);
	if (ret)
		goto error;

	ret = nrf_setup_rx_pool(dev);
	if (ret)
		goto error;

	usb_set_intfdata(interface, dev);

	ret = usb_register_dev(interface, &nrf_class);
	if (ret) {
		dev_err(&interface->dev, "usb_register_dev failed: %d\n", ret);
		goto error;
	}

	dev_info(&interface->dev,
		 "nRF52833: telemetry via %s, commands via %s%s\n",
		 dev->iso_pipe ? "ISO IN" : "bulk IN",
		 dev->int_pipe ? "interrupt OUT" : "",
		 dev->bulk_out_pipe ? " + bulk OUT" : "");

	return 0;

error:
	kref_put(&dev->kref, nrf_delete);
	return ret;
}

static void nrf_disconnect(struct usb_interface *interface)
{
	struct nrf_device *dev = usb_get_intfdata(interface);

	usb_deregister_dev(interface, &nrf_class);

	if (!dev)
		return;

	dev_info(&interface->dev, "nRF52833 disconnected\n");

	mutex_lock(&dev->io_mutex);
	dev->disconnected = 1;
	mutex_unlock(&dev->io_mutex);

	nrf_kill_rx_pool(dev);

	wake_up_interruptible(&dev->ringbuf_wait);
	kref_put(&dev->kref, nrf_delete);
}

static struct usb_driver nrf_driver = {
	.name			= "nrf-usb",
	.probe			= nrf_probe,
	.disconnect		= nrf_disconnect,
	.id_table		= nrf_id_table,
	.supports_autosuspend	= 1,
};
module_usb_driver(nrf_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("nRF52833 USB driver — telemetry in + command out");
MODULE_AUTHOR("CyberTech Labs Ltd.");
