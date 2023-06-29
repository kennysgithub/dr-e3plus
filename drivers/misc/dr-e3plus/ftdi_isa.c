/*
 * Driver for emulating ISA bus on an FTDI 2232H USB
 *
 */

#define DEBUG
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/mutex.h>

#include "ftdi_isa.h"

#ifdef CONFIG_USB_DYNAMIC_MINORS
#define	FTDI_ISA_MINOR_BASE	(0)
#else
#define	FTDI_ISA_MINOR_BASE	(201)	// FIXME?
#endif

#define	READ_NO_DEVICE		(0xFF)	// Simulate read on non-existant address

static const struct usb_device_id id_table[] = {
	{ USB_DEVICE(FTDI_VID, FTDI_2232H_PID) },
	{ },
};

struct ftdi_isa_device {
	struct usb_device *udev;
	struct usb_interface *interface;
	struct usb_host_interface *iface_desc;
	int opened;
	struct mutex intf_mutex;
	u8 bulk_in_ep;
	u8 bulk_out_ep;
	u8 *xfer_buffer;
	int xfer_buflen;
};

static DEFINE_MUTEX(ftdi_isa_mutex);
static struct usb_driver ftdi_isa_driver;
static struct ftdi_isa_device *ftdi_isa_dev;

static int ftdi_isa_recv_command(struct usb_device *dev, u8 req, u16 value, u16 index,
	void *buf, u16 size)
{
	dev_dbg(&dev->dev,
	  "%s(): dev %px req 0x%02X reqtype 0x%02X value 0x%04X, index 0x%04X, buf %px size 0x%02X\n",
	  __func__, dev, req,
	  USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
	  value, index, buf, size);
	return usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
		req,
		USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		value, index, buf, size,
		USB_CTRL_GET_TIMEOUT);
}

static int ftdi_isa_send_command(struct usb_device *dev, u8 req, u16 value, u16 index,
	void *buf, u16 size)
{
	dev_dbg(&dev->dev,
	  "%s(): dev %px req 0x%02X reqtype 0x%02X value 0x%04X, index 0x%04X, buf %px size 0x%02X\n",
	  __func__, dev, req,
	  USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
	  value, index, buf, size);
	return usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
		req,
		USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		value, index, buf, size,
		USB_CTRL_SET_TIMEOUT);
}

static int ftdi_isa_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct ftdi_isa_device *dev = NULL;
	struct usb_interface *interface;

	pr_debug(KERN_INFO "%s(): %d\n", __func__, __LINE__);

	ret = mutex_lock_interruptible(&ftdi_isa_mutex);
	if (ret)
		return ret;

	interface = usb_find_interface(&ftdi_isa_driver, iminor(inode));
	if (!interface) {
		pr_err("%s(): can't find FTDI dev for minor %d\n",
			__func__, iminor(inode));
		ret = -ENODEV;
		goto exit_locked;
	}

	dev = usb_get_intfdata(interface);
	if (!dev) {
		pr_err("%s(): no dev\n", __func__);
		ret = -EBUSY;
		goto exit_locked;
	}

	if (dev->opened) {
		ret = -EBUSY;
		goto exit_locked;
	} else {
		dev->opened = 1;
	};

	ret = 0;
exit_locked:
	mutex_unlock(&ftdi_isa_mutex);
	return ret;
}

static long
ftdi_isa_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	pr_debug(KERN_INFO "%s(): %d\n", __func__, __LINE__);
	return 0L;
}

#ifdef CONFIG_COMPAT
static long
ftdi_isa_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	pr_debug(KERN_INFO "%s(): %d\n", __func__, __LINE__);
	return 0L;
}
#endif

static int ftdi_isa_release(struct inode *inode, struct file *file)
{
	pr_debug(KERN_INFO "%s(): %d\n", __func__, __LINE__);
	return 0;
}

static int ftdi_mcu_write(u8 value, volatile void __iomem *_addr)
{
	unsigned addr = (unsigned) _addr;
	int bytes_xfered;
	int ret;
	u8 *cmdbuf;

	ret = mutex_lock_interruptible(&ftdi_isa_dev->intf_mutex);
	if (ret)
		return ret;

	cmdbuf = ftdi_isa_dev->xfer_buffer;
	cmdbuf[0] = WRITE_EXTENDED;
	cmdbuf[1] = addr >> 8;
	cmdbuf[2] = addr & 0xFF;
	cmdbuf[3] = value;

	ret = usb_bulk_msg(ftdi_isa_dev->udev,
		usb_sndbulkpipe(ftdi_isa_dev->udev, ftdi_isa_dev->bulk_out_ep),
		cmdbuf, 4, &bytes_xfered, USB_CTRL_SET_TIMEOUT);
	if (ret) {
		dev_info(&ftdi_isa_dev->interface->dev,
		  "%s(): ISA write cmd send 0x%04X:0x%02X ret %d bytes %d\n",
		__func__, addr, value, ret, bytes_xfered);
	}

	mutex_unlock(&ftdi_isa_dev->intf_mutex);
	return ret;
}

static int ftdi_mcu_read(const volatile void __iomem *_addr)
{
	unsigned addr = (unsigned) _addr;
	int bytes_xfered;
	int ret;
	u8 *cmdbuf;

	ret = mutex_lock_interruptible(&ftdi_isa_dev->intf_mutex);
	if (ret)
		return ret;

	cmdbuf = ftdi_isa_dev->xfer_buffer;
	cmdbuf[0] = READ_EXTENDED;
	cmdbuf[1] = addr >> 8;
	cmdbuf[2] = addr & 0xFF;
	cmdbuf[3] = SEND_IMMEDIATE;

	ret = usb_bulk_msg(ftdi_isa_dev->udev,
		usb_sndbulkpipe(ftdi_isa_dev->udev, ftdi_isa_dev->bulk_out_ep),
		cmdbuf, 4, &bytes_xfered, USB_CTRL_SET_TIMEOUT);
	if (ret) {
		dev_info(&ftdi_isa_dev->interface->dev,
		  "%s(): ISA read cmd send ret %d bytes %d\n", __func__, ret, bytes_xfered);
		goto out;
	}

	ret = usb_bulk_msg(ftdi_isa_dev->udev,
		usb_rcvbulkpipe(ftdi_isa_dev->udev, ftdi_isa_dev->bulk_in_ep),
		cmdbuf, 4, &bytes_xfered, USB_CTRL_SET_TIMEOUT);
	if (ret) {
		dev_info(&ftdi_isa_dev->interface->dev,
		  "%s(): ISA read cmd recv 0x%04X ret %d bytes %d\n",
			__func__, addr, ret, bytes_xfered);
		goto out;
	}

	dev_dbg(&ftdi_isa_dev->interface->dev,
	  "%s(): addr 0x%02X ret 0x%02X bytes %d\n", __func__,
	  addr, *(u8 *) cmdbuf, bytes_xfered);

	ret = cmdbuf[0];

out:
	mutex_unlock(&ftdi_isa_dev->intf_mutex);
	return ret;
}

u8 ftdi_isa_read(const volatile void __iomem *addr, int inb)
{
	u8 ret = 0;
	if (!ftdi_isa_dev) {
		pr_err("%s(): no device?!\n", __func__);
		return READ_NO_DEVICE;
	}

	dev_dbg(&ftdi_isa_dev->interface->dev, "%s(): addr %px %s\n",
		__func__, addr, inb ? "INB" : "");

	ret = ftdi_mcu_read(addr);
	return (ret < 0) ? READ_NO_DEVICE : ret;
}
EXPORT_SYMBOL(ftdi_isa_read);

void ftdi_isa_write(u8 value, volatile void __iomem *addr, int outb)
{
	if (!ftdi_isa_dev) {
		pr_err("%s(): no device?!\n", __func__);
		return;
	}

	dev_dbg(&ftdi_isa_dev->interface->dev, "%s(): value 0x%02X addr %px %s\n",
		__func__, value, (char *) addr, outb ? "OUTB" : "");

	ftdi_mcu_write(value, addr);
}
EXPORT_SYMBOL_GPL(ftdi_isa_write);

static const struct file_operations ftdi_isa_fops = {
	.owner		= THIS_MODULE,
	.open		= ftdi_isa_open,
	.release	= ftdi_isa_release,
	.llseek		= no_llseek,
	.unlocked_ioctl	= ftdi_isa_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= ftdi_isa_compat_ioctl,
#endif
};

static struct usb_class_driver ftdi_isa_class = {
	.name		= "usb/ftdi_isa%d",
	.fops		= &ftdi_isa_fops,
	.minor_base	= FTDI_ISA_MINOR_BASE,
};

static int ftdi_isa_probe(struct usb_interface *interface,
	const struct usb_device_id *id)
{
	int ret = ENODEV;
	struct usb_device *udev = interface_to_usbdev(interface);
	struct ftdi_isa_device *dev;
	struct usb_endpoint_descriptor *iep, *oep;
	int bytes_xfered;

	/* only handle the first interface */
	if (interface->cur_altsetting->desc.bInterfaceNumber)
		return -ENODEV;

	dev = kzalloc(sizeof(struct ftdi_isa_device), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->iface_desc = interface->cur_altsetting;

	if (usb_find_bulk_in_endpoint(dev->iface_desc, &iep) ||
	    usb_find_bulk_out_endpoint(dev->iface_desc, &oep)) {
		dev_err(&interface->dev, "%s(): Can't get bulk EPs\n",
		__func__);
		goto exit_memfree;
	}

	dev->bulk_in_ep = iep->bEndpointAddress;
	dev->bulk_out_ep = oep->bEndpointAddress;

	mutex_init(&dev->intf_mutex);
	dev->interface = interface;
	dev->udev = usb_get_dev(udev);

	/* Verify MCU Host Mode setup */

	ret = ftdi_isa_send_command(dev->udev,
		SIO_RESET_REQUEST,
		SIO_RESET_SIO,
		1,	/* "Interface A" */
		NULL,
		0);
	if (ret) {
		dev_err(&interface->dev, "FTDI reset ret %d\n", ret);
		goto exit_memfree;
	}

	ret = ftdi_isa_send_command(dev->udev,
		SIO_SET_BITMODE_REQUEST,
		(BITMODE_RESET << 8) | 0,
		1,	/* "Interface A" */
		NULL,
		0);
	if (ret) {
		dev_err(&interface->dev, "FTDI bitmode reset ret %d\n", ret);
		goto exit_memfree;
	}

	ret = ftdi_isa_send_command(dev->udev,
		SIO_SET_BITMODE_REQUEST,
		(BITMODE_MCU << 8) | 0,
		1,	/* "Interface A" */
		NULL,
		0);
	if (ret) {
		dev_err(&interface->dev, "FTDI bitmode MCU set ret %d\n", ret);
		goto exit_memfree;
	}

	dev->xfer_buflen = min(usb_endpoint_maxp(iep), usb_endpoint_maxp(oep));
	dev->xfer_buffer = kmalloc(dev->xfer_buflen, GFP_KERNEL);
	if (!dev->xfer_buffer) {
		dev_err(&interface->dev, "Can't allocate xfer buffer\n");
		goto exit_memfree;
	}

	ret = ftdi_isa_recv_command(dev->udev,
		SIO_GET_LATENCY_TIMER_REQUEST,
		0,
		1,	/* "Interface A" */
		dev->xfer_buffer,
		1);
	if (ret < 0) {
		dev_err(&interface->dev, "FTDI latency timer get ret %d\n", ret);
		goto exit_memfree;
	} else {
		dev_info(&interface->dev, "FTDI latency timer val %d\n", (int) dev->xfer_buffer[0]);
	}

	/* setup clock Divide-by-5 */
	/* setup times are only 16.7uS otherwise */
	dev->xfer_buffer[0] = ENABLE_DIVIDE_BY_5;
	ret = usb_bulk_msg(dev->udev,
		usb_sndbulkpipe(dev->udev, dev->bulk_out_ep),
		dev->xfer_buffer, 1, &bytes_xfered, USB_CTRL_SET_TIMEOUT);
	if (ret) {
		dev_err(&dev->interface->dev,
		  "%s(): Div-by-5 cmd send ret %d bytes %d\n", __func__, ret, bytes_xfered);
		goto exit_usbbuf_free;
	}

	/* Complete registration */
	usb_set_intfdata(interface, dev);
	ret = usb_register_dev(interface, &ftdi_isa_class);
	if (ret)
		goto exit_usbfree;

	mutex_lock(&ftdi_isa_mutex);
	if (!ftdi_isa_dev)
		ftdi_isa_dev = dev;
	mutex_unlock(&ftdi_isa_mutex);

	dev_info(&interface->dev, "FTDI-to-ISA Bus driver intf %d initialized\n",
		dev->iface_desc->desc.bInterfaceNumber);
	return 0;

exit_usbfree:
	usb_set_intfdata(interface, NULL);
exit_usbbuf_free:
	kfree(dev->xfer_buffer);
	usb_put_dev(dev->udev);
exit_memfree:
	kfree(dev);
	return ret;
}

static void ftdi_isa_disconnect(struct usb_interface *interface)
{
	struct ftdi_isa_device *dev;

	/* device is hardwired, so just do the basics, but warn anyway */
	dev_warn(&interface->dev, "WARNING: device disconnected\n");

	mutex_lock(&ftdi_isa_mutex);
	if (!ftdi_isa_dev)
		goto out;

	dev = usb_get_intfdata(interface);
	if (!dev)
		goto out;

	usb_deregister_dev(interface, &ftdi_isa_class);
	usb_set_intfdata(interface, NULL);
	kfree(dev->xfer_buffer);
	usb_put_dev(dev->udev);
	kfree(dev);
out:
	mutex_unlock(&ftdi_isa_mutex);
}

static struct usb_driver ftdi_isa_driver = {
	.name		= "ftdi_isa",
	.probe		= ftdi_isa_probe,
	.disconnect	= ftdi_isa_disconnect,
	.id_table	= id_table,
};

module_usb_driver(ftdi_isa_driver);

MODULE_DEVICE_TABLE(usb, id_table);
MODULE_AUTHOR("Dynamic Ratings, INC.");
MODULE_LICENSE("GPL");
