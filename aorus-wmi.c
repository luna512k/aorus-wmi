// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * aorus-wmi - SMBus adapters for Gigabyte AORUS motherboards
 *
 * Copyright (C) 2026 Luna Webster
 *
 * Enables Linux to access the SMBus devices found on Gigabyte AORUS
 * motherboards, including RGB lighting controllers in RAM and other
 * motherboard components such as memory SPD EEPROMs and temperature
 * sensors.
 *
 * AORUS firmware normally controls the motherboard's two AMD FCH SMBus
 * controllers through ACPI, which prevents Linux from safely using them
 * directly. This means devices connected to those buses, such as
 * RGB-enabled memory modules, may be inaccessible to Linux applications
 * such as OpenRGB.
 *
 * This driver works around that limitation by using the motherboard's WMI
 * interface to ask the firmware to perform SMBus transactions on its
 * behalf. It exposes the two SMBus controllers through the standard Linux
 * i2c subsystem, allowing existing kernel drivers and userspace
 * applications to access the connected devices normally.
 *
 * No kernel command-line options or ACPI resource-check workarounds are
 * required, and the driver does not directly claim or access the SMBus
 * I/O ports.
 */

#include <linux/acpi.h>
#include <linux/cleanup.h>
#include <linux/dmi.h>
#include <linux/i2c.h>
#include <linux/i2c-smbus.h>
#include <linux/jiffies.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <linux/wmi.h>

#define DRIVER_NAME		"aorus-wmi"

/*
 * GUID of the WMBB method entry in the firmware's _WDG table. Each _WDG
 * entry has its own GUID here, so matching the GUID alone is unambiguous.
 */
#define AORUS_WMI_GUID		"DEADBEEF-2001-0000-00A0-C90629100000"

/* Bus selector byte of the argument buffer. */
#define AORUS_WMI_BUS0		0x02	/* SMBus host at I/O 0x0b00 */
#define AORUS_WMI_BUS1		0x03	/* SMBus host at I/O 0x0b20 */

/* WMBB function codes implementing SMBus transactions. */
#define AORUS_WMI_QUICK_WRITE		0x62
#define AORUS_WMI_QUICK_READ		0x63
#define AORUS_WMI_SEND_BYTE		0x65
#define AORUS_WMI_RECEIVE_BYTE		0x66
#define AORUS_WMI_BYTE_WRITE		0x67
#define AORUS_WMI_BYTE_READ		0x68
#define AORUS_WMI_WORD_WRITE		0x69
#define AORUS_WMI_WORD_READ		0x6a
#define AORUS_WMI_BLOCK_WRITE		0x6b
#define AORUS_WMI_BLOCK_READ		0x5f

/*
 * Layout of the WMBB argument buffer, as overlaid by the AML on Arg2.
 * All multi-byte fields are little endian. The offsets are fixed by the
 * firmware: word data lives at offset 4, not 3 - getting this wrong makes
 * writes land on the wrong register while still reporting success.
 *
 *	[0]	bus selector (AORUS_WMI_BUS0 / AORUS_WMI_BUS1)
 *	[1]	slave address, pre-shifted: 7-bit address << 1. The AML sets
 *		the read bit itself for read transactions.
 *	[2]	command / register byte; send byte takes its value here
 *	[3]	byte value; also low byte of the block length
 *	[4..5]	word value
 *	[3..6]	block length (dword)
 *	[7..]	block payload
 *
 * Short buffers have been observed to break some function codes, so
 * always send at least AORUS_WMI_BUF_MIN bytes.
 */
#define AORUS_WMI_BUF_MIN	7
#define AORUS_WMI_BUF_MAX	(AORUS_WMI_BUF_MIN + I2C_SMBUS_BLOCK_MAX)

/* Result conventions of the firmware. */
#define AORUS_WMI_WRITE_OK	1	/* write transaction success */
#define AORUS_WMI_BYTE_ERR	0xffff	/* byte/quick read failure */
#define AORUS_WMI_WORD_ERR	0xffffffff	/* word read failure */
#define AORUS_WMI_BLOCK_ERR	0x8000	/* block read: status word bit 15 */
#define AORUS_WMI_BLOCK_NAK	0x0004	/* block read: status DEV_ERR bit */

/* 7-bit address of the first SPD EEPROM, used by the probe-time selftest. */
#define AORUS_WMI_SPD_ADDR	0x50
#define AORUS_WMI_SPD_SET_PAGE	0x36	/* DDR4 SPD page 0 select */
#define AORUS_WMI_SPD_TYPE	0x23	/* DDR4 SDRAM, SPD byte 0 */

/* Healthy transactions take ~1 ms; the firmware can stall for much longer. */
#define AORUS_WMI_SLOW_MS	50

struct aorus_wmi_adapter {
	struct i2c_adapter adap;
	struct aorus_wmi_data *data;
	u8 bus;
};

struct aorus_wmi_data {
	struct wmi_device *wdev;
	struct pci_dev *pdev;
	/*
	 * The AML serializes access to each SMBus host internally, but
	 * serialize driver side as well: it is cheap, keeps the result
	 * buffer handling sane and guards against our own re-entry.
	 */
	struct mutex lock;
	struct aorus_wmi_adapter bus0;
	struct aorus_wmi_adapter bus1;
};

static bool force;
module_param(force, bool, 0444);
MODULE_PARM_DESC(force,
		 "Bind even on boards not listed in the DMI table; also downgrades a failed SPD self-test to a warning");

/*
 * Boards this driver has been verified on. Extend this list with the DMI
 * board vendor/name of additional Gigabyte boards carrying the \GSA1 WMI
 * device, or load with force=1 to try an unlisted board.
 */
static const struct dmi_system_id aorus_wmi_dmi_table[] = {
	{
		.ident = "Gigabyte X570 AORUS XTREME",
		.matches = {
			DMI_EXACT_MATCH(DMI_BOARD_VENDOR,
					"Gigabyte Technology Co., Ltd."),
			DMI_EXACT_MATCH(DMI_BOARD_NAME,
					"X570 AORUS XTREME"),
		},
	},
	{ }
};

static int aorus_wmi_call(struct aorus_wmi_data *priv, int fn, const u8 *in,
			  size_t in_len, union acpi_object **result)
{
	struct acpi_buffer in_buf = { (acpi_size)in_len, (void *)in };
	struct acpi_buffer out = { ACPI_ALLOCATE_BUFFER, NULL };
	acpi_status status;

	/*
	 * The WMI core evaluates WMBB(instance, fn, in): the function code
	 * is passed as the method_id argument of the WMI API, and the bus
	 * selector / transaction parameters as the input buffer.
	 */
	status = wmidev_evaluate_method(priv->wdev, 0, fn, &in_buf, &out);
	if (ACPI_FAILURE(status)) {
		dev_dbg(&priv->wdev->dev, "WMBB failed: %s\n",
			acpi_format_exception(status));
		return -EIO;
	}

	*result = out.pointer;
	return 0;
}

/* Return the result payload of a WMBB call, or NULL if it is too short. */
static const u8 *aorus_wmi_result(const union acpi_object *result,
				  size_t min_len)
{
	if (!result || result->type != ACPI_TYPE_BUFFER ||
	    result->buffer.length < min_len)
		return NULL;
	return result->buffer.pointer;
}

static s32 aorus_wmi_smbus_xfer(struct i2c_adapter *adap, u16 addr,
				unsigned short flags, char read_write,
				u8 command, int size,
				union i2c_smbus_data *data)
{
	struct aorus_wmi_adapter *a;
	struct aorus_wmi_data *priv;
	union acpi_object *result = NULL;
	u8 buf[AORUS_WMI_BUF_MAX] = {};
	size_t in_len = AORUS_WMI_BUF_MIN;
	unsigned long start = jiffies;
	const u8 *res;
	int fn, ret;

	a = container_of(adap, struct aorus_wmi_adapter, adap);
	priv = a->data;

	/* The firmware only stubs out quick and byte transactions on bus 1. */
	if (a->bus != AORUS_WMI_BUS0 &&
	    (size == I2C_SMBUS_QUICK || size == I2C_SMBUS_BYTE))
		return -EOPNOTSUPP;

	buf[0] = a->bus;
	buf[1] = addr << 1;
	buf[2] = command;

	switch (size) {
	case I2C_SMBUS_QUICK:
		fn = read_write == I2C_SMBUS_READ ? AORUS_WMI_QUICK_READ
						  : AORUS_WMI_QUICK_WRITE;
		break;
	case I2C_SMBUS_BYTE:
		/* Send byte carries its value in the command field. */
		fn = read_write == I2C_SMBUS_READ ? AORUS_WMI_RECEIVE_BYTE
						  : AORUS_WMI_SEND_BYTE;
		break;
	case I2C_SMBUS_BYTE_DATA:
		if (read_write == I2C_SMBUS_WRITE) {
			fn = AORUS_WMI_BYTE_WRITE;
			buf[3] = data->byte;
		} else {
			fn = AORUS_WMI_BYTE_READ;
		}
		break;
	case I2C_SMBUS_WORD_DATA:
		if (read_write == I2C_SMBUS_WRITE) {
			fn = AORUS_WMI_WORD_WRITE;
			put_unaligned_le16(data->word, &buf[4]);
		} else {
			fn = AORUS_WMI_WORD_READ;
		}
		break;
	case I2C_SMBUS_BLOCK_DATA:
		if (read_write == I2C_SMBUS_WRITE) {
			if (data->block[0] > I2C_SMBUS_BLOCK_MAX)
				return -EINVAL;
			fn = AORUS_WMI_BLOCK_WRITE;
			put_unaligned_le32(data->block[0], &buf[3]);
			memcpy(&buf[7], &data->block[1], data->block[0]);
			in_len = AORUS_WMI_BUF_MIN + data->block[0];
		} else {
			fn = AORUS_WMI_BLOCK_READ;
		}
		break;
	default:
		return -EOPNOTSUPP;
	}

	guard(mutex)(&priv->lock);

	ret = aorus_wmi_call(priv, fn, buf, in_len, &result);

	/* Surface a stuck or wedged SMBus host in the log. */
	if (time_after(jiffies, start + msecs_to_jiffies(AORUS_WMI_SLOW_MS)))
		dev_warn_ratelimited(&priv->wdev->dev,
				     "WMBB transaction took %ums\n",
				     jiffies_to_msecs(jiffies - start));

	if (ret)
		return ret;

	if (read_write == I2C_SMBUS_WRITE) {
		/* Write transactions report success as a dword 1. */
		res = aorus_wmi_result(result, 4);
		if (!res)
			ret = -EIO;
		else
			ret = get_unaligned_le32(res) == AORUS_WMI_WRITE_OK ?
			      0 : -ENXIO;
		goto out_free;
	}

	switch (size) {
	case I2C_SMBUS_QUICK:
	case I2C_SMBUS_BYTE:
		res = aorus_wmi_result(result, 2);
		if (!res) {
			ret = -EIO;
			break;
		}
		ret = get_unaligned_le16(res) == AORUS_WMI_BYTE_ERR ?
		      -ENXIO : 0;
		if (!ret && size == I2C_SMBUS_BYTE)
			data->byte = get_unaligned_le16(res);
		break;
	case I2C_SMBUS_BYTE_DATA:
		res = aorus_wmi_result(result, 4);
		if (!res) {
			ret = -EIO;
			break;
		}
		ret = get_unaligned_le32(res) == AORUS_WMI_BYTE_ERR ?
		      -ENXIO : 0;
		if (!ret)
			data->byte = get_unaligned_le32(res);
		break;
	case I2C_SMBUS_WORD_DATA:
		res = aorus_wmi_result(result, 4);
		if (!res) {
			ret = -EIO;
			break;
		}
		ret = get_unaligned_le32(res) == AORUS_WMI_WORD_ERR ?
		      -ENXIO : 0;
		if (!ret)
			data->word = get_unaligned_le32(res);
		break;
	case I2C_SMBUS_BLOCK_DATA:
		/* Result is a status word, a count word, then the data. */
		res = aorus_wmi_result(result, 4);
		if (!res) {
			ret = -EIO;
			break;
		}
		if (get_unaligned_le16(res) & AORUS_WMI_BLOCK_ERR) {
			ret = get_unaligned_le16(res) & AORUS_WMI_BLOCK_NAK ?
			      -ENXIO : -EIO;
			break;
		}
		ret = 0;
		data->block[0] = min3((size_t)get_unaligned_le16(res + 2),
				      (size_t)I2C_SMBUS_BLOCK_MAX,
				      (size_t)(result->buffer.length - 4));
		memcpy(&data->block[1], res + 4, data->block[0]);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

out_free:
	kfree(result);
	return ret;
}

static u32 aorus_wmi_functionality(struct i2c_adapter *adap)
{
	struct aorus_wmi_adapter *a;
	u32 func = I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA |
		   I2C_FUNC_SMBUS_BLOCK_DATA;

	a = container_of(adap, struct aorus_wmi_adapter, adap);

	/* Quick and send/receive byte are only implemented on bus 0. */
	if (a->bus == AORUS_WMI_BUS0)
		func |= I2C_FUNC_SMBUS_QUICK | I2C_FUNC_SMBUS_BYTE;

	return func;
}

static const struct i2c_algorithm aorus_wmi_algorithm = {
	.smbus_xfer	= aorus_wmi_smbus_xfer,
	.functionality	= aorus_wmi_functionality,
};

/*
 * Verify that the firmware behaves as expected before exposing any
 * adapter: a byte-data read of SPD byte 0 on bus 0 must return the DDR4
 * memory type code.
 */
static int aorus_wmi_selftest(struct aorus_wmi_data *priv)
{
	u8 buf[AORUS_WMI_BUF_MIN] = {};
	union acpi_object *result = NULL;
	const u8 *res;
	u32 val;
	int ret;

	guard(mutex)(&priv->lock);

	/*
	 * DDR4 SPD EEPROMs expose 256-byte pages; a page is selected by a
	 * write to 0x36 (page 0) or 0x37 (page 1) on the same bus and the
	 * selection latches until the next selection, even across reboots.
	 * Earlier readers may have left page 1 selected, so select page 0
	 * first. The data byte is ignored by the SPD, and the write is
	 * harmless if no DDR4 SPD is present.
	 */
	buf[0] = AORUS_WMI_BUS0;
	buf[1] = AORUS_WMI_SPD_SET_PAGE << 1;
	buf[2] = 0x00;

	aorus_wmi_call(priv, AORUS_WMI_SEND_BYTE, buf, sizeof(buf), &result);
	kfree(result);
	result = NULL;

	buf[1] = AORUS_WMI_SPD_ADDR << 1;

	ret = aorus_wmi_call(priv, AORUS_WMI_BYTE_READ, buf, sizeof(buf), &result);
	if (ret)
		return ret;

	res = aorus_wmi_result(result, 4);
	if (res) {
		val = get_unaligned_le32(res);
		if (val != AORUS_WMI_BYTE_ERR && (val & 0xff) == AORUS_WMI_SPD_TYPE)
			ret = 0;
		else
			ret = -ENODEV;
		if (ret)
			dev_dbg(&priv->wdev->dev,
				"SPD self-test read 0x%08x\n", val);
	} else {
		ret = -EIO;
	}

	kfree(result);

	return ret;
}

static void aorus_wmi_pci_dev_put(void *pdev)
{
	pci_dev_put(pdev);
}

static int aorus_wmi_add_adapter(struct aorus_wmi_data *priv,
				 struct aorus_wmi_adapter *a, u8 bus,
				 const char *name)
{
	a->data = priv;
	a->bus = bus;

	a->adap.owner = THIS_MODULE;
	a->adap.algo = &aorus_wmi_algorithm;
	/*
	 * Parent the adapters at the FCH SMBus PCI device: the two SMBus
	 * hosts behind WMBB are its I/O ranges, so this reflects the real
	 * hardware topology in sysfs. It also lets userspace identify the
	 * DRAM-capable buses by the controller's PCI ID.
	 */
	if (priv->pdev)
		a->adap.dev.parent = &priv->pdev->dev;
	else
		a->adap.dev.parent = &priv->wdev->dev;
	strscpy(a->adap.name, name, sizeof(a->adap.name));

	return devm_i2c_add_adapter(&priv->wdev->dev, &a->adap);
}

static int aorus_wmi_probe(struct wmi_device *wdev, const void *context)
{
	struct aorus_wmi_data *priv;
	int ret;

	if (!dmi_check_system(aorus_wmi_dmi_table) && !force) {
		dev_dbg(&wdev->dev,
			"unknown board, load with aorus_wmi.force=1 to override\n");
		return -ENODEV;
	}

	priv = devm_kzalloc(&wdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->wdev = wdev;

	/* The FCH SMBus controller providing the hosts behind WMBB. */
	priv->pdev = pci_get_device(PCI_VENDOR_ID_AMD,
				    PCI_DEVICE_ID_AMD_KERNCZ_SMBUS, NULL);
	if (priv->pdev) {
		ret = devm_add_action_or_reset(&wdev->dev,
					       aorus_wmi_pci_dev_put,
					       priv->pdev);
		if (ret)
			return ret;
	}

	ret = devm_mutex_init(&wdev->dev, &priv->lock);
	if (ret)
		return ret;

	ret = aorus_wmi_selftest(priv);
	if (!ret) {
		dev_info(&wdev->dev, "SPD self-test passed\n");
	} else if (force) {
		/*
		 * force=1 is an explicit override of the board checks, so
		 * downgrading the self-test failure lets users probe boards
		 * with different SPD layouts.
		 */
		dev_warn(&wdev->dev,
			 "SPD self-test failed (%pe), continuing due to force=1\n",
			 ERR_PTR(ret));
	} else {
		dev_err(&wdev->dev, "SPD self-test failed (%pe)\n", ERR_PTR(ret));
		return ret;
	}

	ret = aorus_wmi_add_adapter(priv, &priv->bus0, AORUS_WMI_BUS0,
				    "SMBus AORUS WMI adapter 0");
	if (ret)
		return ret;

	/* Instantiate SPD EEPROM and memory temperature sensor clients. */
	i2c_register_spd_write_enable(&priv->bus0.adap);

	return aorus_wmi_add_adapter(priv, &priv->bus1, AORUS_WMI_BUS1,
				     "SMBus AORUS WMI adapter 1");
}

static const struct wmi_device_id aorus_wmi_id_table[] = {
	{ .guid_string = AORUS_WMI_GUID },
	{ }
};
MODULE_DEVICE_TABLE(wmi, aorus_wmi_id_table);

static struct wmi_driver aorus_wmi_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.probe_type	= PROBE_PREFER_ASYNCHRONOUS,
	},
	.id_table	= aorus_wmi_id_table,
	.probe		= aorus_wmi_probe,
};
module_wmi_driver(aorus_wmi_driver);

MODULE_AUTHOR("Luna Webster");
MODULE_DESCRIPTION("Enable access to the SMBus devices on Gigabyte AORUS motherboards via the board firmware's WMI interface");
MODULE_LICENSE("GPL");
