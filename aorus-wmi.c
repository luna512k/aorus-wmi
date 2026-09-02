// SPDX-License-Identifier: GPL-2.0-only
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

#include <linux/build_bug.h>
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
#include <linux/stddef.h>
#include <linux/unaligned.h>
#include <linux/version.h>
#include <linux/wmi.h>

#define DRIVER_NAME		"aorus-wmi"

/*
 * GUID of the WMBB method entry in the firmware's _WDG table. Each _WDG
 * entry has its own GUID here, so matching the GUID alone is unambiguous.
 * The GUID does not establish ABI compatibility across boards: the DMI
 * table below is the compatibility boundary (the firmware exposes no
 * version or capability register for this interface).
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
 * WMBB request format - the binary ABI implemented by the Gigabyte
 * \GSA1 WMBB AML method. Field offsets are firmware-fixed; do not
 * reorder. Byte data lives at [3], word data at [4..5] (little endian),
 * block length dword at [3..6] with payload from [7] - the fields
 * deliberately overlap. The function code travels as the WMI method_id
 * argument, never inside this buffer. Word data at offset 4 (not 3) is
 * load-bearing: getting it wrong makes writes land on the wrong device
 * register while the firmware still reports success.
 * Full ABI: Documentation/wmi/devices/aorus-wmi.rst
 *
 * Short buffers have been observed to break some function codes, so
 * always send at least AORUS_WMI_BUF_MIN bytes.
 */
struct aorus_wmi_req_byte  { u8 bus, addr, cmd, val; }  __packed;
struct aorus_wmi_req_word  { u8 bus, addr, cmd, pad; __le16 val; }  __packed;
struct aorus_wmi_req_block { u8 bus, addr, cmd; __le32 len; u8 payload[]; }  __packed;

static_assert(offsetof(struct aorus_wmi_req_byte,  val)     == 3);
static_assert(offsetof(struct aorus_wmi_req_word,  val)     == 4);
static_assert(offsetof(struct aorus_wmi_req_block, len)     == 3);
static_assert(offsetof(struct aorus_wmi_req_block, payload) == 7);

#define AORUS_WMI_BUF_MIN	7
#define AORUS_WMI_BUF_MAX	(AORUS_WMI_BUF_MIN + I2C_SMBUS_BLOCK_MAX)

/* Result conventions of the firmware. */
#define AORUS_WMI_WRITE_OK	1	/* write transaction success */
#define AORUS_WMI_BYTE_ERR	0xffff	/* byte/quick read failure */
#define AORUS_WMI_WORD_ERR	0xffffffff	/* word read failure */
#define AORUS_WMI_BLOCK_ERR	0x8000	/* block read: status word bit 15 */
#define AORUS_WMI_BLOCK_NAK	0x0004	/* block read: status DEV_ERR bit */

/*
 * Out-of-tree compatibility (dropped before upstream submission):
 * kernels before 7.2 predate wmidev_invoke_method() and do not provide
 * struct wmi_buffer; supply the minimal equivalent for the compat
 * transport below.
 */
#if KERNEL_VERSION(7, 2, 0) > LINUX_VERSION_CODE
struct wmi_buffer {
	size_t length;
	void *data;
};
#endif

/*
 * Healthy transactions take ~1 ms; a hung bus costs the firmware up to
 * ~600 ms before it gives up and kills the transaction (three-tier cost
 * model: healthy/NAK ~0.5-1 ms, stale-status entry up to +200 ms, hung
 * bus ~400-600 ms). Surface a wedged host with a rate-limited warning
 * well above the healthy band.
 */
#define AORUS_WMI_SLOW_MS	50

/* Adapter timeout: covers the ~600 ms firmware worst case with margin. */
#define AORUS_WMI_TIMEOUT	(2 * HZ)

struct aorus_wmi_adapter {
	struct i2c_adapter adap;
	struct aorus_wmi_data *data;
	u8 bus;
	u8 index;	/* adapter-relative bus number, for logging */
};

/*
 * Two lock levels exist on purpose: the kernel mutex below protects the
 * driver's own state (result buffer handling, re-entry), while the AML
 * mutexes (SME0/SME3) guard the physical SMBus hosts inside every WMBB
 * transaction. WMBB is a Serialized AML method, so ACPICA serializes all
 * invocations - both buses - at the method boundary; per-adapter kernel
 * locking would therefore deliver no cross-bus parallelism for the WMI
 * transport. Splitting this lock per adapter remains the designated
 * future optimization, to be reconsidered only after the complete AML
 * call graph (including the unmapped EZV* and GGG* methods) is verified.
 */
struct aorus_wmi_data {
	struct wmi_device *wdev;
	struct pci_dev *pdev;
	/* Two lock levels - see the rationale above. */
	struct mutex lock;
	struct aorus_wmi_adapter bus0;
	struct aorus_wmi_adapter bus1;
	/*
	 * Firmware transport, injectable for KUnit testing. Always
	 * points to aorus_wmi_exec() in production.
	 */
	int (*exec)(struct aorus_wmi_data *priv, int fn, size_t min_size,
		    const u8 *in, size_t in_len, struct wmi_buffer *result);
};

/*
 * Development override for unlisted boards; drop before upstream
 * submission.
 */
static bool force;
module_param(force, bool, 0444);
MODULE_PARM_DESC(force, "Bind even on boards not listed in the DMI table");

/*
 * Pack a WMBB request buffer for one SMBus transaction. Pure function:
 * no allocation, no I/O. Returns the function code (the WMI method_id
 * argument) on success, or a negative errno.
 */
static int aorus_wmi_pack_request(u8 bus, u16 addr, int size, char read_write,
				  u8 command,
				  const union i2c_smbus_data *data,
				  u8 *buf, size_t *in_len)
{
	union {
		struct aorus_wmi_req_byte byte;
		struct aorus_wmi_req_word word;
		struct aorus_wmi_req_block block;
	} *req = (void *)buf;
	int fn;

	/* The firmware only stubs out quick and byte transactions on bus 1. */
	if (bus != AORUS_WMI_BUS0 &&
	    (size == I2C_SMBUS_QUICK || size == I2C_SMBUS_BYTE))
		return -EOPNOTSUPP;

	memset(buf, 0, AORUS_WMI_BUF_MAX);
	*in_len = AORUS_WMI_BUF_MIN;

	/*
	 * The AML sets the read bit itself; the caller passes addr << 1.
	 * The i2c-core SMBus contract guarantees addr <= 0x7F, which makes
	 * the truncation to the u8 wire byte safe.
	 */
	req->byte.bus = bus;
	req->byte.addr = addr << 1;
	req->byte.cmd = command;

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
			req->byte.val = data->byte;
		} else {
			fn = AORUS_WMI_BYTE_READ;
		}
		break;
	case I2C_SMBUS_WORD_DATA:
		if (read_write == I2C_SMBUS_WRITE) {
			fn = AORUS_WMI_WORD_WRITE;
			req->word.val = cpu_to_le16(data->word);
		} else {
			fn = AORUS_WMI_WORD_READ;
		}
		break;
	case I2C_SMBUS_BLOCK_DATA:
		if (read_write == I2C_SMBUS_WRITE) {
			if (data->block[0] > I2C_SMBUS_BLOCK_MAX)
				return -EINVAL;
			fn = AORUS_WMI_BLOCK_WRITE;
			req->block.len = cpu_to_le32(data->block[0]);
			memcpy(req->block.payload, &data->block[1],
			       data->block[0]);
			*in_len = AORUS_WMI_BUF_MIN + data->block[0];
		} else {
			fn = AORUS_WMI_BLOCK_READ;
		}
		break;
	default:
		return -EOPNOTSUPP;
	}

	return fn;
}

/*
 * Minimum result-buffer length per function code, in bytes. Pure
 * function: the WMI core enforces this minimum on the result of every
 * invocation, and the transport seam passes it along.
 */
static size_t aorus_wmi_min_result_size(int fn)
{
	if (fn == AORUS_WMI_QUICK_READ || fn == AORUS_WMI_RECEIVE_BYTE)
		return 2;
	return 4;
}

#ifndef AORUS_WMI_KUNIT_TEST

/*
 * The real WMBB transport: one synchronous WMI method invocation
 * performs ready-wait, transaction, completion poll and status clear.
 * There is no asynchronous completion; the call never times out early,
 * so a new transaction can never race the firmware. The WMI core
 * unmarshals the ACPI result into a flat buffer of at least the
 * requested minimum length (8-byte aligned, freed by the caller).
 *
 * This function is only compiled into the production driver: it
 * references the WMI core, which KUnit/UML builds intentionally do not
 * resolve - tests replace the transport via priv->exec instead.
 *
 * Out-of-tree compatibility section: kernels before 7.2 only provide
 * the deprecated evaluate_method() API. This branch is dropped before
 * upstream submission.
 */
#if KERNEL_VERSION(7, 2, 0) <= LINUX_VERSION_CODE
static int aorus_wmi_exec(struct aorus_wmi_data *priv, int fn, size_t min_size,
			  const u8 *in, size_t in_len, struct wmi_buffer *result)
{
	const struct wmi_buffer in_buf = { .length = in_len, .data = (void *)in };

	return wmidev_invoke_method(priv->wdev, 0, fn, &in_buf, result,
				    min_size);
}
#else /* pre-7.2: emulate the invoke_method() contract over evaluate_method() */
static int aorus_wmi_exec(struct aorus_wmi_data *priv, int fn, const u8 *in,
			  size_t in_len, struct wmi_buffer *result)
{
	struct acpi_buffer in_buf = { (acpi_size)in_len, (void *)in };
	struct acpi_buffer out = { ACPI_ALLOCATE_BUFFER, NULL };
	union acpi_object *obj;
	acpi_status status;
	int ret = 0;

	status = wmidev_evaluate_method(priv->wdev, 0, fn, &in_buf, &out);
	if (ACPI_FAILURE(status))
		return -EIO;

	/*
	 * Replicate the invoke_method() guarantees: buffer-typed result,
	 * at least min_size bytes, linear data buffer owned by the caller.
	 */
	obj = out.pointer;
	if (!obj || obj->type != ACPI_TYPE_BUFFER ||
	    (size_t)obj->buffer.length < aorus_wmi_min_result_size(fn)) {
		kfree(obj);
		return -EIO;
	}

	result->length = obj->buffer.length;
	result->data = kmemdup(obj->buffer.pointer, obj->buffer.length,
			       GFP_KERNEL);
	if (!result->data)
		ret = -ENOMEM;

	kfree(obj);
	return ret;
}
#endif /* LINUX_VERSION_CODE */

#endif /* !AORUS_WMI_KUNIT_TEST */

/*
 * Parse a WMBB result buffer into Linux SMBus convention. Pure function:
 * no allocation, no I/O, and safe for every input - returned block
 * lengths are clamped both to I2C_SMBUS_BLOCK_MAX and to the actual
 * length of the result buffer. The WMI core guarantees the result to
 * hold at least the per-function minimum (see
 * aorus_wmi_min_result_size()); the length check below is defense in
 * depth so the parser stays safe for any transport implementation.
 */
static s32 aorus_wmi_parse_result(int size, char read_write,
				  const struct wmi_buffer *result,
				  union i2c_smbus_data *data)
{
	size_t min_len = size == I2C_SMBUS_QUICK || size == I2C_SMBUS_BYTE ?
			 2 : 4;

	/*
	 * The WMI core guarantees this minimum in production; the check is
	 * defense in depth so the parser stays safe for any transport.
	 */
	if (result->length < min_len)
		return -EIO;

	if (read_write == I2C_SMBUS_WRITE) {
		/* Write transactions report success as a dword 1. */
		return get_unaligned_le32(result->data) == AORUS_WMI_WRITE_OK ?
		       0 : -ENXIO;
	}

	switch (size) {
	case I2C_SMBUS_QUICK:
	case I2C_SMBUS_BYTE:
		if (get_unaligned_le16(result->data) == AORUS_WMI_BYTE_ERR)
			return -ENXIO;
		if (size == I2C_SMBUS_BYTE)
			data->byte = get_unaligned_le16(result->data);
		return 0;
	case I2C_SMBUS_BYTE_DATA:
		if (get_unaligned_le32(result->data) == AORUS_WMI_BYTE_ERR)
			return -ENXIO;
		data->byte = get_unaligned_le32(result->data);
		return 0;
	case I2C_SMBUS_WORD_DATA:
		if (get_unaligned_le32(result->data) == AORUS_WMI_WORD_ERR)
			return -ENXIO;
		data->word = get_unaligned_le32(result->data);
		return 0;
	case I2C_SMBUS_BLOCK_DATA:
		/* Result is a status word, a count word, then the data. */
		if (get_unaligned_le16(result->data) & AORUS_WMI_BLOCK_ERR)
			return get_unaligned_le16(result->data) &
			       AORUS_WMI_BLOCK_NAK ? -ENXIO : -EIO;
		data->block[0] = min3((size_t)get_unaligned_le16(result->data + 2),
				      (size_t)I2C_SMBUS_BLOCK_MAX,
				      result->length - 4);
		memcpy(&data->block[1], result->data + 4, data->block[0]);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static s32 aorus_wmi_smbus_xfer(struct i2c_adapter *adap, u16 addr,
				unsigned short flags, char read_write,
				u8 command, int size,
				union i2c_smbus_data *data)
{
	struct aorus_wmi_adapter *a;
	struct aorus_wmi_data *priv;
	struct wmi_buffer result = { };
	u8 buf[AORUS_WMI_BUF_MAX];
	size_t in_len;
	unsigned long start;
	size_t min_size;
	int fn, ret;

	a = container_of(adap, struct aorus_wmi_adapter, adap);
	priv = a->data;

	fn = aorus_wmi_pack_request(a->bus, addr, size, read_write, command,
				    data, buf, &in_len);
	if (fn < 0)
		return fn;

	min_size = aorus_wmi_min_result_size(fn);

	guard(mutex)(&priv->lock);

	start = jiffies;
	ret = priv->exec(priv, fn, min_size, buf, in_len, &result);

	/*
	 * The WMI core reports a missing result as -ENOMSG and a result
	 * shorter than the per-function minimum as -ENODATA; both are
	 * transport failures per the documented conventions (-EIO) and
	 * must be normalized regardless of which transport impl ran.
	 */
	if (ret == -ENOMSG || ret == -ENODATA)
		ret = -EIO;

	/*
	 * wmidev_invoke_method() is synchronous: the AML always completes
	 * (bounded by the timeout tiers) before returning. Never return
	 * -ETIMEDOUT while the AML may still be executing - a new
	 * transaction could then start against a bus the firmware
	 * considers busy. This warning therefore logs strictly after
	 * completion and never races the firmware.
	 */
	if (time_after(jiffies, start + msecs_to_jiffies(AORUS_WMI_SLOW_MS)))
		dev_warn_ratelimited(&priv->wdev->dev,
				     "WMBB transaction took %ums\n",
				     jiffies_to_msecs(jiffies - start));

	if (!ret)
		ret = aorus_wmi_parse_result(size, read_write, &result, data);

	dev_dbg(&priv->wdev->dev,
		"bus %u addr 0x%02x size %d rw %d cmd 0x%02x fn 0x%02x -> %d (%ums)\n",
		a->index, addr, size, read_write, command, fn, ret,
		jiffies_to_msecs(jiffies - start));

	kfree(result.data);
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

	/*
	 * The decompiled AML exposes no PEC controls (HCNT carries only
	 * the size bits, START and KILL), so I2C_FUNC_SMBUS_PEC is
	 * deliberately omitted.
	 */

	return func;
}

static const struct i2c_algorithm aorus_wmi_algorithm = {
	.smbus_xfer	= aorus_wmi_smbus_xfer,
	.functionality	= aorus_wmi_functionality,
};

#ifndef AORUS_WMI_KUNIT_TEST

/*
 * Boards this driver has been verified on. The GUID alone does not
 * establish ABI compatibility (the firmware exposes no version or
 * capability register for the SMBus interface), so this DMI table is the
 * compatibility boundary. Extend it with the DMI board vendor/name of
 * additional Gigabyte boards carrying the \GSA1 WMI device, or load with
 * force=1 to try an unlisted board.
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

static void aorus_wmi_pci_dev_put(void *pdev)
{
	pci_dev_put(pdev);
}

static int aorus_wmi_add_adapter(struct aorus_wmi_data *priv,
				 struct aorus_wmi_adapter *a, u8 index,
				 const char *name)
{
	a->data = priv;
	a->bus = index == 0 ? AORUS_WMI_BUS0 : AORUS_WMI_BUS1;
	a->index = index;

	a->adap.owner = THIS_MODULE;
	a->adap.algo = &aorus_wmi_algorithm;
	/*
	 * Parent the adapters at the AMD FCH SMBus PCI function
	 * (1022:790b): the AML declares OperationRegion(SMBI, SystemIO,
	 * 0x0B00, 0x10) and OperationRegion(SMG0, SystemIO, 0x0B20, 0x20),
	 * which are exactly the two I/O ranges of that PCI function, and
	 * the function decodes no BARs on these boards - the fixed decodes
	 * are its only hardware presence. Parenting the adapters there
	 * reflects the physical topology, and userspace DRAM tooling
	 * discovers SMBus buses by the parent PCI ID.
	 */
	if (priv->pdev)
		a->adap.dev.parent = &priv->pdev->dev;
	else
		a->adap.dev.parent = &priv->wdev->dev;
	strscpy(a->adap.name, name, sizeof(a->adap.name));
	a->adap.timeout = AORUS_WMI_TIMEOUT;

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
	priv->exec = aorus_wmi_exec;

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

	/*
	 * Registration performs no device writes; the client instantiation
	 * below performs read-only discovery (SPD and temperature
	 * probing). Devices are only ever written by explicit
	 * userspace-initiated operations.
	 */
	ret = aorus_wmi_add_adapter(priv, &priv->bus0, 0,
				    "SMBus AORUS WMI adapter 0");
	if (ret)
		return ret;

	/*
	 * Instantiate SPD EEPROM and memory temperature sensor clients.
	 * The write_enable name is DDR5-centric (SPD5118 write protection);
	 * for DDR4/ee1004 the enable/disable distinction is a no-op - only
	 * the read-only client instantiation matters here.
	 */
	i2c_register_spd_write_enable(&priv->bus0.adap);

	return aorus_wmi_add_adapter(priv, &priv->bus1, 1,
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
MODULE_DESCRIPTION("WMI SMBus driver for Gigabyte motherboards");
MODULE_LICENSE("GPL");

#endif /* !AORUS_WMI_KUNIT_TEST */
