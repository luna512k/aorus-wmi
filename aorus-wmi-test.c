// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the aorus-wmi WMBB wire ABI.
 *
 * These tests run without hardware: the WMI transport is replaced by a
 * fake firmware, and the request marshalling and result parsing are
 * asserted byte-exact against the ABI documented in
 * Documentation/wmi/devices/aorus-wmi.rst. In particular the word-data
 * offset-4 layout is pinned here - moving any request field must fail
 * these tests immediately.
 *
 * Running the suite:
 *
 *   - in-tree: ./tools/testing/kunit/kunit.py run --kunitconfig <path>/kunitconfig
 *     aorus-wmi (with this driver's Kconfig fragment)
 *   - out-of-tree on a CONFIG_KUNIT kernel: make && sudo modprobe aorus-wmi-test;
 *     results appear in the kernel log (ktap)
 *
 * The kunitconfig in the repository root lists the required options.
 */

#include <kunit/test.h>

#include "aorus-wmi.c"

/*
 * Fake firmware. The transport operation in the driver-under-test is
 * pointed at fake_exec(); results are handed over one at a time and
 * ownership of each result buffer transfers to smbus_xfer(), which
 * frees it.
 */
static struct {
	int fail;			/* error to return from exec */
	u8 *next_data;			/* kmalloc'd result, ownership transferred */
	size_t next_len;
	int calls;			/* number of exec invocations */
	int last_fn;
	size_t last_len;
	u8 last_buf[AORUS_WMI_BUF_MAX];
} fake;

struct aorus_wmi_test_env {
	struct aorus_wmi_data priv;
	struct aorus_wmi_adapter ad0;
	struct aorus_wmi_adapter ad1;
};

static int fake_exec(struct aorus_wmi_data *priv, int fn, const u8 *in,
		     size_t in_len, struct wmi_buffer *result)
{
	fake.calls++;
	fake.last_fn = fn;
	fake.last_len = in_len;
	if (in_len <= sizeof(fake.last_buf))
		memcpy(fake.last_buf, in, in_len);

	if (fake.fail)
		return fake.fail;

	result->length = fake.next_len;
	result->data = fake.next_data;
	fake.next_data = NULL;
	fake.next_len = 0;
	return 0;
}

/* Queue a firmware result buffer; ownership passes to smbus_xfer(). */
static void fake_set_result(struct kunit *test, const u8 *bytes, size_t len)
{
	u8 *data;

	data = kmalloc(len, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, data);
	memcpy(data, bytes, len);

	fake.next_data = data;
	fake.next_len = len;
}

static void aorus_wmi_test_mutex_destroy(void *lock)
{
	mutex_destroy(lock);
}

static struct aorus_wmi_test_env *env_init(struct kunit *test, u8 bus)
{
	struct aorus_wmi_test_env *env;
	int ret;

	env = kunit_kzalloc(test, sizeof(*env), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, env);

	mutex_init(&env->priv.lock);
	ret = kunit_add_action(test, aorus_wmi_test_mutex_destroy,
			       &env->priv.lock);
	KUNIT_ASSERT_EQ(test, ret, 0);
	env->priv.wdev = kunit_kzalloc(test, sizeof(struct wmi_device),
				       GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, env->priv.wdev);
	env->priv.exec = fake_exec;

	env->ad0.data = &env->priv;
	env->ad0.bus = AORUS_WMI_BUS0;
	env->ad0.index = 0;
	env->ad1.data = &env->priv;
	env->ad1.bus = AORUS_WMI_BUS1;
	env->ad1.index = 1;

	return env;
}

/* ---- request marshalling ------------------------------------------------ */

static void aorus_wmi_pack_byte_data_write(struct kunit *test)
{
	u8 buf[AORUS_WMI_BUF_MAX];
	size_t in_len;
	union i2c_smbus_data data = { .byte = 0x55 };
	const u8 expect[] = { 0x02, 0xa0, 0xaa, 0x55, 0x00, 0x00, 0x00 };
	int fn;

	fn = aorus_wmi_pack_request(AORUS_WMI_BUS0, 0x50, I2C_SMBUS_BYTE_DATA,
				    I2C_SMBUS_WRITE, 0xaa, &data,
				    buf, &in_len);

	KUNIT_EXPECT_EQ(test, fn, AORUS_WMI_BYTE_WRITE);
	KUNIT_EXPECT_EQ(test, in_len, AORUS_WMI_BUF_MIN);
	KUNIT_EXPECT_MEMEQ(test, buf, expect, sizeof(expect));
}

static void aorus_wmi_pack_byte_data_read(struct kunit *test)
{
	u8 buf[AORUS_WMI_BUF_MAX];
	size_t in_len;
	union i2c_smbus_data data;
	int fn;

	fn = aorus_wmi_pack_request(AORUS_WMI_BUS0, 0x50, I2C_SMBUS_BYTE_DATA,
				    I2C_SMBUS_READ, 0xaa, &data,
				    buf, &in_len);

	KUNIT_EXPECT_EQ(test, fn, AORUS_WMI_BYTE_READ);
	KUNIT_EXPECT_EQ(test, buf[0], AORUS_WMI_BUS0);
	KUNIT_EXPECT_EQ(test, buf[1], 0xa0);	/* pre-shifted address */
	KUNIT_EXPECT_EQ(test, buf[2], 0xaa);
	KUNIT_EXPECT_EQ(test, buf[3], 0x00);	/* read: no byte value */
}

static void aorus_wmi_pack_word_write_offset4(struct kunit *test)
{
	u8 buf[AORUS_WMI_BUF_MAX];
	size_t in_len;
	union i2c_smbus_data data = { .word = 0x2080 };
	int fn;

	/*
	 * The regression test for the historical offset-4 bug: the word
	 * value 0x2080 must be little endian at [4..5] (wire: 80 20,
	 * big-endian pointer 0x8020 for the ENE protocol), never at [3].
	 */
	fn = aorus_wmi_pack_request(AORUS_WMI_BUS0, 0x71, I2C_SMBUS_WORD_DATA,
				    I2C_SMBUS_WRITE, 0x00, &data,
				    buf, &in_len);

	KUNIT_EXPECT_EQ(test, fn, AORUS_WMI_WORD_WRITE);
	KUNIT_EXPECT_EQ(test, buf[3], 0x00);
	KUNIT_EXPECT_EQ(test, buf[4], 0x80);
	KUNIT_EXPECT_EQ(test, buf[5], 0x20);
	KUNIT_EXPECT_EQ(test, buf[6], 0x00);
}

static void aorus_wmi_pack_word_read(struct kunit *test)
{
	u8 buf[AORUS_WMI_BUF_MAX];
	size_t in_len;
	union i2c_smbus_data data;
	int fn;

	fn = aorus_wmi_pack_request(AORUS_WMI_BUS0, 0x71, I2C_SMBUS_WORD_DATA,
				    I2C_SMBUS_READ, 0x05, &data,
				    buf, &in_len);

	KUNIT_EXPECT_EQ(test, fn, AORUS_WMI_WORD_READ);
	KUNIT_EXPECT_EQ(test, buf[2], 0x05);
}

static void aorus_wmi_pack_quick(struct kunit *test)
{
	u8 buf[AORUS_WMI_BUF_MAX];
	size_t in_len;
	union i2c_smbus_data data;
	int fn;

	fn = aorus_wmi_pack_request(AORUS_WMI_BUS0, 0x71, I2C_SMBUS_QUICK,
				    I2C_SMBUS_WRITE, 0, &data, buf, &in_len);
	KUNIT_EXPECT_EQ(test, fn, AORUS_WMI_QUICK_WRITE);
	KUNIT_EXPECT_EQ(test, in_len, AORUS_WMI_BUF_MIN);

	fn = aorus_wmi_pack_request(AORUS_WMI_BUS0, 0x71, I2C_SMBUS_QUICK,
				    I2C_SMBUS_READ, 0, &data, buf, &in_len);
	KUNIT_EXPECT_EQ(test, fn, AORUS_WMI_QUICK_READ);

	/* Quick transactions are firmware stubs on bus 1. */
	fn = aorus_wmi_pack_request(AORUS_WMI_BUS1, 0x71, I2C_SMBUS_QUICK,
				    I2C_SMBUS_WRITE, 0, &data, buf, &in_len);
	KUNIT_EXPECT_EQ(test, fn, -EOPNOTSUPP);
}

static void aorus_wmi_pack_byte_send_recv(struct kunit *test)
{
	u8 buf[AORUS_WMI_BUF_MAX];
	size_t in_len;
	union i2c_smbus_data data;
	int fn;

	/* Send byte carries its value in the command field. */
	fn = aorus_wmi_pack_request(AORUS_WMI_BUS0, 0x71, I2C_SMBUS_BYTE,
				    I2C_SMBUS_WRITE, 0x42, &data,
				    buf, &in_len);
	KUNIT_EXPECT_EQ(test, fn, AORUS_WMI_SEND_BYTE);
	KUNIT_EXPECT_EQ(test, buf[2], 0x42);
	KUNIT_EXPECT_EQ(test, buf[3], 0x00);

	fn = aorus_wmi_pack_request(AORUS_WMI_BUS0, 0x71, I2C_SMBUS_BYTE,
				    I2C_SMBUS_READ, 0, &data, buf, &in_len);
	KUNIT_EXPECT_EQ(test, fn, AORUS_WMI_RECEIVE_BYTE);

	/* Byte transactions are firmware stubs on bus 1. */
	fn = aorus_wmi_pack_request(AORUS_WMI_BUS1, 0x71, I2C_SMBUS_BYTE,
				    I2C_SMBUS_WRITE, 0x42, &data,
				    buf, &in_len);
	KUNIT_EXPECT_EQ(test, fn, -EOPNOTSUPP);
}

static void aorus_wmi_pack_block_write(struct kunit *test)
{
	u8 buf[AORUS_WMI_BUF_MAX];
	size_t in_len;
	union i2c_smbus_data data = { .block = { 3, 0x01, 0x02, 0x03 } };
	int fn;

	fn = aorus_wmi_pack_request(AORUS_WMI_BUS0, 0x71, I2C_SMBUS_BLOCK_DATA,
				    I2C_SMBUS_WRITE, 0x01, &data,
				    buf, &in_len);

	KUNIT_EXPECT_EQ(test, fn, AORUS_WMI_BLOCK_WRITE);
	KUNIT_EXPECT_EQ(test, in_len, AORUS_WMI_BUF_MIN + 3);
	KUNIT_EXPECT_EQ(test, buf[3], 0x03);	/* length dword, LE */
	KUNIT_EXPECT_EQ(test, buf[4], 0x00);
	KUNIT_EXPECT_EQ(test, buf[5], 0x00);
	KUNIT_EXPECT_EQ(test, buf[6], 0x00);
	KUNIT_EXPECT_EQ(test, buf[7], 0x01);	/* payload from [7] */
	KUNIT_EXPECT_EQ(test, buf[8], 0x02);
	KUNIT_EXPECT_EQ(test, buf[9], 0x03);
}

static void aorus_wmi_pack_block_read(struct kunit *test)
{
	u8 buf[AORUS_WMI_BUF_MAX];
	size_t in_len;
	union i2c_smbus_data data;
	int fn;

	fn = aorus_wmi_pack_request(AORUS_WMI_BUS0, 0x71, I2C_SMBUS_BLOCK_DATA,
				    I2C_SMBUS_READ, 0x80, &data,
				    buf, &in_len);

	KUNIT_EXPECT_EQ(test, fn, AORUS_WMI_BLOCK_READ);
	KUNIT_EXPECT_EQ(test, in_len, AORUS_WMI_BUF_MIN);
}

static void aorus_wmi_pack_block_len_boundaries(struct kunit *test)
{
	u8 buf[AORUS_WMI_BUF_MAX];
	size_t in_len;
	union i2c_smbus_data data;
	int len, fn;

	/* Lengths 0..32 are valid, anything above is rejected driver-side. */
	for (len = 0; len <= I2C_SMBUS_BLOCK_MAX; len++) {
		data.block[0] = len;
		fn = aorus_wmi_pack_request(AORUS_WMI_BUS0, 0x71,
					    I2C_SMBUS_BLOCK_DATA,
					    I2C_SMBUS_WRITE, 0x01, &data,
					    buf, &in_len);
		KUNIT_EXPECT_EQ_MSG(test, fn, AORUS_WMI_BLOCK_WRITE,
				    "len %d", len);
		KUNIT_EXPECT_EQ_MSG(test, in_len,
				    AORUS_WMI_BUF_MIN + len, "len %d", len);
	}

	data.block[0] = I2C_SMBUS_BLOCK_MAX + 1;
	fn = aorus_wmi_pack_request(AORUS_WMI_BUS0, 0x71, I2C_SMBUS_BLOCK_DATA,
				    I2C_SMBUS_WRITE, 0x01, &data,
				    buf, &in_len);
	KUNIT_EXPECT_EQ(test, fn, -EINVAL);

	data.block[0] = 255;
	fn = aorus_wmi_pack_request(AORUS_WMI_BUS0, 0x71, I2C_SMBUS_BLOCK_DATA,
				    I2C_SMBUS_WRITE, 0x01, &data,
				    buf, &in_len);
	KUNIT_EXPECT_EQ(test, fn, -EINVAL);
}

static void aorus_wmi_pack_unknown_size(struct kunit *test)
{
	u8 buf[AORUS_WMI_BUF_MAX];
	size_t in_len;
	union i2c_smbus_data data;
	int fn;

	fn = aorus_wmi_pack_request(AORUS_WMI_BUS0, 0x71, I2C_SMBUS_PROC_CALL,
				    I2C_SMBUS_WRITE, 0, &data, buf, &in_len);
	KUNIT_EXPECT_EQ(test, fn, -EOPNOTSUPP);
}

/* ---- result parsing ----------------------------------------------------- */

static void aorus_wmi_parse_write(struct kunit *test)
{
	union i2c_smbus_data data;
	const u8 ok[] = { 0x01, 0x00, 0x00, 0x00 };
	const u8 fail[] = { 0x00, 0x00, 0x00, 0x00 };
	struct wmi_buffer result = { .length = sizeof(ok),
				     .data = (u8 *)ok };
	int ret;

	ret = aorus_wmi_parse_result(I2C_SMBUS_BYTE_DATA, I2C_SMBUS_WRITE,
				     &result, &data);
	KUNIT_EXPECT_EQ(test, ret, 0);

	ret = aorus_wmi_parse_result(I2C_SMBUS_WORD_DATA, I2C_SMBUS_WRITE,
				     &result, &data);
	KUNIT_EXPECT_EQ(test, ret, 0);

	result.data = (u8 *)fail;
	ret = aorus_wmi_parse_result(I2C_SMBUS_BYTE_DATA, I2C_SMBUS_WRITE,
				     &result, &data);
	KUNIT_EXPECT_EQ(test, ret, -ENXIO);
}

static void aorus_wmi_parse_byte_read(struct kunit *test)
{
	union i2c_smbus_data data;
	const u8 ok[] = { 0x23, 0x00, 0x00, 0x00 };
	const u8 nak[] = { 0xff, 0xff, 0x00, 0x00 };
	struct wmi_buffer result = { .length = sizeof(ok),
				     .data = (u8 *)ok };
	int ret;

	ret = aorus_wmi_parse_result(I2C_SMBUS_BYTE_DATA, I2C_SMBUS_READ,
				     &result, &data);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, data.byte, 0x23);

	result.data = (u8 *)nak;
	ret = aorus_wmi_parse_result(I2C_SMBUS_BYTE_DATA, I2C_SMBUS_READ,
				     &result, &data);
	KUNIT_EXPECT_EQ(test, ret, -ENXIO);
}

static void aorus_wmi_parse_word_read(struct kunit *test)
{
	union i2c_smbus_data data;
	const u8 ok[] = { 0xcd, 0xab, 0x00, 0x00 };
	const u8 nak[] = { 0xff, 0xff, 0xff, 0xff };
	struct wmi_buffer result = { .length = sizeof(ok),
				     .data = (u8 *)ok };
	int ret;

	ret = aorus_wmi_parse_result(I2C_SMBUS_WORD_DATA, I2C_SMBUS_READ,
				     &result, &data);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, data.word, 0xabcd);

	result.data = (u8 *)nak;
	ret = aorus_wmi_parse_result(I2C_SMBUS_WORD_DATA, I2C_SMBUS_READ,
				     &result, &data);
	KUNIT_EXPECT_EQ(test, ret, -ENXIO);
}

static void aorus_wmi_parse_quick_read(struct kunit *test)
{
	union i2c_smbus_data data;
	const u8 ok[] = { 0x34, 0x12 };
	const u8 nak[] = { 0xff, 0xff };
	struct wmi_buffer result = { .length = sizeof(ok),
				     .data = (u8 *)ok };
	int ret;

	ret = aorus_wmi_parse_result(I2C_SMBUS_QUICK, I2C_SMBUS_READ,
				     &result, &data);
	KUNIT_EXPECT_EQ(test, ret, 0);

	result.data = (u8 *)nak;
	ret = aorus_wmi_parse_result(I2C_SMBUS_QUICK, I2C_SMBUS_READ,
				     &result, &data);
	KUNIT_EXPECT_EQ(test, ret, -ENXIO);
}

static void aorus_wmi_parse_receive_byte(struct kunit *test)
{
	union i2c_smbus_data data;
	const u8 val[] = { 0x5a, 0x00 };
	struct wmi_buffer result = { .length = sizeof(val),
				     .data = (u8 *)val };
	int ret;

	ret = aorus_wmi_parse_result(I2C_SMBUS_BYTE, I2C_SMBUS_READ,
				     &result, &data);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, data.byte, 0x5a);
}

static void aorus_wmi_parse_block_read(struct kunit *test)
{
	union i2c_smbus_data data;
	/* status word, count word, three data bytes */
	const u8 ok[] = { 0x00, 0x00, 0x03, 0x00, 0xaa, 0xbb, 0xcc };
	const u8 nak[] = { 0x04, 0x80, 0x00, 0x00 };
	const u8 err[] = { 0x01, 0x80, 0x00, 0x00 };
	struct wmi_buffer result = { .length = sizeof(ok),
				     .data = (u8 *)ok };
	int ret;

	ret = aorus_wmi_parse_result(I2C_SMBUS_BLOCK_DATA, I2C_SMBUS_READ,
				     &result, &data);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, data.block[0], 3);
	KUNIT_EXPECT_EQ(test, data.block[1], 0xaa);
	KUNIT_EXPECT_EQ(test, data.block[2], 0xbb);
	KUNIT_EXPECT_EQ(test, data.block[3], 0xcc);

	result.data = (u8 *)nak;	/* bit15 + DEV_ERR */
	ret = aorus_wmi_parse_result(I2C_SMBUS_BLOCK_DATA, I2C_SMBUS_READ,
				     &result, &data);
	KUNIT_EXPECT_EQ(test, ret, -ENXIO);

	result.data = (u8 *)err;	/* bit15 without DEV_ERR */
	ret = aorus_wmi_parse_result(I2C_SMBUS_BLOCK_DATA, I2C_SMBUS_READ,
				     &result, &data);
	KUNIT_EXPECT_EQ(test, ret, -EIO);
}

static void aorus_wmi_parse_block_count_clamps(struct kunit *test)
{
	union i2c_smbus_data data;
	u8 buf[4 + I2C_SMBUS_BLOCK_MAX];
	struct wmi_buffer result = { .length = sizeof(buf), .data = buf };
	int ret, i;

	/* Firmware claims 255 bytes but only sends 32: clamp to 32. */
	buf[0] = 0x00;
	buf[1] = 0x00;
	buf[2] = 0xff;
	buf[3] = 0x00;
	for (i = 0; i < I2C_SMBUS_BLOCK_MAX; i++)
		buf[4 + i] = i;

	ret = aorus_wmi_parse_result(I2C_SMBUS_BLOCK_DATA, I2C_SMBUS_READ,
				     &result, &data);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, data.block[0], I2C_SMBUS_BLOCK_MAX);

	/* Firmware sends fewer bytes than it claims: clamp to the buffer. */
	result.length = 4 + 5;
	buf[2] = 10;
	ret = aorus_wmi_parse_result(I2C_SMBUS_BLOCK_DATA, I2C_SMBUS_READ,
				     &result, &data);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, data.block[0], 5);

	/* Zero-length block is valid. */
	result.length = 4;
	buf[2] = 0;
	ret = aorus_wmi_parse_result(I2C_SMBUS_BLOCK_DATA, I2C_SMBUS_READ,
				     &result, &data);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, data.block[0], 0);
}

/*
 * Short and missing results cannot reach parse_result() - the WMI core
 * enforces the per-function minimum length (-ENODATA) and reports a
 * missing result with -ENOMSG. The driver must normalize both to -EIO.
 */
static void aorus_wmi_fault_short_results(struct kunit *test)
{
	struct aorus_wmi_test_env *env = env_init(test, AORUS_WMI_BUS0);
	union i2c_smbus_data data = { .byte = 0x55 };
	int ret;

	fake.fail = -ENOMSG;
	ret = aorus_wmi_smbus_xfer(&env->ad0.adap, 0x50, 0, I2C_SMBUS_WRITE,
				   0xaa, I2C_SMBUS_BYTE_DATA, &data);
	KUNIT_EXPECT_EQ(test, ret, -EIO);

	fake.fail = -ENODATA;
	ret = aorus_wmi_smbus_xfer(&env->ad0.adap, 0x50, 0, I2C_SMBUS_WRITE,
				   0xaa, I2C_SMBUS_BYTE_DATA, &data);
	KUNIT_EXPECT_EQ(test, ret, -EIO);

	fake.fail = -ENODEV;	/* unexpected core errors pass through */
	ret = aorus_wmi_smbus_xfer(&env->ad0.adap, 0x50, 0, I2C_SMBUS_WRITE,
				   0xaa, I2C_SMBUS_BYTE_DATA, &data);
	KUNIT_EXPECT_EQ(test, ret, -ENODEV);
	fake.fail = 0;

	/* The adapter must remain usable after the failures. */
	fake_set_result(test, (const u8[]){ 0x01, 0x00, 0x00, 0x00 }, 4);
	ret = aorus_wmi_smbus_xfer(&env->ad0.adap, 0x50, 0, I2C_SMBUS_WRITE,
				   0xaa, I2C_SMBUS_BYTE_DATA, &data);
	KUNIT_EXPECT_EQ(test, ret, 0);
}

/* ---- capability / transaction matrix through the fake backend ---------- */

static void aorus_wmi_xfer_bus0_matrix(struct kunit *test)
{
	struct aorus_wmi_test_env *env = env_init(test, AORUS_WMI_BUS0);
	union i2c_smbus_data data = { .byte = 0x55 };
	int ret;

	/* Quick write: firmware reports success as a dword 1. */
	fake_set_result(test, (const u8[]){ 0x01, 0x00, 0x00, 0x00 }, 4);
	ret = aorus_wmi_smbus_xfer(&env->ad0.adap, 0x71, 0, I2C_SMBUS_WRITE,
				   0, I2C_SMBUS_QUICK, &data);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, fake.last_fn, AORUS_WMI_QUICK_WRITE);

	/* Receive byte. */
	fake_set_result(test, (const u8[]){ 0x5a, 0x00 }, 2);
	ret = aorus_wmi_smbus_xfer(&env->ad0.adap, 0x71, 0, I2C_SMBUS_READ,
				   0, I2C_SMBUS_BYTE, &data);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, data.byte, 0x5a);
	KUNIT_EXPECT_EQ(test, fake.last_fn, AORUS_WMI_RECEIVE_BYTE);

	/* Word write: the offset-4 pin, end to end. */
	data.word = 0x2080;
	fake_set_result(test, (const u8[]){ 0x01, 0x00, 0x00, 0x00 }, 4);
	ret = aorus_wmi_smbus_xfer(&env->ad0.adap, 0x71, 0, I2C_SMBUS_WRITE,
				   0x00, I2C_SMBUS_WORD_DATA, &data);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, fake.last_fn, AORUS_WMI_WORD_WRITE);
	KUNIT_EXPECT_EQ(test, fake.last_buf[4], 0x80);
	KUNIT_EXPECT_EQ(test, fake.last_buf[5], 0x20);

	/* Block write. */
	data.block[0] = 2;
	data.block[1] = 0xde;
	data.block[2] = 0xad;
	fake_set_result(test, (const u8[]){ 0x01, 0x00, 0x00, 0x00 }, 4);
	ret = aorus_wmi_smbus_xfer(&env->ad0.adap, 0x71, 0, I2C_SMBUS_WRITE,
				   0x01, I2C_SMBUS_BLOCK_DATA, &data);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, fake.last_fn, AORUS_WMI_BLOCK_WRITE);
	KUNIT_EXPECT_EQ(test, fake.last_len, AORUS_WMI_BUF_MIN + 2);
}

static void aorus_wmi_xfer_bus1_matrix(struct kunit *test)
{
	struct aorus_wmi_test_env *env = env_init(test, AORUS_WMI_BUS1);
	union i2c_smbus_data data;
	int ret, calls;

	/*
	 * Quick and byte transactions are firmware stubs on bus 1: they
	 * must be rejected driver-side without touching the firmware at
	 * all (functionality() does not advertise them either).
	 */
	calls = fake.calls;
	ret = aorus_wmi_smbus_xfer(&env->ad1.adap, 0x71, 0, I2C_SMBUS_WRITE,
				   0, I2C_SMBUS_QUICK, &data);
	KUNIT_EXPECT_EQ(test, ret, -EOPNOTSUPP);
	ret = aorus_wmi_smbus_xfer(&env->ad1.adap, 0x71, 0, I2C_SMBUS_READ,
				   0, I2C_SMBUS_BYTE, &data);
	KUNIT_EXPECT_EQ(test, ret, -EOPNOTSUPP);
	KUNIT_EXPECT_EQ(test, fake.calls, calls);

	/* Byte-data, word-data and block transactions work on bus 1. */
	fake_set_result(test, (const u8[]){ 0x23, 0x00, 0x00, 0x00 }, 4);
	ret = aorus_wmi_smbus_xfer(&env->ad1.adap, 0x50, 0, I2C_SMBUS_READ,
				   0x00, I2C_SMBUS_BYTE_DATA, &data);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, data.byte, 0x23);
	KUNIT_EXPECT_EQ(test, fake.last_buf[0], AORUS_WMI_BUS1);
}

static void aorus_wmi_xfer_unknown_size(struct kunit *test)
{
	struct aorus_wmi_test_env *env = env_init(test, AORUS_WMI_BUS0);
	union i2c_smbus_data data;
	int ret, calls;

	calls = fake.calls;
	ret = aorus_wmi_smbus_xfer(&env->ad0.adap, 0x71, 0, I2C_SMBUS_WRITE,
				   0, I2C_SMBUS_PROC_CALL, &data);
	KUNIT_EXPECT_EQ(test, ret, -EOPNOTSUPP);
	KUNIT_EXPECT_EQ(test, fake.calls, calls);
}

/* ---- fault injection ----------------------------------------------------- */

static void aorus_wmi_fault_wmi_failure(struct kunit *test)
{
	struct aorus_wmi_test_env *env = env_init(test, AORUS_WMI_BUS0);
	union i2c_smbus_data data = { .byte = 0x55 };
	int ret;

	fake.fail = -EIO;
	ret = aorus_wmi_smbus_xfer(&env->ad0.adap, 0x50, 0, I2C_SMBUS_WRITE,
				   0xaa, I2C_SMBUS_BYTE_DATA, &data);
	KUNIT_EXPECT_EQ(test, ret, -EIO);
	fake.fail = 0;

	/* The adapter must remain usable after a firmware failure. */
	fake_set_result(test, (const u8[]){ 0x01, 0x00, 0x00, 0x00 }, 4);
	ret = aorus_wmi_smbus_xfer(&env->ad0.adap, 0x50, 0, I2C_SMBUS_WRITE,
				   0xaa, I2C_SMBUS_BYTE_DATA, &data);
	KUNIT_EXPECT_EQ(test, ret, 0);
}

static int aorus_wmi_test_init(struct kunit *test)
{
	memset(&fake, 0, sizeof(fake));
	return 0;
}

static void aorus_wmi_test_exit(struct kunit *test)
{
	/* Free any result the fake queued but smbus_xfer never consumed. */
	kfree(fake.next_data);
	memset(&fake, 0, sizeof(fake));
}

static struct kunit_case aorus_wmi_test_cases[] = {
	KUNIT_CASE(aorus_wmi_pack_byte_data_write),
	KUNIT_CASE(aorus_wmi_pack_byte_data_read),
	KUNIT_CASE(aorus_wmi_pack_word_write_offset4),
	KUNIT_CASE(aorus_wmi_pack_word_read),
	KUNIT_CASE(aorus_wmi_pack_quick),
	KUNIT_CASE(aorus_wmi_pack_byte_send_recv),
	KUNIT_CASE(aorus_wmi_pack_block_write),
	KUNIT_CASE(aorus_wmi_pack_block_read),
	KUNIT_CASE(aorus_wmi_pack_block_len_boundaries),
	KUNIT_CASE(aorus_wmi_pack_unknown_size),
	KUNIT_CASE(aorus_wmi_parse_write),
	KUNIT_CASE(aorus_wmi_parse_byte_read),
	KUNIT_CASE(aorus_wmi_parse_word_read),
	KUNIT_CASE(aorus_wmi_parse_quick_read),
	KUNIT_CASE(aorus_wmi_parse_receive_byte),
	KUNIT_CASE(aorus_wmi_parse_block_read),
	KUNIT_CASE(aorus_wmi_parse_block_count_clamps),
	KUNIT_CASE(aorus_wmi_fault_short_results),
	KUNIT_CASE(aorus_wmi_xfer_bus0_matrix),
	KUNIT_CASE(aorus_wmi_xfer_bus1_matrix),
	KUNIT_CASE(aorus_wmi_xfer_unknown_size),
	KUNIT_CASE(aorus_wmi_fault_wmi_failure),
	{ }
};

static struct kunit_suite aorus_wmi_test_suite = {
	.name = "aorus-wmi",
	.init = aorus_wmi_test_init,
	.exit = aorus_wmi_test_exit,
	.test_cases = aorus_wmi_test_cases,
};

kunit_test_suite(aorus_wmi_test_suite);

MODULE_AUTHOR("Luna Webster");
MODULE_DESCRIPTION("KUnit tests for the aorus-wmi WMI SMBus driver");
MODULE_LICENSE("GPL");
