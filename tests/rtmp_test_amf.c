/**
 * Copyright (c) 2018 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "rtmp_test.h"

#include "amf.h"

/* AMF0 wire format constants (see AMF0 spec / src/amf.c) */
#define TAG_NUMBER 0x00
#define TAG_BOOLEAN 0x01
#define TAG_STRING 0x02
#define TAG_OBJECT_START 0x03
#define TAG_NULL 0x05
#define TAG_ECMA_ARRAY 0x08
#define TAG_LONG_STRING 0x0c


static void test_amf_encode_number(void)
{
	uint8_t data[16];
	struct rtmp_buffer buf = {
		.buf = data,
		.cap = sizeof(data),
		.len = 0,
		.rd = 0,
	};
	int ret;
	double val = 0.;

	ret = amf_encode(&buf, "%f", 3.14);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(buf.len, 9);
	CU_ASSERT_EQUAL(buf.buf[0], TAG_NUMBER);

	ret = amf_get_number(&buf, &val);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_DOUBLE_EQUAL(val, 3.14, 0.0000001);
	CU_ASSERT_EQUAL(buf.rd, buf.len);
}


static void test_amf_encode_boolean(void)
{
	uint8_t data[16];
	struct rtmp_buffer buf = {
		.buf = data,
		.cap = sizeof(data),
		.len = 0,
		.rd = 0,
	};
	int ret;
	uint8_t val = 0xff;

	ret = amf_encode(&buf, "%u%u", 1, 0);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(buf.len, 4);
	CU_ASSERT_EQUAL(buf.buf[0], TAG_BOOLEAN);
	CU_ASSERT_EQUAL(buf.buf[1], 1);
	CU_ASSERT_EQUAL(buf.buf[2], TAG_BOOLEAN);
	CU_ASSERT_EQUAL(buf.buf[3], 0);

	ret = amf_get_boolean(&buf, &val);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(val, 1);
	ret = amf_get_boolean(&buf, &val);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(val, 0);
}


static void test_amf_encode_string(void)
{
	uint8_t data[64];
	struct rtmp_buffer buf = {
		.buf = data,
		.cap = sizeof(data),
		.len = 0,
		.rd = 0,
	};
	int ret;
	char *str = NULL;

	ret = amf_encode(&buf, "%s", "hello");
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(buf.len, 3 + 5);
	CU_ASSERT_EQUAL(buf.buf[0], TAG_STRING);
	CU_ASSERT_EQUAL(buf.buf[1], 0);
	CU_ASSERT_EQUAL(buf.buf[2], 5);

	ret = amf_get_string(&buf, &str);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(str);
	CU_ASSERT_STRING_EQUAL(str, "hello");
	free(str);
}


static void test_amf_encode_long_string(void)
{
	/* A string bigger than UINT16_MAX must use the long-string (0x0c)
	 * encoding instead of the short-string (0x02) one */
	size_t slen = UINT16_MAX + 10;
	char *str = malloc(slen + 1);
	uint8_t *data;
	struct rtmp_buffer buf;
	int ret;
	char *decoded = NULL;

	CU_ASSERT_PTR_NOT_NULL_FATAL(str);
	memset(str, 'a', slen);
	str[slen] = '\0';

	data = malloc(slen + 16);
	CU_ASSERT_PTR_NOT_NULL_FATAL(data);
	buf.buf = data;
	buf.cap = slen + 16;
	buf.len = 0;
	buf.rd = 0;

	ret = amf_encode(&buf, "%s", str);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(buf.len, 5 + slen);
	CU_ASSERT_EQUAL(buf.buf[0], TAG_LONG_STRING);

	ret = amf_get_string(&buf, &decoded);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(decoded);
	CU_ASSERT_STRING_EQUAL(decoded, str);

	free(decoded);
	free(data);
	free(str);
}


static void test_amf_encode_null(void)
{
	uint8_t data[16];
	struct rtmp_buffer buf = {
		.buf = data,
		.cap = sizeof(data),
		.len = 0,
		.rd = 0,
	};
	int ret;

	ret = amf_encode(&buf, "0");
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(buf.len, 1);
	CU_ASSERT_EQUAL(buf.buf[0], TAG_NULL);

	ret = amf_get_null(&buf);
	CU_ASSERT_EQUAL(ret, 0);
}


static void test_amf_encode_object(void)
{
	uint8_t data[64];
	struct rtmp_buffer buf = {
		.buf = data,
		.cap = sizeof(data),
		.len = 0,
		.rd = 0,
	};
	int ret;
	char *key = NULL;
	double val = 0.;

	ret = amf_encode(&buf, "{%s:%f}", "level", 42.0);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(buf.buf[0], TAG_OBJECT_START);

	ret = amf_get_object_start(&buf);
	CU_ASSERT_EQUAL(ret, 0);
	ret = amf_get_property(&buf, &key);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(key);
	CU_ASSERT_STRING_EQUAL(key, "level");
	free(key);
	ret = amf_get_number(&buf, &val);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_DOUBLE_EQUAL(val, 42.0, 0.0000001);
	ret = amf_get_object_end(&buf);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(buf.rd, buf.len);
}


static void test_amf_encode_ecma_array(void)
{
	/* There is no amf_get_ecma_start(): assert on the exact wire bytes
	 * (tag + big-endian element count), then skip past them manually to
	 * decode the name/value pairs like an object's */
	uint8_t data[64];
	struct rtmp_buffer buf = {
		.buf = data,
		.cap = sizeof(data),
		.len = 0,
		.rd = 0,
	};
	int ret;
	char *key = NULL;
	double val = 0.;

	ret = amf_encode(&buf, "[%d%s:%f]", 1, "width", 1920.0);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(buf.buf[0], TAG_ECMA_ARRAY);
	CU_ASSERT_EQUAL(buf.buf[1], 0);
	CU_ASSERT_EQUAL(buf.buf[2], 0);
	CU_ASSERT_EQUAL(buf.buf[3], 0);
	CU_ASSERT_EQUAL(buf.buf[4], 1);

	buf.rd = 5; /* skip tag + count, no getter exists for it */
	ret = amf_get_property(&buf, &key);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(key);
	CU_ASSERT_STRING_EQUAL(key, "width");
	free(key);
	ret = amf_get_number(&buf, &val);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_DOUBLE_EQUAL(val, 1920.0, 0.0000001);
	ret = amf_get_object_end(&buf);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(buf.rd, buf.len);
}


static void test_amf_encode_separators_ignored(void)
{
	uint8_t data[32];
	struct rtmp_buffer buf = {
		.buf = data,
		.cap = sizeof(data),
		.len = 0,
		.rd = 0,
	};
	int ret;
	double val1 = 0.;
	double val2 = 0.;

	ret = amf_encode(&buf, "%f, :\t\n %f", 1.0, 2.0);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(buf.len, 18);

	ret = amf_get_number(&buf, &val1);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_DOUBLE_EQUAL(val1, 1.0, 0.0000001);
	ret = amf_get_number(&buf, &val2);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_DOUBLE_EQUAL(val2, 2.0, 0.0000001);
}


static void test_amf_encode_golden_buffer(void)
{
	/* Byte-exact regression buffer for a flat "connect"-style message:
	 * a string, a number, and a null. If the wire format of amf_encode
	 * ever changes, this test pins it down explicitly. */
	static const uint8_t expected[] = {
		/* "connect" (7 chars) */
		TAG_STRING,
		0x00,
		0x07,
		'c',
		'o',
		'n',
		'n',
		'e',
		'c',
		't',
		/* 1.0 */
		TAG_NUMBER,
		0x3f,
		0xf0,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		/* null */
		TAG_NULL,
	};
	uint8_t data[64];
	struct rtmp_buffer buf = {
		.buf = data,
		.cap = sizeof(data),
		.len = 0,
		.rd = 0,
	};
	int ret;

	ret = amf_encode(&buf, "%s,%f,0", "connect", 1.0);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(buf.len, sizeof(expected));
	CU_ASSERT_EQUAL(memcmp(buf.buf, expected, sizeof(expected)), 0);
}


/* amf_encode()'s format string is a custom mini-language (not printf),
 * even though it carries AMF_ATTRIBUTE_FORMAT_PRINTF for basic sanity
 * checking on well-formed calls. The tests below deliberately call it with
 * malformed/mismatched format strings to exercise its own error handling,
 * which GCC's printf-style checker (correctly, from its own point of view)
 * flags as suspicious. Silence it for exactly these calls. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat"

static void test_amf_encode_invalid_args(void)
{
	uint8_t data[32];
	struct rtmp_buffer buf;
	int ret;

	/* NULL buffer/format */
	ret = amf_encode(NULL, "%f", 1.0);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	buf.buf = data;
	buf.cap = sizeof(data);
	buf.len = 0;
	buf.rd = 0;
	ret = amf_encode(&buf, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Unknown format char after '%' (no vararg consumed) */
	buf.len = 0;
	ret = amf_encode(&buf, "%z");
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Unknown literal char */
	buf.len = 0;
	ret = amf_encode(&buf, "?");
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Value where a property name was expected (right after '{') */
	buf.len = 0;
	ret = amf_encode(&buf, "{%f}");
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* '}' / ']' without a matching opener */
	buf.len = 0;
	ret = amf_encode(&buf, "%f}", 1.0);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	buf.len = 0;
	ret = amf_encode(&buf, "]");
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* '[' not immediately followed by "%d" */
	buf.len = 0;
	ret = amf_encode(&buf, "[%s");
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Unterminated object at end of format string */
	buf.len = 0;
	ret = amf_encode(&buf, "{%s%f", "a", 1.0);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Unterminated ECMA array at end of format string */
	buf.len = 0;
	ret = amf_encode(&buf, "[%d%s%f", 1, "a", 1.0);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}

#pragma GCC diagnostic pop


/* Same rationale as above: cases[i].fmt is a variable (not a string
 * literal), and is dispatched to whichever amf_encode() call actually
 * matches its arity right below -- GCC cannot verify that itself. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
#pragma GCC diagnostic ignored "-Wformat-security"

static void test_amf_encode_enomem_boundary(void)
{
	/* -ENOMEM must trigger exactly when cap is one byte too small, and
	 * succeed when cap matches exactly, for each token type */
	uint8_t data[16];
	struct rtmp_buffer buf;
	int ret;

	struct {
		const char *fmt;
		size_t exact_cap;
	} cases[] = {
		{"%f", 9},
		{"%u", 2},
		{"%s", 3 + 3 /* "abc" */},
		{"0", 1},
	};

	for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
		buf.buf = data;
		buf.rd = 0;

		buf.cap = cases[i].exact_cap - 1;
		buf.len = 0;
		if (strcmp(cases[i].fmt, "%f") == 0)
			ret = amf_encode(&buf, cases[i].fmt, 1.0);
		else if (strcmp(cases[i].fmt, "%u") == 0)
			ret = amf_encode(&buf, cases[i].fmt, 1);
		else if (strcmp(cases[i].fmt, "%s") == 0)
			ret = amf_encode(&buf, cases[i].fmt, "abc");
		else
			ret = amf_encode(&buf, cases[i].fmt);
		CU_ASSERT_EQUAL(ret, -ENOMEM);

		buf.cap = cases[i].exact_cap;
		buf.len = 0;
		if (strcmp(cases[i].fmt, "%f") == 0)
			ret = amf_encode(&buf, cases[i].fmt, 1.0);
		else if (strcmp(cases[i].fmt, "%u") == 0)
			ret = amf_encode(&buf, cases[i].fmt, 1);
		else if (strcmp(cases[i].fmt, "%s") == 0)
			ret = amf_encode(&buf, cases[i].fmt, "abc");
		else
			ret = amf_encode(&buf, cases[i].fmt);
		CU_ASSERT_EQUAL(ret, 0);
		CU_ASSERT_EQUAL(buf.len, cases[i].exact_cap);
	}
}

#pragma GCC diagnostic pop


static void test_amf_get_msg_name(void)
{
	uint8_t data[32];
	struct rtmp_buffer buf = {
		.buf = data,
		.cap = sizeof(data),
		.len = 0,
		.rd = 0,
	};
	int ret;
	char *name;
	double id = -1.;

	ret = amf_encode(&buf, "%s,%f", "_result", 3.0);
	CU_ASSERT_EQUAL(ret, 0);

	name = amf_get_msg_name(&buf, &id);
	CU_ASSERT_PTR_NOT_NULL_FATAL(name);
	CU_ASSERT_STRING_EQUAL(name, "_result");
	CU_ASSERT_DOUBLE_EQUAL(id, 3.0, 0.0000001);
	free(name);

	/* Malformed message (no leading string at all): amf_get_msg_name()
	 * must return NULL cleanly instead of crashing. This is the
	 * property that fixes the amf_msg() NULL-deref in rtmp.c: any
	 * caller MUST check for a NULL return here. */
	buf.len = 0;
	buf.rd = 0;
	ret = amf_encode(&buf, "0"); /* null, not a string */
	CU_ASSERT_EQUAL(ret, 0);
	name = amf_get_msg_name(&buf, &id);
	CU_ASSERT_PTR_NULL(name);
}


static void test_amf_decode_boundaries(void)
{
	uint8_t data[32];
	struct rtmp_buffer buf;
	int ret;
	double dval;
	uint8_t bval;
	char *str;

	/* NULL args */
	ret = amf_get_number(NULL, &dval);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = amf_get_number(&buf, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = amf_get_boolean(NULL, &bval);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = amf_get_string(NULL, &str);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = amf_get_property(NULL, &str);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = amf_get_object_start(NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = amf_get_null(NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = amf_get_object_end(NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = amf_skip_data(NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Truncated buffers: -ENOMEM */
	buf.buf = data;
	buf.cap = sizeof(data);
	buf.rd = 0;
	buf.len = 0;
	ret = amf_get_number(&buf, &dval);
	CU_ASSERT_EQUAL(ret, -ENOMEM);
	ret = amf_get_boolean(&buf, &bval);
	CU_ASSERT_EQUAL(ret, -ENOMEM);
	ret = amf_get_null(&buf);
	CU_ASSERT_EQUAL(ret, -ENOMEM);
	ret = amf_get_object_start(&buf);
	CU_ASSERT_EQUAL(ret, -ENOMEM);
	ret = amf_get_object_end(&buf);
	CU_ASSERT_EQUAL(ret, -ENOMEM);

	/* Wrong tag, but with enough bytes available for the expected type:
	 * -EBADMSG. Each type needs its own exact byte count, otherwise the
	 * truncation check (-ENOMEM) fires first. */
	memset(data, TAG_BOOLEAN, sizeof(data));
	buf.rd = 0;
	buf.len = 9; /* AMF0_NUMBER_LEN */
	ret = amf_get_number(&buf, &dval);
	CU_ASSERT_EQUAL(ret, -EBADMSG);
	memset(data, TAG_NUMBER, sizeof(data));
	buf.rd = 0;
	buf.len = 2; /* AMF0_BOOLEAN_LEN */
	ret = amf_get_boolean(&buf, &bval);
	CU_ASSERT_EQUAL(ret, -EBADMSG);
	buf.rd = 0;
	buf.len = 1; /* AMF0_NULL_LEN */
	ret = amf_get_null(&buf);
	CU_ASSERT_EQUAL(ret, -EBADMSG);
	buf.rd = 0;
	buf.len = 1; /* AMF0_OBJ_START_LEN */
	ret = amf_get_object_start(&buf);
	CU_ASSERT_EQUAL(ret, -EBADMSG);
	buf.rd = 0;
	buf.len = 3; /* AMF0_OBJ_END_LEN */
	ret = amf_get_object_end(&buf);
	CU_ASSERT_EQUAL(ret, -EBADMSG);
}


static void test_amf_skip_data(void)
{
	uint8_t data[64];
	struct rtmp_buffer buf = {
		.buf = data,
		.cap = sizeof(data),
		.len = 0,
		.rd = 0,
	};
	int ret;

	/* Number, boolean, and null are all skippable */
	ret = amf_encode(&buf, "%f%u0", 1.0, 1);
	CU_ASSERT_EQUAL(ret, 0);
	ret = amf_skip_data(&buf);
	CU_ASSERT_EQUAL(ret, 0);
	ret = amf_skip_data(&buf);
	CU_ASSERT_EQUAL(ret, 0);
	ret = amf_skip_data(&buf);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(buf.rd, buf.len);

	/* Object/ECMA array start tags are not supported by amf_skip_data */
	buf.len = 0;
	buf.rd = 0;
	ret = amf_encode(&buf, "{%s:%f}", "a", 1.0);
	CU_ASSERT_EQUAL(ret, 0);
	ret = amf_skip_data(&buf);
	CU_ASSERT_EQUAL(ret, -ENOSYS);
}


static void test_amf_skip_data_string_advances_rd(void)
{
	/* Regression test: amf_skip_data() used to compute the length of a
	 * skipped string without ever advancing buf->rd past it, silently
	 * desynchronizing every subsequent read from the buffer. */
	uint8_t data[64];
	struct rtmp_buffer buf = {
		.buf = data,
		.cap = sizeof(data),
		.len = 0,
		.rd = 0,
	};
	int ret;
	char *second = NULL;

	ret = amf_encode(&buf, "%s%s", "hello", "world");
	CU_ASSERT_EQUAL(ret, 0);

	ret = amf_skip_data(&buf);
	CU_ASSERT_EQUAL(ret, 0);

	ret = amf_get_string(&buf, &second);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(second);
	CU_ASSERT_STRING_EQUAL(second, "world");
	CU_ASSERT_EQUAL(buf.rd, buf.len);
	free(second);
}


static void test_amf_get_string_long_string_bounds(void)
{
	/* Regression test: amf_get_stringp()'s upfront bounds check only
	 * ensured 2 bytes were available (enough for the short-string
	 * length field), but the long-string (0x0c) branch then read a
	 * 4-byte length at rd+1 unconditionally, over-reading up to 3
	 * bytes past the buffer on a truncated message. */
	uint8_t *data = malloc(4);
	struct rtmp_buffer buf;
	int ret;
	char *str = NULL;

	CU_ASSERT_PTR_NOT_NULL_FATAL(data);
	data[0] = TAG_LONG_STRING;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	buf.buf = data;
	buf.cap = 4;
	buf.len = 4;
	buf.rd = 0;

	ret = amf_get_string(&buf, &str);
	CU_ASSERT_EQUAL(ret, -ENOMEM);
	CU_ASSERT_PTR_NULL(str);

	free(data);
}


CU_TestInfo g_rtmp_test_amf[] = {
	{FN("amf-encode-number"), &test_amf_encode_number},
	{FN("amf-encode-boolean"), &test_amf_encode_boolean},
	{FN("amf-encode-string"), &test_amf_encode_string},
	{FN("amf-encode-long-string"), &test_amf_encode_long_string},
	{FN("amf-encode-null"), &test_amf_encode_null},
	{FN("amf-encode-object"), &test_amf_encode_object},
	{FN("amf-encode-ecma-array"), &test_amf_encode_ecma_array},
	{FN("amf-encode-separators-ignored"),
	 &test_amf_encode_separators_ignored},
	{FN("amf-encode-golden-buffer"), &test_amf_encode_golden_buffer},
	{FN("amf-encode-invalid-args"), &test_amf_encode_invalid_args},
	{FN("amf-encode-enomem-boundary"), &test_amf_encode_enomem_boundary},

	{FN("amf-get-msg-name"), &test_amf_get_msg_name},
	{FN("amf-decode-boundaries"), &test_amf_decode_boundaries},
	{FN("amf-skip-data"), &test_amf_skip_data},

	{FN("amf-skip-data-string-advances-rd"),
	 &test_amf_skip_data_string_advances_rd},
	{FN("amf-get-string-long-string-bounds"),
	 &test_amf_get_string_long_string_bounds},

	CU_TEST_INFO_NULL,
};
