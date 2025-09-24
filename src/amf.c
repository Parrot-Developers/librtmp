/**
 *  Copyright (c) 2018 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Parrot Company nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT COMPANY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "amf.h"

#ifdef _WIN32
#	include <winsock2.h>
#else
#	include <arpa/inet.h>
#endif
#include <errno.h>
#include <stdarg.h>
#include <string.h>

#define ULOG_TAG amf
#include <ulog.h>
ULOG_DECLARE_TAG(amf);

#define AMF0_NUMBER_TAG 0x00
#define AMF0_NUMBER_LEN 9

#define AMF0_BOOLEAN_TAG 0x01
#define AMF0_BOOLEAN_LEN 2

#define AMF0_STRING_TAG 0x02
#define AMF0_STRING_LEN_BASE 3
#define AMF0_LONG_STRING_TAG 0x0c
#define AMF0_LONG_STRING_LEN_BASE 5

#define AMF0_OBJ_START_TAG 0x03
#define AMF0_OBJ_START_LEN 1

#define AMF0_NULL_TAG 0x05
#define AMF0_NULL_LEN 1

#define AMF0_ECMA_TAG 0x08
#define AMF0_ECMA_LEN 5

#define AMF0_OBJ_END_TAG 0x09
#define AMF0_OBJ_END_LEN 3


/* Internal API */

static int amf_put_number(struct rtmp_buffer *buf, double value);

static int amf_put_boolean(struct rtmp_buffer *buf, uint8_t value);

static int amf_put_string(struct rtmp_buffer *buf, const char *string);

static int amf_put_property(struct rtmp_buffer *buf, const char *key);

static int amf_put_object_start(struct rtmp_buffer *buf);

static int amf_put_ecma_start(struct rtmp_buffer *buf, uint32_t len);

static int amf_put_null(struct rtmp_buffer *buf);

static int amf_put_object_end(struct rtmp_buffer *buf);


#ifdef _WIN32
static char *strndup(const char *s, size_t n)
{
	size_t len = strnlen(s, n);
	char *p = malloc(len + 1);
	if (p) {
		memcpy(p, s, len);
		p[len] = '\0';
	}
	return p;
}
#endif


/* Encoder API */

int amf_encode(struct rtmp_buffer *buffer, const char *fmt, ...)
{
	va_list ap;

	char c, d, e;
	int ret = 0;

	int objcount = 0;
	int arraycount = 0;

	/* arg values */
	double number;
	uint8_t boolean;
	const char *str;
	int32_t ecma_len;

	/* 1 if next arg MUST be a property (%s) */
	int needprop = 0;

	/* TODOs:
	 * - Check for mixed Objects/ECMA Arrays (i.e. "{%p[%d%p%s}]" should not
	 * pass)
	 * - Check for wrong ECMA objects len: encode("[%d]", 1) should not pass
	 * --> Maybe autocalc ECMA objects len based on the matching ']' in
	 * format string ?
	 */


	if (!buffer || !fmt)
		return -EINVAL;

	va_start(ap, fmt);
	while (*fmt && ret == 0) {
		c = *fmt++;
		switch (c) {
		case '%':
			d = *fmt++;
			switch (d) {
			case 'f':
				if (needprop) {
					ret = -EINVAL;
					break;
				}
				number = va_arg(ap, double);
				ret = amf_put_number(buffer, number);
				break;
			case 'u':
				if (needprop) {
					ret = -EINVAL;
					break;
				}
				boolean = va_arg(ap, unsigned int);
				ret = amf_put_boolean(buffer, boolean);
				break;
			case 's':
				str = va_arg(ap, const char *);
				if (needprop)
					ret = amf_put_property(buffer, str);
				else
					ret = amf_put_string(buffer, str);
				break;
			default:
				ULOGW("unexpected format char %c", d);
				ret = -EINVAL;
				break;
			}
			/* Update needprop */
			if (needprop)
				needprop = 0;
			else
				needprop = (objcount > 0 || arraycount > 0);
			break;

		case '{':
			if (needprop) {
				ret = -EINVAL;
				break;
			}
			objcount++;
			needprop = 1;
			ret = amf_put_object_start(buffer);
			break;

		case '}':
			if (objcount <= 0) {
				ret = -EINVAL;
				break;
			}
			objcount--;
			needprop = (objcount > 0 || arraycount > 0);
			ret = amf_put_object_end(buffer);
			break;

		case '[':
			if (needprop) {
				ret = -EINVAL;
				break;
			}
			d = *fmt++;
			e = *fmt++;
			if (d != '%' || e != 'd') {
				ret = -EINVAL;
				break;
			}
			arraycount++;
			needprop = 1;
			ecma_len = va_arg(ap, int);
			ret = amf_put_ecma_start(buffer, ecma_len);
			break;

		case ']':
			if (arraycount <= 0) {
				ret = -EINVAL;
				break;
			}
			arraycount--;
			needprop = (objcount > 0 || arraycount > 0);
			ret = amf_put_object_end(buffer);
			break;

		case '0':
			if (needprop) {
				ret = -EINVAL;
				break;
			}
			ret = amf_put_null(buffer);
			needprop = (objcount > 0 || arraycount > 0);
			break;

		case ',':
		case ':':
		case ' ':
		case '\t':
		case '\n':
			/* Ignore these chars, they can be added to format
			 * strings for readability */
			break;

		default:
			ULOGI("got unexpected char %c", c);
			ret = -EINVAL;
			break;
		}
	}
	va_end(ap);

	/* All arrays/objects should be terminated */
	if (arraycount > 0 || objcount > 0)
		ret = -EINVAL;

	return ret;
}


/* Decoder API */

char *amf_get_msg_name(struct rtmp_buffer *buf, double *id)
{
	int ret;
	char *str;

	ret = amf_get_string(buf, &str);
	if (ret != 0) {
		ULOG_ERRNO("amf_get_msg_name", -ret);
		return NULL;
	}

	ret = amf_get_number(buf, id);
	if (ret != 0) {
		ULOG_ERRNO("amf_get_msg_name", -ret);
		free(str);
		return NULL;
	}
	return str;
}


/* Internal implementation */

/* Endianness stuff ... */
#ifndef htonll
#	define htonll(x)                                                      \
		((1 == htonl(1)) ? (x)                                         \
				 : ((uint64_t)htonl((x)&0xFFFFFFFF) << 32) |   \
					   htonl((x) >> 32))
#endif
#ifndef ntohll
#	define ntohll(x)                                                      \
		((1 == ntohl(1)) ? (x)                                         \
				 : ((uint64_t)ntohl((x)&0xFFFFFFFF) << 32) |   \
					   ntohl((x) >> 32))
#endif


static int amf_put_number(struct rtmp_buffer *buf, double value)
{
	union {
		double d;
		uint64_t u;
	} v;
	v.d = value;
	uint64_t value_ne;
	if (!buf)
		return -EINVAL;

	if (buf->len + AMF0_NUMBER_LEN > buf->cap)
		return -ENOMEM;

	buf->buf[buf->len] = AMF0_NUMBER_TAG;


	value_ne = htonll(v.u);

	memcpy(&buf->buf[buf->len + 1], &value_ne, sizeof(value_ne));
	buf->len += AMF0_NUMBER_LEN;
	return 0;
}


static int amf_put_boolean(struct rtmp_buffer *buf, uint8_t value)
{
	if (!buf)
		return -EINVAL;

	if (buf->len + AMF0_BOOLEAN_LEN > buf->cap)
		return -ENOMEM;

	buf->buf[buf->len] = AMF0_BOOLEAN_TAG;
	buf->buf[buf->len + 1] = value;
	buf->len += AMF0_BOOLEAN_LEN;

	return 0;
}


static int amf_put_string_internal(struct rtmp_buffer *buf,
				   const char *string,
				   int put_tag)
{
	size_t slen;
	uint8_t tag;
	size_t total_len;
	size_t stroffset;
	size_t lenoffset;

	if (!buf || !string)
		return -EINVAL;

	slen = strlen(string);
	if (slen <= UINT16_MAX) {
		tag = AMF0_STRING_TAG;
		stroffset = AMF0_STRING_LEN_BASE;
	} else {
		tag = AMF0_LONG_STRING_TAG;
		stroffset = AMF0_LONG_STRING_LEN_BASE;
	}
	lenoffset = 1;
	if (!put_tag) {
		lenoffset--;
		stroffset--;
	}
	total_len = stroffset + slen;

	if (buf->len + total_len > buf->cap)
		return -ENOMEM;

	if (put_tag)
		buf->buf[buf->len] = tag;
	if (tag == AMF0_STRING_TAG) {
		uint16_t slen_ne = htons(slen);
		memcpy(&buf->buf[buf->len + lenoffset],
		       &slen_ne,
		       sizeof(slen_ne));
	} else {
		uint32_t slen_ne = htonl(slen);
		memcpy(&buf->buf[buf->len + lenoffset],
		       &slen_ne,
		       sizeof(slen_ne));
	}
	memcpy(&buf->buf[buf->len + stroffset], string, slen);
	buf->len += total_len;
	return 0;
}


static int amf_put_string(struct rtmp_buffer *buf, const char *string)
{
	return amf_put_string_internal(buf, string, 1);
}


static int amf_put_property(struct rtmp_buffer *buf, const char *key)
{
	return amf_put_string_internal(buf, key, 0);
}


static int amf_put_object_start(struct rtmp_buffer *buf)
{
	if (!buf)
		return -EINVAL;

	if (buf->len + AMF0_OBJ_START_LEN > buf->cap)
		return -ENOMEM;

	buf->buf[buf->len] = AMF0_OBJ_START_TAG;
	buf->len += AMF0_OBJ_START_LEN;

	return 0;
}


static int amf_put_ecma_start(struct rtmp_buffer *buf, uint32_t len)
{
	uint32_t len_ne;

	if (!buf)
		return -EINVAL;

	if (buf->len + AMF0_ECMA_LEN > buf->cap)
		return -ENOMEM;

	buf->buf[buf->len] = AMF0_ECMA_TAG;
	len_ne = htonl(len);
	memcpy(&buf->buf[buf->len + 1], &len_ne, sizeof(len_ne));
	buf->len += AMF0_ECMA_LEN;

	return 0;
}


static int amf_put_null(struct rtmp_buffer *buf)
{
	if (!buf)
		return -EINVAL;

	if (buf->len + AMF0_NULL_LEN > buf->cap)
		return -ENOMEM;

	buf->buf[buf->len] = AMF0_NULL_TAG;
	buf->len += AMF0_NULL_LEN;

	return 0;
}


static int amf_put_object_end(struct rtmp_buffer *buf)
{
	if (!buf)
		return -EINVAL;

	if (buf->len + AMF0_OBJ_END_LEN > buf->cap)
		return -ENOMEM;

	buf->buf[buf->len] = 0;
	buf->buf[buf->len + 1] = 0;
	buf->buf[buf->len + 2] = AMF0_OBJ_END_TAG;
	buf->len += AMF0_OBJ_END_LEN;

	return 0;
}

/* low level getters */

int amf_get_number(struct rtmp_buffer *buf, double *value)
{
	union {
		double d;
		uint64_t u;
	} v;
	uint64_t value_ne;

	if (!buf || !value)
		return -EINVAL;

	if (buf->rd + AMF0_NUMBER_LEN > buf->len)
		return -ENOMEM;

	if (buf->buf[buf->rd] != AMF0_NUMBER_TAG)
		return -EBADMSG;

	memcpy(&value_ne, &buf->buf[buf->rd + 1], sizeof(value_ne));
	v.u = ntohll(value_ne);
	*value = v.d;
	buf->rd += AMF0_NUMBER_LEN;
	return 0;
}


int amf_get_boolean(struct rtmp_buffer *buf, uint8_t *value)
{
	if (!buf || !value)
		return -EINVAL;

	if (buf->rd + AMF0_BOOLEAN_LEN > buf->len)
		return -ENOMEM;

	if (buf->buf[buf->rd] != AMF0_BOOLEAN_TAG)
		return -EBADMSG;

	*value = buf->buf[buf->rd + 1];
	buf->rd += AMF0_BOOLEAN_LEN;
	return 0;
}


static ssize_t amf_get_stringp(struct rtmp_buffer *buf,
			       char **stringp,
			       size_t *len,
			       int check_tag)
{
	uint8_t tag;
	size_t size_offset;
	size_t size_len;
	ssize_t total_len;
	/* Note: stringp is NOT null terminated ! */
	if (!buf || !stringp || !len)
		return -EINVAL;

	/* we always need at least 2 bytes, for non-tag strings */
	if (buf->rd + 2 > buf->len)
		return -ENOMEM;

	tag = buf->buf[buf->rd];

	if (!check_tag) {
		size_offset = 0;
		size_len = 2;
	} else if (tag == AMF0_STRING_TAG) {
		size_offset = 1;
		size_len = 2;
	} else if (tag == AMF0_LONG_STRING_TAG) {
		size_offset = 1;
		size_len = 4;
	} else {
		return -EBADMSG;
	}

	if (size_len == 2) {
		uint16_t val_ne;
		memcpy(&val_ne, &buf->buf[buf->rd + size_offset], size_len);
		*len = ntohs(val_ne);
	} else {
		uint32_t val_ne;
		memcpy(&val_ne, &buf->buf[buf->rd + size_offset], size_len);
		*len = ntohl(val_ne);
	}

	total_len = size_offset + size_len + *len;

	if (buf->rd + total_len > buf->len)
		return -ENOMEM;

	*stringp = (char *)&buf->buf[buf->rd + size_offset + size_len];
	return total_len;
}


int amf_get_string(struct rtmp_buffer *buf, char **string)
{
	ssize_t data_len;
	char *stringp;
	size_t string_len;

	if (!string)
		return -EINVAL;

	data_len = amf_get_stringp(buf, &stringp, &string_len, 1);
	if (data_len < 0)
		return data_len;

	buf->rd += data_len;

	*string = strndup(stringp, string_len);
	if (!*string)
		return -errno;

	return 0;
}


int amf_get_property(struct rtmp_buffer *buf, char **key)
{
	ssize_t data_len;
	char *stringp;
	size_t string_len;

	if (!key)
		return -EINVAL;

	data_len = amf_get_stringp(buf, &stringp, &string_len, 0);
	if (data_len < 0)
		return data_len;

	buf->rd += data_len;

	*key = strndup(stringp, string_len);
	if (!*key)
		return -errno;

	return 0;
}


int amf_get_object_start(struct rtmp_buffer *buf)
{
	if (!buf)
		return -EINVAL;

	if (buf->rd + AMF0_OBJ_START_LEN > buf->len)
		return -ENOMEM;

	if (buf->buf[buf->rd] != AMF0_OBJ_START_TAG)
		return -EBADMSG;

	buf->rd += AMF0_OBJ_START_LEN;
	return 0;
}


int amf_get_null(struct rtmp_buffer *buf)
{
	if (!buf)
		return -EINVAL;

	if (buf->rd + AMF0_NULL_LEN > buf->len)
		return -ENOMEM;

	if (buf->buf[buf->rd] != AMF0_NULL_TAG)
		return -EBADMSG;

	buf->rd += AMF0_NULL_LEN;
	return 0;
}


int amf_get_object_end(struct rtmp_buffer *buf)
{
	uint8_t data[3] = {0, 0, AMF0_OBJ_END_TAG};
	if (!buf)
		return -EINVAL;

	if (buf->rd + AMF0_OBJ_END_LEN > buf->len)
		return -ENOMEM;

	if (memcmp(data, &buf->buf[buf->rd], sizeof(data)) != 0)
		return -EBADMSG;

	buf->rd += AMF0_OBJ_END_LEN;
	return 0;
}


int amf_skip_data(struct rtmp_buffer *buf)
{
	uint8_t tag;
	double number;
	uint8_t boolean;
	char *string;
	size_t len;
	ssize_t sret;

	if (!buf)
		return -EINVAL;

	if (buf->rd + 1 > buf->len)
		return -ENOMEM;

	tag = buf->buf[buf->rd];
	switch (tag) {
	case AMF0_NUMBER_TAG:
		return amf_get_number(buf, &number);
	case AMF0_BOOLEAN_TAG:
		return amf_get_boolean(buf, &boolean);
	case AMF0_STRING_TAG:
	case AMF0_LONG_STRING_TAG:
		sret = amf_get_stringp(buf, &string, &len, 1);
		return sret < 0 ? sret : 0;
	case AMF0_NULL_TAG:
		return amf_get_null(buf);
	default:
		ULOGE("cannot skip tag type %u", tag);
		return -ENOSYS;
	}
}
