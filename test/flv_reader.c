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
#include "flv_reader.h"

#include <arpa/inet.h>
#include <errno.h>
#include <stdlib.h>

#include <libpomp.h>

#define ULOG_TAG flv_reader
#include <ulog.h>
ULOG_DECLARE_TAG(flv_reader);

#define TAG_BUFFER_SZ (128 * 1024)

#define TAG_HEADER_LEN 11

struct flv_reader {
	FILE *f;
	struct flv_reader_cbs cbs;
	void *userdata;
	struct pomp_timer *timer;
	uint32_t timestamp;

	uint8_t *tag_buffer;
	size_t tb_len;
	size_t tb_cap;

	float speed;
	int loop;
	off_t initial_offset;
	uint32_t loop_ts;
};

static enum flv_data_type get_data_type(uint8_t raw)
{
	enum flv_data_type t = FLV_UNKNOWN;
	switch (raw & 0x1f) {
	case 8:
		t = FLV_AUDIO;
		break;
	case 9:
		t = FLV_VIDEO;
		break;
	case 18:
		t = FLV_META;
		break;
	}
	return t;
}

static uint32_t get_tag_ts(uint8_t *tag)
{
	uint32_t ts;

	ts = tag[4] << 16;
	ts += tag[5] << 8;
	ts += tag[6];
	ts += tag[7] << 24;

	return ts;
}

static size_t get_tag_len(uint8_t *tag)
{
	size_t len;

	len = tag[1] << 16;
	len += tag[2] << 8;
	len += tag[3];

	/* This is properly useless since len can not possibly be more than
	 * 24 bits long but silences coverity TAINTED_SCALAR warning. */
	if (len > 0x00FFFFFF)
		len = 0x00FFFFFF;

	return len;
}


static void pomp_timer_cb(struct pomp_timer *timer, void *userdata)
{
	struct flv_reader *r = userdata;
	size_t len;
	int ret;

	uint8_t pt_len[4];

	uint32_t tag_timestamp;
	size_t tag_len;

	uint32_t delta;

	if (!r) {
		ULOG_ERRNO("pomp_timer_cb", EINVAL);
		return;
	}

	if (r->tb_len > 0)
		r->cbs.tag_cb(&r->tag_buffer[TAG_HEADER_LEN],
			      r->tb_len - TAG_HEADER_LEN,
			      get_data_type(r->tag_buffer[0]),
			      r->timestamp,
			      r->userdata);
	r->tb_len = 0;

retry:
	/* Read & discard previous tag len */
	len = fread(pt_len, 1, 4, r->f);
	if (len != 4) {
		ULOG_ERRNO("fread", errno);
		return;
	}

	len = fread(r->tag_buffer, 1, TAG_HEADER_LEN, r->f);
	if (len != TAG_HEADER_LEN) {
		if (feof(r->f)) {
			ULOGI("End of file");
			if (r->loop) {
				ULOGI("Loop back to start");
				ret = fseeko(r->f, r->initial_offset, SEEK_SET);
				if (ret < 0) {
					ret = -errno;
					ULOG_ERRNO("fseeko", -ret);
					return;
				}
				r->loop_ts = r->timestamp + 33;
				goto retry;
			}
			r->cbs.eof_cb(r->userdata);
		} else {
			ULOG_ERRNO("fread", errno);
		}
		return;
	}
	r->tb_len += len;

	tag_len = get_tag_len(r->tag_buffer);
	tag_timestamp = get_tag_ts(r->tag_buffer) + r->loop_ts;

	tag_timestamp /= r->speed;

	if (tag_timestamp < r->timestamp)
		delta = 1;
	else
		delta = tag_timestamp - r->timestamp;
	if (delta == 0)
		delta = 1;

	while (r->tb_len + tag_len > r->tb_cap) {
		size_t nl = r->tb_cap + TAG_BUFFER_SZ;
		void *tmp = realloc(r->tag_buffer, nl);
		if (!tmp) {
			ULOG_ERRNO("realloc", ENOMEM);
			return;
		}
		r->tag_buffer = tmp;
		r->tb_cap = nl;
	}

	len = fread(&r->tag_buffer[TAG_HEADER_LEN], 1, tag_len, r->f);
	if (len != tag_len) {
		ULOGW("Read %zu bytes instead of %zu", len, tag_len);
		return;
	}
	r->tb_len += len;

	r->timestamp = tag_timestamp;
	ret = pomp_timer_set(timer, delta);
	if (ret != 0)
		ULOG_ERRNO("pomp_timer_set", -ret);
}

struct flv_reader *flv_open_file(const char *path,
				 struct pomp_loop *loop,
				 const struct flv_reader_cbs *cbs,
				 void *userdata)
{
	uint8_t flv_header[9];
	size_t len;
	struct flv_reader *r;
	uint32_t header_len;

	if (!path || !loop || !cbs || !cbs->tag_cb || !cbs->eof_cb) {
		ULOG_ERRNO("flv_open_file", EINVAL);
		return NULL;
	}

	r = calloc(1, sizeof(*r));
	if (!r) {
		ULOG_ERRNO("flv_open_file", ENOMEM);
		return NULL;
	}

	r->tag_buffer = malloc(TAG_BUFFER_SZ);
	if (!r->tag_buffer) {
		ULOG_ERRNO("flv_open_file", ENOMEM);
		goto error;
	}
	r->tb_cap = TAG_BUFFER_SZ;

	r->cbs = *cbs;
	r->userdata = userdata;

	r->timer = pomp_timer_new(loop, pomp_timer_cb, r);
	if (!r->timer) {
		ULOG_ERRNO("flv_open_file", ENOMEM);
		goto error;
	}

	r->f = fopen(path, "rb");
	if (!r->f) {
		ULOG_ERRNO("flv_open_file", errno);
		goto error;
	}

	len = fread(flv_header, 1, sizeof(flv_header), r->f);
	if (len != sizeof(flv_header)) {
		ULOG_ERRNO("flv_open_file", EBADMSG);
		goto error;
	}
	if (flv_header[0] != 'F' || flv_header[1] != 'L' ||
	    flv_header[2] != 'V') {
		ULOG_ERRNO("flv_open_file", EBADMSG);
		goto error;
	}

	memcpy(&header_len, &flv_header[5], sizeof(header_len));
	header_len = ntohl(header_len);
	if (header_len != 9) {
		ULOG_ERRNO("flv_open_file", EBADMSG);
		goto error;
	}

	r->initial_offset = ftello(r->f);

	return r;

error:
	flv_close_file(r);
	return NULL;
}

void flv_close_file(struct flv_reader *r)
{
	if (!r)
		return;
	if (r->timer) {
		pomp_timer_clear(r->timer);
		pomp_timer_destroy(r->timer);
	}
	if (r->f)
		fclose(r->f);
	free(r->tag_buffer);
	free(r);
}

int flv_start_read(struct flv_reader *r, float speed, int loop)
{
	if (!r)
		return -EINVAL;

	r->speed = speed;
	r->loop = loop;

	return pomp_timer_set(r->timer, 1);
}

const char *flv_data_type_str(enum flv_data_type type)
{
	switch (type) {
	case FLV_META:
		return "METADATA";
	case FLV_AUDIO:
		return "AUDIO";
	case FLV_VIDEO:
		return "VIDEO";
	default:
		return "UNKNOWN";
	}
}
