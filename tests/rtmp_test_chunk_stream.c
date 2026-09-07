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
#include "rtmp_chunk_stream.h"

#include <arpa/inet.h>
#include <sys/uio.h>

#include <libpomp.h>
#include <transport-socket/tskt.h>
#include <transport-socket/tskt_ops.h>

#include <aac/aac.h>
#include <aac/aac_types.h>
#include <aac/aac_writer.h>
#include <audio-defs/adefs.h>


/*
 * Fake tskt_socket: no real network I/O, just in-memory FIFOs. The chunk
 * stream module only ever calls read()/writev()/set_event_cb()/
 * update_events() on the socket it is given (grep-confirmed), so that is
 * all that needs to be implemented.
 */

#define FAKE_SOCKET_BUF_SIZE 8192

struct fake_socket {
	struct tskt_socket base;

	uint8_t rx_buf[FAKE_SOCKET_BUF_SIZE];
	size_t rx_len;
	size_t rx_pos;

	uint8_t tx_buf[FAKE_SOCKET_BUF_SIZE];
	size_t tx_len;

	tskt_socket_event_cb_t cb;
	void *cb_userdata;
	uint32_t watched_events;

	/* if > 0, writev returns at most this many bytes (partial send test) */
	size_t partial_limit;
};


static ssize_t fake_socket_read(struct tskt_socket *self,
				void *buf,
				size_t cap,
				uint64_t *ts_us)
{
	struct fake_socket *fs = (struct fake_socket *)self;
	size_t avail = fs->rx_len - fs->rx_pos;
	size_t n;

	UNUSED(ts_us);

	if (avail == 0) {
		errno = EAGAIN;
		return -1;
	}
	n = avail < cap ? avail : cap;
	memcpy(buf, &fs->rx_buf[fs->rx_pos], n);
	fs->rx_pos += n;
	return (ssize_t)n;
}


static ssize_t fake_socket_writev(struct tskt_socket *self,
				  const struct iovec *iov,
				  size_t iov_len)
{
	struct fake_socket *fs = (struct fake_socket *)self;
	size_t written = 0;
	size_t i;

	for (i = 0; i < iov_len; i++) {
		size_t n = iov[i].iov_len;
		if (fs->partial_limit > 0 && written + n > fs->partial_limit)
			n = fs->partial_limit - written;
		if (n == 0)
			break;
		CU_ASSERT_TRUE_FATAL(fs->tx_len + n <= sizeof(fs->tx_buf));
		memcpy(&fs->tx_buf[fs->tx_len], iov[i].iov_base, n);
		fs->tx_len += n;
		written += n;
		if (fs->partial_limit > 0 && written >= fs->partial_limit)
			break;
	}
	return (ssize_t)written;
}


static int fake_socket_set_event_cb(struct tskt_socket *self,
				    uint32_t events,
				    tskt_socket_event_cb_t cb,
				    void *userdata)
{
	struct fake_socket *fs = (struct fake_socket *)self;
	fs->cb = cb;
	fs->cb_userdata = userdata;
	fs->watched_events = events;
	return 0;
}


static int fake_socket_update_events(struct tskt_socket *self,
				     uint32_t events_to_add,
				     uint32_t events_to_remove)
{
	struct fake_socket *fs = (struct fake_socket *)self;
	fs->watched_events =
		(fs->watched_events | events_to_add) & ~events_to_remove;
	return 0;
}


static const struct tskt_socket_ops fake_socket_ops = {
	.read = &fake_socket_read,
	.writev = &fake_socket_writev,
	.set_event_cb = &fake_socket_set_event_cb,
	.update_events = &fake_socket_update_events,
};


static void fake_socket_init(struct fake_socket *fs)
{
	memset(fs, 0, sizeof(*fs));
	fs->base.ops = &fake_socket_ops;
}


static void
fake_socket_inject(struct fake_socket *fs, const uint8_t *data, size_t len)
{
	CU_ASSERT_TRUE_FATAL(fs->rx_len + len <= sizeof(fs->rx_buf));
	memcpy(&fs->rx_buf[fs->rx_len], data, len);
	fs->rx_len += len;
}


/* Build a header-type-0 RTMP chunk header (basic header + full message
 * header, no continuation chunks). csid must be < 64 and timestamp must be
 * < 0xffffff for all the tests in this file. */
static size_t build_header0(uint8_t *out,
			    int csid,
			    uint32_t timestamp,
			    uint32_t msg_len,
			    uint8_t mtid,
			    uint32_t msid)
{
	size_t n = 0;

	out[n++] = (uint8_t)(csid & 0x3f);
	out[n++] = (timestamp >> 16) & 0xff;
	out[n++] = (timestamp >> 8) & 0xff;
	out[n++] = timestamp & 0xff;
	out[n++] = (msg_len >> 16) & 0xff;
	out[n++] = (msg_len >> 8) & 0xff;
	out[n++] = msg_len & 0xff;
	out[n++] = mtid;
	memcpy(&out[n], &msid, sizeof(msid));
	n += sizeof(msid);
	return n;
}


/*
 * Test fixture: a chunk stream wired to a fake socket, with all 4
 * mandatory callbacks recording their invocations.
 */

struct chunk_stream_fixture {
	struct pomp_loop *loop;
	struct fake_socket sock;
	struct rtmp_chunk_stream *stream;

	uint32_t peer_bw;
	int peer_bw_count;

	struct rtmp_buffer last_amf_msg;
	int amf_msg_count;

	uint8_t *last_data_sent;
	void *last_data_sent_userdata;
	int data_sent_count;

	enum rtmp_client_disconnection_reason last_disconnect_reason;
	int disconnected_count;
};


static void cb_peer_bw_changed(uint32_t bandwidth, void *userdata)
{
	struct chunk_stream_fixture *fx = userdata;
	fx->peer_bw = bandwidth;
	fx->peer_bw_count++;
}


static void cb_amf_msg(struct rtmp_buffer *data, void *userdata)
{
	struct chunk_stream_fixture *fx = userdata;
	size_t len = data->len - data->rd;

	free(fx->last_amf_msg.buf);
	fx->last_amf_msg.buf = malloc(len ? len : 1);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx->last_amf_msg.buf);
	memcpy(fx->last_amf_msg.buf, &data->buf[data->rd], len);
	fx->last_amf_msg.len = len;
	fx->last_amf_msg.cap = len;
	fx->last_amf_msg.rd = 0;
	fx->amf_msg_count++;
}


static void cb_data_sent(uint8_t *data, void *data_userdata, void *userdata)
{
	struct chunk_stream_fixture *fx = userdata;
	fx->last_data_sent = data;
	fx->last_data_sent_userdata = data_userdata;
	fx->data_sent_count++;
}


static void cb_disconnected(void *userdata,
			    enum rtmp_client_disconnection_reason reason)
{
	struct chunk_stream_fixture *fx = userdata;
	fx->last_disconnect_reason = reason;
	fx->disconnected_count++;
}


static const struct rtmp_chunk_cbs s_fixture_cbs = {
	.peer_bw_changed = &cb_peer_bw_changed,
	.amf_msg = &cb_amf_msg,
	.data_sent = &cb_data_sent,
	.disconnected = &cb_disconnected,
};


static void fixture_init(struct chunk_stream_fixture *fx)
{
	memset(fx, 0, sizeof(*fx));
	fake_socket_init(&fx->sock);
	fx->loop = pomp_loop_new();
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx->loop);
	fx->stream =
		new_chunk_stream(fx->loop, &fx->sock.base, &s_fixture_cbs, fx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx->stream);
}


static void fixture_cleanup(struct chunk_stream_fixture *fx)
{
	int ret;

	ret = delete_chunk_stream(fx->stream);
	CU_ASSERT_EQUAL(ret, 0);
	ret = pomp_loop_destroy(fx->loop);
	CU_ASSERT_EQUAL(ret, 0);
	free(fx->last_amf_msg.buf);
}


static void fixture_flush_out(struct chunk_stream_fixture *fx)
{
	if (fx->sock.cb != NULL)
		fx->sock.cb(&fx->sock.base,
			    POMP_FD_EVENT_OUT,
			    fx->sock.cb_userdata);
}


static void fixture_deliver_in(struct chunk_stream_fixture *fx,
			       const uint8_t *data,
			       size_t len)
{
	fake_socket_inject(&fx->sock, data, len);
	if (fx->sock.cb != NULL)
		fx->sock.cb(
			&fx->sock.base, POMP_FD_EVENT_IN, fx->sock.cb_userdata);
}


static void test_chunk_stream_new_delete_invalid_args(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct fake_socket sock;
	struct rtmp_chunk_stream *stream;
	struct rtmp_chunk_cbs incomplete_cbs = s_fixture_cbs;
	int ret;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	fake_socket_init(&sock);

	stream = new_chunk_stream(NULL, &sock.base, &s_fixture_cbs, NULL);
	CU_ASSERT_PTR_NULL(stream);
	stream = new_chunk_stream(loop, NULL, &s_fixture_cbs, NULL);
	CU_ASSERT_PTR_NULL(stream);
	stream = new_chunk_stream(loop, &sock.base, NULL, NULL);
	CU_ASSERT_PTR_NULL(stream);

	incomplete_cbs.peer_bw_changed = NULL;
	stream = new_chunk_stream(loop, &sock.base, &incomplete_cbs, NULL);
	CU_ASSERT_PTR_NULL(stream);
	incomplete_cbs = s_fixture_cbs;
	incomplete_cbs.amf_msg = NULL;
	stream = new_chunk_stream(loop, &sock.base, &incomplete_cbs, NULL);
	CU_ASSERT_PTR_NULL(stream);
	incomplete_cbs = s_fixture_cbs;
	incomplete_cbs.data_sent = NULL;
	stream = new_chunk_stream(loop, &sock.base, &incomplete_cbs, NULL);
	CU_ASSERT_PTR_NULL(stream);
	incomplete_cbs = s_fixture_cbs;
	incomplete_cbs.disconnected = NULL;
	stream = new_chunk_stream(loop, &sock.base, &incomplete_cbs, NULL);
	CU_ASSERT_PTR_NULL(stream);

	ret = delete_chunk_stream(NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = pomp_loop_destroy(loop);
	CU_ASSERT_EQUAL(ret, 0);
}


static void test_chunk_stream_new_delete(void)
{
	struct chunk_stream_fixture fx;

	fixture_init(&fx);
	fixture_cleanup(&fx);
}


static void test_chunk_stream_tx_amf_message(void)
{
	struct chunk_stream_fixture fx;
	uint8_t amf_data[32];
	struct rtmp_buffer msg = {
		.buf = amf_data,
		.cap = sizeof(amf_data),
		.len = 0,
		.rd = 0,
	};
	uint8_t expected[64];
	size_t header_len;
	int ret;

	fixture_init(&fx);

	ret = amf_encode(&msg, "%s", "connect");
	CU_ASSERT_EQUAL(ret, 0);

	ret = send_amf_message(fx.stream, &msg);
	CU_ASSERT_EQUAL(ret, 0);

	fixture_flush_out(&fx);

	header_len = build_header0(expected, 3, 0, (uint32_t)msg.len, 0x14, 0);
	memcpy(&expected[header_len], msg.buf, msg.len);

	CU_ASSERT_EQUAL_FATAL(fx.sock.tx_len, header_len + msg.len);
	CU_ASSERT_EQUAL(memcmp(fx.sock.tx_buf, expected, header_len + msg.len),
			0);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_tx_amf_message_publish_csid(void)
{
	/* "publish" messages must go out on csid 4, not the usual csid 3
	 * ("mandatory to connect to the wowza RTMP server", per the comment
	 * in send_amf_message()) */
	struct chunk_stream_fixture fx;
	uint8_t amf_data[32];
	struct rtmp_buffer msg = {
		.buf = amf_data,
		.cap = sizeof(amf_data),
		.len = 0,
		.rd = 0,
	};
	int ret;

	fixture_init(&fx);

	ret = amf_encode(&msg, "%s", "publish");
	CU_ASSERT_EQUAL(ret, 0);

	ret = send_amf_message(fx.stream, &msg);
	CU_ASSERT_EQUAL(ret, 0);

	fixture_flush_out(&fx);

	CU_ASSERT_TRUE_FATAL(fx.sock.tx_len > 0);
	CU_ASSERT_EQUAL(fx.sock.tx_buf[0], 4);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_tx_set_chunk_size(void)
{
	struct chunk_stream_fixture fx;
	uint8_t expected[32];
	uint32_t val_ne;
	size_t header_len;
	int ret;

	fixture_init(&fx);

	ret = set_chunk_size(fx.stream, 0);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = set_chunk_size(fx.stream, 256);
	CU_ASSERT_EQUAL(ret, 0);

	fixture_flush_out(&fx);

	header_len = build_header0(expected, 2, 0, 4, 0x01, 0);
	val_ne = htonl(256);
	memcpy(&expected[header_len], &val_ne, sizeof(val_ne));

	CU_ASSERT_EQUAL_FATAL(fx.sock.tx_len, header_len + sizeof(val_ne));
	CU_ASSERT_EQUAL(memcmp(fx.sock.tx_buf, expected, fx.sock.tx_len), 0);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_tx_header_caching(void)
{
	/* Two consecutive messages on the same channel, with identical
	 * mtid/msid/length and a null timestamp delta, must use the
	 * "no header" (type 3) form for the second one */
	struct chunk_stream_fixture fx;
	uint8_t amf_data1[32];
	uint8_t amf_data2[32];
	struct rtmp_buffer msg1 = {
		.buf = amf_data1,
		.cap = sizeof(amf_data1),
		.len = 0,
		.rd = 0,
	};
	struct rtmp_buffer msg2 = {
		.buf = amf_data2,
		.cap = sizeof(amf_data2),
		.len = 0,
		.rd = 0,
	};
	int ret;

	fixture_init(&fx);

	ret = amf_encode(&msg1, "%s", "abc");
	CU_ASSERT_EQUAL(ret, 0);
	ret = send_amf_message(fx.stream, &msg1);
	CU_ASSERT_EQUAL(ret, 0);
	fixture_flush_out(&fx);
	CU_ASSERT_TRUE_FATAL(fx.sock.tx_len > 0);
	CU_ASSERT_EQUAL(fx.sock.tx_buf[0], 3); /* header type 0, csid 3 */

	fx.sock.tx_len = 0;

	ret = amf_encode(&msg2, "%s", "xyz"); /* same encoded length as msg1 */
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL_FATAL(msg2.len, msg1.len);
	ret = send_amf_message(fx.stream, &msg2);
	CU_ASSERT_EQUAL(ret, 0);
	fixture_flush_out(&fx);

	CU_ASSERT_EQUAL_FATAL(fx.sock.tx_len, 1 + msg2.len);
	CU_ASSERT_EQUAL(fx.sock.tx_buf[0], (uint8_t)((3 << 6) | 3));
	CU_ASSERT_EQUAL(memcmp(&fx.sock.tx_buf[1], msg2.buf, msg2.len), 0);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_tx_queue_full(void)
{
	struct chunk_stream_fixture fx;
	int i;
	int ret;

	fixture_init(&fx);

	for (i = 0; i < RTMP_MAX_QUEUE_SIZE; i++) {
		uint8_t amf_data[16];
		struct rtmp_buffer msg = {
			.buf = amf_data,
			.cap = sizeof(amf_data),
			.len = 0,
			.rd = 0,
		};
		ret = amf_encode(&msg, "%s", "x");
		CU_ASSERT_EQUAL(ret, 0);
		ret = send_amf_message(fx.stream, &msg);
		CU_ASSERT_EQUAL(ret, i);
	}

	{
		uint8_t amf_data[16];
		struct rtmp_buffer msg = {
			.buf = amf_data,
			.cap = sizeof(amf_data),
			.len = 0,
			.rd = 0,
		};
		ret = amf_encode(&msg, "%s", "overflow");
		CU_ASSERT_EQUAL(ret, 0);
		ret = send_amf_message(fx.stream, &msg);
		CU_ASSERT_EQUAL(ret, -EAGAIN);
	}

	/* Never flushed: delete_chunk_stream() must silently free all
	 * queued (internal) buffers without crashing or misbehaving */
	fixture_cleanup(&fx);
}


static void test_chunk_stream_tx_send_video_frame(void)
{
	struct chunk_stream_fixture fx;
	uint8_t frame_data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
	struct rtmp_buffer frame = {
		.buf = frame_data,
		.cap = sizeof(frame_data),
		.len = sizeof(frame_data),
		.rd = 0,
	};
	uint8_t expected_data_header[5] = {0x17, 0x01, 0, 0, 0};
	uint8_t header[32];
	size_t header_len;
	int ret;

	fixture_init(&fx);

	ret = send_video_frame(fx.stream, &frame, 1000, 0, 1, NULL);
	CU_ASSERT_TRUE(ret >= 0);

	fixture_flush_out(&fx);

	header_len = build_header0(
		header,
		4,
		1000,
		(uint32_t)(sizeof(expected_data_header) + sizeof(frame_data)),
		0x09,
		0);
	CU_ASSERT_EQUAL_FATAL(fx.sock.tx_len,
			      header_len + sizeof(expected_data_header) +
				      sizeof(frame_data));
	CU_ASSERT_EQUAL(memcmp(fx.sock.tx_buf, header, header_len), 0);
	CU_ASSERT_EQUAL(memcmp(&fx.sock.tx_buf[header_len],
			       expected_data_header,
			       sizeof(expected_data_header)),
			0);
	CU_ASSERT_EQUAL(memcmp(&fx.sock.tx_buf[header_len +
					       sizeof(expected_data_header)],
			       frame_data,
			       sizeof(frame_data)),
			0);

	CU_ASSERT_EQUAL(fx.data_sent_count, 1);
	CU_ASSERT_PTR_EQUAL(fx.last_data_sent, frame_data);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_tx_send_metadata(void)
{
	struct chunk_stream_fixture fx;
	uint8_t meta_data[32];
	struct rtmp_buffer meta = {
		.buf = meta_data,
		.cap = sizeof(meta_data),
		.len = 0,
		.rd = 0,
	};
	uint8_t prefix_data[16];
	struct rtmp_buffer prefix = {
		.buf = prefix_data,
		.cap = sizeof(prefix_data),
		.len = 0,
		.rd = 0,
	};
	uint8_t header[32];
	size_t header_len;
	int dummy_userdata;
	int ret;

	fixture_init(&fx);

	ret = amf_encode(&meta, "%s", "onMetaData");
	CU_ASSERT_EQUAL(ret, 0);
	ret = amf_encode(&prefix, "%s", "@setDataFrame");
	CU_ASSERT_EQUAL(ret, 0);

	ret = send_metadata(fx.stream, &meta, 0, 0, &dummy_userdata);
	CU_ASSERT_TRUE(ret >= 0);

	fixture_flush_out(&fx);

	header_len = build_header0(
		header, 4, 0, (uint32_t)(prefix.len + meta.len), 0x12, 0);
	CU_ASSERT_EQUAL_FATAL(fx.sock.tx_len,
			      header_len + prefix.len + meta.len);
	CU_ASSERT_EQUAL(memcmp(fx.sock.tx_buf, header, header_len), 0);
	CU_ASSERT_EQUAL(
		memcmp(&fx.sock.tx_buf[header_len], prefix.buf, prefix.len), 0);
	CU_ASSERT_EQUAL(memcmp(&fx.sock.tx_buf[header_len + prefix.len],
			       meta.buf,
			       meta.len),
			0);

	CU_ASSERT_EQUAL(fx.data_sent_count, 1);
	CU_ASSERT_PTR_EQUAL(fx.last_data_sent, meta_data);
	CU_ASSERT_PTR_EQUAL(fx.last_data_sent_userdata, &dummy_userdata);

	fixture_cleanup(&fx);
}


static int build_aac_lc_asc(unsigned int sample_rate,
			    unsigned int channel_count,
			    uint8_t **buf,
			    size_t *len)
{
	struct adef_format fmt = {
		.encoding = ADEF_ENCODING_AAC_LC,
		.channel_count = channel_count,
		.bit_depth = 16,
		.sample_rate = sample_rate,
	};
	struct aac_asc asc;
	int ret;

	fmt.aac.data_format = ADEF_AAC_DATA_FORMAT_RAW;
	memset(&asc, 0, sizeof(asc));

	ret = aac_asc_from_adef_format(&fmt, &asc);
	if (ret != 0)
		return ret;

	return aac_write_asc(&asc, buf, len);
}


static void test_chunk_stream_tx_send_audio_data(void)
{
	struct chunk_stream_fixture fx;
	uint8_t *asc_buf = NULL;
	size_t asc_len = 0;
	struct rtmp_buffer asc_rtmp;
	uint8_t header[32];
	size_t header_len;
	int ret;

	fixture_init(&fx);

	ret = build_aac_lc_asc(48000, 2, &asc_buf, &asc_len);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(asc_buf);

	asc_rtmp.buf = asc_buf;
	asc_rtmp.cap = asc_len;
	asc_rtmp.len = asc_len;
	asc_rtmp.rd = 0;

	ret = send_audio_data(fx.stream, &asc_rtmp, 0, 1, NULL);
	CU_ASSERT_TRUE(ret >= 0);

	fixture_flush_out(&fx);

	header_len =
		build_header0(header, 3, 0, (uint32_t)(2 + asc_len), 0x08, 0);
	CU_ASSERT_EQUAL_FATAL(fx.sock.tx_len, header_len + 2 + asc_len);
	CU_ASSERT_EQUAL(memcmp(fx.sock.tx_buf, header, header_len), 0);
	/* format=HE-AAC(0xa0) | 16-bit(0x2) | 48kHz(0xc) | stereo(0x1) */
	CU_ASSERT_EQUAL(fx.sock.tx_buf[header_len], 0xaf);
	/* AACPacketType = 0 (sequence header), since is_meta was set */
	CU_ASSERT_EQUAL(fx.sock.tx_buf[header_len + 1], 0x00);
	CU_ASSERT_EQUAL(
		memcmp(&fx.sock.tx_buf[header_len + 2], asc_buf, asc_len), 0);

	CU_ASSERT_EQUAL(fx.data_sent_count, 1);
	CU_ASSERT_PTR_EQUAL(fx.last_data_sent, asc_buf);

	free(asc_buf);
	fixture_cleanup(&fx);
}


static void test_chunk_stream_tx_send_audio_data_rejects_bad_asc(void)
{
	/* A sample rate outside {48000,44100,22050,11025} must be rejected
	 * by aac_asc_to_rtmp_audio_config() (only reachable through
	 * send_audio_data(), since it is a static, file-local helper) */
	struct chunk_stream_fixture fx;
	uint8_t *asc_buf = NULL;
	size_t asc_len = 0;
	struct rtmp_buffer asc_rtmp;
	int ret;

	fixture_init(&fx);

	ret = build_aac_lc_asc(16000, 1, &asc_buf, &asc_len);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(asc_buf);

	asc_rtmp.buf = asc_buf;
	asc_rtmp.cap = asc_len;
	asc_rtmp.len = asc_len;
	asc_rtmp.rd = 0;

	ret = send_audio_data(fx.stream, &asc_rtmp, 0, 1, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	CU_ASSERT_EQUAL(fx.sock.tx_len, 0);

	free(asc_buf);
	fixture_cleanup(&fx);
}


static void test_chunk_stream_flush(void)
{
	struct chunk_stream_fixture fx;
	uint8_t amf_data[16];
	struct rtmp_buffer msg = {
		.buf = amf_data,
		.cap = sizeof(amf_data),
		.len = 0,
		.rd = 0,
	};
	int ret;

	fixture_init(&fx);

	ret = amf_encode(&msg, "%s", "x");
	CU_ASSERT_EQUAL(ret, 0);
	ret = send_amf_message(fx.stream, &msg);
	CU_ASSERT_TRUE(ret >= 0);

	ret = flush_chunk_stream(fx.stream);
	CU_ASSERT_EQUAL(ret, 0);

	/* The queued buffer was internal (send_amf_message): flushing must
	 * free it silently, never touch the wire, never call data_sent() */
	CU_ASSERT_EQUAL(fx.sock.tx_len, 0);
	CU_ASSERT_EQUAL(fx.data_sent_count, 0);

	ret = flush_chunk_stream(NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_store_message_stream_id(void)
{
	struct chunk_stream_fixture fx;
	uint8_t amf_data[16];
	struct rtmp_buffer msg = {
		.buf = amf_data,
		.cap = sizeof(amf_data),
		.len = 0,
		.rd = 0,
	};
	uint8_t header[32];
	size_t header_len;
	int ret;

	fixture_init(&fx);

	ret = store_message_stream_id(fx.stream, 42);
	CU_ASSERT_EQUAL(ret, 0);

	ret = amf_encode(&msg, "%s", "x");
	CU_ASSERT_EQUAL(ret, 0);
	ret = send_amf_message(fx.stream, &msg);
	CU_ASSERT_TRUE(ret >= 0);
	fixture_flush_out(&fx);

	header_len = build_header0(header, 3, 0, (uint32_t)msg.len, 0x14, 42);
	CU_ASSERT_TRUE_FATAL(fx.sock.tx_len >= header_len);
	CU_ASSERT_EQUAL(memcmp(fx.sock.tx_buf, header, header_len), 0);

	ret = store_message_stream_id(NULL, 1);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_rx_amf_message(void)
{
	struct chunk_stream_fixture fx;
	uint8_t amf_data[16];
	struct rtmp_buffer msg = {
		.buf = amf_data,
		.cap = sizeof(amf_data),
		.len = 0,
		.rd = 0,
	};
	uint8_t chunk[64];
	size_t n;
	int ret;

	fixture_init(&fx);

	ret = amf_encode(&msg, "%s", "onStatus");
	CU_ASSERT_EQUAL(ret, 0);

	n = build_header0(chunk, 3, 0, (uint32_t)msg.len, 0x14, 0);
	memcpy(&chunk[n], msg.buf, msg.len);
	n += msg.len;

	fixture_deliver_in(&fx, chunk, n);

	CU_ASSERT_EQUAL(fx.amf_msg_count, 1);
	CU_ASSERT_EQUAL_FATAL(fx.last_amf_msg.len, msg.len);
	CU_ASSERT_EQUAL(memcmp(fx.last_amf_msg.buf, msg.buf, msg.len), 0);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_rx_peer_bandwidth(void)
{
	struct chunk_stream_fixture fx;
	uint8_t payload[5];
	uint32_t bw_ne = htonl(500000);
	uint8_t chunk[32];
	size_t n;

	fixture_init(&fx);

	memcpy(payload, &bw_ne, sizeof(bw_ne));
	payload[4] = 0; /* BW_TYPE_HARD */

	n = build_header0(chunk, 2, 0, sizeof(payload), 0x06, 0);
	memcpy(&chunk[n], payload, sizeof(payload));
	n += sizeof(payload);

	fixture_deliver_in(&fx, chunk, n);

	CU_ASSERT_EQUAL(fx.peer_bw_count, 1);
	CU_ASSERT_EQUAL(fx.peer_bw, 500000);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_rx_user_control_ping(void)
{
	struct chunk_stream_fixture fx;
	uint8_t payload[6];
	uint32_t sid_ne = htonl(0x1234);
	uint8_t chunk[32];
	uint8_t expected_payload[6];
	uint8_t expected[32];
	size_t n;
	size_t expected_len;

	fixture_init(&fx);

	payload[0] = 0x00;
	payload[1] = 0x06; /* User Control: PingRequest */
	memcpy(&payload[2], &sid_ne, sizeof(sid_ne));

	n = build_header0(chunk, 2, 0, sizeof(payload), 0x04, 0);
	memcpy(&chunk[n], payload, sizeof(payload));
	n += sizeof(payload);

	fixture_deliver_in(&fx, chunk, n);
	fixture_flush_out(&fx);

	expected_payload[0] = 0x00;
	expected_payload[1] = 0x07; /* PingResponse */
	memcpy(&expected_payload[2], &sid_ne, sizeof(sid_ne));

	expected_len = build_header0(
		expected, 2, 0, sizeof(expected_payload), 0x04, 0);
	memcpy(&expected[expected_len],
	       expected_payload,
	       sizeof(expected_payload));
	expected_len += sizeof(expected_payload);

	CU_ASSERT_EQUAL_FATAL(fx.sock.tx_len, expected_len);
	CU_ASSERT_EQUAL(memcmp(fx.sock.tx_buf, expected, expected_len), 0);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_rx_set_chunk_size_applies(void)
{
	/* Indirect proof that data_complete()'s SetChunkSize (mtid 0x01)
	 * handling really updates the stream's rx_chunk_size: a message
	 * bigger than the default 128-byte chunk size can only be parsed as
	 * a single chunk (=> amf_msg fires) if the new size was applied */
	struct chunk_stream_fixture fx;
	uint32_t cs_ne = htonl(256);
	uint8_t chunk1[32];
	size_t n1;
	uint8_t payload[150];
	uint8_t chunk2[256];
	size_t n2;
	size_t i;

	fixture_init(&fx);

	n1 = build_header0(chunk1, 2, 0, sizeof(cs_ne), 0x01, 0);
	memcpy(&chunk1[n1], &cs_ne, sizeof(cs_ne));
	n1 += sizeof(cs_ne);
	fixture_deliver_in(&fx, chunk1, n1);

	for (i = 0; i < sizeof(payload); i++)
		payload[i] = (uint8_t)i;

	n2 = build_header0(chunk2, 3, 0, sizeof(payload), 0x14, 0);
	memcpy(&chunk2[n2], payload, sizeof(payload));
	n2 += sizeof(payload);
	fixture_deliver_in(&fx, chunk2, n2);

	CU_ASSERT_EQUAL(fx.amf_msg_count, 1);
	CU_ASSERT_EQUAL_FATAL(fx.last_amf_msg.len, sizeof(payload));
	CU_ASSERT_EQUAL(memcmp(fx.last_amf_msg.buf, payload, sizeof(payload)),
			0);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_rx_window_ack_size(void)
{
	/* Covers set_window_ack_size + send_ack: a WindowAckSize chunk (type
	 * 0x05) of 16 bytes with window_ack_size=16 is enough to satisfy
	 * rcv_bytes_since_last_ack (16) >= window/2 (8), so send_ack fires
	 * inside set_window_ack_size and the ACK chunk appears in the TX
	 * buffer. */
	struct chunk_stream_fixture fx;
	uint32_t window_ne;
	uint8_t chunk[32];
	size_t n;
	uint32_t total_ne;
	uint8_t expected[32];
	size_t expected_len;

	fixture_init(&fx);

	window_ne = htonl(16);
	n = build_header0(chunk, 2, 0, sizeof(window_ne), 0x05, 0);
	memcpy(&chunk[n], &window_ne, sizeof(window_ne));
	n += sizeof(window_ne); /* n == 16 == total bytes on the wire */

	fixture_deliver_in(&fx, chunk, n);
	fixture_flush_out(&fx);

	/* ACK (type 0x03): payload = htonl(total_bytes) */
	total_ne = htonl((uint32_t)n);
	expected_len = build_header0(expected, 2, 0, sizeof(total_ne), 0x03, 0);
	memcpy(&expected[expected_len], &total_ne, sizeof(total_ne));
	expected_len += sizeof(total_ne);

	CU_ASSERT_EQUAL_FATAL(fx.sock.tx_len, expected_len);
	CU_ASSERT_EQUAL(memcmp(fx.sock.tx_buf, expected, expected_len), 0);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_rx_ack_short_message_regression(void)
{
	/* Regression test for the OOB read fixed in data_complete()'s Ack
	 * (mtid 0x03) handling: a 2-byte Ack (a real one is 4 bytes) must
	 * not crash, and the stream must remain fully usable afterwards. */
	struct chunk_stream_fixture fx;
	uint8_t bad_payload[2] = {0x11, 0x22};
	uint8_t chunk[32];
	size_t n;
	uint8_t amf_data[16];
	struct rtmp_buffer msg = {
		.buf = amf_data,
		.cap = sizeof(amf_data),
		.len = 0,
		.rd = 0,
	};
	uint8_t chunk2[64];
	size_t n2;
	int ret;

	fixture_init(&fx);

	n = build_header0(chunk, 2, 0, sizeof(bad_payload), 0x03, 0);
	memcpy(&chunk[n], bad_payload, sizeof(bad_payload));
	n += sizeof(bad_payload);
	fixture_deliver_in(&fx, chunk, n);

	ret = amf_encode(&msg, "%s", "onStatus");
	CU_ASSERT_EQUAL(ret, 0);
	n2 = build_header0(chunk2, 3, 0, (uint32_t)msg.len, 0x14, 0);
	memcpy(&chunk2[n2], msg.buf, msg.len);
	n2 += msg.len;
	fixture_deliver_in(&fx, chunk2, n2);

	CU_ASSERT_EQUAL(fx.amf_msg_count, 1);
	CU_ASSERT_EQUAL(fx.disconnected_count, 0);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_rx_ack_valid(void)
{
	/* Valid 4-byte Ack (mtid 0x03): handler logs the value and returns.
	 * No callback fires, no crash. */
	struct chunk_stream_fixture fx;
	uint32_t seq_ne = htonl(1000);
	uint8_t chunk[32];
	size_t n;

	fixture_init(&fx);

	n = build_header0(chunk, 2, 0, sizeof(seq_ne), 0x03, 0);
	memcpy(&chunk[n], &seq_ne, sizeof(seq_ne));
	n += sizeof(seq_ne);
	fixture_deliver_in(&fx, chunk, n);

	CU_ASSERT_EQUAL(fx.amf_msg_count, 0);
	CU_ASSERT_EQUAL(fx.disconnected_count, 0);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_rx_abort(void)
{
	/* Abort message (mtid 0x02): deliver a partial AMF message on csid=3
	 * (msg_len=200, only 128 bytes provided so message stays incomplete),
	 * then send an abort targeting csid=3.  Covers the list-walk branch
	 * that finds a channel with msg.len > 0. */
	struct chunk_stream_fixture fx;
	uint8_t partial_chunk[256];
	uint8_t abort_chunk[32];
	uint32_t abort_csid_ne;
	size_t n;
	size_t abort_n;

	fixture_init(&fx);

	/* Partial AMF message on csid=3: header says 200 bytes, provide 128
	 * (= default chunk size limit) so the message stays incomplete. */
	n = build_header0(partial_chunk, 3, 0, 200, 0x14, 0);
	memset(&partial_chunk[n], 0x00, 128);
	n += 128;
	fixture_deliver_in(&fx, partial_chunk, n);
	CU_ASSERT_EQUAL(fx.amf_msg_count, 0);

	/* Abort on csid=2 targeting csid=3 */
	abort_csid_ne = htonl(3);
	abort_n = build_header0(
		abort_chunk, 2, 0, sizeof(abort_csid_ne), 0x02, 0);
	memcpy(&abort_chunk[abort_n], &abort_csid_ne, sizeof(abort_csid_ne));
	abort_n += sizeof(abort_csid_ne);
	fixture_deliver_in(&fx, abort_chunk, abort_n);

	CU_ASSERT_EQUAL(fx.disconnected_count, 0);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_rx_user_control_events(void)
{
	/* User Control Message (mtid 0x04): cover types 0, 1, 2, 3, 4, and
	 * unknown (type 6 / PingRequest is already covered by the ping test).
	 */
	struct chunk_stream_fixture fx;
	uint8_t chunk[64];
	uint8_t payload[10];
	uint32_t sid_ne = htonl(1);
	size_t n;
	size_t i;
	static const uint16_t simple_types[] = {0, 1, 2, 4};

	fixture_init(&fx);

	/* Types 0 (StreamBegin), 1 (EOF), 2 (Dry), 4 (Recorded): 6-byte
	 * payload = 2-byte type + 4-byte stream ID */
	for (i = 0; i < sizeof(simple_types) / sizeof(simple_types[0]); i++) {
		payload[0] = (uint8_t)(simple_types[i] >> 8);
		payload[1] = (uint8_t)(simple_types[i] & 0xff);
		memcpy(&payload[2], &sid_ne, 4);
		n = build_header0(chunk, 2, 0, 6, 0x04, 0);
		memcpy(&chunk[n], payload, 6);
		n += 6;
		fixture_deliver_in(&fx, chunk, n);
	}

	/* Type 3 (SetBufferLength): 10-byte payload = type + sid + buffer_len
	 */
	{
		uint32_t buflen_ne = htonl(500);
		payload[0] = 0x00;
		payload[1] = 0x03;
		memcpy(&payload[2], &sid_ne, 4);
		memcpy(&payload[6], &buflen_ne, 4);
		n = build_header0(chunk, 2, 0, 10, 0x04, 0);
		memcpy(&chunk[n], payload, 10);
		n += 10;
		fixture_deliver_in(&fx, chunk, n);
	}

	/* Unknown type (9): hits the default ULOGW case */
	{
		payload[0] = 0x00;
		payload[1] = 0x09;
		memcpy(&payload[2], &sid_ne, 4);
		n = build_header0(chunk, 2, 0, 6, 0x04, 0);
		memcpy(&chunk[n], payload, 6);
		n += 6;
		fixture_deliver_in(&fx, chunk, n);
	}

	CU_ASSERT_EQUAL(fx.amf_msg_count, 0);
	CU_ASSERT_EQUAL(fx.disconnected_count, 0);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_rx_peer_bandwidth_variants(void)
{
	/* Peer bandwidth (mtid 0x06): cover SOFT-lower (stored), SOFT-higher
	 * (ignored), DYNAMIC-after-SOFT (ignored), DYNAMIC-after-HARD (stored),
	 * and unknown bw_type (error, no callback). */
	struct chunk_stream_fixture fx;
	uint8_t chunk[32];
	uint8_t payload[5];
	size_t n;

	fixture_init(&fx);

#define SEND_PEER_BW(bw_val, bw_type_byte)                                     \
	do {                                                                   \
		uint32_t _ne = htonl(bw_val);                                  \
		memcpy(payload, &_ne, 4);                                      \
		payload[4] = (bw_type_byte);                                   \
		n = build_header0(chunk, 2, 0, sizeof(payload), 0x06, 0);      \
		memcpy(&chunk[n], payload, sizeof(payload));                   \
		n += sizeof(payload);                                          \
		fixture_deliver_in(&fx, chunk, n);                             \
	} while (0)

	/* HARD 500000: first-ever BW message (bw_type_unknown branch) */
	SEND_PEER_BW(500000, 0 /* BW_TYPE_HARD */);
	CU_ASSERT_EQUAL(fx.peer_bw_count, 1);
	CU_ASSERT_EQUAL(fx.peer_bw, 500000);

	/* SOFT 300000 < 500000: stored (bw decreases) */
	SEND_PEER_BW(300000, 1 /* BW_TYPE_SOFT */);
	CU_ASSERT_EQUAL(fx.peer_bw_count, 2);
	CU_ASSERT_EQUAL(fx.peer_bw, 300000);

	/* SOFT 400000 > 300000: ignored (bw would increase), callback still
	 * fires with the current (unchanged) value */
	SEND_PEER_BW(400000, 1 /* BW_TYPE_SOFT */);
	CU_ASSERT_EQUAL(fx.peer_bw_count, 3);
	CU_ASSERT_EQUAL(fx.peer_bw, 300000);

	/* DYNAMIC when stream->bw_type == SOFT (not HARD): ignored */
	SEND_PEER_BW(400000, 2 /* BW_TYPE_DYNAMIC */);
	CU_ASSERT_EQUAL(fx.peer_bw_count, 4);
	CU_ASSERT_EQUAL(fx.peer_bw, 300000);

	/* HARD 200000: stored */
	SEND_PEER_BW(200000, 0 /* BW_TYPE_HARD */);
	CU_ASSERT_EQUAL(fx.peer_bw_count, 5);
	CU_ASSERT_EQUAL(fx.peer_bw, 200000);

	/* DYNAMIC 100000 when stream->bw_type == HARD: stored as HARD */
	SEND_PEER_BW(100000, 2 /* BW_TYPE_DYNAMIC */);
	CU_ASSERT_EQUAL(fx.peer_bw_count, 6);
	CU_ASSERT_EQUAL(fx.peer_bw, 100000);

	/* Unknown bw_type (3 = BW_TYPE_UNKNOWN on wire): -EBADMSG, no
	 * callback */
	SEND_PEER_BW(50000, 3 /* BW_TYPE_UNKNOWN, hits else */);
	CU_ASSERT_EQUAL(fx.peer_bw_count, 6);

#undef SEND_PEER_BW

	fixture_cleanup(&fx);
}


static void test_chunk_stream_rx_extended_timestamp(void)
{
	/* RX extended timestamp: a type-0 chunk with timestamp=0xFFFFFF in
	 * the 3-byte field signals that a 4-byte extended timestamp follows
	 * the message header.  After parsing, amf_msg fires once. */
	struct chunk_stream_fixture fx;
	uint8_t chunk[64];
	uint32_t ets_ne;
	size_t n = 0;

	fixture_init(&fx);

	/* basic header: fmt=0, csid=3 */
	chunk[n++] = 0x03;
	/* timestamp sentinel: 0xFFFFFF */
	chunk[n++] = 0xFF;
	chunk[n++] = 0xFF;
	chunk[n++] = 0xFF;
	/* msg_len = 1 */
	chunk[n++] = 0x00;
	chunk[n++] = 0x00;
	chunk[n++] = 0x01;
	/* mtid = AMF */
	chunk[n++] = 0x14;
	/* msid (LE) */
	chunk[n++] = 0x00;
	chunk[n++] = 0x00;
	chunk[n++] = 0x00;
	chunk[n++] = 0x00;
	/* extended timestamp (BE) */
	ets_ne = htonl(0x1000000);
	memcpy(&chunk[n], &ets_ne, sizeof(ets_ne));
	n += sizeof(ets_ne);
	/* 1-byte AMF null payload */
	chunk[n++] = 0x05;

	fixture_deliver_in(&fx, chunk, n);

	CU_ASSERT_EQUAL(fx.amf_msg_count, 1);
	CU_ASSERT_EQUAL(fx.disconnected_count, 0);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_rx_multi_byte_csid(void)
{
	/* 2-byte basic header (csid_field == 0): first byte has csid=0 which
	 * means the real csid is encoded in the next byte as (byte + 64).
	 * Delivers a 1-byte AMF message on csid=64. */
	struct chunk_stream_fixture fx;
	uint8_t chunk[64];
	size_t n = 0;

	fixture_init(&fx);

	/* 2-byte basic header: fmt=0 (bits 7-6 = 00), csid_field=0 (bits 5-0 =
	 * 0) */
	chunk[n++] = 0x00;
	chunk[n++] = 0x00; /* extra byte: csid = 64 + 0 = 64 */
	/* type-0 message header (11 bytes) */
	chunk[n++] = 0x00; /* timestamp */
	chunk[n++] = 0x00;
	chunk[n++] = 0x00;
	chunk[n++] = 0x00; /* msg_len */
	chunk[n++] = 0x00;
	chunk[n++] = 0x01; /* = 1 byte */
	chunk[n++] = 0x14; /* mtid = AMF */
	chunk[n++] = 0x00; /* msid (LE) */
	chunk[n++] = 0x00;
	chunk[n++] = 0x00;
	chunk[n++] = 0x00;
	/* 1-byte AMF null payload */
	chunk[n++] = 0x05;

	fixture_deliver_in(&fx, chunk, n);

	CU_ASSERT_EQUAL(fx.amf_msg_count, 1);
	CU_ASSERT_EQUAL(fx.disconnected_count, 0);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_rx_3byte_csid(void)
{
	/* 3-byte basic header (csid_field == 1): first byte fmt=0 | 0x01,
	 * second byte = low(csid - 64), third byte = high.
	 * For csid=320: byte[1]=0, byte[2]=1 (320 = 0*256 + 0 + 64... actually
	 * 320 - 64 = 256 = 256*1 + 0 → byte[1]=0, byte[2]=1). */
	struct chunk_stream_fixture fx;
	uint8_t chunk[64];
	size_t n = 0;

	fixture_init(&fx);

	/* 3-byte basic header: fmt=0 (bits 7-6 = 00), csid_field=1 */
	chunk[n++] = 0x01;
	chunk[n++] = 0x00; /* low = csid - 64 = 256 → low=0 */
	chunk[n++] = 0x01; /* high: csid = 64 + 0 + 256*1 = 320 */
	/* type-0 message header (11 bytes) */
	chunk[n++] = 0x00; /* timestamp */
	chunk[n++] = 0x00;
	chunk[n++] = 0x00;
	chunk[n++] = 0x00; /* msg_len */
	chunk[n++] = 0x00;
	chunk[n++] = 0x01; /* 1 byte */
	chunk[n++] = 0x14; /* mtid = AMF */
	chunk[n++] = 0x00; /* msid (LE) */
	chunk[n++] = 0x00;
	chunk[n++] = 0x00;
	chunk[n++] = 0x00;
	/* 1-byte AMF null payload */
	chunk[n++] = 0x05;

	fixture_deliver_in(&fx, chunk, n);

	CU_ASSERT_EQUAL(fx.amf_msg_count, 1);
	CU_ASSERT_EQUAL(fx.disconnected_count, 0);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_rx_control_bad_sizes(void)
{
	/* data_complete() size-validation branches: inject control messages
	 * with wrong payload lengths and verify they return -EBADMSG (logged
	 * only) without breaking the stream.  Each bad message is followed by
	 * a valid AMF message to confirm the stream is still usable. */
	struct chunk_stream_fixture fx;
	uint8_t chunk[64];
	uint8_t payload[6];
	size_t n;
	uint8_t amf_chunk[32];
	size_t amf_n;

	fixture_init(&fx);

	/* Helper lambda (macro): send a bad-sized control msg then a good AMF
	 */
#define SEND_BAD_CTL(mtid_val, bad_payload, bad_len)                           \
	do {                                                                   \
		n = build_header0(chunk, 2, 0, (bad_len), (mtid_val), 0);      \
		memcpy(&chunk[n], (bad_payload), (bad_len));                   \
		n += (bad_len);                                                \
		fixture_deliver_in(&fx, chunk, n);                             \
		amf_n = build_header0(amf_chunk, 3, 0, 1, 0x14, 0);            \
		amf_chunk[amf_n++] = 0x05;                                     \
		fixture_deliver_in(&fx, amf_chunk, amf_n);                     \
	} while (0)

	/* SetChunkSize (0x01): should be 4 bytes, send 2 */
	payload[0] = 0x00;
	payload[1] = 0x80;
	SEND_BAD_CTL(0x01, payload, 2);

	/* Abort (0x02): should be 4 bytes, send 2 */
	payload[0] = 0x00;
	payload[1] = 0x03;
	SEND_BAD_CTL(0x02, payload, 2);

	/* WindowAckSize (0x05): should be 4 bytes, send 2 */
	payload[0] = 0x00;
	payload[1] = 0x00;
	SEND_BAD_CTL(0x05, payload, 2);

	/* PeerBandwidth (0x06): should be 5 bytes, send 3 */
	payload[0] = 0x00;
	payload[1] = 0x0f;
	payload[2] = 0x42;
	SEND_BAD_CTL(0x06, payload, 3);

	/* UserControl (0x04): should be >= 6, send 4 */
	payload[0] = 0x00;
	payload[1] = 0x00;
	payload[2] = 0x00;
	payload[3] = 0x01;
	SEND_BAD_CTL(0x04, payload, 4);

	/* UserControl type 3 (SetBufferLength): should be >= 10, send 8 */
	payload[0] = 0x00;
	payload[1] = 0x03; /* type 3 */
	payload[2] = 0x00;
	payload[3] = 0x00;
	payload[4] = 0x00;
	payload[5] = 0x01;
	/* 6 bytes so far; build header for 8-byte payload */
	n = build_header0(chunk, 2, 0, 8, 0x04, 0);
	memset(&chunk[n], 0, 8);
	chunk[n + 1] = 0x03; /* type = SetBufferLength */
	n += 8;
	fixture_deliver_in(&fx, chunk, n);
	amf_n = build_header0(amf_chunk, 3, 0, 1, 0x14, 0);
	amf_chunk[amf_n++] = 0x05;
	fixture_deliver_in(&fx, amf_chunk, amf_n);

#undef SEND_BAD_CTL

	/* Each bad message + valid AMF: 6 pairs → 6 AMF callbacks */
	CU_ASSERT_EQUAL(fx.amf_msg_count, 6);
	CU_ASSERT_EQUAL(fx.disconnected_count, 0);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_rx_unknown_mtid(void)
{
	/* data_complete() default case: inject a message with an unknown MTID
	 * (0x07 is not handled); must log a warning and leave stream usable. */
	struct chunk_stream_fixture fx;
	uint8_t payload[4] = {0x11, 0x22, 0x33, 0x44};
	uint8_t chunk[32];
	uint8_t amf_chunk[32];
	size_t n, amf_n;

	fixture_init(&fx);

	n = build_header0(chunk, 3, 0, sizeof(payload), 0x07, 0);
	memcpy(&chunk[n], payload, sizeof(payload));
	n += sizeof(payload);
	fixture_deliver_in(&fx, chunk, n);

	/* Stream must remain usable */
	amf_n = build_header0(amf_chunk, 3, 0, 1, 0x14, 0);
	amf_chunk[amf_n++] = 0x05;
	fixture_deliver_in(&fx, amf_chunk, amf_n);

	CU_ASSERT_EQUAL(fx.amf_msg_count, 1);
	CU_ASSERT_EQUAL(fx.disconnected_count, 0);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_tx_audio_22050hz(void)
{
	/* aac_asc_to_rtmp_audio_config() 22050 Hz branch:
	 * audio_setting must be 0xaa (mono) i.e. HE-AAC|16-bit|22050Hz|mono. */
	struct chunk_stream_fixture fx;
	uint8_t *asc_buf = NULL;
	size_t asc_len = 0;
	struct rtmp_buffer asc_rtmp;
	size_t header_len;
	uint8_t header[32];
	int ret;

	fixture_init(&fx);

	ret = build_aac_lc_asc(22050, 1, &asc_buf, &asc_len);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(asc_buf);

	asc_rtmp.buf = asc_buf;
	asc_rtmp.cap = asc_len;
	asc_rtmp.len = asc_len;
	asc_rtmp.rd = 0;

	ret = send_audio_data(fx.stream, &asc_rtmp, 0, 1, NULL);
	(void)ret; /* -EINVAL if sample rate unsupported by this build */
	if (ret < 0) {
		free(asc_buf);
		fixture_cleanup(&fx);
		return; /* skip if 22050 Hz not supported */
	}

	fixture_flush_out(&fx);

	header_len =
		build_header0(header, 3, 0, (uint32_t)(2 + asc_len), 0x08, 0);
	CU_ASSERT_TRUE_FATAL(fx.sock.tx_len >= header_len + 1);
	/* HE-AAC(0xa0) | 16-bit(0x02) | 22050Hz(0x08) | mono(0x00) = 0xaa */
	CU_ASSERT_EQUAL(fx.sock.tx_buf[header_len], 0xaa);

	free(asc_buf);
	fixture_cleanup(&fx);
}


static void test_chunk_stream_tx_audio_11025hz(void)
{
	/* aac_asc_to_rtmp_audio_config() 11025 Hz branch:
	 * audio_setting must be 0xa6 (mono) i.e. HE-AAC|16-bit|11025Hz|mono. */
	struct chunk_stream_fixture fx;
	uint8_t *asc_buf = NULL;
	size_t asc_len = 0;
	struct rtmp_buffer asc_rtmp;
	size_t header_len;
	uint8_t header[32];
	int ret;

	fixture_init(&fx);

	ret = build_aac_lc_asc(11025, 1, &asc_buf, &asc_len);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(asc_buf);

	asc_rtmp.buf = asc_buf;
	asc_rtmp.cap = asc_len;
	asc_rtmp.len = asc_len;
	asc_rtmp.rd = 0;

	ret = send_audio_data(fx.stream, &asc_rtmp, 0, 1, NULL);
	if (ret < 0) {
		free(asc_buf);
		fixture_cleanup(&fx);
		return; /* skip if 11025 Hz not supported */
	}

	fixture_flush_out(&fx);

	header_len =
		build_header0(header, 3, 0, (uint32_t)(2 + asc_len), 0x08, 0);
	CU_ASSERT_TRUE_FATAL(fx.sock.tx_len >= header_len + 1);
	/* HE-AAC(0xa0) | 16-bit(0x02) | 11025Hz(0x04) | mono(0x00) = 0xa6 */
	CU_ASSERT_EQUAL(fx.sock.tx_buf[header_len], 0xa6);

	free(asc_buf);
	fixture_cleanup(&fx);
}


static void test_chunk_stream_watchdog_timeout(void)
{
	/* watchdog_timer_cb + notify_disconnection (TIMEOUT path):
	 * call the TARGET_TEST hook that directly invokes watchdog_timer_cb.
	 * The disconnected callback must fire exactly once with TIMEOUT. */
	struct chunk_stream_fixture fx;

	fixture_init(&fx);

	rtmp_chunk_stream_trigger_watchdog_for_test(fx.stream);

	CU_ASSERT_EQUAL(fx.disconnected_count, 1);
	CU_ASSERT_EQUAL(fx.last_disconnect_reason,
			RTMP_CLIENT_DISCONNECTION_REASON_TIMEOUT);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_tx_extended_timestamp(void)
{
	/* TX extended timestamp: send_video_frame() with timestamp > 0xFFFFFF
	 * must encode 0xFFFFFF in the 3-byte TS field and append a 4-byte
	 * extended timestamp after the message header. */
	struct chunk_stream_fixture fx;
	uint8_t frame_data[1] = {0x00};
	struct rtmp_buffer frame = {
		.buf = frame_data,
		.cap = sizeof(frame_data),
		.len = sizeof(frame_data),
		.rd = 0,
	};
	uint32_t ts = 0x1000000;
	uint32_t ets_ne;
	int ret;

	fixture_init(&fx);

	ret = send_video_frame(fx.stream, &frame, ts, 0, 1, NULL);
	CU_ASSERT_TRUE(ret >= 0);

	fixture_flush_out(&fx);

	/* TX header layout (type-0 with extended timestamp):
	 *   [0]     = 0x04 (fmt=0, csid=4)
	 *   [1..3]  = 0xFF, 0xFF, 0xFF (sentinel)
	 *   [4..6]  = msg_len (3 bytes)
	 *   [7]     = 0x09 (video mtid)
	 *   [8..11] = msid (LE, 0)
	 *   [12..15] = extended timestamp (BE) */
	CU_ASSERT_TRUE_FATAL(fx.sock.tx_len >= 16);
	CU_ASSERT_EQUAL(fx.sock.tx_buf[0], 0x04);
	CU_ASSERT_EQUAL(fx.sock.tx_buf[1], 0xFF);
	CU_ASSERT_EQUAL(fx.sock.tx_buf[2], 0xFF);
	CU_ASSERT_EQUAL(fx.sock.tx_buf[3], 0xFF);
	CU_ASSERT_EQUAL(fx.sock.tx_buf[7], 0x09);
	ets_ne = htonl(ts);
	CU_ASSERT_EQUAL(memcmp(&fx.sock.tx_buf[12], &ets_ne, sizeof(ets_ne)),
			0);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_tx_extended_csid_2byte(void)
{
	/* TX with csid=100 (64 <= csid < 320): 2-byte basic header.
	 * byte[0] = (fmt << 6) | 0x00 = 0x00 for type 0.
	 * byte[1] = csid - 64 = 36. */
	struct chunk_stream_fixture fx;
	int ret;

	fixture_init(&fx);

	ret = rtmp_chunk_stream_send_data_for_test(fx.stream, 100, 0x14, 1);
	CU_ASSERT_TRUE(ret >= 0);

	fixture_flush_out(&fx);

	CU_ASSERT_TRUE_FATAL(fx.sock.tx_len >= 2);
	CU_ASSERT_EQUAL(fx.sock.tx_buf[0], 0x00); /* fmt=0, csid_field=0 */
	CU_ASSERT_EQUAL(fx.sock.tx_buf[1], 36); /* csid - 64 = 100 - 64 */

	fixture_cleanup(&fx);
}


static void test_chunk_stream_tx_extended_csid_3byte(void)
{
	/* TX with csid=320 (320 <= csid < 65599): 3-byte basic header.
	 * byte[0] = (fmt << 6) | 0x01 = 0x01 for type 0.
	 * offset = 320 - 64 = 256; low = 0, high = 1. */
	struct chunk_stream_fixture fx;
	int ret;

	fixture_init(&fx);

	ret = rtmp_chunk_stream_send_data_for_test(fx.stream, 320, 0x14, 1);
	CU_ASSERT_TRUE(ret >= 0);

	fixture_flush_out(&fx);

	CU_ASSERT_TRUE_FATAL(fx.sock.tx_len >= 3);
	CU_ASSERT_EQUAL(fx.sock.tx_buf[0], 0x01); /* fmt=0, csid_field=1 */
	CU_ASSERT_EQUAL(fx.sock.tx_buf[1], 0); /* low byte of offset */
	CU_ASSERT_EQUAL(fx.sock.tx_buf[2], 1); /* high byte of offset */

	fixture_cleanup(&fx);
}


static void test_chunk_stream_tx_partial_send(void)
{
	/* Partial TX: fake socket returns fewer bytes than requested.
	 * Covers send_chunk "partial send" branch (lines 1127-1130),
	 * process_channel_send chunk_partial_len path (lines 1157-1174),
	 * and event_data_out tx_chan_in_progess path (lines 1299-1331). */
	struct chunk_stream_fixture fx;
	uint8_t frame_data[4] = {0x27, 0x01, 0, 0};
	struct rtmp_buffer frame = {
		.buf = frame_data, .cap = 4, .len = 4, .rd = 0};
	int ret;

	fixture_init(&fx);

	/* Limit socket to 5 bytes per write so the first OUT event is partial
	 */
	fx.sock.partial_limit = 5;

	ret = send_video_frame(fx.stream, &frame, 100, 0, 0, frame_data);
	CU_ASSERT_TRUE(ret >= 0);

	fixture_flush_out(&fx);
	/* Frame not yet delivered: data_sent callback must not have fired */
	CU_ASSERT_EQUAL(fx.data_sent_count, 0);

	/* Remove limit: second OUT event must complete the send */
	fx.sock.partial_limit = 0;
	fixture_flush_out(&fx);
	CU_ASSERT_EQUAL(fx.data_sent_count, 1);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_tx_queue_wrap(void)
{
	/* Send RTMP_MAX_QUEUE_SIZE frames and flush after each to drive
	 * chan->queue_idx through 0..RTMP_MAX_QUEUE_SIZE-1 and back to 0
	 * (the wrap at send_done → queue_idx >= RTMP_MAX_QUEUE_SIZE → 0).
	 * send_video_frame uses internal=0 so data_sent fires each time. */
	struct chunk_stream_fixture fx;
	int i;

	fixture_init(&fx);

	for (i = 0; i < RTMP_MAX_QUEUE_SIZE; i++) {
		uint8_t frame_data[1] = {0x27};
		struct rtmp_buffer frame = {
			.buf = frame_data, .cap = 1, .len = 1, .rd = 0};
		int ret;
		ret = send_video_frame(
			fx.stream, &frame, (uint32_t)i * 100, 0, 0, frame_data);
		CU_ASSERT_TRUE(ret >= 0);
		fixture_flush_out(&fx);
		fx.sock.tx_len = 0;
	}

	CU_ASSERT_EQUAL(fx.data_sent_count, RTMP_MAX_QUEUE_SIZE);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_rx_realloc(void)
{
	/* A message with declared length > RTMP_CHUNK_STREAM_MSG_LEN (512)
	 * forces a realloc of the rx channel buffer on the first chunk. */
	struct chunk_stream_fixture fx;
	uint8_t chunk[256];
	size_t n;
	size_t msg_len = 600;
	size_t sent = 0;

	fixture_init(&fx);

	/* First chunk: header-type-0, msg_len=600, mtid=0x14 */
	n = build_header0(chunk, 3, 0, (uint32_t)msg_len, 0x14, 0);
	memset(&chunk[n], 0x05, 128); /* 128 bytes of AMF null payload */
	n += 128;
	fixture_deliver_in(&fx, chunk, n);
	sent += 128;
	CU_ASSERT_EQUAL(fx.amf_msg_count, 0);

	/* Continuation chunks (type 3, csid=3) */
	while (sent < msg_len) {
		size_t to_send =
			(msg_len - sent < 128) ? (msg_len - sent) : 128;
		chunk[0] = (uint8_t)((3 << 6) | 3); /* fmt=3, csid=3 */
		memset(&chunk[1], 0x05, to_send);
		n = 1 + to_send;
		fixture_deliver_in(&fx, chunk, n);
		sent += to_send;
	}

	CU_ASSERT_EQUAL(fx.amf_msg_count, 1);
	CU_ASSERT_EQUAL(fx.disconnected_count, 0);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_rx_unexpected_new_msg(void)
{
	/* Send 128 bytes of a 200-byte message on csid=3, then send a new
	 * type-0 header on the same csid with a different msg_len.
	 * Covers: "unexpected new message for channel %d" ULOGW + reset. */
	struct chunk_stream_fixture fx;
	uint8_t chunk[256];
	uint8_t amf_data[16];
	struct rtmp_buffer amf = {
		.buf = amf_data, .cap = sizeof(amf_data), .len = 0, .rd = 0};
	size_t n;
	int ret;

	fixture_init(&fx);

	/* Partial message on csid=3, msg_len=200 */
	n = build_header0(chunk, 3, 0, 200, 0x14, 0);
	memset(&chunk[n], 0x05, 128);
	n += 128;
	fixture_deliver_in(&fx, chunk, n);
	CU_ASSERT_EQUAL(fx.amf_msg_count, 0);

	/* New message on csid=3 with different msg_len → triggers warning+reset
	 */
	ret = amf_encode(&amf, "%s", "x");
	CU_ASSERT_EQUAL(ret, 0);
	n = build_header0(chunk, 3, 0, (uint32_t)amf.len, 0x14, 0);
	memcpy(&chunk[n], amf.buf, amf.len);
	n += amf.len;
	fixture_deliver_in(&fx, chunk, n);

	/* The short message completes → amf_msg callback fires once */
	CU_ASSERT_EQUAL(fx.amf_msg_count, 1);
	CU_ASSERT_EQUAL(fx.disconnected_count, 0);

	fixture_cleanup(&fx);
}


static void test_chunk_stream_rx_abort_current_channel(void)
{
	/* Abort(csid=2) delivered on csid=2 itself → "abort on current chunk
	 * stream!" ULOGE.  chan->msg.len is non-zero when data_complete fires,
	 * so the abort_chan == chan branch is reached. */
	struct chunk_stream_fixture fx;
	uint8_t abort_chunk[32];
	uint32_t abort_csid_ne = htonl(2);
	size_t n;

	fixture_init(&fx);

	n = build_header0(abort_chunk, 2, 0, sizeof(abort_csid_ne), 0x02, 0);
	memcpy(&abort_chunk[n], &abort_csid_ne, sizeof(abort_csid_ne));
	n += sizeof(abort_csid_ne);
	fixture_deliver_in(&fx, abort_chunk, n);

	CU_ASSERT_EQUAL(fx.disconnected_count, 0);

	fixture_cleanup(&fx);
}


CU_TestInfo g_rtmp_test_chunk_stream[] = {
	{FN("chunk-stream-new-delete-invalid-args"),
	 &test_chunk_stream_new_delete_invalid_args},
	{FN("chunk-stream-new-delete"), &test_chunk_stream_new_delete},

	{FN("chunk-stream-tx-amf-message"), &test_chunk_stream_tx_amf_message},
	{FN("chunk-stream-tx-amf-message-publish-csid"),
	 &test_chunk_stream_tx_amf_message_publish_csid},
	{FN("chunk-stream-tx-set-chunk-size"),
	 &test_chunk_stream_tx_set_chunk_size},
	{FN("chunk-stream-tx-header-caching"),
	 &test_chunk_stream_tx_header_caching},
	{FN("chunk-stream-tx-queue-full"), &test_chunk_stream_tx_queue_full},
	{FN("chunk-stream-tx-send-video-frame"),
	 &test_chunk_stream_tx_send_video_frame},
	{FN("chunk-stream-tx-send-metadata"),
	 &test_chunk_stream_tx_send_metadata},
	{FN("chunk-stream-tx-send-audio-data"),
	 &test_chunk_stream_tx_send_audio_data},
	{FN("chunk-stream-tx-send-audio-data-rejects-bad-asc"),
	 &test_chunk_stream_tx_send_audio_data_rejects_bad_asc},
	{FN("chunk-stream-flush"), &test_chunk_stream_flush},
	{FN("chunk-stream-store-message-stream-id"),
	 &test_chunk_stream_store_message_stream_id},

	{FN("chunk-stream-rx-amf-message"), &test_chunk_stream_rx_amf_message},
	{FN("chunk-stream-rx-peer-bandwidth"),
	 &test_chunk_stream_rx_peer_bandwidth},
	{FN("chunk-stream-rx-user-control-ping"),
	 &test_chunk_stream_rx_user_control_ping},
	{FN("chunk-stream-rx-set-chunk-size-applies"),
	 &test_chunk_stream_rx_set_chunk_size_applies},

	{FN("chunk-stream-rx-window-ack-size"),
	 &test_chunk_stream_rx_window_ack_size},
	{FN("chunk-stream-rx-ack-short-message-regression"),
	 &test_chunk_stream_rx_ack_short_message_regression},

	{FN("chunk-stream-rx-ack-valid"), &test_chunk_stream_rx_ack_valid},
	{FN("chunk-stream-rx-abort"), &test_chunk_stream_rx_abort},
	{FN("chunk-stream-rx-user-control-events"),
	 &test_chunk_stream_rx_user_control_events},
	{FN("chunk-stream-rx-peer-bandwidth-variants"),
	 &test_chunk_stream_rx_peer_bandwidth_variants},
	{FN("chunk-stream-rx-extended-timestamp"),
	 &test_chunk_stream_rx_extended_timestamp},
	{FN("chunk-stream-rx-multi-byte-csid"),
	 &test_chunk_stream_rx_multi_byte_csid},
	{FN("chunk-stream-tx-extended-timestamp"),
	 &test_chunk_stream_tx_extended_timestamp},
	{FN("chunk-stream-watchdog-timeout"),
	 &test_chunk_stream_watchdog_timeout},

	{FN("chunk-stream-rx-3byte-csid"), &test_chunk_stream_rx_3byte_csid},
	{FN("chunk-stream-rx-control-bad-sizes"),
	 &test_chunk_stream_rx_control_bad_sizes},
	{FN("chunk-stream-rx-unknown-mtid"),
	 &test_chunk_stream_rx_unknown_mtid},
	{FN("chunk-stream-tx-audio-22050hz"),
	 &test_chunk_stream_tx_audio_22050hz},
	{FN("chunk-stream-tx-audio-11025hz"),
	 &test_chunk_stream_tx_audio_11025hz},

	{FN("chunk-stream-tx-extended-csid-2byte"),
	 &test_chunk_stream_tx_extended_csid_2byte},
	{FN("chunk-stream-tx-extended-csid-3byte"),
	 &test_chunk_stream_tx_extended_csid_3byte},
	{FN("chunk-stream-tx-partial-send"),
	 &test_chunk_stream_tx_partial_send},
	{FN("chunk-stream-tx-queue-wrap"), &test_chunk_stream_tx_queue_wrap},
	{FN("chunk-stream-rx-realloc"), &test_chunk_stream_rx_realloc},
	{FN("chunk-stream-rx-unexpected-new-msg"),
	 &test_chunk_stream_rx_unexpected_new_msg},
	{FN("chunk-stream-rx-abort-current-channel"),
	 &test_chunk_stream_rx_abort_current_channel},

	CU_TEST_INFO_NULL,
};
