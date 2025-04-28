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
#include <rtmp.h>

#include "amf.h"
#include "rtmp_chunk_stream.h"

#include <arpa/inet.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <aac/aac.h>
#include <audio-defs/adefs.h>
#include <futils/list.h>
#include <libpomp.h>

#define ULOG_TAG rtmp_chunk_stream
#include <ulog.h>
ULOG_DECLARE_TAG(rtmp_chunk_stream);

#define RTMP_CHUNK_STREAM_MSG_LEN 512
#define RTMP_CHUNK_HEADER_MAX_LEN 18


enum bw_type {
	BW_TYPE_HARD = 0,
	BW_TYPE_SOFT,
	BW_TYPE_DYNAMIC,
	BW_TYPE_UNKNOWN,
};


struct tx_buffer {
	struct rtmp_buffer data_header;
	struct rtmp_buffer data;
	void *frame_userdata;

	uint8_t mtid;
	uint32_t msid;
	uint32_t timestamp;

	int internal;
	uint32_t next_chunk_size;
};


struct rtmp_chunk_tx_chan {
	int csid;
	struct list_node node;

	uint8_t prev_mtid;
	uint32_t prev_msid;
	size_t prev_len;
	uint32_t prev_delta;
	uint32_t prev_timestamp;

	int first;

	struct tx_buffer queue[RTMP_MAX_QUEUE_SIZE];
	int queue_idx;
	int queue_len;
	size_t chunk_partial_len;
	struct rtmp_buffer header;
};


struct rtmp_chunk_rx_chan {
	int csid;
	struct list_node node;

	uint8_t mtid;
	uint32_t msid;
	size_t len;
	uint32_t timestamp;
	uint32_t delta;

	struct rtmp_buffer msg;
};


struct rtmp_chunk_stream {
	/* From constructor */
	struct pomp_loop *loop;
	struct pomp_timer *watchdog_timer;
	struct tskt_socket *tsock;
	struct rtmp_chunk_cbs cbs;
	void *userdata;

	/* Runtime */
	struct list_node rx_channels;
	uint32_t rx_chunk_size;

	struct list_node tx_channels;
	uint32_t tx_chunk_size;

	int tx_chan_in_progess;
	int prev_csid;

	uint32_t window_ack_size;
	uint32_t total_bytes;
	uint32_t rcv_bytes_since_last_ack;

	uint32_t bw;
	enum bw_type bw_type;

	/* recv buffer, should be big enough for one chunk */
	struct rtmp_buffer rcvbuf;

	int pomp_watch_write;

	/* audio */
	bool audio_setup;
	uint8_t audio_setting;

	/* message stream id
	 * (See 5.3.1.2.5. Common Header Fields in Adobe’s Real Time Messaging
	 *  Protocol) */
	uint32_t published_msid;
};


static int send_data(struct rtmp_chunk_stream *stream,
		     int csid,
		     uint8_t mtid,
		     uint32_t msid,
		     uint32_t timestamp,
		     struct rtmp_buffer *data_header,
		     struct rtmp_buffer *data,
		     void *frame_userdata,
		     int internal,
		     int32_t next_chunk_size);


static int send_ack(struct rtmp_chunk_stream *stream);


static int clone_buffer(struct rtmp_buffer *src, struct rtmp_buffer *dst)
{
	if (!src || !dst)
		return -EINVAL;

	dst->cap = src->len - src->rd;
	dst->len = dst->cap;
	dst->rd = 0;
	dst->buf = malloc(dst->cap);
	if (!dst->buf)
		return -ENOMEM;
	memcpy(dst->buf, &src->buf[src->rd], dst->len);
	return 0;
}


static int clone_data(void *src, size_t len, struct rtmp_buffer *dst)
{
	if (!src || !dst)
		return -EINVAL;

	dst->cap = len;
	dst->len = len;
	dst->rd = 0;
	dst->buf = malloc(dst->cap);
	if (!dst->buf)
		return -ENOMEM;
	memcpy(dst->buf, src, dst->len);
	return 0;
}


static struct rtmp_chunk_tx_chan *new_chunk_tx_chan(int csid)
{
	struct rtmp_chunk_tx_chan *chan;

	chan = calloc(1, sizeof(*chan));
	if (!chan) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}

	chan->csid = csid;

	list_init(&chan->node);

	chan->first = 1;

	chan->header.buf = malloc(RTMP_CHUNK_HEADER_MAX_LEN);
	if (!chan->header.buf) {
		ULOG_ERRNO("malloc", ENOMEM);
		free(chan);
		return NULL;
	}
	chan->header.cap = RTMP_CHUNK_HEADER_MAX_LEN;

	return chan;
}


static void flush_chunk_tx_chan(struct rtmp_chunk_stream *stream,
				struct rtmp_chunk_tx_chan *chan)
{
	int i;
	if (!chan)
		return;

	/* Unref all waiting buffers */
	for (i = 0; i < chan->queue_len; i++) {
		int idx = (chan->queue_idx + i) % RTMP_MAX_QUEUE_SIZE;
		if (chan->queue[idx].internal)
			free(chan->queue[idx].data.buf);
		else
			stream->cbs.data_sent(chan->queue[idx].data.buf,
					      chan->queue[idx].frame_userdata,
					      stream->userdata);
		chan->queue[idx].data.buf = NULL;
		chan->queue[idx].frame_userdata = NULL;
		if (chan->queue[idx].data_header.cap) {
			free(chan->queue[idx].data_header.buf);
			chan->queue[idx].data_header.buf = NULL;
		}
	}
	chan->queue_idx = 0;
	chan->queue_len = 0;
	if (chan->csid == stream->tx_chan_in_progess)
		stream->tx_chan_in_progess = 0;
}


static void delete_chunk_tx_chan(struct rtmp_chunk_stream *stream,
				 struct rtmp_chunk_tx_chan *chan)
{
	if (!chan)
		return;

	/* Unref all waiting buffers */
	flush_chunk_tx_chan(stream, chan);

	free(chan->header.buf);
	free(chan);
}


static struct rtmp_chunk_rx_chan *new_chunk_rx_chan(int csid)
{
	struct rtmp_chunk_rx_chan *chan;

	chan = calloc(1, sizeof(*chan));
	if (!chan) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}

	chan->csid = csid;
	list_init(&chan->node);

	chan->msg.buf = malloc(RTMP_CHUNK_STREAM_MSG_LEN);
	if (!chan->msg.buf) {
		free(chan);
		ULOG_ERRNO("malloc", ENOMEM);
		return NULL;
	}
	chan->msg.cap = RTMP_CHUNK_STREAM_MSG_LEN;

	return chan;
}


static void delete_chunk_rx_chan(struct rtmp_chunk_rx_chan *chan)
{
	if (!chan)
		return;

	free(chan->msg.buf);
	free(chan);
}


static int update_pomp_event(struct rtmp_chunk_stream *stream)
{
	int need_out = 0;
	struct rtmp_chunk_tx_chan *chan;

	list_walk_entry_forward(&stream->tx_channels, chan, node)
	{
		if (chan->queue_len > 0) {
			need_out = 1;
			break;
		}
	}

	if (need_out == stream->pomp_watch_write)
		return 0;

	stream->pomp_watch_write = need_out;

	return tskt_socket_update_events(stream->tsock,
					 need_out ? POMP_FD_EVENT_OUT : 0,
					 need_out ? 0 : POMP_FD_EVENT_OUT);
}


static void notify_disconnection(struct rtmp_chunk_stream *stream,
				 enum rtmp_client_disconnection_reason reason)
{
	/* First remove FD_EVENT_OUT from the pomp loop */
	(void)tskt_socket_update_events(stream->tsock, 0, POMP_FD_EVENT_OUT);

	stream->pomp_watch_write = 0;
	/* Then, call the disconnect callback */
	stream->cbs.disconnected(stream->userdata, reason);
}


static int fill_header_buffer(struct rtmp_chunk_tx_chan *chan,
			      uint8_t mtid,
			      uint32_t msid,
			      int *prev_csid,
			      size_t len,
			      uint32_t timestamp,
			      struct rtmp_buffer *header)
{
	int header_type;
	uint8_t basic_header[3];
	size_t bh_len;
	int need_extended_ts;
	int32_t timestamp_delta;
	uint8_t *b;
	uint32_t embedded_ts;

	if (!chan || !header || !prev_csid)
		return -EINVAL;

	if (header->cap < RTMP_CHUNK_HEADER_MAX_LEN || header->len > 0)
		return -EINVAL;

	timestamp_delta = timestamp - chan->prev_timestamp;
	if (timestamp_delta < 0 || chan->first || (chan->prev_mtid != mtid)) {
		/* Full header forced if we are going backward */
		header_type = 0;
	} else if ((chan->prev_mtid == mtid) && (chan->prev_msid == msid) &&
		   (chan->prev_len == len) &&
		   (chan->prev_delta == (uint32_t)timestamp_delta)) {
		/* No Header */
		header_type = 3;
	} else if ((chan->prev_mtid == mtid) && (chan->prev_msid == msid) &&
		   (chan->prev_len == len) && (timestamp == 0)) {
		/* Timestamp delta only */
		header_type = 2;
	} else if ((chan->prev_msid == msid) && (timestamp == 0)) {
		/* Everything but message stream id */
		header_type = 1;
	} else {
		/* Everything */
		header_type = 0;
	}

	if (chan->csid < 2) {
		return -EINVAL;
	} else if (chan->csid < 64) {
		basic_header[0] = (header_type << 6) | (chan->csid & 0x3f);
		bh_len = 1;
	} else if (chan->csid < 320) {
		basic_header[0] = (header_type << 6);
		basic_header[1] = (chan->csid - 64);
		bh_len = 2;
	} else if (chan->csid < 65599) {
		int offset_chan_id = chan->csid - 64;
		uint8_t low = (offset_chan_id & 0xff);
		uint8_t high = (offset_chan_id >> 8);
		basic_header[0] = (header_type << 6) | 0x01;
		basic_header[1] = low;
		basic_header[2] = high;
		bh_len = 3;
	} else {
		return -EINVAL;
	}

	memcpy(header->buf, basic_header, bh_len);
	header->len += bh_len;

	/* Header type 0 embeds an absolute timestamp, other headers embed a
	 * delta */
	embedded_ts = header_type != 0 ? (uint32_t)timestamp_delta : timestamp;
	/* For header 0, set prev_delta as 0 */
	if (header_type == 0)
		timestamp_delta = 0;

	/* If embedded timestamp is more than 3 bytes long, we need an extended
	 * timestamp field */
	need_extended_ts = (embedded_ts > 0xffffff);

	b = &header->buf[header->len];
	if (header_type < 3) {
		/* Timestamp included for header types 0, 1 & 2 */
		if (need_extended_ts) {
			b[0] = 0xff;
			b[1] = 0xff;
			b[2] = 0xff;
		} else {
			b[0] = embedded_ts >> 16 & 0xff;
			b[1] = embedded_ts >> 8 & 0xff;
			b[2] = embedded_ts & 0xff;
		}
		header->len += 3;
	}

	b = &header->buf[header->len];
	if (header_type < 2) {
		/* Length & media type id included for header types 0 & 1 */
		b[0] = len >> 16 & 0xff;
		b[1] = len >> 8 & 0xff;
		b[2] = len & 0xff;
		b[3] = mtid;
		header->len += 4;
	}

	b = &header->buf[header->len];
	if (header_type == 0) {
		/* Media stream id included only for header type 0 */
		memcpy(b, &msid, sizeof(msid));
		header->len += sizeof(msid);
	}

	b = &header->buf[header->len];
	if (need_extended_ts && header_type != 3) {
		/* Only include an extended timestamp value if its required
		 * (timestamp > 0xffffff) & the header type actually has a
		 * timestamp field (type != 3) */
		uint32_t ets_ne = htonl(embedded_ts);
		memcpy(b, &ets_ne, sizeof(ets_ne));
		header->len += sizeof(ets_ne);
	}

	/* Save values to channel */
	chan->prev_len = len;
	chan->prev_mtid = mtid;
	chan->prev_msid = msid;
	chan->prev_timestamp = timestamp;
	chan->prev_delta = (uint32_t)timestamp_delta;
	chan->first = 0;
	*prev_csid = chan->csid;

	return 0;
}


static struct rtmp_chunk_tx_chan *
get_tx_channel(struct rtmp_chunk_stream *stream, int csid)
{
	struct rtmp_chunk_tx_chan *chan;
	int found = 0;

	if (!stream)
		return NULL;

	list_walk_entry_forward(&stream->tx_channels, chan, node)
	{
		if (chan->csid == csid) {
			found = 1;
			break;
		}
	}

	if (!found) {
		chan = new_chunk_tx_chan(csid);
		list_add_before(&stream->tx_channels, &chan->node);
	}

	return chan;
}


static struct rtmp_chunk_rx_chan *
get_rx_channel(struct rtmp_chunk_stream *stream, int csid)
{
	struct rtmp_chunk_rx_chan *chan;
	int found = 0;

	if (!stream)
		return NULL;

	list_walk_entry_forward(&stream->rx_channels, chan, node)
	{
		if (chan->csid == csid) {
			found = 1;
			break;
		}
	}

	if (!found) {
		chan = new_chunk_rx_chan(csid);
		list_add_before(&stream->rx_channels, &chan->node);
	}

	return chan;
}


static int send_ack_if_needed(struct rtmp_chunk_stream *stream)
{
	int ret;

	if (!stream)
		return -EINVAL;
	if ((!stream->window_ack_size) ||
	    (stream->rcv_bytes_since_last_ack < (stream->window_ack_size / 2)))
		return 0;

	ret = send_ack(stream);
	if (ret == 0)
		stream->rcv_bytes_since_last_ack = 0;

	return ret;
}


static int set_rx_chunk_size(struct rtmp_chunk_stream *stream,
			     uint32_t chunk_size)
{
	size_t buf_size;

	if (!stream)
		return -EINVAL;

	buf_size = chunk_size + RTMP_CHUNK_HEADER_MAX_LEN;

	if (buf_size > stream->rcvbuf.cap) {
		void *tmp = realloc(stream->rcvbuf.buf, buf_size);
		if (!tmp)
			return -ENOMEM;
		stream->rcvbuf.buf = tmp;
		stream->rcvbuf.cap = buf_size;
	}

	stream->rx_chunk_size = chunk_size;

	ULOGI("rx chunk size set to %" PRIu32 " bytes", stream->rx_chunk_size);
	return 0;
}


static int set_window_ack_size(struct rtmp_chunk_stream *stream,
			       uint32_t window_ack_size)
{
	if (!stream)
		return -EINVAL;

	stream->window_ack_size = window_ack_size;

	ULOGI("window ack size set to %" PRIu32 " bytes",
	      stream->window_ack_size);

	return send_ack_if_needed(stream);
}


static int data_complete(struct rtmp_chunk_stream *stream,
			 struct rtmp_chunk_rx_chan *chan)
{
	int ret = 0;
	uint32_t data_ne;
	uint32_t chunk_size;
	uint32_t abort_csid;
	uint32_t window_size;
	uint32_t rcv_bytes_since_last_ack;
	uint32_t bw;
	uint8_t type;
	uint32_t stream_id;
	uint32_t buff_len;
	enum bw_type bw_type;
	struct rtmp_chunk_rx_chan *abort_chan;
	struct rtmp_buffer buf;

	if (!stream || !chan)
		return -EINVAL;

	switch (chan->mtid) {
	case 0x01: /* Set Chunk size */
		if (chan->msg.len != sizeof(data_ne)) {
			ULOGW("bad SetChunkSize size (%zu instead of %zu)",
			      chan->msg.len,
			      sizeof(data_ne));
			ret = -EBADMSG;
			break;
		}
		memcpy(&data_ne, chan->msg.buf, sizeof(data_ne));
		chunk_size = ntohl(data_ne);

		if (chunk_size != stream->rx_chunk_size)
			ret = set_rx_chunk_size(stream, chunk_size);
		break;

	case 0x02: /* Abort message */
		if (chan->msg.len != sizeof(data_ne)) {
			ULOGW("bad Abort size (%zu instead of %zu)",
			      chan->msg.len,
			      sizeof(data_ne));
			ret = -EBADMSG;
			break;
		}
		memcpy(&data_ne, chan->msg.buf, sizeof(data_ne));
		abort_csid = ntohl(data_ne);

		list_walk_entry_forward(&stream->rx_channels, abort_chan, node)
		{
			if (abort_chan->csid != (int)abort_csid)
				continue;
			if (abort_chan->msg.len == 0)
				continue;
			if (abort_chan == chan) {
				ULOGE("abort on current chunk stream !");
				continue;
			}
			ULOGI("abort on chunk stream %d", abort_chan->csid);
			chan->msg.len = 0;
		}
		break;

	case 0x03: /* Ack */
		memcpy(&data_ne, chan->msg.buf, sizeof(data_ne));
		rcv_bytes_since_last_ack = ntohl(data_ne);
		ULOGD("ack: %u (server), %u (client)",
		      rcv_bytes_since_last_ack,
		      stream->rcv_bytes_since_last_ack);
		break;

	case 0x04: /* User control message */
		if (chan->msg.len != sizeof(data_ne) + 2) {
			ULOGW("bad user control size: (%zu instead of %zu)",
			      chan->msg.len,
			      sizeof(data_ne) + 2);
			ret = -EBADMSG;
			break;
		}

		/* See 7.1.7.User Control Message Events in rtmp specification
		 */
		memcpy(&data_ne, chan->msg.buf + 2, sizeof(data_ne));
		type = ntohs(chan->msg.buf[0]);
		stream_id = ntohl(data_ne);

		switch (type) {
		case 0:
			ULOGI("stream Begin (ID: %u)", stream_id);
			break;
		case 1:
			ULOGI("stream EOF (ID: %u)", stream_id);
			break;
		case 2:
			ULOGI("stream Dry (ID: %u)", stream_id);
			break;
		case 3:
			memcpy(&data_ne, chan->msg.buf + 4, sizeof(data_ne));
			buff_len = ntohs(data_ne);
			ULOGI("setBuffer Length: %ums (ID: %u)",
			      buff_len,
			      stream_id);
			break;
		case 4:
			ULOGI("streamIs Recorded (ID: %u)", stream_id);
			break;
		case 6:
			ULOGI("pingRequest (ID: %u)", stream_id);
			buf.cap = 6;
			buf.buf = calloc(buf.cap, 1);
			if (!buf.buf)
				return -ENOMEM;
			buf.rd = 0;
			buf.len = buf.cap;

			buf.buf[0] = 0x07;
			data_ne = htonl(stream_id);
			memcpy(&buf.buf[1], &data_ne, sizeof(data_ne));

			ret = send_data(stream,
					2,
					0x04,
					stream->published_msid,
					0,
					NULL,
					&buf,
					NULL,
					1,
					0);
			if (ret < 0)
				free(buf.buf);
			break;
		default:
			ULOGW("unknown user control message %u (ID: %u)",
			      type,
			      stream_id);
			break;
		};
		break;

	case 0x05: /* Window ack size */
		if (chan->msg.len != sizeof(data_ne)) {
			ULOGW("bad WindowAckSize size (%zu instead of %zu)",
			      chan->msg.len,
			      sizeof(data_ne));
			ret = -EBADMSG;
			break;
		}
		memcpy(&data_ne, chan->msg.buf, sizeof(data_ne));
		window_size = ntohl(data_ne);

		ret = set_window_ack_size(stream, window_size);
		break;

	case 0x06: /* Peer bandwidth */
		if (chan->msg.len != sizeof(data_ne) + 1) {
			ULOGW("bad WindowAckSize size (%zu instead of %zu)",
			      chan->msg.len,
			      sizeof(data_ne) + 1);
			ret = -EBADMSG;
			break;
		}
		memcpy(&data_ne, chan->msg.buf, sizeof(data_ne));
		bw = ntohl(data_ne);
		bw_type = (enum bw_type)chan->msg.buf[sizeof(data_ne)];
		/* Save as hard limit if:
		 * - It is the first BW message, or
		 * - It is an hard BW message, or
		 * - It is a dynamic BW message, and the previous limit was hard
		 * Save as soft limit if:
		 * - It is a soft BW message, and the bw is smaller than the
		 * saved bw Ignore if:
		 * - It is a soft BW message, with higher bw than the saved bw
		 * - It is a dynamic BW message, and the saved type is not Hard.
		 * Otherwise, report a EBADMSG error
		 */
		if (stream->bw_type == BW_TYPE_UNKNOWN ||
		    bw_type == BW_TYPE_HARD ||
		    (stream->bw_type == BW_TYPE_HARD &&
		     bw_type == BW_TYPE_DYNAMIC)) {
			stream->bw_type = BW_TYPE_HARD;
			stream->bw = bw;
		} else if (bw_type == BW_TYPE_SOFT && bw < stream->bw) {
			stream->bw = bw;
			stream->bw_type = bw_type;
		} else if (bw_type == BW_TYPE_SOFT ||
			   (bw_type == BW_TYPE_DYNAMIC &&
			    stream->bw_type != BW_TYPE_HARD)) {
			/* Ignore the message */
		} else {
			ret = -EBADMSG;
			break;
		}

		stream->cbs.peer_bw_changed(stream->bw, stream->userdata);
		break;

	case 0x14: /* AMF0 message */
		stream->cbs.amf_msg(&chan->msg, stream->userdata);
		break;

	default:
		ULOGW("unknown mtid: %u", chan->mtid);
		break;
	}

	return ret;
}


/* Check whether _x bytes of data (or more) are available in _d. If that's not
 * the case, return 0 bytes consumed */
#define CHECK_DLEN(_d, _x)                                                     \
	do {                                                                   \
		if ((_d->rd + _x) > _d->len)                                   \
			return 0;                                              \
	} while (0)


static ssize_t stream_consume_rcv_data(struct rtmp_chunk_stream *stream,
				       struct rtmp_buffer *data)
{
	uint8_t d;
	int header_type;
	int csid;
	size_t header_len;
	struct rtmp_chunk_rx_chan *chan;

	int has_extended_ts = 0;
	int ts_ok;

	size_t missing_len;
	size_t chunk_len;

	size_t total_len;

	/* message header fields */
	uint32_t timestamp;
	size_t msg_len;
	uint8_t mtid;
	uint32_t msid;
	int isdelta;

	static const size_t header_len_table[4] = {
		[0] = 11,
		[1] = 7,
		[2] = 3,
		[3] = 0,
	};

	if (!stream || !data)
		return -EINVAL;

	CHECK_DLEN(data, 1);
	d = data->buf[data->rd++];
	csid = d & 0x3f;
	header_type = d >> 6;
	if (header_type < 0 || header_type > 3)
		return -EINVAL;

	total_len = 1;
	if (csid == 0) {
		/* basic header on 2 bytes */
		CHECK_DLEN(data, 1);
		csid = data->buf[data->rd++] + 64;
		total_len = 2;
	} else if (csid == 1) {
		/* basic header on 3 bytes */
		CHECK_DLEN(data, 2);
		csid = data->buf[data->rd++] + 64;
		csid += (256 * data->buf[data->rd++]);
		total_len = 3;
	}

	chan = get_rx_channel(stream, csid);
	if (!chan)
		return -ENOMEM;

	isdelta = (header_type != 0);
	header_len = header_len_table[header_type];
	CHECK_DLEN(data, header_len);

	total_len += header_len;

	if (header_type < 3) {
		/* Timestamp present in headers 0, 1 and 2 */
		timestamp = data->buf[data->rd++] << 16;
		timestamp += data->buf[data->rd++] << 8;
		timestamp += data->buf[data->rd++];
		has_extended_ts = (timestamp == 0xffffff);
	} else {
		timestamp = chan->delta;
	}
	if (header_type < 2) {
		/* msg_len & mtid present in headers 0 and 1 */
		msg_len = data->buf[data->rd++] << 16;
		msg_len += data->buf[data->rd++] << 8;
		msg_len += data->buf[data->rd++];
		if (msg_len > 0x00FFFFFF)
			return -EINVAL;
		mtid = data->buf[data->rd++];
	} else {
		msg_len = chan->len;
		mtid = chan->mtid;
	}
	if (header_type < 1) {
		/* msid only present in header 0 */
		memcpy(&msid, &data->buf[data->rd], sizeof(msid));
		data->rd += sizeof(msid);
	} else {
		msid = chan->msid;
	}

	if (has_extended_ts) {
		uint32_t ts_ne;
		CHECK_DLEN(data, sizeof(ts_ne));
		memcpy(&ts_ne, &data->buf[data->rd], sizeof(ts_ne));
		data->rd += sizeof(ts_ne);
		timestamp = ntohl(ts_ne);
		total_len += sizeof(ts_ne);
	}

	ts_ok = isdelta ? (timestamp == chan->delta)
			: (timestamp == chan->timestamp);

	/* check data validity: if any header data changed, it must be a
	 * new message */
	if (((chan->len != msg_len) || (chan->msid != msid) ||
	     (chan->mtid != mtid) || (!ts_ok)) &&
	    chan->msg.len > 0) {
		ULOGW("unexpected new message for channel %d", csid);
		chan->msg.len = 0;
	}

	/* Check if we have a full chunk (or part) */
	missing_len = msg_len - chan->msg.len;
	chunk_len = stream->rx_chunk_size > missing_len ? missing_len
							: stream->rx_chunk_size;

	CHECK_DLEN(data, chunk_len);
	total_len += chunk_len;

	/* save data in channel */
	chan->mtid = mtid;
	chan->msid = msid;
	chan->len = msg_len;
	if (isdelta) {
		chan->delta = timestamp;
		/* Increment timestamp only on new message */
		if (chan->msg.len == 0)
			chan->timestamp += timestamp;
	} else {
		chan->timestamp = timestamp;
		chan->delta = 0;
	}
	/* Realloc channel buffer if needed */
	if (chan->msg.cap < chan->len) {
		void *tmp = realloc(chan->msg.buf, chan->len);
		if (!tmp)
			return -ENOMEM;
		chan->msg.buf = tmp;
		chan->msg.cap = chan->len;
	}

	/* Copy data into chan->msg */
	memcpy(&chan->msg.buf[chan->msg.len], &data->buf[data->rd], chunk_len);
	chan->msg.len += chunk_len;

	/* Check if message is complete */
	if (chan->msg.len == chan->len) {
		int ret = data_complete(stream, chan);
		if (ret < 0)
			ULOG_ERRNO("data_complete", -ret);
		chan->msg.rd = 0;
		chan->msg.len = 0;
	}

	return total_len;
}


static void event_data_in(struct rtmp_chunk_stream *stream)
{
	size_t avail;
	ssize_t slen;
	ssize_t consumed;

	if (!stream) {
		ULOG_ERRNO("event_data_in", EINVAL);
		return;
	}

	avail = stream->rcvbuf.cap - stream->rcvbuf.len;

	if (avail == 0) {
		ULOGE("buffer full ... this is bad");
		exit(1);
		return;
	}

	slen = tskt_socket_read(stream->tsock,
				&stream->rcvbuf.buf[stream->rcvbuf.len],
				avail,
				NULL);

	if (slen < 0) {
		int err = -errno;

		if (errno == EAGAIN)
			return;

		ULOG_ERRNO("tskt_socket_read", errno);
		if (err == -ECONNRESET)
			notify_disconnection(
				stream,
				RTMP_CLIENT_DISCONNECTION_REASON_NETWORK_ERROR);
		return;
	}
	stream->rcvbuf.len += slen;
	stream->rcv_bytes_since_last_ack += slen;
	stream->total_bytes += slen;
	send_ack_if_needed(stream);

	do {
		consumed = stream_consume_rcv_data(stream, &stream->rcvbuf);
		if (consumed < 0) {
			ULOG_ERRNO("consume_rcv_data", -(int)consumed);
			return;
		}

		if (consumed == 0)
			break;

		if ((size_t)consumed < stream->rcvbuf.len) {
			/* Data remaining, move them to beginning of buffer */
			size_t rem = stream->rcvbuf.len - consumed;
			memmove(stream->rcvbuf.buf,
				&stream->rcvbuf.buf[consumed],
				rem);
			stream->rcvbuf.len = rem;
		} else {
			/* All buffer consumed, just reset len */
			stream->rcvbuf.len = 0;
		}
		/* rd is always reset to 0 */
		stream->rcvbuf.rd = 0;
	} while (1);
	/* rd is always reset to 0 */
	stream->rcvbuf.rd = 0;
}


static int send_chunk(struct rtmp_chunk_stream *stream,
		      struct rtmp_buffer *header,
		      struct rtmp_buffer *data_header,
		      struct rtmp_buffer *data,
		      size_t chunk_size,
		      size_t already_sent)
{
	int flags;
	ssize_t sret;
	size_t full_len;
	int iov_num = 0;
	struct iovec iov[3];
	struct msghdr msg = {
		.msg_name = NULL,
		.msg_namelen = 0,
		.msg_iov = iov,
		.msg_iovlen = 0 /* filled later */,
		.msg_control = NULL,
		.msg_controllen = 0,
		.msg_flags = 0,
	};
	size_t rem_dh_len;
	size_t chunk_dh_len = 0;
	size_t rem_data_len;
	size_t chunk_data_len;
	size_t original_offset = already_sent;
	size_t send_len;

	if (!stream || !header || !data_header || !data)
		return -EINVAL;

	full_len = chunk_size + header->len;

	send_len = 0;
	/* Prepare iov for header */
	if (already_sent < header->len) {
		iov[iov_num].iov_base = &header->buf[already_sent];
		iov[iov_num].iov_len = header->len - already_sent;
		send_len += iov[iov_num].iov_len;
		iov_num++;
		already_sent = 0;
	} else {
		already_sent -= header->len;
	}
	full_len -= header->len;

	/* If no data header, skip to data */
	if (data_header->cap == 0)
		goto data;

	/* Prepare iov for data header */
	rem_dh_len = data_header->len - data_header->rd;
	chunk_dh_len = rem_dh_len < full_len ? rem_dh_len : full_len;
	if (already_sent < chunk_dh_len) {
		iov[iov_num].iov_base =
			&data_header->buf[data_header->rd + already_sent];
		iov[iov_num].iov_len = chunk_dh_len - already_sent;
		send_len += iov[iov_num].iov_len;
		iov_num++;
		already_sent = 0;
	} else {
		already_sent -= chunk_dh_len;
	}
	full_len -= chunk_dh_len;

data:
	/* Prepare iov for data */
	rem_data_len = data->len - data->rd;
	chunk_data_len = rem_data_len < full_len ? rem_data_len : full_len;
	if (already_sent < chunk_data_len) {
		iov[iov_num].iov_base = &data->buf[data->rd + already_sent];
		iov[iov_num].iov_len = chunk_data_len - already_sent;
		send_len += iov[iov_num].iov_len;
		iov_num++;
	}
	/* No need to update already_sent here */

	/* Do the send */
	msg.msg_iovlen = iov_num;
#ifdef MSG_NOSIGNAL
	flags = MSG_NOSIGNAL;
#else
	flags = 0;
#endif
	sret = tskt_socket_writev(stream->tsock, iov, iov_num);
	if (sret < 0) {
		return -errno;
	} else if ((size_t)sret < send_len) {
		/* Partial send, return new send offset */
		return original_offset + sret;
	}

	/* Update the buffers ->rd only on success */
	data_header->rd += chunk_dh_len;
	data->rd += chunk_data_len;

	return 0;
}


static int process_channel_send(struct rtmp_chunk_stream *stream,
				struct rtmp_chunk_tx_chan *chan)
{
	int ret = 0;
	int i;
	int nb_full_chunks;
	struct tx_buffer *buffer;
	size_t dh_rem_len = 0;
	size_t data_rem_len;
	size_t rem_len;
	size_t full_len;

	if (!stream || !chan)
		return -EINVAL;

	buffer = &chan->queue[chan->queue_idx];

	if (chan->chunk_partial_len) {
		/* Finish sending current chunk */
		ret = send_chunk(stream,
				 &chan->header,
				 &buffer->data_header,
				 &buffer->data,
				 stream->tx_chunk_size,
				 chan->chunk_partial_len);

		if (ret < 0) {
			ULOG_ERRNO("send_chunk", -ret);
			goto error;
		} else if (ret > 0) {
			chan->chunk_partial_len = ret;
			return -EAGAIN;
		}
	}
	chan->chunk_partial_len = 0;

	full_len = 0;
	if (buffer->data_header.cap > 0) {
		dh_rem_len = buffer->data_header.len - buffer->data_header.rd;
		full_len += buffer->data_header.len;
	}
	data_rem_len = buffer->data.len - buffer->data.rd;
	full_len += buffer->data.len;

	if (data_rem_len == 0)
		goto send_done;

	rem_len = dh_rem_len + data_rem_len;

	nb_full_chunks = rem_len / stream->tx_chunk_size;

	for (i = 0; i < nb_full_chunks; i++) {
		chan->header.len = 0;
		ret = fill_header_buffer(chan,
					 buffer->mtid,
					 buffer->msid,
					 &stream->prev_csid,
					 full_len,
					 buffer->timestamp,
					 &chan->header);
		if (ret < 0)
			goto error;

		ret = send_chunk(stream,
				 &chan->header,
				 &buffer->data_header,
				 &buffer->data,
				 stream->tx_chunk_size,
				 0);

		if (ret < 0) {
			if (ret != -EAGAIN)
				ULOG_ERRNO("send_chunk", -ret);
			goto error;
		} else if (ret > 0) {
			chan->chunk_partial_len = ret;
			return -EAGAIN;
		}
	}

	rem_len -= stream->tx_chunk_size * nb_full_chunks;
	if (rem_len > 0) {
		chan->header.len = 0;
		ret = fill_header_buffer(chan,
					 buffer->mtid,
					 buffer->msid,
					 &stream->prev_csid,
					 full_len,
					 buffer->timestamp,
					 &chan->header);
		if (ret < 0)
			goto error;

		ret = send_chunk(stream,
				 &chan->header,
				 &buffer->data_header,
				 &buffer->data,
				 rem_len,
				 0);

		if (ret < 0) {
			if (ret != -EAGAIN)
				ULOG_ERRNO("send_chunk", -ret);
			goto error;
		} else if (ret > 0) {
			chan->chunk_partial_len = ret;
			return -EAGAIN;
		}
	}

send_done:

	if (buffer->next_chunk_size > 0) {
		stream->tx_chunk_size = buffer->next_chunk_size;
		ULOGI("tx chunk size set to %" PRIu32 " bytes",
		      stream->tx_chunk_size);
	}
	if (!buffer->internal)
		stream->cbs.data_sent(buffer->data.buf,
				      buffer->frame_userdata,
				      stream->userdata);
	else
		free(buffer->data.buf);
	buffer->data.buf = NULL;
	buffer->data.cap = 0;
	buffer->data.len = 0;
	buffer->frame_userdata = NULL;
	if (buffer->data_header.cap) {
		free(buffer->data_header.buf);
		buffer->data_header.buf = NULL;
		buffer->data_header.cap = 0;
		buffer->data_header.len = 0;
	}
	chan->queue_idx++;
	if (chan->queue_idx >= RTMP_MAX_QUEUE_SIZE)
		chan->queue_idx = 0;
	chan->queue_len--;
	chan->prev_timestamp = buffer->timestamp;

	return 0;

error:
	return ret;
}


static void event_data_out(struct rtmp_chunk_stream *stream)
{
	struct rtmp_chunk_tx_chan *chan;
	int ret;

	if (!stream) {
		ULOG_ERRNO("event_data_out", EINVAL);
		return;
	}

	if (stream->tx_chan_in_progess) {
		int found = 0;
		list_walk_entry_forward(&stream->tx_channels, chan, node)
		{
			if (chan->queue_len == 0)
				continue;
			if (chan->csid != stream->tx_chan_in_progess)
				continue;
			found = 1;
			break;
		}
		if (!found) {
			ULOGE("got a partial chunk sent on an "
			      "unknown channel (%d)",
			      stream->tx_chan_in_progess);
			goto loop;
		}

		ret = process_channel_send(stream, chan);
		if (ret != 0) {
			/* TODO Check ! */
			if (ret != -EAGAIN) {
				ULOG_ERRNO("process_channel_send", -ret);
				notify_disconnection(
					stream,
					/* codecheck_ignore[LONG_LINE] */
					RTMP_CLIENT_DISCONNECTION_REASON_NETWORK_ERROR);
				return;
			}
			return;
		}
		stream->tx_chan_in_progess = 0;
	}

loop:
	list_walk_entry_forward(&stream->tx_channels, chan, node)
	{
		if (chan->queue_len == 0)
			continue;

		ret = process_channel_send(stream, chan);
		if (ret != 0) {
			/* TODO Check ! */
			if (ret != -EAGAIN) {
				ULOG_ERRNO("process_channel_send", -ret);
				notify_disconnection(
					stream,
					/* codecheck_ignore[LONG_LINE] */
					RTMP_CLIENT_DISCONNECTION_REASON_NETWORK_ERROR);
				return;
			} else {
				stream->tx_chan_in_progess = chan->csid;
			}
			return;
		}
	}

	update_pomp_event(stream);
}


static void
tskt_event_cb(struct tskt_socket *sock, uint32_t revents, void *userdata)
{
	int err;
	struct rtmp_chunk_stream *stream = userdata;

	if (revents & POMP_FD_EVENT_IN)
		event_data_in(stream);
	if (revents & POMP_FD_EVENT_OUT)
		event_data_out(stream);

	/* Rearm watchdog */
	err = pomp_timer_set(stream->watchdog_timer,
			     WATCHDOG_TIMER_DURATION_MS);
	if (err < 0)
		ULOG_ERRNO("pomp_timer_set", -err);
}


static void watchdog_timer_cb(struct pomp_timer *timer, void *userdata)
{
	struct rtmp_chunk_stream *stream = userdata;

	ULOGW("%s: no event received on socket for %.2fs, disconnecting",
	      __func__,
	      (float)WATCHDOG_TIMER_DURATION_MS / 1000.);

	notify_disconnection(stream, RTMP_CLIENT_DISCONNECTION_REASON_TIMEOUT);
}


struct rtmp_chunk_stream *new_chunk_stream(struct pomp_loop *loop,
					   struct tskt_socket *tsock,
					   const struct rtmp_chunk_cbs *cbs,
					   void *userdata)
{
	int ret;
	struct rtmp_chunk_stream *stream = NULL;

	if (!loop || !cbs || (tsock == NULL)) {
		ret = -EINVAL;
		goto error;
	}

	if (!cbs->peer_bw_changed || !cbs->amf_msg || !cbs->data_sent ||
	    !cbs->disconnected) {
		ret = -EINVAL;
		goto error;
	}

	stream = calloc(1, sizeof(*stream));
	if (!stream) {
		ret = -ENOMEM;
		goto error;
	}

	stream->loop = loop;
	stream->tsock = tsock;
	stream->cbs = *cbs;
	stream->userdata = userdata;

	stream->watchdog_timer =
		pomp_timer_new(stream->loop, watchdog_timer_cb, stream);
	if (stream->watchdog_timer == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("pomp_timer_new", -ret);
		goto error;
	}

	list_init(&stream->rx_channels);
	list_init(&stream->tx_channels);
	stream->rx_chunk_size = 128;
	stream->tx_chunk_size = 128;

	stream->rcvbuf.buf =
		malloc(stream->rx_chunk_size + RTMP_CHUNK_HEADER_MAX_LEN);
	if (!stream->rcvbuf.buf) {
		ret = -ENOMEM;
		goto error;
	}
	stream->rcvbuf.cap = stream->rx_chunk_size + RTMP_CHUNK_HEADER_MAX_LEN;

	ret = tskt_socket_set_event_cb(
		tsock, POMP_FD_EVENT_IN, tskt_event_cb, stream);
	if (ret < 0)
		goto error;

	return stream;

error:
	ULOG_ERRNO("new_chunk_stream", -ret);
	delete_chunk_stream(stream);
	return NULL;
}


static int send_data(struct rtmp_chunk_stream *stream,
		     int csid,
		     uint8_t mtid,
		     uint32_t msid,
		     uint32_t timestamp,
		     struct rtmp_buffer *data_header,
		     struct rtmp_buffer *data,
		     void *frame_userdata,
		     int internal,
		     int32_t next_chunk_size)
{
	struct rtmp_chunk_tx_chan *chan;
	struct tx_buffer *buffer;
	int idx;
	int ret;

	if (!stream || !data || csid < 0)
		return -EINVAL;

	chan = get_tx_channel(stream, csid);

	if (chan->queue_len >= RTMP_MAX_QUEUE_SIZE)
		return -EAGAIN;

	/* Queue data */
	idx = (chan->queue_idx + chan->queue_len) % RTMP_MAX_QUEUE_SIZE;

	buffer = &chan->queue[idx];
	if (data_header)
		buffer->data_header = *data_header;
	else
		buffer->data_header.cap = 0;
	buffer->data = *data;
	buffer->frame_userdata = frame_userdata;
	buffer->internal = internal;
	buffer->msid = msid;
	buffer->mtid = mtid;
	buffer->timestamp = timestamp;
	buffer->next_chunk_size = next_chunk_size;

	chan->queue_len++;

	ret = update_pomp_event(stream);
	if (ret != 0)
		return ret;

	/* return the number of already waiting frames */
	return chan->queue_len - 1;
}


int set_chunk_size(struct rtmp_chunk_stream *stream, uint32_t chunk_size)
{
	uint32_t chunk_size_ne = htonl(chunk_size);
	struct rtmp_buffer buf;
	int ret;

	if (!stream)
		return -EINVAL;

	if (chunk_size < 1)
		return -EINVAL;

	ret = clone_data(&chunk_size_ne, sizeof(chunk_size_ne), &buf);
	if (ret != 0)
		return ret;

	ret = send_data(stream,
			2,
			0x01,
			stream->published_msid,
			0,
			NULL,
			&buf,
			NULL,
			1,
			chunk_size);
	if (ret < 0)
		free(buf.buf);

	return ret;
}


static int send_abort(struct rtmp_chunk_stream *stream, uint32_t csid)
{
	uint32_t csid_ne = htonl(csid);
	struct rtmp_buffer buf;
	int ret;

	ret = clone_data(&csid_ne, sizeof(csid_ne), &buf);
	if (ret != 0)
		return ret;

	ret = send_data(stream,
			2,
			0x02,
			stream->published_msid,
			0,
			NULL,
			&buf,
			NULL,
			1,
			0);
	if (ret < 0)
		free(buf.buf);

	return ret;
}


static int send_ack(struct rtmp_chunk_stream *stream)
{
	uint32_t seq_ne = htonl(stream->total_bytes);
	struct rtmp_buffer buf;
	int ret;

	ret = clone_data(&seq_ne, sizeof(seq_ne), &buf);
	if (ret != 0)
		return ret;

	ret = send_data(stream,
			2,
			0x03,
			stream->published_msid,
			0,
			NULL,
			&buf,
			NULL,
			1,
			0);
	if (ret < 0)
		free(buf.buf);

	return ret;
}


static int send_window_ack_size(struct rtmp_chunk_stream *stream,
				uint32_t window)
{
	uint32_t win_ne = htonl(window);
	struct rtmp_buffer buf;
	int ret;

	ret = clone_data(&win_ne, sizeof(win_ne), &buf);
	if (ret != 0)
		return ret;
	ret = send_data(stream,
			2,
			0x05,
			stream->published_msid,
			0,
			NULL,
			&buf,
			NULL,
			1,
			0);
	if (ret < 0)
		free(buf.buf);

	return ret;
}


int send_metadata(struct rtmp_chunk_stream *stream,
		  struct rtmp_buffer *data,
		  uint32_t timestamp,
		  int internal,
		  void *frame_userdata)
{
	int ret;
	struct rtmp_buffer b;
	b.cap = 16;
	b.buf = malloc(b.cap);
	if (!b.buf)
		return -ENOMEM;
	b.rd = 0;
	b.len = 0;
	ret = amf_encode(&b, "%s", "@setDataFrame");
	if (ret != 0)
		goto failure;

	ret = send_data(stream,
			4,
			0x12,
			stream->published_msid,
			timestamp,
			&b,
			data,
			frame_userdata,
			internal,
			0);
	if (ret < 0)
		goto failure;
	return ret;

failure:
	free(b.buf);

	return ret;
}


int send_video_frame(struct rtmp_chunk_stream *stream,
		     struct rtmp_buffer *frame,
		     uint32_t timestamp,
		     int is_meta,
		     int is_key,
		     void *frame_userdata)
{
	int ret;
	struct rtmp_buffer b;

	b.cap = 5;
	b.buf = calloc(b.cap, 1);
	if (!b.buf)
		return -ENOMEM;
	b.rd = 0;
	b.len = b.cap;

	b.buf[0] = is_key ? 0x17 : 0x27;
	b.buf[1] = is_meta ? 0x00 : 0x01;

	ret = send_data(stream,
			4,
			0x09,
			stream->published_msid,
			timestamp,
			&b,
			frame,
			frame_userdata,
			0,
			0);
	if (ret < 0)
		free(b.buf);

	return ret;
}


static int aac_asc_to_rtmp_audio_config(struct rtmp_buffer *data,
					uint8_t *audio_setting)
{
	int ret = 0;
	struct aac_asc asc;
	struct adef_format audio_format;

	if ((audio_setting == NULL) || (data == NULL))
		return -EINVAL;

	ret = aac_parse_asc(data->buf, data->len, &asc);
	if (ret < 0)
		return ret;

	ret = aac_asc_to_adef_format(&asc, &audio_format);
	if (ret < 0)
		return ret;

	if (!adef_is_format_valid(&audio_format))
		return -EINVAL;

	if ((audio_format.encoding != ADEF_ENCODING_AAC_LC) ||
	    ((audio_format.channel_count != 1) &&
	     (audio_format.channel_count != 2)) ||
	    (audio_format.bit_depth != 16))
		return -EINVAL;

	if ((audio_format.sample_rate != 48000) &&
	    (audio_format.sample_rate != 44100) &&
	    (audio_format.sample_rate != 22050) &&
	    (audio_format.sample_rate != 11025))
		return -EINVAL;

	/* Format: HE-AAC */
	*audio_setting = 0xa0;

	/* Sample size: 16 bits */
	*audio_setting |= 0x2;

	switch (audio_format.sample_rate) {
	case 48000:
	case 44100:
		*audio_setting |= 0xc;
		break;

	case 22050:
		*audio_setting |= 0x8;
		break;

	case 11025:
		*audio_setting |= 0x4;
		break;

	default:
		break;
	}

	if (audio_format.channel_count == 2)
		*audio_setting |= 0x1;

	return 0;
}


int send_audio_data(struct rtmp_chunk_stream *stream,
		    struct rtmp_buffer *data,
		    uint32_t timestamp,
		    int is_meta,
		    void *frame_userdata)
{
	int ret = 0;
	struct rtmp_buffer b;

	b.cap = 2;
	b.buf = calloc(b.cap, 1);
	if (!b.buf)
		return -ENOMEM;
	b.rd = 0;
	b.len = b.cap;

	/* the first packet to be sent should be the audio config */
	if (!stream->audio_setup) {
		ret = aac_asc_to_rtmp_audio_config(data,
						   &stream->audio_setting);
		if (ret < 0)
			goto failure;
		stream->audio_setup = true;
	}


	b.buf[0] = stream->audio_setting;
	b.buf[1] = is_meta ? 0x00 : 0x01;

	ret = send_data(stream,
			3,
			0x08,
			stream->published_msid,
			timestamp,
			&b,
			data,
			frame_userdata,
			0,
			0);
	if (ret < 0)
		goto failure;

	return ret;

failure:
	free(b.buf);
	return ret;
}


int send_amf_message(struct rtmp_chunk_stream *stream, struct rtmp_buffer *msg)
{
	int ret;
	bool found = false;
	uint8_t *p = NULL;
	unsigned int i;
	struct rtmp_buffer buf;
	ret = clone_buffer(msg, &buf);
	if (ret != 0)
		return ret;

	for (i = 0, p = buf.buf; (buf.len - i) >= sizeof("publish") - 1;
	     i++, p++) {
		if ((*p == 'p') && !memcmp("publish", p, sizeof("publish") - 1))
			found = true;
	}

	/* It seems mandatory to set csid to 0x4 on "publish" message to connect
	 * to the wowza RTMP server*/
	ret = send_data(stream,
			found ? 4 : 3,
			0x14,
			stream->published_msid,
			0,
			NULL,
			&buf,
			NULL,
			1,
			0);
	if (ret < 0)
		free(buf.buf);

	return ret;
}


int flush_chunk_stream(struct rtmp_chunk_stream *stream)
{
	struct rtmp_chunk_tx_chan *tchan, *ttmp;

	if (!stream)
		return -EINVAL;

	list_walk_entry_forward_safe(&stream->tx_channels, tchan, ttmp, node)
	{
		flush_chunk_tx_chan(stream, tchan);
	}

	return 0;
}


int delete_chunk_stream(struct rtmp_chunk_stream *stream)
{
	int ret, err;
	struct rtmp_chunk_rx_chan *rchan, *rtmp;
	struct rtmp_chunk_tx_chan *tchan, *ttmp;

	if (!stream)
		return -EINVAL;

	ret = tskt_socket_update_events(
		stream->tsock, 0, POMP_FD_EVENT_OUT | POMP_FD_EVENT_IN);
	if (ret != 0)
		return ret;

	ret = tskt_socket_set_event_cb(stream->tsock, 0, NULL, NULL);
	if (ret != 0)
		return ret;

	list_walk_entry_forward_safe(&stream->tx_channels, tchan, ttmp, node)
	{
		list_del(&tchan->node);
		delete_chunk_tx_chan(stream, tchan);
	}

	list_walk_entry_forward_safe(&stream->rx_channels, rchan, rtmp, node)
	{
		list_del(&rchan->node);
		delete_chunk_rx_chan(rchan);
	}

	err = pomp_timer_clear(stream->watchdog_timer);
	if (err < 0)
		ULOG_ERRNO("pomp_timer_clear", -err);

	err = pomp_timer_destroy(stream->watchdog_timer);
	if (err < 0)
		ULOG_ERRNO("pomp_timer_destroy", -err);

	free(stream->rcvbuf.buf);
	free(stream);
	return 0;
}


int store_message_stream_id(struct rtmp_chunk_stream *stream, uint32_t msid)
{
	if (!stream)
		return -EINVAL;

	stream->published_msid = msid;
	return 0;
}
