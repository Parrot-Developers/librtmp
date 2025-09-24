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
#include "rtmp_internal.h"

#ifdef _WIN32
#	include <winsock2.h>
#else
#	include <arpa/inet.h>
#	include <netdb.h>
#	include <netinet/in.h>
#	include <sys/socket.h>
#endif
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <libpomp.h>
#include <openssl/err.h>
#include <openssl/evp.h>
#include <openssl/ssl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include <futils/futils.h>
#include <transport-packet/tpkt.h>
#include <transport-socket/tskt.h>
#include <transport-tls/ttls.h>

#define ULOG_TAG rtmp
#include <ulog.h>
ULOG_DECLARE_TAG(rtmp);

#define HANDSHAKE_SIZE 1536

#define RTMP_ONSTATUS_PUBLISH_CODE "NetStream.Publish.Start"


static void
async_rtmp_client_disconnect(struct rtmp_client *client,
			     enum rtmp_client_disconnection_reason reason);


static char *anonymize_str(const char *str);


static void tskt_resolv_cb(struct tskt_resolv *self,
			   int id,
			   enum tskt_resolv_error result,
			   int naddrs,
			   const char *const *addrs,
			   void *userdata);


enum rtmp_internal_state {
	RTMP_CONN_IDLE = 0,
	RTMP_CONN_WAIT_DNS,
	RTMP_CONN_WAIT_TCP,
	RTMP_CONN_WAIT_S0,
	RTMP_CONN_WAIT_S1,
	RTMP_CONN_WAIT_S2,
	RTMP_CONN_WAIT_FMS,
	RTMP_CONN_READY,
};


static inline char *xstrdup(const char *src)
{
	if (!src)
		return NULL;
	return strdup(src);
}


static enum rtmp_client_conn_state
rtmp_internal_to_connection_state(enum rtmp_internal_state state)
{
	switch (state) {
	case RTMP_CONN_WAIT_DNS:
	case RTMP_CONN_WAIT_TCP:
	case RTMP_CONN_WAIT_S0:
	case RTMP_CONN_WAIT_S1:
	case RTMP_CONN_WAIT_S2:
	case RTMP_CONN_WAIT_FMS:
		return RTMP_CLIENT_CONN_STATE_CONNECTING;
	case RTMP_CONN_READY:
		return RTMP_CLIENT_CONN_STATE_CONNECTED;
	case RTMP_CONN_IDLE:
	default:
		return RTMP_CLIENT_CONN_STATE_DISCONNECTED;
	}
}


const char *rtmp_client_conn_state_str(enum rtmp_client_conn_state state)
{
	/* clang-format off */
	switch (state) {
	RTMP_ENUM_CASE(RTMP_CLIENT_CONN_STATE_, DISCONNECTED);
	RTMP_ENUM_CASE(RTMP_CLIENT_CONN_STATE_, CONNECTING);
	RTMP_ENUM_CASE(RTMP_CLIENT_CONN_STATE_, CONNECTED);

	default: return "UNKNOWN";
	}
	/* clang-format on */
}


const char *rtmp_client_disconnection_reason_str(
	enum rtmp_client_disconnection_reason reason)
{
	/* clang-format off */
	switch (reason) {
	RTMP_ENUM_CASE(RTMP_CLIENT_DISCONNECTION_REASON_, CLIENT_REQUEST);
	RTMP_ENUM_CASE(RTMP_CLIENT_DISCONNECTION_REASON_, SERVER_REQUEST);
	RTMP_ENUM_CASE(RTMP_CLIENT_DISCONNECTION_REASON_, NETWORK_ERROR);
	RTMP_ENUM_CASE(RTMP_CLIENT_DISCONNECTION_REASON_, REFUSED);
	RTMP_ENUM_CASE(RTMP_CLIENT_DISCONNECTION_REASON_, ALREADY_IN_USE);
	RTMP_ENUM_CASE(RTMP_CLIENT_DISCONNECTION_REASON_, TIMEOUT);

	case RTMP_CLIENT_DISCONNECTION_REASON_UNKNOWN: /* NO BREAK */
	default: return "UNKNOWN";
	}
	/* clang-format on */
}


static const char *internal_state_to_str(enum rtmp_internal_state state)
{
	/* clang-format off */
	switch (state) {
	RTMP_ENUM_CASE(RTMP_CONN_, IDLE);
	RTMP_ENUM_CASE(RTMP_CONN_, WAIT_DNS);
	RTMP_ENUM_CASE(RTMP_CONN_, WAIT_TCP);
	RTMP_ENUM_CASE(RTMP_CONN_, WAIT_S0);
	RTMP_ENUM_CASE(RTMP_CONN_, WAIT_S1);
	RTMP_ENUM_CASE(RTMP_CONN_, WAIT_S2);
	RTMP_ENUM_CASE(RTMP_CONN_, WAIT_FMS);
	RTMP_ENUM_CASE(RTMP_CONN_, READY);

	default: return "UNKNOWN";
	}
	/* clang-format on */
}


struct rtmp_client {
	/* Pomp loop */
	struct pomp_loop *loop;

	/* Callbacks */
	struct rtmp_callbacks cbs;
	void *userdata;

	/* RTMP socket */
	int sock;
	struct rtmp_buffer buffer;

	/* Internal state */
	enum rtmp_internal_state state;
	enum rtmp_client_conn_state public_state;
	enum rtmp_client_disconnection_reason disconnection_reason;

	/* RTMP Address */
	char *uri;
	char *host;
	int port;
	char *app;
	char *key;

	/* RTMPS */
	bool secure;
	SSL_CTX *ssl_ctx;
	struct tskt_socket *tsock;

	/* Chunk stream reader/writer */
	struct rtmp_chunk_stream *stream;

	/* AMF messages id */
	int amf_msg_id;
	double connect_id;
	double create_stream_id;

	/* Saved for delete_stream */
	double published_stream_id;

	struct tskt_resolv *tskt_resolv;
	int tskt_resolv_req_id;
	struct pomp_timer *dns_timer;
};


static double get_next_amf_id(struct rtmp_client *client)
{
	double d;
	if (!client) {
		ULOG_ERRNO("get_next_amf_id", EINVAL);
		return 0.0;
	}

	d = ++(client->amf_msg_id);
	return d;
}


static void set_state(struct rtmp_client *client,
		      enum rtmp_internal_state state)
{
	enum rtmp_client_conn_state pub;

	if (!client)
		return;

	if (client->state == state)
		return;

	pub = rtmp_internal_to_connection_state(state);

	if (pub == RTMP_CLIENT_CONN_STATE_DISCONNECTED) {
		ULOGI("state change to %s (%s, reason=%s)",
		      rtmp_client_conn_state_str(pub),
		      internal_state_to_str(state),
		      rtmp_client_disconnection_reason_str(
			      client->disconnection_reason));
	} else {
		ULOGI("state change to %s (%s)",
		      rtmp_client_conn_state_str(pub),
		      internal_state_to_str(state));
	}

	client->state = state;
	if (pub != client->public_state && client->cbs.connection_state)
		client->cbs.connection_state(
			pub, client->disconnection_reason, client->userdata);
	client->public_state = pub;
}


static int send_full(struct rtmp_client *client, void *buf, size_t len)
{
	ssize_t ret;

	if ((client == NULL) || (client->tsock == NULL) || (buf == NULL) ||
	    !len)
		return -EINVAL;

	ret = tskt_socket_write(client->tsock, buf, len);
	if (ret < 0)
		return -errno;
	if ((size_t)ret != len)
		return -EIO;
	return 0;
}


static int send_c0(struct rtmp_client *client)
{
	uint8_t c0 = 3;

	if (!client || (client->tsock == NULL))
		return -EINVAL;

	return send_full(client, &c0, sizeof(c0));
}


static int send_c1(struct rtmp_client *client)
{
	int ret = 0;
	uint8_t *buf;
	uint32_t *tmp;

	if (!client || (client->tsock == NULL))
		return -EINVAL;

	buf = malloc(HANDSHAKE_SIZE);
	if (!buf)
		return -ENOMEM;

	tmp = (uint32_t *)buf;
	tmp[0] = 0;
	tmp[1] = 0; /* Constant value 0 */

	futils_random_bytes(&buf[8], HANDSHAKE_SIZE - 8);

	ret = send_full(client, buf, HANDSHAKE_SIZE);
	free(buf);
	return ret;
}


static int send_c2(struct rtmp_client *client, uint8_t *buf, size_t len)
{
	if (!client || (client->tsock == NULL) || (buf == NULL) ||
	    (len != HANDSHAKE_SIZE))
		return -EINVAL;

	return send_full(client, buf, len);
}


static void dns_timer_cb(struct pomp_timer *timer, void *userdata)
{
	struct rtmp_client *client = userdata;
	int err = 0;

	ULOGE("%s: DNS request timed out after %.2fs, disconnecting",
	      __func__,
	      (float)WATCHDOG_TIMER_DURATION_MS / 1000.);

	err = tskt_resolv_cancel(client->tskt_resolv,
				 client->tskt_resolv_req_id);
	if (err < 0)
		ULOG_ERRNO("tskt_resolv_cancel", -err);

	(void)rtmp_client_disconnect(client,
				     RTMP_CLIENT_DISCONNECTION_REASON_TIMEOUT);
}


struct rtmp_client *rtmp_client_new(struct pomp_loop *loop,
				    const struct rtmp_callbacks *cbs,
				    void *userdata)
{
	struct rtmp_client *client = NULL;
	int err = 0;

	ULOG_ERRNO_RETURN_VAL_IF(loop == NULL, EINVAL, NULL);
	ULOG_ERRNO_RETURN_VAL_IF(cbs == NULL, EINVAL, NULL);
	ULOG_ERRNO_RETURN_VAL_IF(cbs->connection_state == NULL, EINVAL, NULL);
	ULOG_ERRNO_RETURN_VAL_IF(cbs->data_unref == NULL, EINVAL, NULL);

	client = calloc(1, sizeof(*client));
	if (!client) {
		ULOG_ERRNO("calloc", ENOMEM);
		goto error;
	}

	client->loop = loop;
	client->cbs = *cbs;
	client->userdata = userdata;
	set_state(client, RTMP_CONN_IDLE);

	client->buffer.buf = malloc(HANDSHAKE_SIZE);
	if (!client->buffer.buf) {
		ULOG_ERRNO("malloc", ENOMEM);
		goto error;
	}
	client->buffer.cap = HANDSHAKE_SIZE;

	client->dns_timer = pomp_timer_new(client->loop, &dns_timer_cb, client);
	if (client->dns_timer == NULL) {
		ULOG_ERRNO("pomp_timer_new", -err);
		goto error;
	}

	err = tskt_resolv_new(&client->tskt_resolv);
	if (err < 0) {
		ULOG_ERRNO("tskt_resolv_new", -err);
		goto error;
	}

	return client;

error:
	if (client == NULL)
		return NULL;
	if (client->buffer.buf != NULL)
		free(client->buffer.buf);
	if (client->dns_timer != NULL) {
		err = pomp_timer_destroy(client->dns_timer);
		if (err < 0)
			ULOG_ERRNO("pomp_timer_destroy", -err);
	}
	free(client);
	return NULL;
}


void rtmp_client_destroy(struct rtmp_client *client)
{
	int err;

	if (client == NULL)
		return;

	if (client->state != RTMP_CONN_IDLE) {
		err = rtmp_client_disconnect(
			client,
			RTMP_CLIENT_DISCONNECTION_REASON_CLIENT_REQUEST);
		if (err < 0)
			ULOG_ERRNO("rtmp_client_disconnect", -err);
	}

	/* Remove any leftover idle callbacks */
	err = pomp_loop_idle_remove_by_cookie(client->loop, client);
	if (err < 0)
		ULOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);

	if (client->dns_timer != NULL) {
		err = pomp_timer_clear(client->dns_timer);
		if (err < 0)
			ULOG_ERRNO("pomp_timer_clear", -err);
		err = pomp_timer_destroy(client->dns_timer);
		if (err < 0)
			ULOG_ERRNO("pomp_timer_destroy", -err);
	}

	if (client->tskt_resolv != NULL)
		tskt_resolv_unref(client->tskt_resolv);

	free(client->uri);
	free(client->host);
	free(client->app);
	free(client->key);
	free(client->buffer.buf);

	free(client);
}


int rtmp_client_flush(struct rtmp_client *client)
{
	ULOG_ERRNO_RETURN_ERR_IF(client == NULL, EINVAL);

	if (client->stream)
		return flush_chunk_stream(client->stream);
	else
		return 0;
}


static void peer_bw_changed(uint32_t bandwidth, void *userdata)
{
	struct rtmp_client *client = userdata;

	if (client->cbs.peer_bw_changed)
		client->cbs.peer_bw_changed(bandwidth, client->userdata);
	else
		ULOGI("peer BW changed to %" PRIu32 " B/s", bandwidth);
}


static const struct {
	const char *code;
	const char *desc;
	enum rtmp_client_disconnection_reason reason;
} pattern_to_reason_map[] = {
	{"NetConnection.Connect.Rejected",
	 NULL,
	 RTMP_CLIENT_DISCONNECTION_REASON_REFUSED},
	{"NetStream.Publish.Denied",
	 "Stream name is already in use",
	 RTMP_CLIENT_DISCONNECTION_REASON_ALREADY_IN_USE},
};


static enum rtmp_client_disconnection_reason
server_error_to_client_disconnection_reason(const char *code, const char *desc)
{
	if ((code == NULL) && (desc == NULL))
		return RTMP_CLIENT_DISCONNECTION_REASON_UNKNOWN;

	/* Search by desc first (if provided) */
	if (desc != NULL) {
		for (size_t i = 0; i < SIZEOF_ARRAY(pattern_to_reason_map);
		     i++) {
			if (pattern_to_reason_map[i].desc == NULL)
				continue;
			if (!strncasecmp(pattern_to_reason_map[i].desc,
					 desc,
					 strlen(pattern_to_reason_map[i].desc)))
				return pattern_to_reason_map[i].reason;
		}
	}
	if (code != NULL) {
		for (size_t i = 0; i < SIZEOF_ARRAY(pattern_to_reason_map);
		     i++) {
			if (pattern_to_reason_map[i].code == NULL)
				continue;
			if (!strncasecmp(pattern_to_reason_map[i].code,
					 code,
					 strlen(pattern_to_reason_map[i].code)))
				return pattern_to_reason_map[i].reason;
		}
	}

	return RTMP_CLIENT_DISCONNECTION_REASON_UNKNOWN;
}


/* Called from stream_consume_rcv_data loop, disconnection must be async */
static void handle_error(struct rtmp_client *client,
			 struct rtmp_buffer *data,
			 const char *name,
			 double id)
{
	enum rtmp_client_disconnection_reason disconnection_reason =
		RTMP_CLIENT_DISCONNECTION_REASON_UNKNOWN;
	char *key = NULL, *value = NULL;
	char *code = NULL;
	char *desc = NULL;
	int is_error = 0;
	int ret;

	ULOGI("handle error");

	/* Skip the NULL command object & start the Info object */
	ret = amf_get_null(data);
	if (ret != 0) {
		ULOG_ERRNO("amf_get_null", -ret);
		goto error;
	}
	ret = amf_get_object_start(data);
	if (ret != 0) {
		ULOG_ERRNO("amf_get_object_start", -ret);
		goto error;
	}

	while (1) {
		free(key);
		free(value);
		key = value = NULL;
		ret = amf_get_property(data, &key);
		if (ret != 0)
			break;
		if (key[0] == '\0')
			break;
		ret = amf_get_string(data, &value);
		if (ret != 0) {
			/* We only handle string value, skip other values */
			amf_skip_data(data);
			continue;
		}

		if (strcmp("level", key) == 0)
			is_error = (strcmp("error", value) == 0);
		else if (strcmp("code", key) == 0)
			code = xstrdup(value);
		else if (strcmp("description", key) == 0)
			desc = xstrdup(value);
	}
	free(key);
	free(value);

	if (is_error) {
		ULOGE("server error: code: '%s', desc: '%s'",
		      code ? code : "Unknown",
		      desc ? desc : "Unknown");
		disconnection_reason =
			server_error_to_client_disconnection_reason(code, desc);
		goto error;
	}

	if (!code) {
		ULOGE("missing 'code' property in server answer");
		disconnection_reason =
			RTMP_CLIENT_DISCONNECTION_REASON_INTERNAL_ERROR;
		goto error;
	}

	if (strcmp(RTMP_ONSTATUS_PUBLISH_CODE, code) != 0) {
		ULOGE("bad answer code: %s, expected %s",
		      code,
		      RTMP_ONSTATUS_PUBLISH_CODE);
		disconnection_reason =
			RTMP_CLIENT_DISCONNECTION_REASON_INTERNAL_ERROR;
		goto error;
	}
	free(desc);
	free(code);

	set_state(client, RTMP_CONN_READY);
	return;

error:
	free(desc);
	free(code);
	async_rtmp_client_disconnect(client, disconnection_reason);
}


/* Called from stream_consume_rcv_data loop, disconnection must be async */
static void handle_connect_result(struct rtmp_client *client,
				  struct rtmp_buffer *data,
				  const char *name,
				  double id)
{
	double cmd_id;
	int ret;

	ULOGI("handle connect result");

	/* Send releaseStream / FCPublish / createStream */
	cmd_id = get_next_amf_id(client);
	client->buffer.len = 0;
	ret = amf_encode(&client->buffer,
			 "%s,%f,0,%s",
			 "releaseStream",
			 cmd_id,
			 client->key);
	if (ret != 0) {
		ULOG_ERRNO("amf_encode", -ret);
		goto error;
	}
	ret = send_amf_message(client->stream, &client->buffer);
	if (ret < 0) {
		ULOG_ERRNO("send_amf_message", -ret);
		goto error;
	}

	cmd_id = get_next_amf_id(client);
	client->buffer.len = 0;
	ret = amf_encode(&client->buffer,
			 "%s,%f,0,%s",
			 "FCPublish",
			 cmd_id,
			 client->key);
	if (ret != 0) {
		ULOG_ERRNO("amf_encode", -ret);
		goto error;
	}
	ret = send_amf_message(client->stream, &client->buffer);
	if (ret < 0) {
		ULOG_ERRNO("send_amf_message", -ret);
		goto error;
	}

	client->create_stream_id = get_next_amf_id(client);
	client->buffer.len = 0;
	ret = amf_encode(&client->buffer,
			 "%s,%f,0,%s",
			 "createStream",
			 client->create_stream_id,
			 client->key);
	if (ret != 0) {
		ULOG_ERRNO("amf_encode", -ret);
		goto error;
	}
	ret = send_amf_message(client->stream, &client->buffer);
	if (ret < 0) {
		ULOG_ERRNO("send_amf_message", -ret);
		goto error;
	}

	return;

error:
	async_rtmp_client_disconnect(
		client, RTMP_CLIENT_DISCONNECTION_REASON_NETWORK_ERROR);
}


/* Called from stream_consume_rcv_data loop, disconnection must be async */
static void handle_create_stream_result(struct rtmp_client *client,
					struct rtmp_buffer *data,
					const char *name,
					double id)
{
	int ret;
	double cmd_id;
	int32_t msid;

	ULOGI("handle create_stream result");

	/* Remaining message format should be NULL followed by the stream ID */
	ret = amf_get_null(data);
	if (ret != 0) {
		ULOG_ERRNO("amf_get_null", -ret);
		goto error;
	}
	ret = amf_get_number(data, &client->published_stream_id);
	if (ret != 0) {
		ULOG_ERRNO("amf_get_number", -ret);
		goto error;
	}

	/* Store the message stream id used for the followings exchanges */
	msid = (uint32_t)client->published_stream_id;
	ret = store_message_stream_id(client->stream, msid);
	if (ret != 0) {
		ULOG_ERRNO("store_message_stream_id", -ret);
		goto error;
	}

	cmd_id = get_next_amf_id(client);
	client->buffer.len = 0;
	ret = amf_encode(&client->buffer,
			 "%s,%f,0,%s,%s",
			 "publish",
			 cmd_id,
			 client->key,
			 "live");
	if (ret != 0) {
		ULOG_ERRNO("amf_encode", -ret);
		goto error;
	}
	ret = send_amf_message(client->stream, &client->buffer);
	if (ret < 0) {
		ULOG_ERRNO("send_amf_message", -ret);
		goto error;
	}

	return;

error:
	async_rtmp_client_disconnect(
		client, RTMP_CLIENT_DISCONNECTION_REASON_NETWORK_ERROR);
}


/* Called from stream_consume_rcv_data loop, disconnection must be async */
static void handle_status_update(struct rtmp_client *client,
				 struct rtmp_buffer *data,
				 const char *name,
				 double id)
{
	enum rtmp_client_disconnection_reason disconnection_reason =
		RTMP_CLIENT_DISCONNECTION_REASON_UNKNOWN;
	char *key = NULL, *value = NULL;
	char *code = NULL;
	char *desc = NULL;
	int is_error = 0;
	int ret;

	ULOGI("handle onStatus");

	/* Skip the NULL command object & start the Info object */
	ret = amf_get_null(data);
	if (ret != 0) {
		ULOG_ERRNO("amf_get_null", -ret);
		goto error;
	}
	ret = amf_get_object_start(data);
	if (ret != 0) {
		ULOG_ERRNO("amf_get_object_start", -ret);
		goto error;
	}

	while (1) {
		free(key);
		free(value);
		key = value = NULL;
		ret = amf_get_property(data, &key);
		if (ret != 0)
			break;
		if (key[0] == '\0')
			break;
		ret = amf_get_string(data, &value);
		if (ret != 0) {
			/* We only handle string value, skip other values */
			amf_skip_data(data);
			continue;
		}

		if (strcmp("level", key) == 0)
			is_error = (strcmp("error", value) == 0);
		else if (strcmp("code", key) == 0)
			code = xstrdup(value);
		else if (strcmp("description", key) == 0)
			desc = xstrdup(value);
	}
	free(key);
	free(value);

	if (is_error) {
		ULOGE("server error: code: '%s', desc: '%s'",
		      code ? code : "Unknown",
		      desc ? desc : "Unknown");
		disconnection_reason =
			server_error_to_client_disconnection_reason(code, desc);
		goto error;
	}

	if (!code) {
		ULOGE("missing 'code' property in server answer");
		disconnection_reason =
			RTMP_CLIENT_DISCONNECTION_REASON_INTERNAL_ERROR;
		goto error;
	}

	if (strcmp(RTMP_ONSTATUS_PUBLISH_CODE, code) != 0) {
		ULOGE("bad answer code: %s, expected %s",
		      code,
		      RTMP_ONSTATUS_PUBLISH_CODE);
		disconnection_reason =
			RTMP_CLIENT_DISCONNECTION_REASON_INTERNAL_ERROR;
		goto error;
	}
	free(desc);
	free(code);

	set_state(client, RTMP_CONN_READY);
	return;

error:
	free(desc);
	free(code);
	async_rtmp_client_disconnect(client, disconnection_reason);
}


/* Called from stream_consume_rcv_data loop, disconnection must be async */
static void handle_bwdone(struct rtmp_client *client,
			  struct rtmp_buffer *data,
			  const char *name,
			  double id)
{
	int ret;
	double cmd_id;
	ULOGI("handle onBWDone");

	cmd_id = get_next_amf_id(client);
	client->buffer.len = 0;
	ret = amf_encode(&client->buffer, "%s%f0", "_checkbw", cmd_id);
	if (ret != 0) {
		ULOG_ERRNO("amf_encode", -ret);
		goto error;
	}
	ret = send_amf_message(client->stream, &client->buffer);
	if (ret < 0) {
		ULOG_ERRNO("send_amf_message", -ret);
		goto error;
	}

	return;

error:
	async_rtmp_client_disconnect(
		client, RTMP_CLIENT_DISCONNECTION_REASON_NETWORK_ERROR);
}


/* Called from stream_consume_rcv_data loop, disconnection must be async */
static void amf_msg(struct rtmp_buffer *data, void *userdata)
{
	struct rtmp_client *client = userdata;
	char *name;
	double id;
	name = amf_get_msg_name(data, &id);

	if (strcmp("_result", name) == 0) {
		if (id == client->connect_id)
			handle_connect_result(client, data, name, id);
		else if (id == client->create_stream_id)
			handle_create_stream_result(client, data, name, id);
		else
			ULOGW("got a result for an unfollowed call (%f)", id);
	} else if (strcmp("_error", name) == 0) {
		handle_error(client, data, name, id);
	} else if (strcmp("onStatus", name) == 0) {
		handle_status_update(client, data, name, id);
	} else if (strcmp("onBWDone", name) == 0) {
		handle_bwdone(client, data, name, id);
	} else if (strcmp("close", name) == 0) {
		async_rtmp_client_disconnect(
			client,
			RTMP_CLIENT_DISCONNECTION_REASON_SERVER_REQUEST);
	} else {
		ULOGW("unexpected message '%s'", name);
	}

	free(name);
}


static void data_sent(uint8_t *data, void *data_userdata, void *userdata)
{
	struct rtmp_client *client = userdata;
	client->cbs.data_unref(data, data_userdata, client->userdata);
}


struct internal_disconnect_params {
	struct rtmp_client *client;
	enum rtmp_client_disconnection_reason reason;
};


static void call_rtmp_client_disconnect(void *userdata)
{
	struct internal_disconnect_params *params = userdata;
	int err = rtmp_client_disconnect(params->client, params->reason);
	if (err < 0)
		ULOG_ERRNO("rtmp_client_disconnect", -err);
	free(params);
}


static void
async_rtmp_client_disconnect(struct rtmp_client *client,
			     enum rtmp_client_disconnection_reason reason)
{
	struct internal_disconnect_params *params = calloc(1, sizeof(*params));
	if (params == NULL) {
		ULOG_ERRNO("calloc", ENOMEM);
		return;
	}
	params->client = client;
	params->reason = reason;
	int err = pomp_loop_idle_add_with_cookie(
		client->loop, call_rtmp_client_disconnect, params, client);
	if (err < 0)
		ULOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


/* Called from stream_consume_rcv_data loop, disconnection must be async */
static void
rtmp_chunk_stream_disconnected(void *userdata,
			       enum rtmp_client_disconnection_reason reason)
{
	struct rtmp_client *client = userdata;
	async_rtmp_client_disconnect(client, reason);
}


static const struct rtmp_chunk_cbs chunk_cbs = {
	.peer_bw_changed = peer_bw_changed,
	.amf_msg = amf_msg,
	.data_sent = data_sent,
	.disconnected = rtmp_chunk_stream_disconnected,
};


static int parse_uri(const char *uri,
		     bool *secure,
		     char **host,
		     uint16_t *port,
		     char **app,
		     char **key)
{
	int ret;
	bool _secure = false;
	char *raw, *tmp;
	char *raw_addr, *_app, *_key;
	char *_host, *port_s;
	uint16_t _port;
	uint32_t offset = 7;

	ULOG_ERRNO_RETURN_ERR_IF(uri == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(secure == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(host == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(port == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(app == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(key == NULL, EINVAL);

	if (strncmp(uri, "rtmps://", 8) == 0) {
		_secure = true;
		offset = 8;
	}

	if ((strncmp(uri, "rtmp://", 7) != 0) && !_secure)
		return -EPROTO;

	raw = xstrdup(&uri[offset]);
	if (!raw) {
		ret = -ENOMEM;
		goto exit;
	}

	/* Format is host[:port]/app/key */
	raw_addr = strtok_r(raw, "/", &tmp);
	if (!raw_addr) {
		ret = -EPROTO;
		goto exit;
	}
	_app = strtok_r(NULL, "/", &tmp);
	if (!_app) {
		ret = -EPROTO;
		goto exit;
	}
	_key = strtok_r(NULL, "", &tmp);
	if (!_key) {
		ret = -EPROTO;
		goto exit;
	}

	/* search for port number in address */
	_host = strtok_r(raw_addr, ":", &tmp);
	port_s = strtok_r(NULL, "", &tmp);
	if (port_s) {
		uint16_t p;
		int match = sscanf(port_s, "%hu", &p);
		if (match != 1) {
			ret = -EPROTO;
			goto exit;
		}
		_port = p;
	} else {
		_port = 0;
	}

	*secure = _secure;
	*host = xstrdup(_host);
	*port = _port;
	*app = xstrdup(_app);
	*key = xstrdup(_key);

	ret = 0;

exit:
	free(raw);
	return ret;
}


static int process_uri(struct rtmp_client *client,
		       const char *uri,
		       uint16_t *port,
		       struct in_addr *addr)
{
	int ret = 0;
	bool secure = false;
	char *host = NULL;
	uint16_t _port = 0;
	char *app = NULL, *key = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(uri == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(port == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(addr == NULL, EINVAL);

	ret = parse_uri(uri, &secure, &host, &_port, &app, &key);
	if (ret < 0) {
		ULOG_ERRNO("parse_uri", -ret);
		goto exit;
	}

	if (_port == 0)
		_port = DEFAULT_RTMP_PORT;

	*port = htons(_port);
	client->secure = secure;
	client->uri = strdup(uri);
	client->host = host;
	client->port = _port;
	client->app = app;
	client->key = key;

	ret = pomp_timer_set(client->dns_timer, WATCHDOG_TIMER_DURATION_MS);
	if (ret < 0) {
		ULOG_ERRNO("pomp_timer_set", -ret);
		goto exit;
	}

	ret = tskt_resolv_getaddrinfo(client->tskt_resolv,
				      host,
				      client->loop,
				      &tskt_resolv_cb,
				      client,
				      &client->tskt_resolv_req_id);
	if (ret < 0) {
		ULOG_ERRNO("tskt_resolv_getaddrinfo", -ret);
		goto exit;
	}

	set_state(client, RTMP_CONN_WAIT_DNS);

	return ret;

exit:
	if (ret < 0) {
		free(host);
		free(app);
		free(key);
	}
	return ret;
}


static void handle_wait_tcp(struct rtmp_client *client)
{
	int ret;

	if (!client)
		return;

	(void)tskt_socket_update_events(client->tsock, POMP_FD_EVENT_IN, 0);

	set_state(client, RTMP_CONN_WAIT_S0);

	ret = send_c0(client);
	if (ret < 0) {
		ULOG_ERRNO("send_c0", -ret);
		goto error;
	}
	ret = send_c1(client);
	if (ret < 0) {
		ULOG_ERRNO("send_c1", -ret);
		goto error;
	}

	return;

error:
	(void)rtmp_client_disconnect(
		client, RTMP_CLIENT_DISCONNECTION_REASON_NETWORK_ERROR);
}


static void handle_wait_s0(struct rtmp_client *client)
{
	uint8_t s0;
	ssize_t len;

	if (!client)
		return;

	len = tskt_socket_read(client->tsock, &s0, sizeof(s0), NULL);
	if ((len < 0) && (errno == EAGAIN))
		return;

	if ((len < 0) || (len != sizeof(s0))) {
		ULOG_ERRNO("tskt_socket_read", errno);
		goto error;
	}

	if (s0 != 3) {
		ULOGE("bad RTMP version from server, got %u, expected 3", s0);
		goto error;
	}

	set_state(client, RTMP_CONN_WAIT_S1);

	return;

error:
	(void)rtmp_client_disconnect(
		client, RTMP_CLIENT_DISCONNECTION_REASON_NETWORK_ERROR);
}


static void handle_wait_s1(struct rtmp_client *client)
{
	size_t missing_len;
	ssize_t read_len;
	int ret;

	if (!client)
		return;

	missing_len = HANDSHAKE_SIZE - client->buffer.len;

	read_len = tskt_socket_read(client->tsock,
				    &client->buffer.buf[client->buffer.len],
				    missing_len,
				    NULL);
	if ((read_len < 0) && (errno == EAGAIN))
		return;

	if (read_len < 0) {
		ULOG_ERRNO("tskt_socket_read", errno);
		goto error;
	}

	client->buffer.len += read_len;

	if (client->buffer.len < HANDSHAKE_SIZE) {
		ULOGI("got %zu bytes out of %u for S1",
		      client->buffer.len,
		      HANDSHAKE_SIZE);
		return;
	}

	/* We have a complete s1 in client->buffer.buf */

	set_state(client, RTMP_CONN_WAIT_S2);
	ret = send_c2(client, client->buffer.buf, client->buffer.len);
	if (ret != 0) {
		ULOG_ERRNO("send_c2", -ret);
		goto error;
	}
	client->buffer.len = 0;

	return;

error:
	(void)rtmp_client_disconnect(
		client, RTMP_CLIENT_DISCONNECTION_REASON_NETWORK_ERROR);
}


static void handle_wait_s2(struct rtmp_client *client)
{
	size_t missing_len;
	ssize_t read_len;
	int ret;
	char *tcUrl = NULL;

	if (!client)
		return;

	missing_len = HANDSHAKE_SIZE - client->buffer.len;
	read_len = tskt_socket_read(client->tsock,
				    &client->buffer.buf[client->buffer.len],
				    missing_len,
				    NULL);
	if ((read_len < 0) && (errno == EAGAIN))
		return;

	if (read_len < 0) {
		ULOG_ERRNO("tskt_socket_read", errno);
		goto error;
	}

	client->buffer.len += read_len;

	if (client->buffer.len < HANDSHAKE_SIZE) {
		ULOGI("got %zu bytes out of %u for S2",
		      client->buffer.len,
		      HANDSHAKE_SIZE);
		return;
	}

	/* We have a complete s2 in client->buffer.buf */

	client->buffer.len = 0;
	client->buffer.rd = 0;

	client->connect_id = get_next_amf_id(client);

	ret = asprintf(&tcUrl,
		       "rtmp://%s:%d/%s",
		       client->host,
		       client->port,
		       client->app);
	if (ret == -1) {
		ULOG_ERRNO("asprintf", ENOMEM);
		return;
	}

	ret = amf_encode(&client->buffer,
			 "%s,%f,{%s:%s,%s:%s,%s:%s,%s:%s}",
			 "connect",
			 client->connect_id,
			 "app",
			 client->app,
			 "type",
			 "nonprivate",
			 "flashVer",
			 "FMLE/3.0 (compatible; librtmp)",
			 "tcUrl",
			 tcUrl);
	free(tcUrl);
	if (ret != 0) {
		ULOG_ERRNO("amf_encode", -ret);
		goto error;
	}

	/* Open the chunk stream. Remove the fd from the loop here, the chunk
	 * stream needs to have its own callback */
	(void)tskt_socket_set_event_cb(client->tsock, 0, NULL, NULL);

	client->stream = new_chunk_stream(
		client->loop, client->tsock, &chunk_cbs, client);
	if (!client->stream)
		goto error;

	ret = send_amf_message(client->stream, &client->buffer);
	if (ret < 0) {
		ULOG_ERRNO("send_amf_message", -ret);
		goto error;
	}

	ret = set_chunk_size(client->stream, 256);
	if (ret < 0) {
		ULOG_ERRNO("set_chunk_size", -ret);
		goto error;
	}

	set_state(client, RTMP_CONN_WAIT_FMS);

	return;

error:
	(void)rtmp_client_disconnect(
		client, RTMP_CLIENT_DISCONNECTION_REASON_NETWORK_ERROR);
}


/* tls client i/o events handler */
static void
tskt_event_cb(struct tskt_socket *sock, uint32_t revents, void *userdata)
{
	struct rtmp_client *client = (struct rtmp_client *)userdata;
	enum rtmp_client_disconnection_reason disconnection_reason =
		RTMP_CLIENT_DISCONNECTION_REASON_UNKNOWN;

	if (revents & POMP_FD_EVENT_ERR) {
		/* print socket error */
		int err = tskt_socket_get_error(client->tsock);
		ULOG_ERRNO("socket error", err);
		switch (err) {
		case ECONNREFUSED:
			disconnection_reason =
				RTMP_CLIENT_DISCONNECTION_REASON_REFUSED;
			break;
		case EPIPE:
			disconnection_reason =
				RTMP_CLIENT_DISCONNECTION_REASON_NETWORK_ERROR;
			break;
		default:
			break;
		}
		/* close socket */
		goto close_socket;
	}

	if (revents & POMP_FD_EVENT_OUT) {
		/* write remaining data */
		switch (client->state) {
		case RTMP_CONN_WAIT_TCP:
			handle_wait_tcp(client);
			break;
		case RTMP_CONN_WAIT_S0:
			handle_wait_s0(client);
			break;
		case RTMP_CONN_WAIT_S1:
			handle_wait_s1(client);
			break;
		case RTMP_CONN_WAIT_S2:
			handle_wait_s2(client);
			break;
		default:
			break;
		}
	}

	return;

close_socket:
	(void)rtmp_client_disconnect(client, disconnection_reason);
}


int rtmp_client_connect(struct rtmp_client *client, const char *url)
{
	struct sockaddr_in addr;

	ULOG_ERRNO_RETURN_ERR_IF(client == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(url == NULL, EINVAL);

	if (client->state != RTMP_CONN_IDLE)
		return -EALREADY;

	client->disconnection_reason = RTMP_CLIENT_DISCONNECTION_REASON_UNKNOWN;

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	return process_uri(client, url, &addr.sin_port, &addr.sin_addr);
}


int rtmp_client_disconnect(struct rtmp_client *client,
			   enum rtmp_client_disconnection_reason reason)
{
	int err;

	ULOG_ERRNO_RETURN_ERR_IF(client == NULL, EINVAL);

	if (client->state == RTMP_CONN_IDLE)
		return -EALREADY;

	client->disconnection_reason = reason;

	if ((client->state == RTMP_CONN_READY) && client->stream) {
		int ret;
		double cmd_id = get_next_amf_id(client);

		/* Send delete stream */
		client->buffer.len = 0;
		ret = amf_encode(&client->buffer,
				 "%s,%f,0,%f",
				 "deleteStream",
				 cmd_id,
				 client->published_stream_id);
		if (ret != 0) {
			ULOG_ERRNO("amf_encode", -ret);
		} else {
			ret = send_amf_message(client->stream, &client->buffer);
			if (ret < 0)
				ULOG_ERRNO("send_amf_message", -ret);
		}
	}

	/* We remove the fd from the loop when creating a chunk_stream. */
	if (client->stream != NULL) {
		err = delete_chunk_stream(client->stream);
		if (err < 0)
			ULOG_ERRNO("delete_chunk_stream", -err);
		client->stream = NULL;
	} else if (client->tsock != NULL) {
		err = tskt_socket_update_events(
			client->tsock, 0, POMP_FD_EVENT_OUT);
		if (err < 0)
			ULOG_ERRNO("tskt_socket_update_events", -err);
		err = tskt_socket_set_event_cb(client->tsock, 0, NULL, NULL);
		if (err < 0)
			ULOG_ERRNO("tskt_socket_set_event_cb", -err);
	}

	if (client->tsock != NULL) {
		err = tskt_socket_destroy(client->tsock);
		if (err < 0)
			ULOG_ERRNO("tskt_socket_destroy", -err);
		client->tsock = NULL;
	}

	if (client->ssl_ctx != NULL) {
		SSL_CTX_free(client->ssl_ctx);
		client->ssl_ctx = NULL;
	}

	if (client->secure) {
		err = ttls_deinit();
		if (err < 0)
			ULOG_ERRNO("ttls_deinit", -err);
	}

	set_state(client, RTMP_CONN_IDLE);
	return 0;
}


int rtmp_client_send_metadata(struct rtmp_client *client,
			      double duration,
			      int width,
			      int height,
			      double framerate,
			      int audio_sample_rate,
			      int audio_sample_size)
{
	struct rtmp_buffer b;
	int ret;
	double fr;

	ULOG_ERRNO_RETURN_ERR_IF(client == NULL, EINVAL);

	if (client->state != RTMP_CONN_READY)
		return -EAGAIN;

	b.cap = 256;
	b.buf = malloc(b.cap);
	if (!b.buf)
		return -ENOMEM;
	b.len = 0;
	b.rd = 0;

	fr = (framerate == 0) ? 29.97 : framerate;

	ret = amf_encode(&b,
			 "%s[%d,%s:%f,%s:%f,%s:%f,%s:%f,"
			 "%s:%f,%s:%f,%s:%f,%s:%u,%s:%f,%s:%f,%s:%f]",
			 "onMetaData",
			 11,
			 "duration",
			 duration,
			 "width",
			 (double)width,
			 "height",
			 (double)height,
			 "framerate",
			 framerate,
			 "videocodecid",
			 7.0 /* h.264 */,
			 "audiosamplerate",
			 (double)audio_sample_rate,
			 "audiosamplesize",
			 (double)audio_sample_size,
			 "stereo",
			 1,
			 "audiocodecid",
			 10.0 /* AAC */,
			 "AspectRatioX",
			 1.,
			 "AspectRatioY",
			 1.);
	if (ret != 0) {
		free(b.buf);
		return ret;
	}

	return send_metadata(client->stream, &b, 0, 1, NULL);
}


int rtmp_client_send_packedmetadata(struct rtmp_client *client,
				    const uint8_t *buf,
				    size_t len,
				    uint32_t timestamp,
				    void *frame_userdata)
{
	ULOG_ERRNO_RETURN_ERR_IF(client == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	struct rtmp_buffer b = {
		.buf = (uint8_t *)buf,
		.cap = len,
		.len = len,
		.rd = 0,
	};

	if (client->state != RTMP_CONN_READY)
		return -EAGAIN;

	return send_metadata(client->stream, &b, timestamp, 0, frame_userdata);
}


int rtmp_client_send_video_avcc(struct rtmp_client *client,
				const uint8_t *buf,
				size_t len,
				void *frame_userdata)
{
	ULOG_ERRNO_RETURN_ERR_IF(client == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	struct rtmp_buffer b = {
		.buf = (uint8_t *)buf,
		.cap = len,
		.len = len,
		.rd = 0,
	};

	if (client->state != RTMP_CONN_READY)
		return -EAGAIN;

	return send_video_frame(client->stream, &b, 0, 1, 1, frame_userdata);
}


int rtmp_client_send_video_frame(struct rtmp_client *client,
				 const uint8_t *buf,
				 size_t len,
				 uint32_t timestamp,
				 void *frame_userdata)
{
	ULOG_ERRNO_RETURN_ERR_IF(client == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	struct rtmp_buffer b = {
		.buf = (uint8_t *)buf,
		.cap = len,
		.len = len,
		.rd = 0,
	};
	size_t offset = 0;
	uint32_t nal_size;
	uint8_t nal_type;
	int is_key = 0;

	if (client->state != RTMP_CONN_READY)
		return -EAGAIN;

	/* Check if we have an IDR NALU or not */
	while (offset < len) {
		memcpy(&nal_size, &buf[offset], sizeof(nal_size));
		nal_size = ntohl(nal_size);
		nal_type = buf[offset + sizeof(nal_size)] & 0x1f;
		if (nal_type == 5) {
			is_key = 1;
			break;
		}
		offset += nal_size + sizeof(nal_size);
	}

	return send_video_frame(
		client->stream, &b, timestamp, 0, is_key, frame_userdata);
}


int rtmp_client_send_audio_specific_config(struct rtmp_client *client,
					   const uint8_t *buf,
					   size_t len,
					   void *frame_userdata)
{
	ULOG_ERRNO_RETURN_ERR_IF(client == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	struct rtmp_buffer b = {
		.buf = (uint8_t *)buf,
		.cap = len,
		.len = len,
		.rd = 0,
	};

	if (client->state != RTMP_CONN_READY)
		return -EAGAIN;

	return send_audio_data(client->stream, &b, 0, 1, frame_userdata);
}


int rtmp_client_send_audio_data(struct rtmp_client *client,
				const uint8_t *buf,
				size_t len,
				uint32_t timestamp,
				void *frame_userdata)
{
	ULOG_ERRNO_RETURN_ERR_IF(client == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	struct rtmp_buffer b = {
		.buf = (uint8_t *)buf,
		.cap = len,
		.len = len,
		.rd = 0,
	};

	if (client->state != RTMP_CONN_READY)
		return -EAGAIN;

	return send_audio_data(
		client->stream, &b, timestamp, 0, frame_userdata);
}


static char *anonymize_str(const char *str)
{
	size_t len;
	char *output = NULL;

	if (str == NULL)
		return NULL;

	len = strlen(str);
	output = strdup(str);

	if (len < 4)
		return output;

	for (size_t i = 2; i < (len - 2); i++)
		output[i] = '*';

	return output;
}


int rtmp_anonymize_uri(const char *uri, char **anonymized)
{
	int ret;

	ULOG_ERRNO_RETURN_ERR_IF(uri == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(anonymized == NULL, EINVAL);

	bool secure = false;
	char *host = NULL;
	uint16_t _port = 0;
	char *app = NULL, *key = NULL;

	char *anonymized_url = NULL;
	char *anonymized_app = NULL;
	char *anonymized_key = NULL;

	ret = parse_uri(uri, &secure, &host, &_port, &app, &key);
	if (ret < 0) {
		ULOG_ERRNO("parse_uri", -ret);
		goto out;
	}

	anonymized_app = anonymize_str(app);
	anonymized_key = anonymize_str(key);

	if (_port != 0) {
		/* rtmp[s]://[host]:[port]/[app]/[key] */
		ret = asprintf(&anonymized_url,
			       "rtmp%s://%s:%u/%s/%s",
			       secure ? "s" : "",
			       host,
			       _port,
			       anonymized_app,
			       anonymized_key);
	} else {
		/* rtmp[s]://[host]/[app]/[key] */
		ret = asprintf(&anonymized_url,
			       "rtmp%s://%s/%s/%s",
			       secure ? "s" : "",
			       host,
			       anonymized_app,
			       anonymized_key);
	}
	if (ret < 0) {
		ret = -ENOMEM;
		ULOG_ERRNO("asprintf", -ret);
		goto out;
	}

	ret = 0;
	*anonymized = anonymized_url;

out:
	free(host);
	free(app);
	free(key);
	free(anonymized_app);
	free(anonymized_key);
	return ret;
}


static void tskt_resolv_cb(struct tskt_resolv *self,
			   int id,
			   enum tskt_resolv_error result,
			   int naddrs,
			   const char *const *addrs,
			   void *userdata)
{
	int err = 0;
	bool tls_init = false;
	int fd = -1;
	struct tskt_socket *tsock = NULL;
	struct rtmp_client *client = (struct rtmp_client *)userdata;
	char *anonymized_uri = NULL;
	char *anonymized_app = NULL, *anonymized_key = NULL;
#ifdef SO_NOSIGPIPE
	int flags = 1;
#endif

	err = pomp_timer_clear(client->dns_timer);
	if (err < 0) {
		ULOG_ERRNO("pomp_timer_clear", -err);
		goto error;
	}

	if (result != TSKT_RESOLV_ERROR_OK || naddrs == 0) {
		ULOGE("%s (err=%d), (naddrs=%d)", __func__, result, naddrs);
		goto error;
	}

	err = rtmp_anonymize_uri(client->uri, &anonymized_uri);
	if (err < 0) {
		ULOG_ERRNO("rtmp_anonymize_uri", -err);
		goto error;
	}

	anonymized_app = anonymize_str(client->app);
	anonymized_key = anonymize_str(client->key);

	ULOGI("address resolution:");
	ULOGI("input (anonymized): '%s'", anonymized_uri);
	ULOGI("host              : '%s'", client->host);
	ULOGI("resolved address  : '%s'", addrs[0]);
	ULOGI("resolved port     : %hu", client->port);
	ULOGI("app (anonymized)  : '%s'", anonymized_app);
	ULOGI("key (anonymized)  : '%s'", anonymized_key);
	free(anonymized_uri);
	free(anonymized_app);
	free(anonymized_key);

	/* create tcp socket */
	err = tskt_socket_new_tcp(client->loop, &tsock);
	if (err < 0)
		goto error;

	/* connect to server */
	err = tskt_socket_connect(tsock, NULL, 0, addrs[0], client->port);
	if (err < 0)
		goto error;

	fd = tskt_socket_get_fd(tsock);

	if (client->secure) {
		OPENSSL_init_ssl(0, NULL);
		err = ttls_init();
		if (err < 0)
			goto error;
		tls_init = true;

		/* create TLS client context */
		client->ssl_ctx = SSL_CTX_new(TLS_client_method());
		if (client->ssl_ctx == NULL)
			goto error;
	}

#ifdef SO_NOSIGPIPE
	flags = 1;
	err = setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &flags, sizeof(flags));
	if (err != 0) {
		err = -errno;
		goto error;
	}
#endif

	if (client->cbs.socket_cb)
		client->cbs.socket_cb(fd, client->userdata);

	if (client->secure) {
		/* create TLS socket */
		err = ttls_socket_new_with_ctx(
			client->ssl_ctx, tsock, &client->tsock);
		if (err < 0)
			goto error;
	} else {
		client->tsock = tsock;
	}
	tsock = NULL;

	/* monitor i/o events */
	err = tskt_socket_set_event_cb(
		client->tsock, POMP_FD_EVENT_OUT, tskt_event_cb, client);
	if (err < 0)
		goto error;

	set_state(client, RTMP_CONN_WAIT_TCP);
	return;

error:
	err = rtmp_client_disconnect(
		client, RTMP_CLIENT_DISCONNECTION_REASON_NETWORK_ERROR);
	if (err < 0)
		ULOG_ERRNO("rtmp_client_disconnect", -err);
}
