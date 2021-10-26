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

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <libpomp.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <futils/random.h>

#define ULOG_TAG rtmp
#include <ulog.h>
ULOG_DECLARE_TAG(rtmp);

#define HANDSHAKE_SIZE 1536

#define RTMP_ONSTATUS_PUBLISH_CODE "NetStream.Publish.Start"

enum rtmp_internal_state {
	RTMP_CONN_IDLE = 0,
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

static enum rtmp_connection_state
internal_to_public(enum rtmp_internal_state state)
{
	switch (state) {
	case RTMP_CONN_WAIT_S0:
	case RTMP_CONN_WAIT_S1:
	case RTMP_CONN_WAIT_S2:
	case RTMP_CONN_WAIT_FMS:
		return RTMP_CONNECTING;
	case RTMP_CONN_READY:
		return RTMP_CONNECTED;
	case RTMP_CONN_IDLE:
	default:
		return RTMP_DISCONNECTED;
	}
}

static const char *internal_to_string(enum rtmp_internal_state state)
{
	switch (state) {
	case RTMP_CONN_IDLE:
		return "IDLE";
	case RTMP_CONN_WAIT_TCP:
		return "WAIT_TCP";
	case RTMP_CONN_WAIT_S0:
		return "WAIT_S0";
	case RTMP_CONN_WAIT_S1:
		return "WAIT_S1";
	case RTMP_CONN_WAIT_S2:
		return "WAIT_S2";
	case RTMP_CONN_WAIT_FMS:
		return "WAIT_FMS";
	case RTMP_CONN_READY:
		return "READY";
	default:
		return "UNKNOWN";
	}
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
	enum rtmp_connection_state public_state;

	/* RTMP Address */
	char *host;
	int port;
	char *app;
	char *key;

	/* Chunk stream reader/writer */
	struct rtmp_chunk_stream *stream;

	/* AMF messages id */
	int amf_msg_id;
	double connect_id;
	double create_stream_id;

	/* Saved for delete_stream */
	double published_stream_id;
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
	enum rtmp_connection_state pub;
	if (!client)
		return;

	ULOGI("Going to state %s", internal_to_string(state));

	pub = internal_to_public(state);

	client->state = state;
	if (pub != client->public_state && client->cbs.connection_state)
		client->cbs.connection_state(pub, client->userdata);
	client->public_state = pub;
}

static int send_full(int fd, void *buf, size_t len)
{
	ssize_t ret;
	do {
		ret = send(fd, buf, len, 0);
	} while (ret < 0 && errno == EINTR);
	if (ret < 0)
		return -errno;
	if ((size_t)ret != len)
		return -EIO;
	return 0;
}

static int send_c0(struct rtmp_client *client)
{
	uint8_t c0 = 3;

	if (!client || client->sock < 0)
		return -EINVAL;

	return send_full(client->sock, &c0, sizeof(c0));
}

static int send_c1(struct rtmp_client *client)
{
	int ret = 0;
	uint8_t *buf;
	uint32_t *tmp;

	if (!client || client->sock < 0)
		return -EINVAL;

	buf = malloc(HANDSHAKE_SIZE);
	if (!buf)
		return -ENOMEM;

	tmp = (uint32_t *)buf;
	tmp[0] = 0;
	tmp[1] = 0; /* Constant value 0 */

	futils_random_bytes(&buf[8], HANDSHAKE_SIZE - 8);

	ret = send_full(client->sock, buf, HANDSHAKE_SIZE);
	free(buf);
	return ret;
}

static int send_c2(struct rtmp_client *client, uint8_t *buf, size_t len)
{
	if (!client || client->sock < 0 || !buf || len != HANDSHAKE_SIZE)
		return -EINVAL;

	return send_full(client->sock, buf, len);
}

struct rtmp_client *rtmp_client_new(struct pomp_loop *loop,
				    const struct rtmp_callbacks *cbs,
				    void *userdata)
{
	struct rtmp_client *client;

	if (!cbs || !loop) {
		ULOG_ERRNO("rtmp_client_new", EINVAL);
		return NULL;
	}

	client = calloc(1, sizeof(*client));
	if (!client) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}

	client->loop = loop;
	client->cbs = *cbs;
	client->userdata = userdata;
	set_state(client, RTMP_CONN_IDLE);
	client->sock = -1;

	client->buffer.buf = malloc(HANDSHAKE_SIZE);
	if (!client->buffer.buf) {
		free(client);
		return NULL;
	}
	client->buffer.cap = HANDSHAKE_SIZE;

	return client;
}

RTMP_API void rtmp_client_destroy(struct rtmp_client *client)
{
	if (!client)
		return;
	EINVAL;

	if (client->state != RTMP_CONN_IDLE)
		rtmp_client_disconnect(client);

	free(client->host);
	free(client->app);
	free(client->key);
	free(client->buffer.buf);

	free(client);
}

static void peer_bw_changed(uint32_t bandwidth, void *userdata)
{
	struct rtmp_client *client = userdata;

	if (client->cbs.peer_bw_changed)
		client->cbs.peer_bw_changed(bandwidth, client->userdata);
	else
		ULOGI("Peer BW changed to %" PRIu32 " B/s", bandwidth);
}

static void handle_connect_result(struct rtmp_client *client,
				  struct rtmp_buffer *data,
				  const char *name,
				  double id)
{
	double cmd_id;
	int ret;

	ULOGI("Handle connect result");

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
	rtmp_client_disconnect(client);
}

static void handle_create_stream_result(struct rtmp_client *client,
					struct rtmp_buffer *data,
					const char *name,
					double id)
{
	int ret;
	double cmd_id;
	ULOGI("Handle create_stream result");

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
	rtmp_client_disconnect(client);
}

static void handle_status_update(struct rtmp_client *client,
				 struct rtmp_buffer *data,
				 const char *name,
				 double id)
{
	char *key = NULL, *value = NULL;
	char *code = NULL;
	char *desc = NULL;
	int is_error = 0;
	int ret;
	ULOGI("Handle onStatus");

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
		ULOGE("Server error: %s",
		      desc ? desc : (code ? code : "Unknown error"));
		goto error;
	}

	if (!code) {
		ULOGE("Missing 'code' property in server answer");
		goto error;
	}

	if (strcmp(RTMP_ONSTATUS_PUBLISH_CODE, code) != 0) {
		ULOGE("Bad answer code : %s, expected %s",
		      code,
		      RTMP_ONSTATUS_PUBLISH_CODE);
		goto error;
	}
	free(desc);
	free(code);

	set_state(client, RTMP_CONN_READY);
	return;
error:
	free(desc);
	free(code);
	rtmp_client_disconnect(client);
}

static void handle_bwdone(struct rtmp_client *client,
			  struct rtmp_buffer *data,
			  const char *name,
			  double id)
{
	int ret;
	double cmd_id;
	ULOGI("Handle onBWDone");


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
	rtmp_client_disconnect(client);
}

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
			ULOGW("Got a result for an unfollowed call (%f)", id);
	} else if (strcmp("_error", name) == 0) {
		if (id == client->connect_id) {
			ULOGE("connect failed");
			rtmp_client_disconnect(client);
		} else if (id == client->create_stream_id) {
			ULOGE("create_stream failed");
			rtmp_client_disconnect(client);
		} else {
			ULOGW("Got an error for an unfollowed call (%f)", id);
		}
	} else if (strcmp("onStatus", name) == 0) {
		handle_status_update(client, data, name, id);
	} else if (strcmp("onBWDone", name) == 0) {
		handle_bwdone(client, data, name, id);
	} else {
		ULOGW("Unexpected message %s", name);
	}

	free(name);
}

static void data_sent(uint8_t *data, void *data_userdata, void *userdata)
{
	struct rtmp_client *client = userdata;
	client->cbs.data_unref(data, data_userdata, client->userdata);
}

static void pomp_idle_disconnect(void *userdata)
{
	struct rtmp_client *client = userdata;
	rtmp_client_disconnect(client);
}

static void rtmp_chunk_stream_disconnected(void *userdata)
{
	struct rtmp_client *client = userdata;
	pomp_loop_idle_add(client->loop, pomp_idle_disconnect, client);
}

static const struct rtmp_chunk_cbs chunk_cbs = {
	.peer_bw_changed = peer_bw_changed,
	.amf_msg = amf_msg,
	.data_sent = data_sent,
	.disconnected = rtmp_chunk_stream_disconnected,
};

static int parse_uri(struct rtmp_client *client,
		     const char *uri,
		     uint16_t *port,
		     struct in_addr *addr)
{
	int ret = 0;
	char *raw, *tmp;
	char *raw_addr, *app, *key;
	char *host, *port_s;

	struct addrinfo *infos, *i;

	if (!uri || !port || !addr)
		return -EINVAL;

	ULOGI("Parsing %s", uri);

	if (strncmp(uri, "rtmp://", 7) != 0)
		return -EINVAL;

	raw = xstrdup(&uri[7]);
	if (!raw) {
		ret = -EINVAL;
		goto exit;
	}

	/* Format is host[:port]/app/key */
	raw_addr = strtok_r(raw, "/", &tmp);
	if (!raw_addr) {
		ret = -EINVAL;
		goto exit;
	}
	app = strtok_r(NULL, "/", &tmp);
	if (!app) {
		ret = -EINVAL;
		goto exit;
	}
	key = strtok_r(NULL, "", &tmp);
	if (!key) {
		ret = -EINVAL;
		goto exit;
	}

	/* search for port number in address */
	host = strtok_r(raw_addr, ":", &tmp);
	port_s = strtok_r(NULL, "", &tmp);
	if (port_s) {
		uint16_t p;
		int match = sscanf(port_s, "%hu", &p);
		if (match != 1) {
			ret = -EINVAL;
			goto exit;
		}
		*port = htons(p);
	} else {
		*port = htons(DEFAULT_RTMP_PORT);
	}

	ret = getaddrinfo(host, NULL, NULL, &infos);
	if (ret != 0) {
		ret = -EFAULT;
		goto exit;
	}

	i = infos;
	for (i = infos; i; i = i->ai_next) {
		struct sockaddr_in *ai;
		if (i->ai_family != AF_INET)
			continue;
		ai = (struct sockaddr_in *)i->ai_addr;
		addr->s_addr = ai->sin_addr.s_addr;
		break;
	}

	freeaddrinfo(infos);

	if (!i) {
		ret = -EFAULT;
		goto exit;
	}

	client->host = xstrdup(host);
	client->port = ntohs(*port);
	client->app = xstrdup(app);
	client->key = xstrdup(key);

	ULOGI("Address resolution :");
	ULOGI("Input string : %s", uri);
	ULOGI("host   : %s", host);
	ULOGI("port_s : %s", port_s);
	ULOGI("Resolved address : %x (%s)", addr->s_addr, inet_ntoa(*addr));
	ULOGI("Resolved port    : %hu", ntohs(*port));
	ULOGI("App : %s", client->app);
	ULOGI("Key : %s", client->key);

exit:
	free(raw);
	return ret;
}

static void handle_wait_tcp(struct rtmp_client *client)
{
	int sockerr, ret;
	socklen_t len;

	if (!client)
		return;

	len = sizeof(sockerr);
	ret = getsockopt(client->sock, SOL_SOCKET, SO_ERROR, &sockerr, &len);
	if (ret < 0) {
		ULOG_ERRNO("getsockopt", errno);
		goto error;
	}

	if (sockerr != 0) {
		ULOG_ERRNO("connect", sockerr);
		goto error;
	}

	pomp_loop_update(client->loop, client->sock, POMP_FD_EVENT_IN);

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
	rtmp_client_disconnect(client);
}

static void handle_wait_s0(struct rtmp_client *client)
{
	uint8_t s0;
	ssize_t len;

	if (!client)
		return;

	len = recv(client->sock, &s0, sizeof(s0), 0);
	if (len < 0) {
		ULOG_ERRNO("recv", errno);
		goto error;
	}
	if (len != sizeof(s0)) {
		ULOG_ERRNO("recv", ENODATA);
		goto error;
	}

	if (s0 != 3) {
		ULOGE("Bad RTMP version from server, got %u, expected 3", s0);
		goto error;
	}

	set_state(client, RTMP_CONN_WAIT_S1);

	return;
error:
	rtmp_client_disconnect(client);
}

static void handle_wait_s1(struct rtmp_client *client)
{
	size_t missing_len;
	ssize_t read_len;
	int ret;

	if (!client)
		return;

	missing_len = HANDSHAKE_SIZE - client->buffer.len;
	read_len = recv(client->sock,
			&client->buffer.buf[client->buffer.len],
			missing_len,
			0);
	if (read_len < 0) {
		ULOG_ERRNO("read", errno);
		goto error;
	}

	client->buffer.len += read_len;

	if (client->buffer.len < HANDSHAKE_SIZE) {
		ULOGI("Got %zu bytes out of %u for S1",
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
	rtmp_client_disconnect(client);
}

static void handle_wait_s2(struct rtmp_client *client)
{
	size_t missing_len;
	ssize_t read_len;
	int ret;
	char tcUrl[64];

	if (!client)
		return;

	missing_len = HANDSHAKE_SIZE - client->buffer.len;
	read_len = recv(client->sock,
			&client->buffer.buf[client->buffer.len],
			missing_len,
			0);
	if (read_len < 0) {
		ULOG_ERRNO("read", errno);
		goto error;
	}

	client->buffer.len += read_len;

	if (client->buffer.len < HANDSHAKE_SIZE) {
		ULOGI("Got %zu bytes out of %u for S2",
		      client->buffer.len,
		      HANDSHAKE_SIZE);
		return;
	}

	/* We have a complete s2 in client->buffer.buf */

	client->buffer.len = 0;
	client->buffer.rd = 0;

	client->connect_id = get_next_amf_id(client);

	snprintf(tcUrl,
		 sizeof(tcUrl),
		 "rtmp://%s:%d/%s",
		 client->host,
		 client->port,
		 client->app);

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
	if (ret != 0) {
		ULOG_ERRNO("amf_encode", -ret);
		goto error;
	}

	/* Open the chunk stream. Remove the fd from the loop here, the chunk
	 * stream needs to have its own callback */
	pomp_loop_remove(client->loop, client->sock);
	client->stream = new_chunk_stream(
		client->loop, client->sock, &chunk_cbs, client);
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
	rtmp_client_disconnect(client);
}

static void pomp_event_cb(int fd, uint32_t revents, void *userdata)
{
	struct rtmp_client *client = userdata;
	if (!client || fd != client->sock)
		return;

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

RTMP_API int rtmp_client_connect(struct rtmp_client *client, const char *url)
{
	int ret;
	int flags;
	struct sockaddr_in addr;

	if (!client || !url)
		return -EINVAL;

	if (client->state != RTMP_CONN_IDLE)
		return -EALREADY;

	client->sock = socket(AF_INET, SOCK_STREAM, 0);
	if (client->sock < 0)
		return -errno;

	flags = fcntl(client->sock, F_GETFL, 0);
	if (flags == -1) {
		ret = -errno;
		goto error_close;
	}
	ret = fcntl(client->sock, F_SETFL, flags | O_NONBLOCK);
	if (ret == -1) {
		ret = -errno;
		goto error_close;
	}

#ifdef SO_NOSIGPIPE
	flags = 1;
	ret = setsockopt(
		client->sock, SOL_SOCKET, SO_NOSIGPIPE, &flags, sizeof(flags));
	if (ret != 0) {
		ret = -errno;
		goto error_close;
	}
#endif

	if (client->cbs.socket_cb)
		client->cbs.socket_cb(client->sock, client->userdata);

	ret = pomp_loop_add(client->loop,
			    client->sock,
			    POMP_FD_EVENT_OUT,
			    pomp_event_cb,
			    client);
	if (ret != 0)
		goto error_close;

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	ret = parse_uri(client, url, &addr.sin_port, &addr.sin_addr);
	if (ret != 0)
		goto error;

	ret = connect(
		client->sock, (const struct sockaddr *)&addr, sizeof(addr));
	if (ret < 0)
		ret = -errno;

	/* EINPROGRESS should be silenced */
	if (ret == -EINPROGRESS)
		ret = 0;
	if (ret != 0)
		goto error;

	set_state(client, RTMP_CONN_WAIT_TCP);

	return ret;

error:
	pomp_loop_remove(client->loop, client->sock);
error_close:
	close(client->sock);
	return ret;
}

RTMP_API int rtmp_client_disconnect(struct rtmp_client *client)
{
	if (!client || client->state == RTMP_CONN_IDLE)
		return -EINVAL;

	if (client->state == RTMP_CONN_READY) {
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
	if (!client->stream)
		pomp_loop_remove(client->loop, client->sock);
	else
		delete_chunk_stream(client->stream);
	client->stream = NULL;

	close(client->sock);
	set_state(client, RTMP_CONN_IDLE);
	return 0;
}

RTMP_API int rtmp_client_send_metadata(struct rtmp_client *client,
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

	if (!client)
		return -EINVAL;
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
			 "%s:%f,%s:%f,%s:%f,%s:%u,%s:%f]",
			 "onMetaData",
			 9,
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
			 10.0 /* AAC */);
	if (ret != 0) {
		free(b.buf);
		return ret;
	}

	return send_metadata(client->stream, &b, 0, 1, NULL);
}

RTMP_API int rtmp_client_send_packedmetadata(struct rtmp_client *client,
					     const uint8_t *buf,
					     size_t len,
					     uint32_t timestamp,
					     void *frame_userdata)
{
	struct rtmp_buffer b = {
		.buf = (uint8_t *)buf,
		.cap = len,
		.len = len,
		.rd = 0,
	};
	if (!client || !buf)
		return -EINVAL;

	if (client->state != RTMP_CONN_READY)
		return -EAGAIN;

	return send_metadata(client->stream, &b, timestamp, 0, frame_userdata);
}

RTMP_API int rtmp_client_send_video_avcc(struct rtmp_client *client,
					 const uint8_t *buf,
					 size_t len,
					 void *frame_userdata)
{
	struct rtmp_buffer b = {
		.buf = (uint8_t *)buf,
		.cap = len,
		.len = len,
		.rd = 0,
	};

	if (!client || !buf)
		return -EINVAL;

	if (client->state != RTMP_CONN_READY)
		return -EAGAIN;

	return send_video_frame(client->stream,
				&b,
				(uint32_t)client->published_stream_id,
				0,
				1,
				1,
				frame_userdata);
}

RTMP_API int rtmp_client_send_video_frame(struct rtmp_client *client,
					  const uint8_t *buf,
					  size_t len,
					  uint32_t timestamp,
					  void *frame_userdata)
{
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

	if (!client || !buf)
		return -EINVAL;

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

	return send_video_frame(client->stream,
				&b,
				(uint32_t)client->published_stream_id,
				timestamp,
				0,
				is_key,
				frame_userdata);
}

RTMP_API int rtmp_client_send_audio_specific_config(struct rtmp_client *client,
						    const uint8_t *buf,
						    size_t len,
						    void *frame_userdata)
{
	struct rtmp_buffer b = {
		.buf = (uint8_t *)buf,
		.cap = len,
		.len = len,
		.rd = 0,
	};
	if (!client || !buf)
		return -EINVAL;

	if (client->state != RTMP_CONN_READY)
		return -EAGAIN;

	return send_audio_data(client->stream,
			       &b,
			       (uint32_t)client->published_stream_id,
			       0,
			       1,
			       frame_userdata);
}

RTMP_API int rtmp_client_send_audio_data(struct rtmp_client *client,
					 const uint8_t *buf,
					 size_t len,
					 uint32_t timestamp,
					 void *frame_userdata)
{
	struct rtmp_buffer b = {
		.buf = (uint8_t *)buf,
		.cap = len,
		.len = len,
		.rd = 0,
	};
	if (!client || !buf)
		return -EINVAL;

	if (client->state != RTMP_CONN_READY)
		return -EAGAIN;

	return send_audio_data(client->stream,
			       &b,
			       (uint32_t)client->published_stream_id,
			       timestamp,
			       0,
			       frame_userdata);
}

RTMP_API const char *
rtmp_connection_state_to_string(enum rtmp_connection_state state)
{
	switch (state) {
	case RTMP_DISCONNECTED:
		return "RTMP_DISCONNECTED";
	case RTMP_CONNECTING:
		return "RTMP_CONNECTING";
	case RTMP_CONNECTED:
		return "RTMP_CONNECTED";
	default:
		return "UNKNOWN";
	}
}
