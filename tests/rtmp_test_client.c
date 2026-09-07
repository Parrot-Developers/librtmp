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
#include "rtmp_priv.h"

#include <libpomp.h>
#include <transport-socket/tskt.h>

#include <futils/timetools.h>

#include <aac/aac.h>
#include <aac/aac_types.h>
#include <aac/aac_writer.h>
#include <audio-defs/adefs.h>

/* RTMP simple handshake size (see rtmp.c's own HANDSHAKE_SIZE, not exported
 * via any header) */
#define HANDSHAKE_LEN 1536


/*
 * Elapsed-time-bounded polling helpers. Per the lesson already learned
 * writing libaudio-decode's suite (see CLAUDE.md): an iteration-count
 * budget silently turns into a per-event budget whenever the polled
 * events can fire back-to-back; bound by real time instead.
 */

static uint64_t elapsed_us_since(const struct timespec *start)
{
	struct timespec now;
	uint64_t diff_us = 0;
	int sign = 0;

	time_get_monotonic(&now);
	time_timespec_diff_us(start, &now, &diff_us, &sign);
	return diff_us;
}


/* Pump the loop unconditionally for a fixed duration, e.g. to let an
 * asynchronous round-trip settle when there is no fixture-visible signal
 * to wait on (only used for the "and nothing bad happened" regression
 * check, never as a substitute for a real completion predicate). */
static void pump_for(struct pomp_loop *loop, int duration_ms)
{
	struct timespec start;

	time_get_monotonic(&start);
	while (elapsed_us_since(&start) < (uint64_t)duration_ms * 1000)
		pomp_loop_wait_and_process(loop, 20);
}


static bool pump_while(struct pomp_loop *loop,
		       bool (*done)(void *arg),
		       void *arg,
		       int timeout_ms)
{
	struct timespec start;

	time_get_monotonic(&start);
	while (!done(arg)) {
		pomp_loop_wait_and_process(loop, 20);
		if (elapsed_us_since(&start) >= (uint64_t)timeout_ms * 1000)
			break;
	}
	return done(arg);
}


static bool socket_read_exact(struct pomp_loop *loop,
			      struct tskt_socket *sock,
			      uint8_t *buf,
			      size_t len,
			      int timeout_ms)
{
	struct timespec start;
	size_t got = 0;

	time_get_monotonic(&start);
	while (got < len) {
		ssize_t n = tskt_socket_read(sock, &buf[got], len - got, NULL);
		if (n > 0) {
			got += (size_t)n;
			continue;
		}
		if (n < 0 && errno != EAGAIN)
			return false;
		pomp_loop_wait_and_process(loop, 20);
		if (elapsed_us_since(&start) >= (uint64_t)timeout_ms * 1000)
			break;
	}
	return got == len;
}


static bool socket_write_exact(struct pomp_loop *loop,
			       struct tskt_socket *sock,
			       const uint8_t *buf,
			       size_t len,
			       int timeout_ms)
{
	struct timespec start;
	size_t sent = 0;

	time_get_monotonic(&start);
	while (sent < len) {
		ssize_t n = tskt_socket_write(sock, &buf[sent], len - sent);
		if (n > 0) {
			sent += (size_t)n;
			continue;
		}
		if (n < 0 && errno != EAGAIN)
			return false;
		pomp_loop_wait_and_process(loop, 20);
		if (elapsed_us_since(&start) >= (uint64_t)timeout_ms * 1000)
			break;
	}
	return sent == len;
}


/*
 * Loopback RTMP "server" fixture: a real listening socket on
 * 127.0.0.1:<ephemeral>, driving the RTMP simple handshake with raw
 * socket I/O, then a server-side rtmp_chunk_stream (the exact same
 * in-tree code the client uses) to exchange AMF0 command messages.
 */

/* How the server should respond to "publish" from the client */
enum loopback_publish_response {
	PUBLISH_NORMAL = 0, /* onStatus(NetStream.Publish.Start) -- success */
	PUBLISH_ERROR_REJECTED, /* _error(NetConnection.Connect.Rejected) */
	PUBLISH_ERROR_ALREADY_IN_USE, /* onStatus(level:error, Denied+desc) */
	PUBLISH_ERROR_NO_CODE, /* _error(level:status), no code field */
	PUBLISH_ERROR_WRONG_CODE, /* _error(level:status, code:WrongCode) */
	PUBLISH_ERROR_SUCCESS_CODE, /* _error(level:status,
				       code:NetStream.Publish.Start) */
	PUBLISH_STATUS_NO_CODE, /* onStatus(level:status), no code field */
	PUBLISH_STATUS_WRONG_CODE, /* onStatus(level:status, code:WrongCode) */
	PUBLISH_CLOSE, /* "close" message */
};

struct loopback_server {
	struct pomp_loop *loop; /* shared with the client, not owned */
	struct tskt_socket *listen_sock;
	struct tskt_socket *peer_sock;
	uint16_t port;

	struct rtmp_chunk_stream *stream;

	/* behavior knob: if true, reply to "connect" with a malformed AMF
	 * message (a lone null value, no leading string) instead of the
	 * normal _result -- used by the amf_msg()-NULL-deref regression
	 * test */
	bool send_malformed_reply;

	/* if true, write a raw SetPeerBandwidth chunk to peer_sock after the
	 * handshake (before creating the server chunk stream) */
	bool send_peer_bw;

	/* how to respond to "publish" (default = PUBLISH_NORMAL) */
	enum loopback_publish_response publish_response;

	/* if true, send S0 version byte = 0 instead of 3 to test the client's
	 * bad-version detection in handle_wait_s0 */
	bool send_bad_s0;

	/* if true, fragment S1 and S2 into two pieces each, yielding the
	 * loop between them to trigger the partial-read log in handle_wait_s1
	 * and handle_wait_s2 */
	bool send_fragmented_handshake;

	/* if true, send a _result(999.0) with an unknown transaction ID just
	 * before the real _result(connect_id) in the connect handler -- covers
	 * the "got a result for an unfollowed call" branch in amf_msg() */
	bool send_extra_result_unknown_id;

	/* if true, send a "netStatus" message after the normal publish onStatus
	 * -- covers the "unexpected message" ULOGW branch in amf_msg() */
	bool send_extra_unknown_amf;

	int amf_msg_count;
};


static bool server_accept_done(void *arg)
{
	struct loopback_server *srv = arg;
	int ret;

	if (srv->peer_sock != NULL)
		return true;
	ret = tskt_socket_accept(
		srv->listen_sock, NULL, 0, NULL, &srv->peer_sock);
	return ret == 0;
}


static void server_peer_bw_cb(uint32_t bandwidth, void *userdata)
{
	UNUSED(bandwidth);
	UNUSED(userdata);
}


static void
server_data_sent_cb(uint8_t *data, void *data_userdata, void *userdata)
{
	UNUSED(data);
	UNUSED(data_userdata);
	UNUSED(userdata);
}


static void server_disconnected_cb(void *userdata,
				   enum rtmp_client_disconnection_reason reason)
{
	UNUSED(userdata);
	UNUSED(reason);
}


static void server_amf_msg_cb(struct rtmp_buffer *data, void *userdata)
{
	struct loopback_server *srv = userdata;
	struct rtmp_buffer tmp = *data;
	char *name;
	double id = -1.;

	srv->amf_msg_count++;
	name = amf_get_msg_name(&tmp, &id);

	if (srv->send_malformed_reply) {
		uint8_t buf[8];
		struct rtmp_buffer reply = {
			.buf = buf, .cap = sizeof(buf), .len = 0, .rd = 0};
		/* A lone null value: no leading string, so the client's
		 * amf_get_msg_name() must return NULL on it */
		amf_encode(&reply, "0");
		send_amf_message(srv->stream, &reply);
		free(name);
		return;
	}

	if (name == NULL) {
		free(name);
		return;
	}

	if (strcmp(name, "connect") == 0) {
		if (srv->send_extra_result_unknown_id) {
			/* Send _result with a bogus transaction ID first; the
			 * client must log a warning and then still process the
			 * real _result below. */
			uint8_t buf2[24];
			struct rtmp_buffer reply2 = {.buf = buf2,
						     .cap = sizeof(buf2),
						     .len = 0,
						     .rd = 0};
			amf_encode(&reply2, "%s,%f,0", "_result", 999.0);
			send_amf_message(srv->stream, &reply2);
		}

		/* "_result"(10) + number(9) + null(1) = 20 bytes */
		uint8_t buf[24];
		struct rtmp_buffer reply = {
			.buf = buf, .cap = sizeof(buf), .len = 0, .rd = 0};
		amf_encode(&reply, "%s,%f,0", "_result", id);
		send_amf_message(srv->stream, &reply);

		/* Send onBWDone to exercise handle_bwdone on the client side;
		 * the client responds with _checkbw which we ignore below. */
		/* "onBWDone"(11) + number(9) + null(1) = 21 bytes */
		uint8_t bwdone_buf[24];
		struct rtmp_buffer bwdone = {.buf = bwdone_buf,
					     .cap = sizeof(bwdone_buf),
					     .len = 0,
					     .rd = 0};
		amf_encode(&bwdone, "%s,%f,0", "onBWDone", 0.0);
		send_amf_message(srv->stream, &bwdone);
	} else if (strcmp(name, "createStream") == 0) {
		/* "_result"(10) + number(9) + null(1) + number(9) = 29 bytes */
		uint8_t buf[32];
		struct rtmp_buffer reply = {
			.buf = buf, .cap = sizeof(buf), .len = 0, .rd = 0};
		amf_encode(&reply, "%s,%f,0,%f", "_result", id, 1.0);
		send_amf_message(srv->stream, &reply);
	} else if (strcmp(name, "publish") == 0) {
		switch (srv->publish_response) {
		case PUBLISH_ERROR_REJECTED: {
			uint8_t buf[96];
			struct rtmp_buffer reply = {.buf = buf,
						    .cap = sizeof(buf),
						    .len = 0,
						    .rd = 0};
			amf_encode(&reply,
				   "%s,%f,0,{%s:%s,%s:%s}",
				   "_error",
				   id,
				   "level",
				   "error",
				   "code",
				   "NetConnection.Connect.Rejected");
			send_amf_message(srv->stream, &reply);
			break;
		}
		case PUBLISH_ERROR_ALREADY_IN_USE: {
			/* onStatus with level:"error",
			 * code:"NetStream.Publish.Denied", description:"Stream
			 * name is already in use" -> ALREADY_IN_USE via the
			 * desc-first path in
			 * server_error_to_client_disconnection_reason */
			uint8_t buf[128];
			struct rtmp_buffer reply = {.buf = buf,
						    .cap = sizeof(buf),
						    .len = 0,
						    .rd = 0};
			amf_encode(&reply,
				   "%s,%f,0,{%s:%s,%s:%s,%s:%s}",
				   "onStatus",
				   0.0,
				   "level",
				   "error",
				   "code",
				   "NetStream.Publish.Denied",
				   "description",
				   "Stream name is already in use");
			send_amf_message(srv->stream, &reply);
			break;
		}
		case PUBLISH_ERROR_NO_CODE: {
			/* _error with level:"status" and no code ->
			 * handle_error's if (!code) branch -> INTERNAL_ERROR */
			uint8_t buf[48];
			struct rtmp_buffer reply = {.buf = buf,
						    .cap = sizeof(buf),
						    .len = 0,
						    .rd = 0};
			amf_encode(&reply,
				   "%s,%f,0,{%s:%s}",
				   "_error",
				   id,
				   "level",
				   "status");
			send_amf_message(srv->stream, &reply);
			break;
		}
		case PUBLISH_ERROR_WRONG_CODE: {
			/* _error with level:"status" and wrong code ->
			 * handle_error's strcmp branch -> INTERNAL_ERROR */
			uint8_t buf[64];
			struct rtmp_buffer reply = {.buf = buf,
						    .cap = sizeof(buf),
						    .len = 0,
						    .rd = 0};
			amf_encode(&reply,
				   "%s,%f,0,{%s:%s,%s:%s}",
				   "_error",
				   id,
				   "level",
				   "status",
				   "code",
				   "WrongCode");
			send_amf_message(srv->stream, &reply);
			break;
		}
		case PUBLISH_ERROR_SUCCESS_CODE: {
			/* _error with level:"status" and the right code ->
			 * handle_error happy path -> set_state(CONN_READY) ->
			 * client CONNECTED */
			uint8_t buf[80];
			struct rtmp_buffer reply = {.buf = buf,
						    .cap = sizeof(buf),
						    .len = 0,
						    .rd = 0};
			amf_encode(&reply,
				   "%s,%f,0,{%s:%s,%s:%s}",
				   "_error",
				   id,
				   "level",
				   "status",
				   "code",
				   "NetStream.Publish.Start");
			send_amf_message(srv->stream, &reply);
			break;
		}
		case PUBLISH_STATUS_NO_CODE: {
			/* onStatus with level:"status" and no code ->
			 * handle_status_update's if (!code) branch ->
			 * INTERNAL_ERROR */
			uint8_t buf[48];
			struct rtmp_buffer reply = {.buf = buf,
						    .cap = sizeof(buf),
						    .len = 0,
						    .rd = 0};
			amf_encode(&reply,
				   "%s,%f,0,{%s:%s}",
				   "onStatus",
				   0.0,
				   "level",
				   "status");
			send_amf_message(srv->stream, &reply);
			break;
		}
		case PUBLISH_STATUS_WRONG_CODE: {
			/* onStatus with level:"status" and wrong code ->
			 * handle_status_update's strcmp branch ->
			 * INTERNAL_ERROR */
			uint8_t buf[64];
			struct rtmp_buffer reply = {.buf = buf,
						    .cap = sizeof(buf),
						    .len = 0,
						    .rd = 0};
			amf_encode(&reply,
				   "%s,%f,0,{%s:%s,%s:%s}",
				   "onStatus",
				   0.0,
				   "level",
				   "status",
				   "code",
				   "WrongCode");
			send_amf_message(srv->stream, &reply);
			break;
		}
		case PUBLISH_CLOSE: {
			/* Send "close" to exercise amf_msg()'s "close" branch
			 * -> async disconnect with SERVER_REQUEST reason */
			uint8_t buf[24];
			struct rtmp_buffer reply = {.buf = buf,
						    .cap = sizeof(buf),
						    .len = 0,
						    .rd = 0};
			amf_encode(&reply, "%s,%f,0", "close", 0.0);
			send_amf_message(srv->stream, &reply);
			break;
		}
		default: /* PUBLISH_NORMAL */
		{
			uint8_t buf[96];
			struct rtmp_buffer reply = {.buf = buf,
						    .cap = sizeof(buf),
						    .len = 0,
						    .rd = 0};
			amf_encode(&reply,
				   "%s,%f,0,{%s:%s,%s:%s}",
				   "onStatus",
				   0.0,
				   "level",
				   "status",
				   "code",
				   "NetStream.Publish.Start");
			send_amf_message(srv->stream, &reply);

			if (srv->send_extra_unknown_amf) {
				/* Send an unrecognized AMF message to cover the
				 * ULOGW("unexpected message '%s'") branch in
				 * amf_msg() */
				uint8_t u_buf[24];
				struct rtmp_buffer u_reply = {
					.buf = u_buf,
					.cap = sizeof(u_buf),
					.len = 0,
					.rd = 0};
				amf_encode(
					&u_reply, "%s,%f,0", "netStatus", 0.0);
				send_amf_message(srv->stream, &u_reply);
			}
			break;
		}
		}
	}

	free(name);
}


static const struct rtmp_chunk_cbs s_server_cbs = {
	.peer_bw_changed = &server_peer_bw_cb,
	.amf_msg = &server_amf_msg_cb,
	.data_sent = &server_data_sent_cb,
	.disconnected = &server_disconnected_cb,
};


static void loopback_server_init(struct loopback_server *srv,
				 struct pomp_loop *loop)
{
	int ret;

	memset(srv, 0, sizeof(*srv));
	srv->loop = loop;

	ret = tskt_socket_new_tcp(loop, &srv->listen_sock);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	ret = tskt_socket_listen(srv->listen_sock, "127.0.0.1", 0);
	CU_ASSERT_EQUAL_FATAL(ret, 0);

	srv->port = tskt_socket_get_local_port(srv->listen_sock);
	CU_ASSERT_TRUE_FATAL(srv->port != 0);
}


/* Accepts the incoming connection, performs the RTMP simple handshake, and
 * starts the server-side chunk stream. Returns true once ready to exchange
 * AMF0 messages with the client. */
static bool loopback_server_run_until_ready(struct loopback_server *srv,
					    int timeout_ms)
{
	uint8_t c0c1[1 + HANDSHAKE_LEN];
	uint8_t s0s1s2[1 + 2 * HANDSHAKE_LEN];
	uint8_t c2[HANDSHAKE_LEN];

	if (!pump_while(srv->loop, &server_accept_done, srv, timeout_ms))
		return false;

	if (!socket_read_exact(
		    srv->loop, srv->peer_sock, c0c1, sizeof(c0c1), timeout_ms))
		return false;
	if (c0c1[0] != 3)
		return false;

	s0s1s2[0] = srv->send_bad_s0 ? 0 : 3;
	memset(&s0s1s2[1], 0, 2 * HANDSHAKE_LEN);

	if (srv->send_fragmented_handshake) {
		/* S0 + first 10 bytes of S1 -- client processes partial S1 */
		if (!socket_write_exact(srv->loop,
					srv->peer_sock,
					s0s1s2,
					1 + 10,
					timeout_ms))
			return false;
		pomp_loop_wait_and_process(srv->loop, 20);
		/* Rest of S1 -- client processes full S1, sends C2 */
		if (!socket_write_exact(srv->loop,
					srv->peer_sock,
					&s0s1s2[1 + 10],
					HANDSHAKE_LEN - 10,
					timeout_ms))
			return false;
		pomp_loop_wait_and_process(srv->loop, 20);
		/* First 10 bytes of S2 -- client processes partial S2 */
		if (!socket_write_exact(srv->loop,
					srv->peer_sock,
					&s0s1s2[1 + HANDSHAKE_LEN],
					10,
					timeout_ms))
			return false;
		pomp_loop_wait_and_process(srv->loop, 20);
		/* Rest of S2 -- client processes full S2, sends connect */
		if (!socket_write_exact(srv->loop,
					srv->peer_sock,
					&s0s1s2[1 + HANDSHAKE_LEN + 10],
					HANDSHAKE_LEN - 10,
					timeout_ms))
			return false;
	} else {
		if (!socket_write_exact(srv->loop,
					srv->peer_sock,
					s0s1s2,
					sizeof(s0s1s2),
					timeout_ms))
			return false;
	}

	if (!socket_read_exact(
		    srv->loop, srv->peer_sock, c2, sizeof(c2), timeout_ms))
		return false;

	if (srv->send_peer_bw) {
		/* Raw type-0 chunk: csid=2, type=0x06 (Set Peer Bandwidth),
		 * payload = 4-byte bandwidth (1 MB/s, big-endian) + 1-byte
		 * limit type HARD.  Written before the server chunk stream is
		 * created so there is no interleaving with its own TX queue. */
		static const uint8_t peer_bw_msg[] = {
			0x02, /* basic header: fmt=0, csid=2 */
			0x00,
			0x00,
			0x00, /* timestamp */
			0x00,
			0x00,
			0x05, /* message length = 5 bytes */
			0x06, /* message type: Set Peer Bandwidth */
			0x00,
			0x00,
			0x00,
			0x00, /* stream id */
			0x00,
			0x0F,
			0x42,
			0x40, /* bandwidth = 1,000,000 B/s */
			0x00, /* limit type: HARD */
		};
		if (!socket_write_exact(srv->loop,
					srv->peer_sock,
					peer_bw_msg,
					sizeof(peer_bw_msg),
					timeout_ms))
			return false;
	}

	srv->stream =
		new_chunk_stream(srv->loop, srv->peer_sock, &s_server_cbs, srv);
	return srv->stream != NULL;
}


static void loopback_server_stop(struct loopback_server *srv)
{
	if (srv->stream != NULL)
		delete_chunk_stream(srv->stream);
	if (srv->peer_sock != NULL)
		tskt_socket_destroy(srv->peer_sock);
	if (srv->listen_sock != NULL)
		tskt_socket_destroy(srv->listen_sock);
}


/*
 * Client fixture: a real rtmp_client, driven against the loopback server
 * above over the same pomp_loop.
 */

struct client_fixture {
	struct rtmp_client *client;

	int conn_state_count;
	enum rtmp_client_conn_state last_state;
	enum rtmp_client_disconnection_reason last_reason;

	int data_unref_count;
	int socket_cb_count;

	int peer_bw_count;
	uint32_t last_peer_bw;
};


static void
client_connection_state_cb(enum rtmp_client_conn_state state,
			   enum rtmp_client_disconnection_reason reason,
			   void *userdata)
{
	struct client_fixture *fx = userdata;
	fx->last_state = state;
	fx->last_reason = reason;
	fx->conn_state_count++;
}


static void
client_data_unref_cb(uint8_t *data, void *buffer_userdata, void *userdata)
{
	struct client_fixture *fx = userdata;
	UNUSED(data);
	UNUSED(buffer_userdata);
	fx->data_unref_count++;
}


static void client_socket_cb(int fd, void *userdata)
{
	struct client_fixture *fx = userdata;
	UNUSED(fd);
	fx->socket_cb_count++;
}


static void client_peer_bw_changed_cb(uint32_t bandwidth, void *userdata)
{
	struct client_fixture *fx = userdata;
	fx->peer_bw_count++;
	fx->last_peer_bw = bandwidth;
}


static const struct rtmp_callbacks s_client_cbs = {
	.socket_cb = &client_socket_cb,
	.connection_state = &client_connection_state_cb,
	.peer_bw_changed = NULL,
	.data_unref = &client_data_unref_cb,
};


static const struct rtmp_callbacks s_client_cbs_with_peerbw = {
	.socket_cb = &client_socket_cb,
	.connection_state = &client_connection_state_cb,
	.peer_bw_changed = &client_peer_bw_changed_cb,
	.data_unref = &client_data_unref_cb,
};


struct wait_for_state_arg {
	struct client_fixture *fx;
	enum rtmp_client_conn_state target;
};


static bool wait_for_state_done(void *arg)
{
	struct wait_for_state_arg *a = arg;
	return a->fx->last_state == a->target;
}


static bool wait_for_state(struct pomp_loop *loop,
			   struct client_fixture *fx,
			   enum rtmp_client_conn_state target,
			   int timeout_ms)
{
	struct wait_for_state_arg arg = {.fx = fx, .target = target};
	return pump_while(loop, &wait_for_state_done, &arg, timeout_ms);
}


static bool amf_count_at_least_done(void *arg)
{
	struct loopback_server *srv = arg;
	return srv->amf_msg_count >= 1;
}


/*
 * Pure / argument-validation tests
 */

static void test_client_new_delete_invalid_args(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct rtmp_client *client;
	struct rtmp_callbacks incomplete_cbs = s_client_cbs;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	client = rtmp_client_new(NULL, &s_client_cbs, NULL);
	CU_ASSERT_PTR_NULL(client);
	client = rtmp_client_new(loop, NULL, NULL);
	CU_ASSERT_PTR_NULL(client);

	incomplete_cbs.connection_state = NULL;
	client = rtmp_client_new(loop, &incomplete_cbs, NULL);
	CU_ASSERT_PTR_NULL(client);

	incomplete_cbs = s_client_cbs;
	incomplete_cbs.data_unref = NULL;
	client = rtmp_client_new(loop, &incomplete_cbs, NULL);
	CU_ASSERT_PTR_NULL(client);

	/* NULL-safe */
	rtmp_client_destroy(NULL);

	pomp_loop_destroy(loop);
}


static void test_client_fresh_flush_and_txbuf(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct client_fixture fx;
	int ret;

	memset(&fx, 0, sizeof(fx));
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	fx.client = rtmp_client_new(loop, &s_client_cbs, &fx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx.client);

	/* No stream/socket yet: both must succeed trivially */
	ret = rtmp_client_flush(fx.client);
	CU_ASSERT_EQUAL(ret, 0);

	ret = rtmp_client_set_socket_txbuf_size(fx.client, 65536);
	CU_ASSERT_EQUAL(ret, 0);

	ret = rtmp_client_flush(NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = rtmp_client_set_socket_txbuf_size(NULL, 1);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	rtmp_client_destroy(fx.client);
	pomp_loop_destroy(loop);
}


static void test_client_conn_state_str(void)
{
	const char *s;

	s = rtmp_client_conn_state_str(RTMP_CLIENT_CONN_STATE_DISCONNECTED);
	CU_ASSERT_STRING_EQUAL(s, "DISCONNECTED");
	s = rtmp_client_conn_state_str(RTMP_CLIENT_CONN_STATE_CONNECTING);
	CU_ASSERT_STRING_EQUAL(s, "CONNECTING");
	s = rtmp_client_conn_state_str(RTMP_CLIENT_CONN_STATE_CONNECTED);
	CU_ASSERT_STRING_EQUAL(s, "CONNECTED");
	s = rtmp_client_conn_state_str((enum rtmp_client_conn_state)999);
	CU_ASSERT_STRING_EQUAL(s, "UNKNOWN");
}


static void test_client_disconnection_reason_str(void)
{
	const char *s;

	s = rtmp_client_disconnection_reason_str(
		RTMP_CLIENT_DISCONNECTION_REASON_UNKNOWN);
	CU_ASSERT_STRING_EQUAL(s, "UNKNOWN");
	s = rtmp_client_disconnection_reason_str(
		RTMP_CLIENT_DISCONNECTION_REASON_CLIENT_REQUEST);
	CU_ASSERT_STRING_EQUAL(s, "CLIENT_REQUEST");
	s = rtmp_client_disconnection_reason_str(
		RTMP_CLIENT_DISCONNECTION_REASON_SERVER_REQUEST);
	CU_ASSERT_STRING_EQUAL(s, "SERVER_REQUEST");
	s = rtmp_client_disconnection_reason_str(
		RTMP_CLIENT_DISCONNECTION_REASON_NETWORK_ERROR);
	CU_ASSERT_STRING_EQUAL(s, "NETWORK_ERROR");
	s = rtmp_client_disconnection_reason_str(
		RTMP_CLIENT_DISCONNECTION_REASON_REFUSED);
	CU_ASSERT_STRING_EQUAL(s, "REFUSED");
	s = rtmp_client_disconnection_reason_str(
		RTMP_CLIENT_DISCONNECTION_REASON_ALREADY_IN_USE);
	CU_ASSERT_STRING_EQUAL(s, "ALREADY_IN_USE");
	s = rtmp_client_disconnection_reason_str(
		RTMP_CLIENT_DISCONNECTION_REASON_TIMEOUT);
	CU_ASSERT_STRING_EQUAL(s, "TIMEOUT");

	/* INTERNAL_ERROR has no explicit case: falls through to the same
	 * "UNKNOWN" as the default/UNKNOWN cases */
	s = rtmp_client_disconnection_reason_str(
		RTMP_CLIENT_DISCONNECTION_REASON_INTERNAL_ERROR);
	CU_ASSERT_STRING_EQUAL(s, "UNKNOWN");

	s = rtmp_client_disconnection_reason_str(
		(enum rtmp_client_disconnection_reason)999);
	CU_ASSERT_STRING_EQUAL(s, "UNKNOWN");
}


static void test_client_anonymize_url_more_edges(void)
{
	int ret;
	char *out = NULL;

	/* app/key shorter than 4 chars: anonymize_str() leaves them
	 * unchanged (its "len < 4" passthrough) */
	ret = rtmp_anonymize_url("rtmp://host/ab/abcde", &out);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(out);
	CU_ASSERT_STRING_EQUAL(out, "rtmp://host/ab/ab*de");
	free(out);

	/* rtmps with an explicit port, same boundary */
	out = NULL;
	ret = rtmp_anonymize_url("rtmps://host:443/ab/abcde", &out);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(out);
	CU_ASSERT_STRING_EQUAL(out, "rtmps://host:443/ab/ab*de");
	free(out);
}


static void test_client_connect_invalid_args(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct client_fixture fx;
	int ret;

	memset(&fx, 0, sizeof(fx));
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	fx.client = rtmp_client_new(loop, &s_client_cbs, &fx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx.client);

	ret = rtmp_client_connect(NULL, "rtmp://host/app/key");
	CU_ASSERT_EQUAL(ret, -EINVAL);
	ret = rtmp_client_connect(fx.client, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Malformed URL: parse_url() fails synchronously, no loop pumping
	 * needed */
	ret = rtmp_client_connect(fx.client, "http://host/app/key");
	CU_ASSERT_EQUAL(ret, -EPROTO);

	rtmp_client_destroy(fx.client);
	pomp_loop_destroy(loop);
}


/*
 * Full loopback connect / send / disconnect flow
 */

static void test_client_connect_send_frames_disconnect(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct client_fixture fx;
	struct loopback_server srv;
	char url[64];
	int ret;

	memset(&fx, 0, sizeof(fx));
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	loopback_server_init(&srv, loop);

	fx.client = rtmp_client_new(loop, &s_client_cbs, &fx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx.client);

	snprintf(url, sizeof(url), "rtmp://127.0.0.1:%u/live/stream", srv.port);
	ret = rtmp_client_connect(fx.client, url);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_TRUE_FATAL(loopback_server_run_until_ready(&srv, 5000));

	CU_ASSERT_TRUE_FATAL(wait_for_state(
		loop, &fx, RTMP_CLIENT_CONN_STATE_CONNECTED, 5000));
	CU_ASSERT_EQUAL(fx.socket_cb_count, 1);

	/* Cover set_socket_txbuf_size() with a real socket (tsock != NULL) */
	ret = rtmp_client_set_socket_txbuf_size(fx.client, 65536);
	CU_ASSERT_EQUAL(ret, 0);

	/* Exercise one call of each send_* entry point on a real, connected
	 * client. Each must be accepted (>= 0: number of waiting frames) */
	ret = rtmp_client_send_metadata(
		fx.client, 0, 1920, 1080, 30.0, 48000, 16);
	CU_ASSERT_TRUE(ret >= 0);

	{
		uint8_t payload[4] = {0xaa, 0xbb, 0xcc, 0xdd};
		ret = rtmp_client_send_packedmetadata(
			fx.client, payload, sizeof(payload), 0, payload);
		CU_ASSERT_TRUE(ret >= 0);
	}
	{
		uint8_t avcc[4] = {0x01, 0x42, 0x00, 0x1f};
		ret = rtmp_client_send_video_avcc(
			fx.client, avcc, sizeof(avcc), avcc);
		CU_ASSERT_TRUE(ret >= 0);
	}
	{
		/* One NALU, size-prefixed, not an IDR (type != 5) */
		uint8_t frame[4 + 2] = {0, 0, 0, 2, 0x41, 0x9a};
		ret = rtmp_client_send_video_frame(
			fx.client, frame, sizeof(frame), 1000, frame);
		CU_ASSERT_TRUE(ret >= 0);
	}
	{
		/* IDR NALU: nal_type = 0x65 & 0x1f = 5 -- covers the is_key=1
		 * branch in rtmp_client_send_video_frame() */
		uint8_t idr_frame[4 + 2] = {0, 0, 0, 2, 0x65, 0x00};
		ret = rtmp_client_send_video_frame(fx.client,
						   idr_frame,
						   sizeof(idr_frame),
						   2000,
						   idr_frame);
		CU_ASSERT_TRUE(ret >= 0);
	}
	{
		uint8_t *asc_buf = NULL;
		size_t asc_len = 0;
		int aret;
		struct adef_format fmt = {
			.encoding = ADEF_ENCODING_AAC_LC,
			.channel_count = 2,
			.bit_depth = 16,
			.sample_rate = 48000,
		};
		struct aac_asc asc;

		fmt.aac.data_format = ADEF_AAC_DATA_FORMAT_RAW;
		memset(&asc, 0, sizeof(asc));
		aret = aac_asc_from_adef_format(&fmt, &asc);
		CU_ASSERT_EQUAL_FATAL(aret, 0);
		aret = aac_write_asc(&asc, &asc_buf, &asc_len);
		CU_ASSERT_EQUAL_FATAL(aret, 0);

		ret = rtmp_client_send_audio_specific_config(
			fx.client, asc_buf, asc_len, asc_buf);
		CU_ASSERT_TRUE(ret >= 0);

		free(asc_buf);
	}
	{
		uint8_t audio[4] = {1, 2, 3, 4};
		ret = rtmp_client_send_audio_data(
			fx.client, audio, sizeof(audio), 1000, audio);
		CU_ASSERT_TRUE(ret >= 0);
	}

	/* Cover rtmp_client_flush() with client->stream != NULL */
	ret = rtmp_client_flush(fx.client);
	CU_ASSERT_EQUAL(ret, 0);

	ret = rtmp_client_disconnect(
		fx.client, RTMP_CLIENT_DISCONNECTION_REASON_CLIENT_REQUEST);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(fx.last_state, RTMP_CLIENT_CONN_STATE_DISCONNECTED);
	CU_ASSERT_EQUAL(fx.last_reason,
			RTMP_CLIENT_DISCONNECTION_REASON_CLIENT_REQUEST);

	/* A second disconnect on an already-idle client is a no-op error */
	ret = rtmp_client_disconnect(
		fx.client, RTMP_CLIENT_DISCONNECTION_REASON_CLIENT_REQUEST);
	CU_ASSERT_EQUAL(ret, -EALREADY);

	rtmp_client_destroy(fx.client);
	loopback_server_stop(&srv);
	pomp_loop_destroy(loop);
}


static void test_client_connect_already(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct client_fixture fx;
	struct loopback_server srv;
	char url[64];
	int ret;

	memset(&fx, 0, sizeof(fx));
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	loopback_server_init(&srv, loop);

	fx.client = rtmp_client_new(loop, &s_client_cbs, &fx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx.client);

	snprintf(url, sizeof(url), "rtmp://127.0.0.1:%u/live/stream", srv.port);
	ret = rtmp_client_connect(fx.client, url);
	CU_ASSERT_EQUAL(ret, 0);

	/* Connecting again while already connecting must fail */
	ret = rtmp_client_connect(fx.client, url);
	CU_ASSERT_EQUAL(ret, -EALREADY);

	ret = rtmp_client_disconnect(fx.client,
				     RTMP_CLIENT_DISCONNECTION_REASON_UNKNOWN);
	CU_ASSERT_EQUAL(ret, 0);

	rtmp_client_destroy(fx.client);
	loopback_server_stop(&srv);
	pomp_loop_destroy(loop);
}


/*
 * Regression tests
 */

static void test_client_malformed_server_message_regression(void)
{
	/* Regression test for the amf_msg() NULL-deref fixed in rtmp.c: a
	 * server that replies to "connect" with a malformed AMF message (no
	 * leading string, so amf_get_msg_name() returns NULL) must not
	 * crash the client. The client simply never reaches CONNECTED. */
	struct pomp_loop *loop = pomp_loop_new();
	struct client_fixture fx;
	struct loopback_server srv;
	char url[64];
	int ret;

	memset(&fx, 0, sizeof(fx));
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	loopback_server_init(&srv, loop);
	srv.send_malformed_reply = true;

	fx.client = rtmp_client_new(loop, &s_client_cbs, &fx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx.client);

	snprintf(url, sizeof(url), "rtmp://127.0.0.1:%u/live/stream", srv.port);
	ret = rtmp_client_connect(fx.client, url);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_TRUE_FATAL(loopback_server_run_until_ready(&srv, 5000));

	/* Wait for the server to receive the client's "connect" (and send
	 * its malformed reply back from server_amf_msg_cb()), then pump a
	 * bit longer to let that reply actually reach the client and be
	 * processed by its amf_msg() handler -- this is exactly the call
	 * that used to crash on a NULL amf_get_msg_name() result. */
	CU_ASSERT_TRUE_FATAL(
		pump_while(loop, &amf_count_at_least_done, &srv, 5000));
	pump_for(loop, 1000);

	/* No crash reaching this point is the primary assertion. The
	 * client must not have been fooled into CONNECTED either. */
	CU_ASSERT_NOT_EQUAL(fx.last_state, RTMP_CLIENT_CONN_STATE_CONNECTED);

	ret = rtmp_client_disconnect(fx.client,
				     RTMP_CLIENT_DISCONNECTION_REASON_UNKNOWN);
	CU_ASSERT_EQUAL(ret, 0);

	rtmp_client_destroy(fx.client);
	loopback_server_stop(&srv);
	pomp_loop_destroy(loop);
}


static void test_client_send_video_frame_truncated_regression(void)
{
	/* Regression test for the OOB read fixed in
	 * rtmp_client_send_video_frame(): a buffer too short to contain
	 * even one size-prefixed NALU header must not be read out of
	 * bounds. */
	struct pomp_loop *loop = pomp_loop_new();
	struct client_fixture fx;
	struct loopback_server srv;
	char url[64];
	int ret;
	uint8_t truncated[3] = {0, 0, 0}; /* too short for size(4) + type(1) */

	memset(&fx, 0, sizeof(fx));
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	loopback_server_init(&srv, loop);

	fx.client = rtmp_client_new(loop, &s_client_cbs, &fx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx.client);

	snprintf(url, sizeof(url), "rtmp://127.0.0.1:%u/live/stream", srv.port);
	ret = rtmp_client_connect(fx.client, url);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_TRUE_FATAL(loopback_server_run_until_ready(&srv, 5000));
	CU_ASSERT_TRUE_FATAL(wait_for_state(
		loop, &fx, RTMP_CLIENT_CONN_STATE_CONNECTED, 5000));

	ret = rtmp_client_send_video_frame(
		fx.client, truncated, sizeof(truncated), 0, truncated);
	CU_ASSERT_TRUE(ret >= 0);

	ret = rtmp_client_disconnect(
		fx.client, RTMP_CLIENT_DISCONNECTION_REASON_CLIENT_REQUEST);
	CU_ASSERT_EQUAL(ret, 0);

	rtmp_client_destroy(fx.client);
	loopback_server_stop(&srv);
	pomp_loop_destroy(loop);
}


static void test_client_server_sends_peer_bw(void)
{
	/* peer_bw_changed coverage: the server writes a raw SetPeerBandwidth
	 * chunk after the handshake; the client processes it and must still
	 * reach CONNECTED. */
	struct pomp_loop *loop = pomp_loop_new();
	struct client_fixture fx;
	struct loopback_server srv;
	char url[64];
	int ret;

	memset(&fx, 0, sizeof(fx));
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	loopback_server_init(&srv, loop);
	srv.send_peer_bw = true;

	fx.client = rtmp_client_new(loop, &s_client_cbs_with_peerbw, &fx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx.client);

	snprintf(url, sizeof(url), "rtmp://127.0.0.1:%u/live/stream", srv.port);
	ret = rtmp_client_connect(fx.client, url);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_TRUE_FATAL(loopback_server_run_until_ready(&srv, 5000));
	CU_ASSERT_TRUE_FATAL(wait_for_state(
		loop, &fx, RTMP_CLIENT_CONN_STATE_CONNECTED, 5000));

	/* The server wrote a SetPeerBandwidth chunk, so the non-NULL callback
	 * branch in peer_bw_changed() (rtmp.c) must have fired. */
	CU_ASSERT_TRUE(fx.peer_bw_count > 0);
	CU_ASSERT_EQUAL(fx.last_peer_bw, 1000000);

	ret = rtmp_client_disconnect(
		fx.client, RTMP_CLIENT_DISCONNECTION_REASON_CLIENT_REQUEST);
	CU_ASSERT_EQUAL(ret, 0);

	rtmp_client_destroy(fx.client);
	loopback_server_stop(&srv);
	pomp_loop_destroy(loop);
}


static void test_client_server_refused(void)
{
	/* handle_error + server_error_to_client_disconnection_reason +
	 * call_rtmp_client_disconnect coverage: the server responds to
	 * "publish" with _error + NetConnection.Connect.Rejected; the client
	 * must disconnect asynchronously with reason REFUSED. */
	struct pomp_loop *loop = pomp_loop_new();
	struct client_fixture fx;
	struct loopback_server srv;
	char url[64];
	int ret;

	memset(&fx, 0, sizeof(fx));
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	loopback_server_init(&srv, loop);
	srv.publish_response = PUBLISH_ERROR_REJECTED;

	fx.client = rtmp_client_new(loop, &s_client_cbs, &fx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx.client);

	snprintf(url, sizeof(url), "rtmp://127.0.0.1:%u/live/stream", srv.port);
	ret = rtmp_client_connect(fx.client, url);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_TRUE_FATAL(loopback_server_run_until_ready(&srv, 5000));
	CU_ASSERT_TRUE_FATAL(wait_for_state(
		loop, &fx, RTMP_CLIENT_CONN_STATE_DISCONNECTED, 5000));
	CU_ASSERT_EQUAL(fx.last_reason,
			RTMP_CLIENT_DISCONNECTION_REASON_REFUSED);

	rtmp_client_destroy(fx.client);
	loopback_server_stop(&srv);
	pomp_loop_destroy(loop);
}


static void test_client_dns_timeout(void)
{
	/* dns_timer_cb coverage: connect with a valid URL format (DNS won't
	 * fire because we trigger the timer manually before pumping the loop),
	 * then call the test hook that invokes dns_timer_cb directly.  The
	 * client must disconnect with TIMEOUT. */
	struct pomp_loop *loop = pomp_loop_new();
	struct client_fixture fx;
	int ret;

	memset(&fx, 0, sizeof(fx));
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	fx.client = rtmp_client_new(loop, &s_client_cbs, &fx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx.client);

	/* Connect to a hostname -- do NOT pump the loop; the timer fires
	 * synchronously before the DNS callback. */
	ret = rtmp_client_connect(fx.client,
				  "rtmp://some.nonexistent.host/app/key");
	CU_ASSERT_EQUAL(ret, 0);

	/* Directly invoke dns_timer_cb via the TARGET_TEST hook */
	rtmp_client_trigger_dns_timeout_for_test(fx.client);

	CU_ASSERT_EQUAL(fx.last_state, RTMP_CLIENT_CONN_STATE_DISCONNECTED);
	CU_ASSERT_EQUAL(fx.last_reason,
			RTMP_CLIENT_DISCONNECTION_REASON_TIMEOUT);

	/* Pump briefly to drain any leftover async events */
	pump_for(loop, 100);

	rtmp_client_destroy(fx.client);
	pomp_loop_destroy(loop);
}


/* Helper: wait for disconnection due to a specific server response.
 * The server must already be configured (publish_response set) before
 * calling this. */
static void
run_publish_failure_test(enum loopback_publish_response response,
			 enum rtmp_client_disconnection_reason expected_reason)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct client_fixture fx;
	struct loopback_server srv;
	char url[64];
	int ret;

	memset(&fx, 0, sizeof(fx));
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	loopback_server_init(&srv, loop);
	srv.publish_response = response;

	fx.client = rtmp_client_new(loop, &s_client_cbs, &fx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx.client);

	snprintf(url, sizeof(url), "rtmp://127.0.0.1:%u/live/stream", srv.port);
	ret = rtmp_client_connect(fx.client, url);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_TRUE_FATAL(loopback_server_run_until_ready(&srv, 5000));
	CU_ASSERT_TRUE_FATAL(wait_for_state(
		loop, &fx, RTMP_CLIENT_CONN_STATE_DISCONNECTED, 5000));
	CU_ASSERT_EQUAL(fx.last_reason, expected_reason);

	rtmp_client_destroy(fx.client);
	loopback_server_stop(&srv);
	pomp_loop_destroy(loop);
}


static void test_client_server_already_in_use(void)
{
	/* handle_status_update is_error=true + desc-first lookup in
	 * server_error_to_client_disconnection_reason: server sends onStatus
	 * with level:"error", code:"NetStream.Publish.Denied", and
	 * description:"Stream name is already in use" */
	run_publish_failure_test(
		PUBLISH_ERROR_ALREADY_IN_USE,
		RTMP_CLIENT_DISCONNECTION_REASON_ALREADY_IN_USE);
}


static void test_client_handle_error_no_code(void)
{
	/* handle_error if(!code) branch: _error with level:"status" but no
	 * code property -> INTERNAL_ERROR disconnect */
	run_publish_failure_test(
		PUBLISH_ERROR_NO_CODE,
		RTMP_CLIENT_DISCONNECTION_REASON_INTERNAL_ERROR);
}


static void test_client_handle_error_wrong_code(void)
{
	/* handle_error strcmp branch: _error with level:"status" and a code
	 * that does not match NetStream.Publish.Start -> INTERNAL_ERROR */
	run_publish_failure_test(
		PUBLISH_ERROR_WRONG_CODE,
		RTMP_CLIENT_DISCONNECTION_REASON_INTERNAL_ERROR);
}


static void test_client_handle_error_success_code(void)
{
	/* handle_error happy path: _error with level:"status" and
	 * code:"NetStream.Publish.Start" -> set_state(CONN_READY) -> CONNECTED
	 */
	struct pomp_loop *loop = pomp_loop_new();
	struct client_fixture fx;
	struct loopback_server srv;
	char url[64];
	int ret;

	memset(&fx, 0, sizeof(fx));
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	loopback_server_init(&srv, loop);
	srv.publish_response = PUBLISH_ERROR_SUCCESS_CODE;

	fx.client = rtmp_client_new(loop, &s_client_cbs, &fx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx.client);

	snprintf(url, sizeof(url), "rtmp://127.0.0.1:%u/live/stream", srv.port);
	ret = rtmp_client_connect(fx.client, url);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_TRUE_FATAL(loopback_server_run_until_ready(&srv, 5000));
	CU_ASSERT_TRUE_FATAL(wait_for_state(
		loop, &fx, RTMP_CLIENT_CONN_STATE_CONNECTED, 5000));

	ret = rtmp_client_disconnect(
		fx.client, RTMP_CLIENT_DISCONNECTION_REASON_CLIENT_REQUEST);
	CU_ASSERT_EQUAL(ret, 0);

	rtmp_client_destroy(fx.client);
	loopback_server_stop(&srv);
	pomp_loop_destroy(loop);
}


static void test_client_handle_status_no_code(void)
{
	/* handle_status_update if(!code) branch: onStatus with level:"status"
	 * but no code property -> INTERNAL_ERROR disconnect */
	run_publish_failure_test(
		PUBLISH_STATUS_NO_CODE,
		RTMP_CLIENT_DISCONNECTION_REASON_INTERNAL_ERROR);
}


static void test_client_handle_status_wrong_code(void)
{
	/* handle_status_update strcmp branch: onStatus with level:"status" and
	 * a code that does not match NetStream.Publish.Start -> INTERNAL_ERROR
	 */
	run_publish_failure_test(
		PUBLISH_STATUS_WRONG_CODE,
		RTMP_CLIENT_DISCONNECTION_REASON_INTERNAL_ERROR);
}


static void test_client_server_close_message(void)
{
	/* amf_msg "close" branch: server sends a "close" message in response
	 * to publish -> async_rtmp_client_disconnect(SERVER_REQUEST) */
	run_publish_failure_test(
		PUBLISH_CLOSE, RTMP_CLIENT_DISCONNECTION_REASON_SERVER_REQUEST);
}


static void test_client_bad_s0(void)
{
	/* handle_wait_s0 bad-version branch: server sends S0 = 0 instead of
	 * 3 -> client disconnects with NETWORK_ERROR. */
	struct pomp_loop *loop = pomp_loop_new();
	struct client_fixture fx;
	struct loopback_server srv;
	char url[64];
	int ret;

	memset(&fx, 0, sizeof(fx));
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	loopback_server_init(&srv, loop);
	srv.send_bad_s0 = true;

	fx.client = rtmp_client_new(loop, &s_client_cbs, &fx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx.client);

	snprintf(url, sizeof(url), "rtmp://127.0.0.1:%u/live/stream", srv.port);
	ret = rtmp_client_connect(fx.client, url);
	CU_ASSERT_EQUAL(ret, 0);

	/* Server writes bad S0; server_run returns false (C2 never arrives).
	 * The client sees the bad version byte and disconnects. */
	(void)loopback_server_run_until_ready(&srv, 5000);
	CU_ASSERT_TRUE_FATAL(wait_for_state(
		loop, &fx, RTMP_CLIENT_CONN_STATE_DISCONNECTED, 5000));
	CU_ASSERT_EQUAL(fx.last_reason,
			RTMP_CLIENT_DISCONNECTION_REASON_NETWORK_ERROR);

	rtmp_client_destroy(fx.client);
	loopback_server_stop(&srv);
	pomp_loop_destroy(loop);
}


static void test_client_fragmented_handshake(void)
{
	/* handle_wait_s1 / handle_wait_s2 partial-read branches: server sends
	 * S1 and S2 in two fragments each.  The client must still reach
	 * CONNECTED despite receiving handshake bytes in pieces. */
	struct pomp_loop *loop = pomp_loop_new();
	struct client_fixture fx;
	struct loopback_server srv;
	char url[64];
	int ret;

	memset(&fx, 0, sizeof(fx));
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	loopback_server_init(&srv, loop);
	srv.send_fragmented_handshake = true;

	fx.client = rtmp_client_new(loop, &s_client_cbs, &fx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx.client);

	snprintf(url, sizeof(url), "rtmp://127.0.0.1:%u/live/stream", srv.port);
	ret = rtmp_client_connect(fx.client, url);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_TRUE_FATAL(loopback_server_run_until_ready(&srv, 5000));
	CU_ASSERT_TRUE_FATAL(wait_for_state(
		loop, &fx, RTMP_CLIENT_CONN_STATE_CONNECTED, 5000));

	ret = rtmp_client_disconnect(
		fx.client, RTMP_CLIENT_DISCONNECTION_REASON_CLIENT_REQUEST);
	CU_ASSERT_EQUAL(ret, 0);

	rtmp_client_destroy(fx.client);
	loopback_server_stop(&srv);
	pomp_loop_destroy(loop);
}


static void test_client_chunk_stream_disconnect(void)
{
	/* rtmp_chunk_stream_disconnected + notify_disconnection + watchdog
	 * path: connect to CONNECTED, then trigger the chunk stream watchdog
	 * directly. The chunk stream calls its disconnected callback
	 * synchronously, which calls async_rtmp_client_disconnect(TIMEOUT).
	 * Pump the loop once to let the idle callback fire and verify client
	 * reaches DISCONNECTED. */
	struct pomp_loop *loop = pomp_loop_new();
	struct client_fixture fx;
	struct loopback_server srv;
	char url[64];
	int ret;

	memset(&fx, 0, sizeof(fx));
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	loopback_server_init(&srv, loop);

	fx.client = rtmp_client_new(loop, &s_client_cbs, &fx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx.client);

	snprintf(url, sizeof(url), "rtmp://127.0.0.1:%u/live/stream", srv.port);
	ret = rtmp_client_connect(fx.client, url);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_TRUE_FATAL(loopback_server_run_until_ready(&srv, 5000));
	CU_ASSERT_TRUE_FATAL(wait_for_state(
		loop, &fx, RTMP_CLIENT_CONN_STATE_CONNECTED, 5000));

	/* Trigger the chunk stream watchdog to simulate a stream-level timeout
	 */
	rtmp_client_trigger_chunk_stream_watchdog_for_test(fx.client);

	/* The disconnect is async (idle callback) -- pump the loop */
	CU_ASSERT_TRUE_FATAL(wait_for_state(
		loop, &fx, RTMP_CLIENT_CONN_STATE_DISCONNECTED, 2000));
	CU_ASSERT_EQUAL(fx.last_reason,
			RTMP_CLIENT_DISCONNECTION_REASON_TIMEOUT);

	rtmp_client_destroy(fx.client);
	loopback_server_stop(&srv);
	pomp_loop_destroy(loop);
}


static void test_client_result_unknown_id(void)
{
	/* amf_msg() "_result" unknown-ID branch: the server sends _result(999)
	 * before the real _result(connect_id).  The client must warn and then
	 * still process the real result, eventually reaching CONNECTED. */
	struct pomp_loop *loop = pomp_loop_new();
	struct client_fixture fx;
	struct loopback_server srv;
	char url[64];
	int ret;

	memset(&fx, 0, sizeof(fx));
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	loopback_server_init(&srv, loop);
	srv.send_extra_result_unknown_id = true;

	fx.client = rtmp_client_new(loop, &s_client_cbs, &fx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx.client);

	snprintf(url, sizeof(url), "rtmp://127.0.0.1:%u/live/stream", srv.port);
	ret = rtmp_client_connect(fx.client, url);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_TRUE_FATAL(loopback_server_run_until_ready(&srv, 5000));
	CU_ASSERT_TRUE_FATAL(wait_for_state(
		loop, &fx, RTMP_CLIENT_CONN_STATE_CONNECTED, 5000));

	ret = rtmp_client_disconnect(
		fx.client, RTMP_CLIENT_DISCONNECTION_REASON_CLIENT_REQUEST);
	CU_ASSERT_EQUAL(ret, 0);

	rtmp_client_destroy(fx.client);
	loopback_server_stop(&srv);
	pomp_loop_destroy(loop);
}


static void test_client_unexpected_amf_message(void)
{
	/* amf_msg() "unexpected message" branch: the server sends a "netStatus"
	 * message (unrecognized name) after publish.  The client must warn and
	 * remain CONNECTED. */
	struct pomp_loop *loop = pomp_loop_new();
	struct client_fixture fx;
	struct loopback_server srv;
	char url[64];
	int ret;

	memset(&fx, 0, sizeof(fx));
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	loopback_server_init(&srv, loop);
	srv.send_extra_unknown_amf = true;

	fx.client = rtmp_client_new(loop, &s_client_cbs, &fx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx.client);

	snprintf(url, sizeof(url), "rtmp://127.0.0.1:%u/live/stream", srv.port);
	ret = rtmp_client_connect(fx.client, url);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_TRUE_FATAL(loopback_server_run_until_ready(&srv, 5000));
	CU_ASSERT_TRUE_FATAL(wait_for_state(
		loop, &fx, RTMP_CLIENT_CONN_STATE_CONNECTED, 5000));

	/* Pump briefly to ensure the unexpected "netStatus" message is
	 * processed */
	pump_for(loop, 200);

	CU_ASSERT_EQUAL(fx.last_state, RTMP_CLIENT_CONN_STATE_CONNECTED);

	ret = rtmp_client_disconnect(
		fx.client, RTMP_CLIENT_DISCONNECTION_REASON_CLIENT_REQUEST);
	CU_ASSERT_EQUAL(ret, 0);

	rtmp_client_destroy(fx.client);
	loopback_server_stop(&srv);
	pomp_loop_destroy(loop);
}


static void test_client_dns_failure(void)
{
	/* tskt_resolv_cb error path: trigger DNS failure hook → client must
	 * reach DISCONNECTED with NETWORK_ERROR reason (synchronous path). */
	struct pomp_loop *loop = pomp_loop_new();
	struct client_fixture fx;
	struct loopback_server srv;
	char url[64];
	int ret;

	memset(&fx, 0, sizeof(fx));
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	loopback_server_init(&srv, loop);
	fx.client = rtmp_client_new(loop, &s_client_cbs, &fx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx.client);

	snprintf(url, sizeof(url), "rtmp://127.0.0.1:%u/live/stream", srv.port);
	ret = rtmp_client_connect(fx.client, url);
	CU_ASSERT_EQUAL(ret, 0);

	/* Trigger the DNS failure before the real DNS callback fires */
	rtmp_client_trigger_dns_failure_for_test(fx.client);

	/* rtmp_client_disconnect is called synchronously by the error path */
	CU_ASSERT_EQUAL(fx.last_state, RTMP_CLIENT_CONN_STATE_DISCONNECTED);
	CU_ASSERT_EQUAL(fx.last_reason,
			RTMP_CLIENT_DISCONNECTION_REASON_NETWORK_ERROR);

	rtmp_client_destroy(fx.client);
	loopback_server_stop(&srv);
	pomp_loop_destroy(loop);
}


static void test_client_connection_refused(void)
{
	/* tskt_event_cb POMP_FD_EVENT_ERR + ECONNREFUSED branch: start a
	 * server, stop it before the client connects, then connect the client
	 * to the freed port.  The client must reach DISCONNECTED/REFUSED. */
	struct pomp_loop *loop = pomp_loop_new();
	struct client_fixture fx;
	struct loopback_server srv;
	uint16_t port;
	char url[64];
	int ret;

	memset(&fx, 0, sizeof(fx));
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	loopback_server_init(&srv, loop);
	port = srv.port;
	loopback_server_stop(&srv);

	/* Small pump to let the OS process the socket close */
	pump_for(loop, 50);

	fx.client = rtmp_client_new(loop, &s_client_cbs, &fx);
	CU_ASSERT_PTR_NOT_NULL_FATAL(fx.client);

	snprintf(url, sizeof(url), "rtmp://127.0.0.1:%u/live/stream", port);
	ret = rtmp_client_connect(fx.client, url);
	CU_ASSERT_EQUAL(ret, 0);

	CU_ASSERT_TRUE_FATAL(wait_for_state(
		loop, &fx, RTMP_CLIENT_CONN_STATE_DISCONNECTED, 5000));
	CU_ASSERT_EQUAL(fx.last_reason,
			RTMP_CLIENT_DISCONNECTION_REASON_REFUSED);

	rtmp_client_destroy(fx.client);
	pomp_loop_destroy(loop);
}


CU_TestInfo g_rtmp_test_client[] = {
	{FN("client-new-delete-invalid-args"),
	 &test_client_new_delete_invalid_args},
	{FN("client-fresh-flush-and-txbuf"),
	 &test_client_fresh_flush_and_txbuf},
	{FN("client-conn-state-str"), &test_client_conn_state_str},
	{FN("client-disconnection-reason-str"),
	 &test_client_disconnection_reason_str},
	{FN("client-anonymize-url-more-edges"),
	 &test_client_anonymize_url_more_edges},
	{FN("client-connect-invalid-args"), &test_client_connect_invalid_args},

	{FN("client-connect-send-frames-disconnect"),
	 &test_client_connect_send_frames_disconnect},
	{FN("client-connect-already"), &test_client_connect_already},

	{FN("client-server-sends-peer-bw"), &test_client_server_sends_peer_bw},
	{FN("client-server-refused"), &test_client_server_refused},

	{FN("client-dns-timeout"), &test_client_dns_timeout},
	{FN("client-server-already-in-use"),
	 &test_client_server_already_in_use},
	{FN("client-handle-error-no-code"), &test_client_handle_error_no_code},
	{FN("client-handle-error-wrong-code"),
	 &test_client_handle_error_wrong_code},
	{FN("client-handle-error-success-code"),
	 &test_client_handle_error_success_code},
	{FN("client-handle-status-no-code"),
	 &test_client_handle_status_no_code},
	{FN("client-handle-status-wrong-code"),
	 &test_client_handle_status_wrong_code},
	{FN("client-server-close-message"), &test_client_server_close_message},
	{FN("client-bad-s0"), &test_client_bad_s0},
	{FN("client-fragmented-handshake"), &test_client_fragmented_handshake},

	{FN("client-chunk-stream-disconnect"),
	 &test_client_chunk_stream_disconnect},
	{FN("client-result-unknown-id"), &test_client_result_unknown_id},
	{FN("client-unexpected-amf-message"),
	 &test_client_unexpected_amf_message},

	{FN("client-dns-failure"), &test_client_dns_failure},
	{FN("client-connection-refused"), &test_client_connection_refused},

	{FN("client-malformed-server-message-regression"),
	 &test_client_malformed_server_message_regression},
	{FN("client-send-video-frame-truncated-regression"),
	 &test_client_send_video_frame_truncated_regression},

	CU_TEST_INFO_NULL,
};
