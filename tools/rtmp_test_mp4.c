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
#include <execinfo.h>

#include <errno.h>
#include <libpomp.h>
#include <rtmp.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>

#include "mp4_reader.h"

#define ULOG_TAG rtmp_test
#include <ulog.h>
ULOG_DECLARE_TAG(rtmp_test);

/* Define to 0 to skip RTMP connection & run only locally */
#define DO_SEND 1


struct rtmp_test_ctx {
	int run;
	int rtmp_connected;
	struct pomp_loop *loop;
	struct rtmp_client *rtmp;
	struct mp4_reader *reader;
};


static void socket_cb(int fd, void *userdata)
{
	ULOGI("socket CB(%d)", fd);
}


static void
connection_state(enum rtmp_client_conn_state state,
		 enum rtmp_client_disconnection_reason disconnection_reason,
		 void *userdata)
{
	struct rtmp_test_ctx *ctx = userdata;
	ULOGI("connection state: %s", rtmp_client_conn_state_str(state));

	if (state != RTMP_CLIENT_CONN_STATE_DISCONNECTED)
		ctx->rtmp_connected = 1;

	if (state == RTMP_CLIENT_CONN_STATE_CONNECTED) {
		int ret;
		ULOGI("RTMP connected, start reading mp4 file");
		ret = mp4_start_read(ctx->reader, 1);
		if (ret != 0)
			ULOG_ERRNO("mp4_start_read", -ret);
	} else if (state == RTMP_CLIENT_CONN_STATE_DISCONNECTED &&
		   ctx->rtmp_connected) {
		ULOGI("lost RTMP connection");
		ctx->run = 0;
		pomp_loop_wakeup(ctx->loop);
	}
}


static void peer_bw_changed(uint32_t bandwidth, void *userdata)
{
	ULOGI("peer BW changed to %" PRIu32 " Bytes per second", bandwidth);
}


static void data_unref(uint8_t *data, void *buffer_userdata, void *userdata)
{
	free(buffer_userdata);
}


static const struct rtmp_callbacks rtmp_cbs = {
	.socket_cb = socket_cb,
	.connection_state = connection_state,
	.peer_bw_changed = peer_bw_changed,
	.data_unref = data_unref,
};


static void mp4_config(double duration,
		       int width,
		       int height,
		       double framerate,
		       int audiosamplerate,
		       int audiosamplesize,
		       void *userdata)
{
	int ret;
	struct rtmp_test_ctx *ctx = userdata;

	ULOGW("MP4_CONFIG: [[ duration=%f, res=%dx%d, framerate=%f,"
	      "audio rate=%dHz, audio size=%dbits ]]",
	      duration,
	      width,
	      height,
	      framerate,
	      audiosamplerate,
	      audiosamplesize);

	ret = rtmp_client_send_metadata(ctx->rtmp,
					duration,
					width,
					height,
					framerate,
					audiosamplerate,
					audiosamplesize);
	if (ret != 0)
		ULOG_ERRNO("rtmp_client_send_metadata", -ret);
}


static void mp4_element(uint8_t *buffer,
			size_t len,
			enum mp4_data_type type,
			uint32_t timestamp,
			void *userdata)
{
	int ret = 0;
	uint8_t *buf;
	struct rtmp_test_ctx *ctx = userdata;
	ULOGI("got an element of type %s, len %zu, timestamp %" PRIu32 "ms",
	      mp4_data_type_str(type),
	      len,
	      timestamp);

	buf = malloc(len);
	if (!buf) {
		ret = -ENOMEM;
		ULOG_ERRNO("malloc", -ret);
		return;
	}
	memcpy(buf, buffer, len);

	switch (type) {
	case MP4_AVCC:
		ret = rtmp_client_send_video_avcc(ctx->rtmp, buf, len, buf);
		break;
	case MP4_ASC:
		ret = rtmp_client_send_audio_specific_config(
			ctx->rtmp, buf, len, buf);
		break;
	case MP4_AUDIO:
		ret = rtmp_client_send_audio_data(
			ctx->rtmp, buf, len, timestamp, buf);
		break;
	case MP4_VIDEO:
		ret = rtmp_client_send_video_frame(
			ctx->rtmp, buf, len, timestamp, buf);
		break;
	default:
		ret = -ENOSYS;
	}
	if (ret < 0) {
		free(buf);
		ULOG_ERRNO("send mp4 element", -ret);
	} else if (ret > 0) {
		ULOGI("already %d frames waiting", ret);
	}
}


static void mp4_eof(void *userdata)
{
	struct rtmp_test_ctx *ctx = userdata;
	ULOGI("end of MP4 file");
	ctx->run = 0;
	pomp_loop_wakeup(ctx->loop);
}


static const struct mp4_reader_cbs mp4_cbs = {
	.config_cb = mp4_config,
	.element_cb = mp4_element,
	.eof_cb = mp4_eof,
};


static struct rtmp_test_ctx ctx;


static void sighandler(int signal)
{
	if (ctx.run)
		ctx.run = 0;
	else
		exit(1);
	pomp_loop_wakeup(ctx.loop);
}


static void sighandler_pipe(int signal)
{
	void *array[30];
	size_t size;

	size = backtrace(array, 30);

	/* print out all the frames to stderr */
	fprintf(stderr, "Error: signal %d:\n", signal);
	backtrace_symbols_fd(array, size, STDERR_FILENO);
	exit(1);
}


int main(int argc, char *argv[])
{
	char *url;
	char *mp4;

	int ret;
	int status_code = EXIT_SUCCESS;

	if (argc < 3) {
		ULOGE("usage: %s mp4_file url", argv[0]);
		status_code = EXIT_FAILURE;
		goto exit;
	}

	mp4 = argv[1];
	url = argv[2];

	memset(&ctx, 0x0, sizeof(ctx));
	ctx.run = 1;

	signal(SIGINT, sighandler);
	signal(SIGPIPE, sighandler_pipe);

	ctx.loop = pomp_loop_new();
	if (!ctx.loop) {
		ULOG_ERRNO("pomp_loop_new", ENOMEM);
		status_code = EXIT_FAILURE;
		goto exit;
	}

	ctx.rtmp = rtmp_client_new(ctx.loop, &rtmp_cbs, &ctx);
	if (!ctx.rtmp) {
		ULOG_ERRNO("rtmp_client_new", ENOMEM);
		status_code = EXIT_FAILURE;
		goto exit;
	}

#if DO_SEND
	ret = rtmp_client_connect(ctx.rtmp, url);
	if (ret != 0) {
		ULOG_ERRNO("rtmp_client_connect", -ret);
		status_code = EXIT_FAILURE;
		goto exit;
	}
#endif

	ctx.reader = mp4_open_file(mp4, ctx.loop, &mp4_cbs, &ctx);
	if (!ctx.reader) {
		status_code = EXIT_FAILURE;
		goto exit;
	}

#if !DO_SEND
	mp4_start_read(ctx.reader, 0);
#endif

	ULOGI("starting loop");
	while (ctx.run)
		pomp_loop_wait_and_process(ctx.loop, -1);
	ULOGI("ending loop");

	ret = rtmp_client_disconnect(
		ctx.rtmp, RTMP_CLIENT_DISCONNECTION_REASON_CLIENT_REQUEST);
	if (ret != 0) {
		ULOG_ERRNO("rtmp_client_disconnect", -ret);
		status_code = EXIT_FAILURE;
	}
exit:
	mp4_close_file(ctx.reader);

	if (ctx.rtmp)
		rtmp_client_destroy(ctx.rtmp);
	if (ctx.loop) {
		ret = pomp_loop_destroy(ctx.loop);
		if (ret != 0) {
			ULOG_ERRNO("pomp_loop_destroy", -ret);
			status_code = EXIT_FAILURE;
		}
	}

	return status_code;
}
