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
#ifndef _RTMP_CHUNK_STREAM_H_
#define _RTMP_CHUNK_STREAM_H_

#include <stdbool.h>
#include <transport-socket/tskt.h>

#include "rtmp_internal.h"

struct rtmp_chunk_stream;
struct pomp_loop;


struct rtmp_chunk_cbs {
	void (*peer_bw_changed)(uint32_t bandwidth, void *userdata);
	void (*amf_msg)(struct rtmp_buffer *data, void *userdata);
	void (*data_sent)(uint8_t *data, void *data_userdata, void *userdata);
	void (*disconnected)(void *userdata,
			     enum rtmp_client_disconnection_reason reason);
};


struct rtmp_chunk_stream *new_chunk_stream(struct pomp_loop *loop,
					   struct tskt_socket *tsock,
					   const struct rtmp_chunk_cbs *cbs,
					   void *userdata);

int set_chunk_size(struct rtmp_chunk_stream *stream, uint32_t chunk_size);

int send_metadata(struct rtmp_chunk_stream *stream,
		  struct rtmp_buffer *data,
		  uint32_t timestamp,
		  int internal,
		  void *frame_userdata);
int send_video_frame(struct rtmp_chunk_stream *stream,
		     struct rtmp_buffer *frame,
		     uint32_t timestamp,
		     int is_meta,
		     int is_key,
		     void *frame_userdata);
int send_audio_data(struct rtmp_chunk_stream *stream,
		    struct rtmp_buffer *data,
		    uint32_t timestamp,
		    int is_meta,
		    void *frame_userdata);
int send_amf_message(struct rtmp_chunk_stream *stream, struct rtmp_buffer *msg);

int flush_chunk_stream(struct rtmp_chunk_stream *stream);

int delete_chunk_stream(struct rtmp_chunk_stream *stream);

int store_message_stream_id(struct rtmp_chunk_stream *stream, uint32_t msid);


#endif /* _RTMP_CHUNK_STREAM_H_ */
