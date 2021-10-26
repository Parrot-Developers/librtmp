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
#ifndef _MP4_READER_H_
#define _MP4_READER_H_

#include <inttypes.h>
#include <stdio.h>

struct pomp_loop;
struct pomp_timer;

enum mp4_data_type {
	MP4_AVCC,
	MP4_ASC,
	MP4_VIDEO,
	MP4_AUDIO,
};

struct mp4_reader_cbs {
	void (*config_cb)(double duration,
			  int width,
			  int height,
			  double framerate,
			  int audiosamplerate,
			  int audiosamplesize,
			  void *userdata);
	void (*element_cb)(uint8_t *buffer,
			   size_t len,
			   enum mp4_data_type type,
			   uint32_t timestamp,
			   void *userdata);
	void (*eof_cb)(void *userdata);
};

struct mp4_reader;

struct mp4_reader *mp4_open_file(const char *path,
				 struct pomp_loop *loop,
				 const struct mp4_reader_cbs *cbs,
				 void *userdata);
void mp4_close_file(struct mp4_reader *r);

int mp4_start_read(struct mp4_reader *r, int loop);

const char *mp4_data_type_str(enum mp4_data_type type);

#endif /* _MP4_READER_H_ */
