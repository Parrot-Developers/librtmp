#include "mp4_reader.h"

#include <errno.h>
#include <libmp4.h>
#include <libpomp.h>

#define ULOG_TAG mp4_reader
#include <ulog.h>
ULOG_DECLARE_TAG(mp4_reader);

static uint8_t dummy_audio_specific_config[] = {0x12, 0x10, 0x56, 0xe5, 0x00};
static uint8_t dummy_audio_sample[] = {0x21, 0x10, 0x04, 0x60, 0x8c, 0x1c};
static int dummy_audiosamplerate = 44100;
static int dummy_audiosamplesize = 16;

struct mp4_reader {
	struct pomp_loop *loop;
	struct mp4_demux *demux;
	struct mp4_reader_cbs cbs;
	void *userdata;
	struct pomp_timer *video_timer;
	struct pomp_timer *audio_timer;

	int video_track;
	int video_track_id;
	uint32_t video_track_timescale;
	int audio_track;
	int audio_track_id;
	uint32_t audio_track_timescale;
	int dummy_audio;
	uint32_t dummy_ts;

	uint8_t *video_sample_buffer;
	size_t vsb_len;

	uint8_t *audio_sample_buffer;
	size_t asb_len;

	int do_loop;
	uint32_t loop_ts;
};

const char *mp4_data_type_str(enum mp4_data_type type)
{
	switch (type) {
	case MP4_AVCC:
		return "Avcc";
	case MP4_ASC:
		return "AudioSpecificConfig";
	case MP4_VIDEO:
		return "Video";
	case MP4_AUDIO:
		return "Audio";
	default:
		return "UNKNOWN";
	}
}

static void video_timer_cb(struct pomp_timer *timer, void *userdata)
{
	int ret;
	struct mp4_reader *r = userdata;
	struct mp4_track_sample ts;
	uint32_t frame_ts;
	uint32_t dts_ms, next_dts_ms;

	ret = mp4_demux_get_track_sample(r->demux,
					 r->video_track_id,
					 1,
					 r->video_sample_buffer,
					 r->vsb_len,
					 NULL,
					 0,
					 &ts);
	if (ret != 0) {
		ULOG_ERRNO("mp4_demux_get_track_next_sample", -ret);
		return;
	}

	dts_ms = mp4_sample_time_to_usec(ts.dts, r->video_track_timescale) /
		 1000;
	frame_ts = dts_ms + r->loop_ts;
	r->cbs.element_cb(r->video_sample_buffer,
			  ts.size,
			  MP4_VIDEO,
			  frame_ts,
			  r->userdata);

	if (ts.next_dts == 0) {
		if (r->do_loop) {
			mp4_demux_seek(r->demux, 0, 1);
			r->loop_ts = frame_ts + 33;
			pomp_timer_set(r->video_timer, 33);
			pomp_timer_set(r->audio_timer, 23);
		} else {
			r->cbs.eof_cb(r->userdata);
		}
		return;
	}
	next_dts_ms =
		mp4_sample_time_to_usec(ts.next_dts, r->video_track_timescale) /
		1000;

	pomp_timer_set(r->video_timer, next_dts_ms - dts_ms);
}

static void audio_timer_cb(struct pomp_timer *timer, void *userdata)
{
	int ret;
	struct mp4_reader *r = userdata;
	struct mp4_track_sample ts;
	uint32_t frame_ts;
	uint32_t dts_ms, next_dts_ms;

	if (r->dummy_audio) {
		r->cbs.element_cb(dummy_audio_sample,
				  sizeof(dummy_audio_sample),
				  MP4_AUDIO,
				  r->dummy_ts,
				  r->userdata);
		r->dummy_ts += 23;
		pomp_timer_set(r->audio_timer, 23);
		return;
	}

	ret = mp4_demux_get_track_sample(r->demux,
					 r->audio_track_id,
					 1,
					 r->audio_sample_buffer,
					 r->asb_len,
					 NULL,
					 0,
					 &ts);
	if (ret != 0) {
		ULOG_ERRNO("mp4_demux_get_track_next_sample", -ret);
		return;
	}

	dts_ms = mp4_sample_time_to_usec(ts.dts, r->audio_track_timescale) /
		 1000;
	frame_ts = dts_ms + r->loop_ts;
	r->cbs.element_cb(r->video_sample_buffer,
			  ts.size,
			  MP4_AUDIO,
			  frame_ts,
			  r->userdata);

	if (ts.next_dts == 0)
		return;
	next_dts_ms =
		mp4_sample_time_to_usec(ts.next_dts, r->audio_track_timescale) /
		1000;

	pomp_timer_set(r->audio_timer, next_dts_ms - dts_ms);
}

struct mp4_reader *mp4_open_file(const char *path,
				 struct pomp_loop *loop,
				 const struct mp4_reader_cbs *cbs,
				 void *userdata)
{
	int ret;
	struct mp4_reader *r;
	int track_count;
	int track;
	int found_video = 0, found_audio = 0;

	if (!path || !loop || !cbs || !cbs->config_cb || !cbs->element_cb ||
	    !cbs->eof_cb) {
		ULOG_ERRNO("mp4_open_file", EINVAL);
		return NULL;
	}

	r = calloc(1, sizeof(*r));
	if (!r) {
		ULOG_ERRNO("calloc", ENOMEM);
		return NULL;
	}

	r->loop = loop;
	r->cbs = *cbs;
	r->userdata = userdata;

	r->vsb_len = 512 * 1024;
	r->video_sample_buffer = malloc(r->vsb_len);
	if (!r->video_sample_buffer) {
		ULOG_ERRNO("malloc", ENOMEM);
		goto error;
	}

	r->asb_len = 1024;
	r->audio_sample_buffer = malloc(r->vsb_len);
	if (!r->audio_sample_buffer) {
		ULOG_ERRNO("malloc", ENOMEM);
		goto error;
	}

	ret = mp4_demux_open(path, &r->demux);
	if (ret != 0) {
		ULOG_ERRNO("mp4_demux_open", -ret);
		goto error;
	}

	r->video_timer = pomp_timer_new(loop, video_timer_cb, r);
	if (!r->video_timer) {
		ULOG_ERRNO("pomp_timer_new", ENOMEM);
		goto error;
	}

	r->audio_timer = pomp_timer_new(loop, audio_timer_cb, r);
	if (!r->audio_timer) {
		ULOG_ERRNO("pomp_timer_new", ENOMEM);
		goto error;
	}

	/* search audio/video tracks in MP4 */
	track_count = mp4_demux_get_track_count(r->demux);
	for (track = 0; track < track_count; track++) {
		struct mp4_track_info info;
		mp4_demux_get_track_info(r->demux, track, &info);
		if (info.type == MP4_TRACK_TYPE_VIDEO) {
			found_video = 1;
			ULOGI("Video track with id %d", track);
			r->video_track = track;
			r->video_track_id = info.id;
			r->video_track_timescale = info.timescale;
		}
		if (info.type == MP4_TRACK_TYPE_AUDIO) {
			found_audio = 1;
			r->audio_track = track;
			r->audio_track_id = info.id;
			r->audio_track_timescale = info.timescale;
		}
	}

	if (!found_video) {
		ULOGE("No video track found !");
		goto error;
	}
	if (!found_audio) {
		ULOGW("No audio track, using dummy audio track");
		r->dummy_audio = 1;
	}

	return r;
error:
	mp4_close_file(r);
	return NULL;
}

void mp4_close_file(struct mp4_reader *r)
{
	if (!r)
		return;

	if (r->video_timer) {
		pomp_timer_clear(r->video_timer);
		pomp_timer_destroy(r->video_timer);
	}
	if (r->audio_timer) {
		pomp_timer_clear(r->audio_timer);
		pomp_timer_destroy(r->audio_timer);
	}

	if (r->demux)
		mp4_demux_close(r->demux);

	free(r->audio_sample_buffer);
	free(r->video_sample_buffer);

	free(r);
}

static void mp4_config_idle(void *userdata)
{
	struct mp4_reader *r = userdata;
	struct mp4_media_info m_info;
	struct mp4_track_info a_info;
	struct mp4_track_info v_info;

	struct mp4_video_decoder_config vdc;

	uint8_t avcc[64];
	unsigned int avcc_len = sizeof(avcc);
	uint8_t *asc;
	unsigned int asc_len;

	double duration;
	double framerate;

	mp4_demux_get_media_info(r->demux, &m_info);
	if (r->dummy_audio) {
		a_info.audio_sample_rate = dummy_audiosamplerate;
		a_info.audio_sample_size = dummy_audiosamplesize;
		asc = dummy_audio_specific_config;
		asc_len = sizeof(dummy_audio_specific_config);
	} else {
		mp4_demux_get_track_info(r->demux, r->audio_track, &a_info);
		mp4_demux_get_track_audio_specific_config(
			r->demux, r->audio_track_id, &asc, &asc_len);
	}
	mp4_demux_get_track_info(r->demux, r->video_track, &v_info);
	mp4_demux_get_track_video_decoder_config(
		r->demux, r->video_track_id, &vdc);
	mp4_generate_avc_decoder_config(vdc.avc.sps,
					vdc.avc.sps_size,
					vdc.avc.pps,
					vdc.avc.pps_size,
					avcc,
					&avcc_len);

	/* Duration in seconds, floating point */
	duration = m_info.duration / 1000000.0;
	framerate = v_info.sample_count / duration;

	r->cbs.config_cb(duration,
			 v_info.video_width,
			 v_info.video_height,
			 framerate,
			 a_info.audio_sample_rate,
			 a_info.audio_sample_size,
			 r->userdata);
	r->cbs.element_cb(avcc, (size_t)avcc_len, MP4_AVCC, 0, r->userdata);
	r->cbs.element_cb(asc, (size_t)asc_len, MP4_ASC, 0, r->userdata);

	pomp_timer_set(r->audio_timer, 1);
	pomp_timer_set(r->video_timer, 1);
}

int mp4_start_read(struct mp4_reader *r, int loop)
{
	if (!r)
		return -EINVAL;

	r->do_loop = !!loop;

	pomp_loop_idle_add(r->loop, mp4_config_idle, r);
	return 0;
}
