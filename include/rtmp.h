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
#ifndef _RTMP_H_
#define _RTMP_H_

#include <stdint.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** To be used for all public API */
#ifdef RTMP_API_EXPORTS
#	ifdef _WIN32
#		define RTMP_API __declspec(dllexport)
#	else /* !_WIN32 */
#		define RTMP_API __attribute__((visibility("default")))
#	endif /* !_WIN32 */
#else /* !RTP_API_EXPORTS */
#	define RTMP_API
#endif /* !RTP_API_EXPORTS */

/* Forward declarations */
struct rtmp_client;
struct pomp_loop;

/** Connection state */
enum rtmp_connection_state {
	RTMP_DISCONNECTED, /**< Client is disconnected */
	RTMP_CONNECTING, /**< Connection in progress */
	RTMP_CONNECTED, /**< Client is connected */
};

/**
 * Gets the string description of a connection state.
 *
 * @param state : state to convert.
 *
 * @return string description of the connection state.
 */
RTMP_API const char *
rtmp_connection_state_to_string(enum rtmp_connection_state state);


/**
 * Callbacks structure for an rtmp_client.
 */
struct rtmp_callbacks {
	/**
	 * Callback called on socket creation. (optional)
	 *
	 * @param fd : the created socket file descriptor.
	 * @param userdata : userdata passed in rtmp_client_new.
	 */
	void (*socket_cb)(int fd, void *userdata);

	/**
	 * Callback called when the connection state changes. (mandatory)
	 *
	 * When this callback is called with RTMP_CONNECTED, it is safe to call
	 * the rtmp_client_send_xxx() functions.
	 * The rtmp_client won't try to automatically reconnect when
	 * disconnected.
	 *
	 * @param state : the client connection state.
	 * @param userdata : userdata passed in rtmp_client_new.
	 */
	void (*connection_state)(enum rtmp_connection_state state,
				 void *userdata);

	/**
	 * Callback called when the peer (the rtmp server) sends a new bandwidth
	 * limit message.
	 *
	 * This bandwidth limit may be higher than the actual network capacity,
	 * but is an upper bandwidth limit from the receiver.
	 *
	 * @param bandwidth : the max bandwidth supported by the server (B/s).
	 * @param userdata : userdata passed in rtmp_client_new.
	 */
	void (*peer_bw_changed)(uint32_t bandwidth, void *userdata);

	/**
	 * Callback called when a metadata/frame/audio buffer is fully sent and
	 * can be reused. (mandatory)
	 *
	 * @param data : the data which is no longer needed by rtmp_client.
	 * @param buffer_userdata : user data passed along the buffer in a
	 * rtmp_client_send_xxx() function.
	 * @param userdata : userdata passed in rtmp_client_new.
	 */
	void (*data_unref)(uint8_t *data,
			   void *buffer_userdata,
			   void *userdata);
};

/**
 * Creates a new rtmp_client.
 *
 * @param loop : pomp_loop which will be used by the client. Must be externally
 * provided & run (no fallback to an internal loop).
 * @param cbs : the callbacks that the client will use to communicate with the
 * application. Some callbacks are mandatory.
 * @param userdata : an opaque piece of data passed back to the callbacks.
 *
 * @return rtmp_client structure or NULL in case of error.
 */
RTMP_API struct rtmp_client *rtmp_client_new(struct pomp_loop *loop,
					     const struct rtmp_callbacks *cbs,
					     void *userdata);

/**
 * Destroys an rtmp_client.
 *
 * If the RTMP client is still connected, it is disconnected first.
 *
 * @param client : the rtmp_client to destroy.
 */
RTMP_API void rtmp_client_destroy(struct rtmp_client *client);

/**
 * Connects an rtmp_client to the given rtmp URL.
 *
 * If the client is already connected or connecting, an error is returned.
 *
 * @param client : the rtmp_client to connect.
 * @param url : the rtmp url to connect to.
 *
 * @return 0 on success, negative errno on error.
 */
RTMP_API int rtmp_client_connect(struct rtmp_client *client, const char *url);

/**
 * Disconnects an rtmp_client.
 *
 * If used when the client is in RTMP_CONNECTING state, this function will abort
 * the connection.
 *
 * @param client : the rtmp_client to disconnect.
 *
 * @return 0 on success, negative errno on error.
 */
RTMP_API int rtmp_client_disconnect(struct rtmp_client *client);


/**
 * Sends a metadata packet to the server.
 *
 * This function must be called on an RTMP_CONNECTED client.
 *
 * @param client : the connected rtmp_client which will send the data.
 * @param duration : media duration in seconds (0 for unlimited duration).
 * @param width : video width in pixels.
 * @param height : video height in pixels.
 * @param framerate : video nominal framerate. If 0, "29.97" will be sent.
 * @param audio_sample_rate : audio sample rate, in Hz.
 * @param audio_sample_size : audio sample size, in bits.
 *
 * @return the number of waiting metadata/audio frames on success, negative
 * errno on error.
 */
RTMP_API int rtmp_client_send_metadata(struct rtmp_client *client,
				       double duration,
				       int width,
				       int height,
				       double framerate,
				       int audio_sample_rate,
				       int audio_sample_size);


/**
 * Sends a metadata packet to the server.
 *
 * This function must be called on an RTMP_CONNECTED client.
 *
 * @param client : the connected rtmp_client which will send the data.
 * @param buf : pointer to an AMF0-encoded metadata buffer.
 * @param len : length of the AMF0-encoded metadata buffer.
 * @param timestamp : timestamp of the metadata, in milliseconds, from the rtmp
 * connection.
 * @param frame_userdata : userdata passed back to the data_unref() callback.
 * buf must NOT be changed before being passed to data_unref().
 *
 * @return the number of waiting metadata/audio frames on success, negative
 * errno on error.
 */
RTMP_API int rtmp_client_send_packedmetadata(struct rtmp_client *client,
					     const uint8_t *buf,
					     size_t len,
					     uint32_t timestamp,
					     void *frame_userdata);

/**
 * Sends a video avcC configuration structure to the server.
 *
 * This function must be called on an RTMP_CONNECTED client.
 *
 * @param client : the connected rtmp_client which will send the data.
 * @param buf : pointer to an avcC buffer (See ISO 14496-15, 5.2.4.1.).
 * @param len : length of the avcC buffer.
 * @param frame_userdata : userdata passed back to the data_unref() callback.
 * buf must NOT be changed before being passed to data_unref().
 *
 * @return the number of waiting frames on success, negative errno on error.
 */
RTMP_API int rtmp_client_send_video_avcc(struct rtmp_client *client,
					 const uint8_t *buf,
					 size_t len,
					 void *frame_userdata);

/**
 * Sends a video frame to the server.
 *
 * This function must be called on an RTMP_CONNECTED client.
 *
 * @param client : the connected rtmp_client which will send the data.
 * @param buf : pointer to a video frame.
 * @param len : length of the video buffer.
 * @param timestamp : timestamp of the frame, in milliseconds, from the rtmp
 * connection.
 * @param frame_userdata : userdata passed back to the data_unref() callback.
 * buf must NOT be changed before being passed to data_unref().
 *
 * @return the number of waiting frames on success, negative errno on error.
 */
RTMP_API int rtmp_client_send_video_frame(struct rtmp_client *client,
					  const uint8_t *buf,
					  size_t len,
					  uint32_t timestamp,
					  void *frame_userdata);

/**
 * Sends an AudioSpecifiConfig buffer to the server.
 * (See ISO/IEC 14496-3 1.6.2)
 *
 * This function must be called on an RTMP_CONNECTED client.
 *
 * @param client : the connected rtmp_client which will send the data.
 * @param buf : pointer to an AudioSpecificConfig buffer.
 * @param len : length of the AudioSpecificConfig buffer.
 * @param frame_userdata : userdata passed back to the data_unref() callback.
 * buf must NOT be changed before being passed to data_unref().
 *
 * @return the number of waiting metadata/audio frames on success, negative
 * errno on error.
 */
RTMP_API int rtmp_client_send_audio_specific_config(struct rtmp_client *client,
						    const uint8_t *buf,
						    size_t len,
						    void *frame_userdata);

/**
 * Sends an audio chunk to the server.
 *
 * This function must be called on an RTMP_CONNECTED client.
 *
 * @param client : the connected rtmp_client which will send the data.
 * @param buf : pointer to an audio chunk.
 * @param len : length of the audio buffer.
 * @param timestamp : timestamp of the audio chunk, in milliseconds, from the
 * rtmp connection.
 * @param frame_userdata : userdata passed back to the data_unref() callback.
 * buf must NOT be changed before being passed to data_unref().
 *
 * @return the number of waiting metadata/audio frames on success, negative
 * errno on error.
 */
RTMP_API int rtmp_client_send_audio_data(struct rtmp_client *client,
					 const uint8_t *buf,
					 size_t len,
					 uint32_t timestamp,
					 void *frame_userdata);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _RTMP_H_ */
