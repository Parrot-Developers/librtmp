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
#ifndef _AMF_H_
#define _AMF_H_

#include "rtmp_internal.h"

/** Wrapper for gcc printf attribute */
#ifndef AMF_ATTRIBUTE_FORMAT_PRINTF
#	if defined(__GNUC__) && defined(__MINGW32__) && !defined(__clang__)
#		define AMF_ATTRIBUTE_FORMAT_PRINTF(_x, _y)                    \
			__attribute__((__format__(__gnu_printf__, _x, _y)))
#	elif defined(__GNUC__)
#		define AMF_ATTRIBUTE_FORMAT_PRINTF(_x, _y)                    \
			__attribute__((__format__(__printf__, _x, _y)))
#	else
#		define AMF_ATTRIBUTE_FORMAT_PRINTF(_x, _y)
#	endif
#endif /* !AMF_ATTRIBUTE_FORMAT_PRINTF */


/**
 * Encode an AMF message into the given buffer.
 *
 * Available formatters & associated arg type:
 * - '%f' : Number (double)
 * - '%u' : Boolean (uint8_t)
 * - '%s' : String (const char *).
 *   -> '%s' is also used as property names inside AMF objects
 * - '{': Start of an AMF object. (no arg)
 *     AMF Objects have pairs of properties name/values, which must be both
 *     specified in the format string (e.g. %s%f for a Number property)
 * - '}': End of an AMF object. (no arg)
 * - '[%d': Start of ECMA array (int), with a given element count
 *     ECMA arrays have an element count, followed by this number of name/values
 *     pairs, as for AMF objects.
 * - ']': End of ECMA array. (no arg)
 * - '0': Null (no arg)
 * The following chars are ignored between tokens, and are here to improve
 * readability of format strings:
 * - ',' (comma)
 * - ':' (colon)
 * - ' ' (space)
 * - '\t' (tab)
 * - '\n' (new line)
 */
int amf_encode(struct rtmp_buffer *buf, const char *fmt, ...)
	AMF_ATTRIBUTE_FORMAT_PRINTF(2, 3);

/**
 * Get the message name (first encoded string) and call ID (second encoded
 * Number) from an AMF buffer.
 *
 * As the strings are not null-terminated inside the buffer, this function will
 * allocate a new buffer and copy the string content into it. This buffer must
 * be later freed by a call to 'free()'.
 *
 * In case of an error a NULL pointer is returned.
 *
 * This function uses & updates the ->rd attribute of buf
 */
char *amf_get_msg_name(struct rtmp_buffer *buf, double *id);

/* Low level getters */
int amf_get_number(struct rtmp_buffer *buf, double *value);
int amf_get_boolean(struct rtmp_buffer *buf, uint8_t *value);
int amf_get_string(struct rtmp_buffer *buf,
		   char **string); /* Allocates memory */
int amf_get_property(struct rtmp_buffer *buf,
		     char **key); /* Allocates memory */
int amf_get_object_start(struct rtmp_buffer *buf);
int amf_get_null(struct rtmp_buffer *buf);
int amf_get_object_end(struct rtmp_buffer *buf);

int amf_skip_data(struct rtmp_buffer *buf);

#endif /* _AMF_H_ */
