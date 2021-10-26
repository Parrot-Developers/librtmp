#  librtmp - Real Time Messaging Protocol

_librtmp_ is a C library implementing a Real Time Messaging Protocol (RTMP)
client, able to send a video to a remote RTMP server.

## Dependencies

The library depends on the following Alchemy modules:

* libulog
* libpomp
* libfutils

The _rtmp_test_mp4_ sample also depends on:

* libmp4

## Testing

### Library usage

Two samples are provided:

* _rtmp_test_mp4_
* _rtmp_test_flv_

Both samples take a video file (.mp4 or .flv) and a RTSP uri as their
arguments, and will stream the file in loop to this URI.

For .mp4 files, a dummy empty audio track is used, for .flv files, an
audio track must be present in the file.

## Docs

The library implements a part of the RTMP specification, available at
https://www.adobe.com/devnet/rtmp.html. The library only implements the
client part, publishing a stream to an RTMP server (e.g. Youtube Live)

The library also implements a (partial) AMF0 codec
(https://www.adobe.com/devnet/swf.html) and a basic FLV muxer
(https://www.adobe.com/devnet/f4v.html). A basic FLV demuxer is
implemented in the _rtmp_test_flv_ tool.