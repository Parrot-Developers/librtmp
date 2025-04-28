LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := librtmp
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := RTMP (Real-Time Messaging Protocol) library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_CFLAGS := -DRTMP_API_EXPORTS -D_GNU_SOURCE -fvisibility=hidden -std=gnu99
LOCAL_SRC_FILES := \
	src/rtmp.c \
	src/amf.c \
	src/rtmp_chunk_stream.c
LOCAL_LIBRARIES := \
	libaac \
	libaudio-defs \
	libcrypto \
	libfutils \
	libpomp \
	libtransport-packet \
	libtransport-socket \
	libtransport-tls \
	libulog

include $(BUILD_LIBRARY)


ifeq ("$(TARGET_OS_FLAVOUR)","native")

include $(CLEAR_VARS)

LOCAL_MODULE := rtmp_test_flv
LOCAL_SRC_FILES := \
	tools/rtmp_test_flv.c \
	tools/flv_reader.c
LOCAL_LIBRARIES := librtmp libpomp libulog
include $(BUILD_EXECUTABLE)


include $(CLEAR_VARS)

LOCAL_MODULE := rtmp_test_mp4
LOCAL_SRC_FILES := \
	tools/rtmp_test_mp4.c \
	tools/mp4_reader.c
LOCAL_LIBRARIES := librtmp libpomp libulog libmp4

include $(BUILD_EXECUTABLE)

endif


ifdef TARGET_TEST

include $(CLEAR_VARS)

LOCAL_MODULE := tst-librtmp
LOCAL_CFLAGS += -DTARGET_TEST -D_GNU_SOURCE
LOCAL_SRC_FILES := \
	tests/rtmp_test_utils.c \
	tests/rtmp_test.c
LOCAL_LIBRARIES := \
	libcunit \
	librtmp

include $(BUILD_EXECUTABLE)

endif
