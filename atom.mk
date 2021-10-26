LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := librtmp
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := RTMP (Real-Time Messaging Protocol) library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_CFLAGS := -DRTMP_API_EXPORTS -fvisibility=hidden -std=gnu99
LOCAL_SRC_FILES := \
	src/rtmp.c \
	src/amf.c \
	src/rtmp_chunk_stream.c
LOCAL_LIBRARIES := libfutils libpomp libulog

include $(BUILD_LIBRARY)

ifeq ("$(TARGET_OS_FLAVOUR)","native")
	include $(CLEAR_VARS)
	LOCAL_MODULE := rtmp_test_flv
	LOCAL_SRC_FILES := test/rtmp_test_flv.c test/flv_reader.c
	LOCAL_LIBRARIES := librtmp libpomp libulog
	include $(BUILD_EXECUTABLE)

	include $(CLEAR_VARS)
	LOCAL_MODULE := rtmp_test_mp4
	LOCAL_SRC_FILES := test/rtmp_test_mp4.c test/mp4_reader.c
	LOCAL_LIBRARIES := librtmp libpomp libulog libmp4
	include $(BUILD_EXECUTABLE)
endif
