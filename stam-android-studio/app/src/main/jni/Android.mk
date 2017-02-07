LOCAL_PATH := $(call my-dir)
PROJECT_ROOT_FROM_JNI:= ../../../../..
PROJECT_ROOT:= $(call my-dir)/../../../../..

LOCAL_ALLOW_UNDEFINED_SYMBOLS := true

include $(CLEAR_VARS)

LOCAL_MODULE    := libstam_tango
LOCAL_SHARED_LIBRARIES := libtango_client_api
LOCAL_SHARED_LIBRARIES += libtango_support_api
LOCAL_CFLAGS    := -std=c++11

LOCAL_C_INCLUDES += $(LOCAL_PATH)/header/stam
LOCAL_C_INCLUDES += $(LOCAL_PATH)/header/tango
LOCAL_C_INCLUDES += $(LOCAL_PATH)/include
LOCAL_C_INCLUDES += $(PROJECT_ROOT)/tango_gl/include
LOCAL_C_INCLUDES += $(PROJECT_ROOT)/third_party/glm/
					
LOCAL_LDLIBS    := -llog -lGLESv2 -L$(SYSROOT)/usr/lib
LOCAL_LDLIBS    += -lstdc++ # C++ stardard library
LOCAL_LDLIBS    += -llog -lz -ljnigraphics -ldl -landroid

LOCAL_STATIC_LIBRARIES += ceres
LOCAL_STATIC_LIBRARIES += opencv_videoio
LOCAL_STATIC_LIBRARIES += opencv_video
LOCAL_STATIC_LIBRARIES += opencv_imgcodecs
LOCAL_STATIC_LIBRARIES += libjasper
LOCAL_STATIC_LIBRARIES += libIlmImf
LOCAL_STATIC_LIBRARIES += libwebp
LOCAL_STATIC_LIBRARIES += libtiff
LOCAL_STATIC_LIBRARIES += libpng
LOCAL_STATIC_LIBRARIES += libjpeg
LOCAL_STATIC_LIBRARIES += opencv_xfeatures2d
LOCAL_STATIC_LIBRARIES += opencv_features2d
LOCAL_STATIC_LIBRARIES += opencv_calib3d
LOCAL_STATIC_LIBRARIES += opencv_flann
LOCAL_STATIC_LIBRARIES += opencv_objdetect
LOCAL_STATIC_LIBRARIES += opencv_imgproc
LOCAL_STATIC_LIBRARIES += opencv_highgui
LOCAL_STATIC_LIBRARIES += opencv_core
LOCAL_STATIC_LIBRARIES += opencv_hal
LOCAL_STATIC_LIBRARIES += tbb

LOCAL_SRC_FILES := source/tango/augmented_reality_app.cc \
                   source/tango/jni_interface.cc \
                   source/tango/pose_data.cc \
                   source/tango/yuv_drawable.cc \
                   source/tango/tango_event_data.cc \
                   source/stam/STAM.cpp \
                   source/stam/types.cpp \
                   source/stam/utils.cpp \
                   source/stam/VideoSource.cpp \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/axis.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/bounding_box.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/camera.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/conversions.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/drawable_object.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/frustum.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/gesture_camera.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/grid.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/goal_marker.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/line.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/mesh.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/shaders.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/trace.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/transform.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/util.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango_gl/video_overlay.cc

LOCAL_C_INCLUDES += $(PROJECT_ROOT)/eigen \
					$(PROJECT_ROOT)/ceres-solver/include \
					$(PROJECT_ROOT)/ceres-solver/config \
					$(PROJECT_ROOT)/ceres-solver/internal/ceres/miniglog \
					$(PROJECT_ROOT)/ceres-solver/internal

LOCAL_CPPFLAGS += -ffunction-sections -fdata-sections
LOCAL_LDFLAGS += -Wl,--gc-sections

include $(BUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE    := ceres
LOCAL_SRC_FILES := libs/3rdparty/$(TARGET_ARCH_ABI)/libceres.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_videoio
LOCAL_SRC_FILES := libs/$(TARGET_ARCH_ABI)/libopencv_videoio.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_video
LOCAL_SRC_FILES := libs/$(TARGET_ARCH_ABI)/libopencv_video.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_imgcodecs
LOCAL_SRC_FILES := libs/$(TARGET_ARCH_ABI)/libopencv_imgcodecs.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libjasper
LOCAL_SRC_FILES := libs/3rdparty/$(TARGET_ARCH_ABI)/liblibjasper.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := IlmImf
LOCAL_SRC_FILES := libs/3rdparty/$(TARGET_ARCH_ABI)/libIlmImf.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libwebp
LOCAL_SRC_FILES := libs/3rdparty/$(TARGET_ARCH_ABI)/liblibwebp.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libtiff
LOCAL_SRC_FILES := libs/3rdparty/$(TARGET_ARCH_ABI)/liblibtiff.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libpng
LOCAL_SRC_FILES := libs/3rdparty/$(TARGET_ARCH_ABI)/liblibpng.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libjpeg
LOCAL_SRC_FILES := libs/3rdparty/$(TARGET_ARCH_ABI)/liblibjpeg.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_xfeatures2d
LOCAL_SRC_FILES := libs/$(TARGET_ARCH_ABI)/libopencv_xfeatures2d.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_features2d
LOCAL_SRC_FILES := libs/$(TARGET_ARCH_ABI)/libopencv_features2d.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_calib3d
LOCAL_SRC_FILES := libs/$(TARGET_ARCH_ABI)/libopencv_calib3d.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_flann
LOCAL_SRC_FILES := libs/$(TARGET_ARCH_ABI)/libopencv_flann.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_objdetect
LOCAL_SRC_FILES := libs/$(TARGET_ARCH_ABI)/libopencv_objdetect.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_imgproc
LOCAL_SRC_FILES := libs/$(TARGET_ARCH_ABI)/libopencv_imgproc.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_highgui
LOCAL_SRC_FILES := libs/$(TARGET_ARCH_ABI)/libopencv_highgui.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_core
LOCAL_SRC_FILES := libs/$(TARGET_ARCH_ABI)/libopencv_core.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_hal
LOCAL_SRC_FILES := libs/$(TARGET_ARCH_ABI)/libopencv_hal.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := tbb
LOCAL_SRC_FILES := libs/3rdparty/$(TARGET_ARCH_ABI)/libtbb.a
include $(PREBUILT_STATIC_LIBRARY)

$(call import-add-path, $(PROJECT_ROOT))
$(call import-module, tango_client_api)
$(call import-module, tango_support_api)