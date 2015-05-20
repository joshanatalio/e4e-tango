#
# Copyright 2014 Google Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
LOCAL_PATH:= $(call my-dir)
PROJECT_ROOT:= $(call my-dir)/../../../../..
include $(CLEAR_VARS)
OPENCV_CAMERA_MODULES := on
OPENCV_INSTALL_MODULES := on
include $(OPENCV_ROOT)/sdk/native/jni/OpenCV.mk

LOCAL_MODULE    := libpoint_cloud_jni_example
LOCAL_SHARED_LIBRARIES += tango_client_api
LOCAL_CFLAGS    := -std=c++11 -fexceptions

LOCAL_SRC_FILES := tango_data.cpp \
                   tango_pointcloud.cpp \
                   pointcloud.cpp \
                   $(TANGO_ROOT)/tango-gl/axis.cpp \
                   $(TANGO_ROOT)/tango-gl/camera.cpp \
                   $(TANGO_ROOT)/tango-gl/drawable_object.cpp \
                   $(TANGO_ROOT)/tango-gl/frustum.cpp \
                   $(TANGO_ROOT)/tango-gl/grid.cpp \
                   $(TANGO_ROOT)/tango-gl/line.cpp \
                   $(TANGO_ROOT)/tango-gl/shaders.cpp \
                   $(TANGO_ROOT)/tango-gl/transform.cpp \
                   $(TANGO_ROOT)/tango-gl/util.cpp

LOCAL_C_INCLUDES := $(TANGO_ROOT)/tango-gl/include \
                    $(TANGO_ROOT)/third-party/glm/ \
                    $(OPENCV_ROOT)/sdk/native/jni/include/ \
                    $(BOOST_ROOT)/boost_1_45_0/ \
                    $(PCL_ROOT)/include/pcl-1.6/ \
                    $(EIGEN_ROOT)/ \
                    $(UTIL_ROOT)/ \
                    $(TURBO_ROOT)/build/arm_neon/installdir/include/

LOCAL_LDLIBS    := -llog -lGLESv2 -L$(SYSROOT)/usr/lib
# Boost Libraries
LOCAL_LDLIBS    += -L$(BOOST_ROOT)/libs/armeabi-v7a/ -lboost_thread
# PCL Libraries
LOCAL_LDLIBS    += -L$(PCL_ROOT)/lib/ -lpcl_io -lpcl_io_ply -lpcl_common -lpcl_octree
LOCAL_LDLIBS    += -L$(TURBO_ROOT)/build/arm_neon/libs/static/ -lturbojpeg
include $(BUILD_SHARED_LIBRARY)



$(call import-add-path, $(TANGO_ROOT))
$(call import-module,tango_client_api)
