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

# TANGO_ROOT := Should be defined in your ENVIRONMENT
TEGRA_ROOT := /home/drichmond/Android/NVPACK/
OPENCV_ROOT := /home/drichmond/Research/repositories/sourceforge/OpenCV-android-sdk/
#$(TEGRA_ROOT)/OpenCV-2.4.8.2-Tegra-sdk/
CUDA_ROOT := $(TEGRA_ROOT)/cuda-6.5/
PCL_ROOT   := /home/drichmond/Research/repositories/git/pcl-superbuild/CMakeExternals/Install/pcl-android/
BOOST_ROOT := /home/drichmond/Research/repositories/git/pcl-superbuild/CMakeExternals/Source/boost/
EIGEN_ROOT := /home/drichmond/Research/repositories/git/pcl-superbuild/CMakeExternals/Source/eigen/

# LibTango Declarations
include $(CLEAR_VARS)
LOCAL_MODULE := libtango-prebuilt
LOCAL_SRC_FILES := $(TANGO_ROOT)/tango_client_api/lib/libtango_client_api.so
LOCAL_EXPORT_C_INCLUDES := $(TANGO_ROOT)/tango_client_api/include
include $(PREBUILT_SHARED_LIBRARY)


include $(CLEAR_VARS)
OPENCV_CAMERA_MODULES := on
OPENCV_INSTALL_MODULES := on
include $(OPENCV_ROOT)/sdk/native/jni/OpenCV.mk

LOCAL_MODULE    := libpoint_cloud_jni_example
LOCAL_SHARED_LIBRARIES += libtango-prebuilt
LOCAL_CFLAGS    := -std=c++11 -fexceptions

LOCAL_C_INCLUDES := $(TANGO_ROOT)/tango-service-sdk/include/ \
                    $(TANGO_ROOT)/tango-gl/include \
                    $(TANGO_ROOT)/third-party/glm/ \
                    $(OPENCV_ROOT)/sdk/native/jni/include/ \
                    $(OPENCV_ROOT)/sdk/native/jni/include/ \
                    $(BOOST_ROOT)/boost_1_45_0/ \
                    $(PCL_ROOT)/include/pcl-1.6/ \
                    $(EIGEN_ROOT)/ \
                    $(CUDA_ROOT)/include/

LOCAL_SRC_FILES := tango_data.cpp \
                   tango_pointcloud.cpp \
                   pointcloud.cpp \
                   $(TANGO_ROOT)/tango-gl/axis.cpp \
                   $(TANGO_ROOT)/tango-gl/camera.cpp \
                   $(TANGO_ROOT)/tango-gl/frustum.cpp \
                   $(TANGO_ROOT)/tango-gl/grid.cpp \
                   $(TANGO_ROOT)/tango-gl/transform.cpp \
                   $(TANGO_ROOT)/tango-gl/util.cpp

LOCAL_LDLIBS    := -llog -lGLESv2 -L$(SYSROOT)/usr/lib
# Boost Libraries
LOCAL_LDLIBS    += -L$(BOOST_ROOT)/libs/armeabi-v7a/ -lboost_thread
# PCL Libraries
LOCAL_LDLIBS    += -L$(PCL_ROOT)/lib/ -lpcl_io -lpcl_io_ply -lpcl_common -lpcl_octree
# CUDA Libraries
LOCAL_LDLIBS    += -L$(CUDA_ROOT)/targets/armv7-linux-androideabi/lib/ -lcudart_static
include $(BUILD_SHARED_LIBRARY)

$(call import-module,android/native_app_glue)
