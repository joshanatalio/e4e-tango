# e4e-tango
Project files for the E4E Tango Projects

Before compiling, you should download and compile these libraries:
Android NDK (r10d)
Boost for Android (> 1.53)
PCL for Android (1.6.0)
EIGEN for Android (???)
(For the above libraries, follow these instructions: http://www.vtk.org/Wiki/VES/Point_Cloud_Library
from this repository: https://github.com/patmarion/pcl-superbuild
with these modifications: http://stackoverflow.com/questions/29266543/building-pcl-for-android/29398845#29398845)
OpenCV for Android (http://sourceforge.net/projects/opencvlibrary/files/opencv-android/2.4.10/)

You will also need to set the environment variables: 

ANDROID_NDK_ROOT (and NDK_ROOT to be safe)
ANDROID_SDK_ROOT (and SDK_ROOT to be safe)
TANGO_ROOT
OPENCV_ROOT 
PCL_ROOT (Comes with the pcl-superbuild above)
BOOST_ROOT (Comes with the pcl-superbuild above)
EIGEN_ROOT (Comes with the pcl-superbuild above)

My .bashrc looks like this:
export ANDROID_SDK_ROOT=/home/drichmond/Android/Sdk/
export ANDROID_NDK_ROOT=/home/drichmond/Android/android-ndk-r10d
export TANGO_ROOT=/home/drichmond/Research/repositories/git/tango-examples-c/
export OPENCV_ROOT=/home/drichmond/Research/repositories/sourceforge/OpenCV-android-sdk/
export PCL_ROOT=/home/drichmond/Research/repositories/git/pcl-superbuild/CMakeExternals/Install/pcl-android/
export BOOST_ROOT=/home/drichmond/Research/repositories/git/pcl-superbuild/CMakeExternals/Source/boost/
export EIGEN_ROOT=/home/drichmond/Research/repositories/git/pcl-superbuild/CMakeExternals/Source/eigen/
