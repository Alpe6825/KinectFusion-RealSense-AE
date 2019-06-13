# KinectFusion-RealSense-AE

## Dependencies
* OpenCV 4.1 with Contrib
* RealSense SDK 2.0

## Build OpenCV with CMake

| Name   |      value      |
|----------|-------------|
| OPENCV_ENABLE_NONFREE |   ☑ |
| OPENCV_EXTRA_MODULES_PATH |    <path-to-opencv_contrib>/modules   |
| WITH_OPENCL | ☑ |
| WITH_LIBREALSENSE | ☑ |
| LIBREALSENSE_INCLUDE_DIR | *C:/Program Files (x86)/Intel RealSense SDK 2.0/include* |
| LIBREALSENSE_LIBRARIES | *C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/x64/realsense2.lib* (for 64bit) |
    

#
Based on:
https://qiita.com/UnaNancyOwen/items/680e56e7bacddce2eb52
