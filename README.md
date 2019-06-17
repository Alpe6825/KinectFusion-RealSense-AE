# KinectFusion-RealSense-AE

<img src="https://i.ibb.co/hWsvV1j/Unbenannt.png" alt="Unbenannt" border="0" width="500" />


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
    
## DLLs:

Copy the follwing Dlls to the folder, which contains the .exe.

- opencv_calib3d410.dll
- opencv_core410.dll
- opencv_features2d410.dll
- opencv_flann410.dll
- opencv_highgui410.dll
- opencv_imgcodecs410.dll
- opencv_imgproc410.dll
- opencv_rgbd410.dll
- opencv_videoio410.dll
- realsense2.dll


#
Based on:
https://qiita.com/UnaNancyOwen/items/680e56e7bacddce2eb52
