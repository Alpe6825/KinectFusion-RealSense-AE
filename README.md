# KinectFusion-RealSense-AE

<img src="https://i.ibb.co/hWsvV1j/Unbenannt.png" alt="Unbenannt" border="0" width="500" />


## Dependencies
* OpenCV 4.1 with Contrib
* RealSense SDK 2.0
* PointCloud Library
* OpenNI2

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

<b>OpenCV:</b>
- opencv_calib3d410.dll
- opencv_core410.dll
- opencv_features2d410.dll
- opencv_flann410.dll
- opencv_highgui410.dll
- opencv_imgcodecs410.dll
- opencv_imgproc410.dll
- opencv_rgbd410.dll
- opencv_videoio410.dll

<b>RealSense:</b>
- realsense2.dll

<b>PointCloud Library:</b>
- pcl_common_release.dll
- pcl_io_ply_release.dll
- pcl_io_release.dll
- pcl_kdtree_release.dll
- pcl_octree_release.dll
- pcl_search_release.dll
- pcl_surface_release.dll

<b>OpenNI2:</b>
- OpenNI2.dll

#
Based on:
https://qiita.com/UnaNancyOwen/items/680e56e7bacddce2eb52
