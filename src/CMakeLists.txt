#   CmakeLists for KinectFusion-RealSense-AE
message(STATUS "How to build: \n
- use OpenCV 4.1 in Release Build\n
- build RealSense in a subdirectory "build" of the root dir as Release build\n")

set(OpenCV_INSTALL_ROOT "Where is OpenCV installed (e.g. D:/Alexander_Pech/OpenCV/opencv-4.1.0/build/install" CACHE FILEPATH "Root directory of OpenCV")
find_path(OpenCV_INCLUDE opencv2/core.hpp HINTS "${OpenCV_INSTALL_ROOT}/include")
find_path(OpenCV_LIBRARY_PATH opencv_core410.lib HINTS 	"${OpenCV_INSTALL_ROOT}/x64/vc16/lib"
														"${OpenCV_INSTALL_ROOT}/x64/vc15/lib")

set(RealSense2_INSTALL_ROOT "Where is RealSense2 (e.g. D:/Alexander_Pech/librealsense-master" CACHE FILEPATH "Root directory of RealSense2")
find_path(RealSense2_INCLUDE librealsense2/rs.h HINTS "${RealSense2_INSTALL_ROOT}/include")
find_path(RealSense2_LIBRARY_PATH realsense2.lib HINTS 	"${RealSense2_INSTALL_ROOT}/build/Release"
														"${RealSense2_INSTALL_ROOT}/built/Release")

include_directories(${CMAKE_BINARY_DIR}/src ${OpenCV_INCLUDE} ${RealSense2_INCLUDE})
link_directories(${CMAKE_BINARY_DIR}/lib "${CMAKE_BINARY_DIR}/lib" ${OpenCV_LIBRARY_PATH} ${RealSense2_LIBRARY_PATH})
file(GLOB_RECURSE KFRSAE_HEADER "${CMAKE_CURRENT_SRC_DIR}" *.h)
file(GLOB_RECURSE KFRSAE_SRC "${CMAKE_CURRENT_SRC_DIR}" *.cpp)
add_executable(KinectFusion-RealSense-AE ${KFRSAE_HEADER} ${KFRSAE_SRC})

set(CMAKE_DEBUG_POSTFIX "d")

target_link_libraries(KinectFusion-RealSense-AE
    debug opencv_core410${CMAKE_DEBUG_POSTFIX} optimized opencv_core410
    debug opencv_highgui410${CMAKE_DEBUG_POSTFIX} optimized opencv_highgui410
    debug opencv_videoio410${CMAKE_DEBUG_POSTFIX} optimized opencv_videoio410
    debug opencv_rgbd410${CMAKE_DEBUG_POSTFIX} optimized opencv_rgbd410
    debug realsense2${CMAKE_DEBUG_POSTFIX} optimized realsense2
)