#include <iostream>
#include <cmath>
#include <limits>

#include "objExporter.h";

// (1) Include Header
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd.hpp>
//#include <opencv2/core/core.hpp>

#include <pcl/point_types.h>

int main(int argc, char** argv)
{
	// (2) Set Optimized
	cv::setUseOptimized(true);

	// (3) Open Video Capture
	cv::VideoCapture capture(cv::VideoCaptureAPIs::CAP_INTELPERC);
	if (!capture.isOpened()) {
		return -1;
	}

	// (4) Retrieve Camera Parameters
	const uint32_t width = static_cast<uint32_t>(capture.get(cv::CAP_INTELPERC_DEPTH_GENERATOR + cv::VideoCaptureProperties::CAP_PROP_FRAME_WIDTH));
	const uint32_t height = static_cast<uint32_t>(capture.get(cv::CAP_INTELPERC_DEPTH_GENERATOR + cv::VideoCaptureProperties::CAP_PROP_FRAME_HEIGHT));
	const float fx = static_cast<float>(capture.get(cv::CAP_PROP_INTELPERC_DEPTH_FOCAL_LENGTH_HORZ));
	const float fy = static_cast<float>(capture.get(cv::CAP_PROP_INTELPERC_DEPTH_FOCAL_LENGTH_VERT));
	const float cx = width / 2.0f - 0.5f;
	const float cy = height / 2.0f - 0.5f;
	const cv::Matx33f camera_matrix = cv::Matx33f(fx, 0.0f, cx, 0.0f, fy, cy, 0.0f, 0.0f, 1.0f);

	// (5) Initialize KinFu Parameters
	cv::Ptr<cv::kinfu::Params> params;
	params = cv::kinfu::Params::defaultParams(); // Default Parameters
	//params = cv::kinfu::Params::coarseParams(); // Coarse Parameters

	params->frameSize = cv::Size(width, height); // Frame Size
	params->intr = camera_matrix;             // Camera Intrinsics
	params->depthFactor = 1000.0f;                   // Depth Factor (1000/meter)

	// (6) Create KinFu
	cv::Ptr<cv::kinfu::KinFu> kinfu;
	kinfu = cv::kinfu::KinFu::create(params);

	while (true) {
		// (7) Grab All Frames and Retrieve Depth Frame
		capture.grab();

		cv::UMat frame;
		capture.retrieve(frame, cv::CAP_INTELPERC_DEPTH_MAP);
		if (frame.empty()) {
			continue;
		}

		// (8) Flip Image
		cv::flip(frame, frame, 1);

		// (9) Update Frame
		if (!kinfu->update(frame)) {
			std::cout << "reset" << std::endl;
			kinfu->reset();
			continue;
		}

		// (10) Rendering
		cv::UMat render;
		kinfu->render(render);

		// (11) Show Image
		cv::imshow("Kinect Fusion", render);
		const int32_t key = cv::waitKey(1);
		if (key == 'r') {
			kinfu->reset();
		}
		if (key == 'q') {
			break;
		}
		if (key == 's') {
			std::cout << "safe" << std::endl;

			cv::Mat points, normals;

			kinfu->getCloud(points, normals);

			std::cout << "Points: " << std::endl << points.rows << " " << points.cols << std::endl;
			std::cout << "Normals: " << std::endl << normals.rows << std::endl;

			//std::cout << "Points: " << std::endl << points << std::endl;
			//std::cout << "Normals: " << std::endl << normals << std::endl;

			objExporter("test.obj", points, normals);

			break;

		}
	}

	cv::destroyAllWindows();

	std::cin.get();

	return 0;
}