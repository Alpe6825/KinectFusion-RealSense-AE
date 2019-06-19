#include <iostream>
#include <cmath>
#include <limits>

#include "objExporter.h";
#include "pcdExporter.h";

// (1) Include Header
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd.hpp>

#include <pcl/surface/marching_cubes_hoppe.h>


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
			std::cout << "Normals: " << std::endl << normals.rows << std::endl << std::endl;

			std::cout << "PointCloud Speichern in .obj (press 'o'), .pcd (press 'p')" << std::endl;

			char select = std::cin.get();

			switch (select)
			{
			case 'o':
				objExporter("test.obj", points, normals);
				break;
			case 'p':
				pcdExporter("test.pcd", points, normals);
				break;
			default:
				std::cout << "kein Format gewählt";
				break;
			}

			break;

		}
		if (key == 'm') {
			std::cout << "Marching Cubs" << std::endl;

			//https://github.com/rhololkeolke/EECS-466-Project/blob/master/src/pcl_reconstruction/pcl_marching_cubes.cpp

			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);

				cv::Mat points, normals;
				kinfu->getCloud(points, normals);
				std::cout << "Points: " << std::endl << points.rows << " " << points.cols << std::endl;
				std::cout << "Normals: " << std::endl << normals.rows << std::endl;
				for (int i = 0; i < points.rows; i++) {

					pcl::PointNormal pointNormal;
					pointNormal.x = points.at<float>(i, 0);
					pointNormal.y = points.at<float>(i, 1);
					pointNormal.z = points.at<float>(i, 2);
					pointNormal.normal_x = normals.at<float>(i, 0);
					pointNormal.normal_y = normals.at<float>(i, 1);
					pointNormal.normal_z = normals.at<float>(i, 2);

					cloud_with_normals->push_back(pointNormal);
				}

				
			std::cout << cloud_with_normals->points.size() << " points in Point Cloud" << std::endl;

			
			pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
			tree->setInputCloud(cloud_with_normals);

			pcl::MarchingCubesHoppe<pcl::PointNormal> mc; 
			pcl::PolygonMesh triangles;

			mc.setGridResolution(100, 100, 100);
			mc.setIsoLevel(0);
			mc.setPercentageExtendGrid(0);

			mc.setInputCloud(cloud_with_normals);
			mc.setSearchMethod(tree);
			mc.reconstruct(triangles);

			std::cout << triangles.polygons.size() << " polygons in reconstructed mesh" << std::endl;

			objExporter("test.obj", points, normals, triangles);
			
		    break;

		}
	}

	cv::destroyAllWindows();

	std::cin.get();

	return 0;
}