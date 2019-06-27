#include <iostream>
#include <cmath>
#include <limits>

#include "objExporter.h";
#include "pcdExporter.h";

// (1) Include Header
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd.hpp>

#include <pcl/surface/marching_cubes_hoppe.h>



bool restart = false;
static void setRestart(int, void*) {
	restart = true;
}

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
	params->intr = camera_matrix;				 // Camera Intrinsics
	params->depthFactor = 1000.0f;               // Depth Factor (1000/meter)
	

	/*###################### GUI ##############################################*/

	//Params in Int
	//int bilateral_kernel_size;
	int bilateral_sigma_depth = (int)(params->bilateral_sigma_depth * 100);
	int bilateral_sigma_spatial = (int)(params->bilateral_sigma_spatial * 10);;
	int depthFactor = (int)params->depthFactor;
	int frameSize;
	int icpAngleThresh;
	int icpDistThresh;
	int icpIterations;
	int intr;
	int lightPose;
	int pyramidLevels;
	int raycast_step_factor;
	int truncateThreshold = (int)params->truncateThreshold;
	//int tsdf_max_weight;
	int tsdf_min_camera_movement = (int)params->tsdf_min_camera_movement;
	int tsdf_trunc_dist = (int)(params->tsdf_trunc_dist * 100);
	//int volumeDims;
	int volumePose;
	int voxelSize;

	cv::namedWindow("KinFu Params", cv::WINDOW_NORMAL);

	cv::createTrackbar("bilateral_kernel_size", "KinFu Params", &params->bilateral_kernel_size, 10, setRestart);
	cv::createTrackbar("bilateral_sigma_depth", "KinFu Params", &bilateral_sigma_depth, 10, setRestart);
	cv::createTrackbar("bilateral_sigma_spatial", "KinFu Params", &bilateral_sigma_spatial, 100, setRestart);
	
	//cv::createTrackbar("frameSize _ x", "KinFu Params", &params->frameSize[0], 100, setRestart);
	//cv::createTrackbar("frameSize _ y", "KinFu Params", &params->frameSize[1], 100, setRestart);

	
	cv::createTrackbar("depthFactor", "KinFu Params", &depthFactor, 2000, setRestart);

	cv::createTrackbar("truncateThreshold", "KinFu Params", &truncateThreshold, 512, setRestart);
	cv::createTrackbar("tsdf_max_weight", "KinFu Params", &params->tsdf_max_weight, 512, setRestart);
	cv::createTrackbar("tsdf_min_camera_movement", "KinFu Params", &tsdf_min_camera_movement, 10, setRestart);
	cv::createTrackbar("tsdf_trunc_dist", "KinFu Params", &tsdf_trunc_dist, 10, setRestart);
	
	cv::createTrackbar("volumeDims X", "KinFu Params", &params->volumeDims[0], 512, setRestart);
	cv::createTrackbar("volumeDims Y", "KinFu Params", &params->volumeDims[1], 512, setRestart);
	cv::createTrackbar("volumeDims Z", "KinFu Params", &params->volumeDims[2], 512, setRestart);
	



	restart:
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
		if (key == 'p') {
			cv::kinfu::Params currentParams = kinfu->getParams();

			std::cout << "bilateral_kernel_size:\t" << currentParams.bilateral_kernel_size << std::endl;
			std::cout << "bilateral_sigma_depth:\t" << currentParams.bilateral_sigma_depth << std::endl;
			std::cout << "bilateral_sigma_spatial:\t" << currentParams.bilateral_sigma_spatial << std::endl;
			std::cout << "depthFactor:\t" << currentParams.depthFactor << std::endl;
			std::cout << "frameSize:\t" << currentParams.frameSize << std::endl;
			std::cout << "icpAngleThresh:\t" << currentParams.icpAngleThresh << std::endl;
			std::cout << "icpDistThresh:\t" << currentParams.icpDistThresh << std::endl;
			std::cout << "icpIterations:\t" /*<< currentParams.icpIterations*/ << std::endl;
			std::cout << "intr:\t" << currentParams.intr << std::endl;
			std::cout << "lightPose:\t" << currentParams.lightPose << std::endl;
			std::cout << "pyramidLevels:\t" << currentParams.pyramidLevels << std::endl;
			std::cout << "raycast_step_factor:\t" << currentParams.raycast_step_factor << std::endl;
			std::cout << "truncateThreshold:\t" << currentParams.truncateThreshold << std::endl;
			std::cout << "tsdf_max_weight:\t" << currentParams.tsdf_max_weight << std::endl;
			std::cout << "tsdf_min_camera_movement:\t" << currentParams.tsdf_min_camera_movement << std::endl;
			std::cout << "tsdf_trunc_dist:\t" << currentParams.tsdf_trunc_dist << std::endl;
			std::cout << "volumeDims:\t" << currentParams.volumeDims << std::endl;
			std::cout << "volumePose:\t" /*<< currentParams.volumePose*/ << std::endl;
			std::cout << "voxelSize:\t" << currentParams.voxelSize << std::endl;
		}
		if (key == 't') {
			params->truncateThreshold = 1;
			goto restart;
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
		/*if (key == 'm') {
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

		}*/
		if (restart == true) {
			restart = false;


			params->bilateral_sigma_depth = (float)bilateral_sigma_depth/100;
			params->bilateral_sigma_spatial = (float)bilateral_sigma_spatial / 10;

			params->depthFactor = (float)depthFactor;

			params->truncateThreshold = (float)truncateThreshold;
			params->tsdf_min_camera_movement = (float)tsdf_min_camera_movement;
			params->tsdf_trunc_dist = (float)tsdf_trunc_dist / 100;
		
			goto restart;
		}
	}

	cv::destroyAllWindows();

	std::cin.get();

	return 0;
}
