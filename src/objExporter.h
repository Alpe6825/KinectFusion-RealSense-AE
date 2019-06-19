#pragma once
#include <opencv2/opencv.hpp>
#include <pcl/surface/marching_cubes_hoppe.h>

class objExporter
{
public:
	objExporter(std::string, cv::Mat, cv::Mat);
	objExporter(std::string, cv::Mat, cv::Mat, pcl::PolygonMesh);

private:

};