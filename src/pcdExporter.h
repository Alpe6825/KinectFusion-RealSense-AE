#pragma once
#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class pcdExporter
{
public:
	pcdExporter(std::string, cv::Mat, cv::Mat);
private:

};