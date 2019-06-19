
#include <pcdExporter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

pcdExporter::pcdExporter(std::string filename, cv::Mat points, cv::Mat normals)
{
	pcl::PointCloud<pcl::PointNormal> cloud_with_normals;// (new pcl::PointCloud<pcl::PointNormal>);

	for (int i = 0; i < points.rows; i++) {

		pcl::PointNormal pointNormal;
		pointNormal.x = points.at<float>(i, 0);
		pointNormal.y = points.at<float>(i, 1);
		pointNormal.z = points.at<float>(i, 2);
		pointNormal.normal_x = normals.at<float>(i, 0);
		pointNormal.normal_y = normals.at<float>(i, 1);
		pointNormal.normal_z = normals.at<float>(i, 2);

		cloud_with_normals.push_back(pointNormal);
	}

	pcl::io::savePCDFileASCII("test_ascii.pcd", cloud_with_normals);
	pcl::io::savePCDFileBinary("test_binary.pcd", cloud_with_normals);
}