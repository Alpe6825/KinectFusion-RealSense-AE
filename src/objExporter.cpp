#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "objExporter.h";
#include <pcl/surface/marching_cubes_hoppe.h>

using namespace std;

objExporter::objExporter(std::string filename, cv::Mat points, cv::Mat normals)
{
	ofstream objFile(filename);


	if (!objFile.is_open())
	{
		cout << "Unable to open OBJ-file";
	}
	else
	{
		for (int i = 0; i < points.rows; i++) {
			objFile << "v " << points.at<float>(i, 0) << " " << points.at<float>(i, 1) << " " << points.at<float>(i, 2) << endl;
		}
		for (int i = 0; i < normals.rows; i++) {
			//objFile << "vn " << normals.at<float>(i, 0) << " " << normals.at<float>(i, 1) << " " << normals.at<float>(i, 2) << endl;
		}
	}

	objFile.close();
}



objExporter::objExporter(std::string filename, cv::Mat points, cv::Mat normals, pcl::PolygonMesh triangles)
{
	ofstream objFile(filename);


	if (!objFile.is_open())
	{
		cout << "Unable to open OBJ-file";
	}
	else
	{
		for (int i = 0; i < points.rows; i++) {
			objFile << "v " << points.at<float>(i, 0) << " " << points.at<float>(i, 1) << " " << points.at<float>(i, 2) << endl;
		}
		for (int i = 0; i < normals.rows; i++) {
			//objFile << "vn " << normals.at<float>(i, 0) << " " << normals.at<float>(i, 1) << " " << normals.at<float>(i, 2) << endl;
		}
		for (int i = 0; i < triangles.polygons.size(); i++) {
			objFile << "f " << triangles.polygons[i].vertices[0] + 1 << " " << triangles.polygons[i].vertices[1] + 1 << " " << triangles.polygons[i].vertices[2] + 1 << endl;
		}
	}

	objFile.close();
}

