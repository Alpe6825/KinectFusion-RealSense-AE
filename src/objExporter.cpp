#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "objExporter.h";

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
	}

	objFile.close();
}
