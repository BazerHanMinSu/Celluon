#pragma once

#include "KinectController.h"
//#include "WarpingController.h"
#include "MappingTableController.h"
#include <fstream>
#include <vector>
#include <stack>

class Application
{
public:
	Application();
	~Application();

	void Run();

	void CropDepthImage(Mat inImg, Mat &cropImg);

	void OutFileOfPoints();


	void MakeObjFile(Mat _depthImg, string _filePath);
	void MakeObjFile(Point3f** realPosArr, int width, int height, string _filePath);
	void MakeObjFile(Point2f** realPosArr, int width, int height, string _filePath);
	void MakeObjFile(bool** realPosArr, int width, int height, string _filePath);
	void MakeObjFile(MappingPos** realPosArr, int width, int height, string _filePath);

	void GetMedianDepthFrame(Mat& dstImg, int length = 30, int maskSize = 3);
	void MeanDepthFrame(int length);
	void SetUserLocation(double _x, double _y, double _z);

	Point2f* m_BoundaryMathedPoint;
private:

	KinectController m_kinect;	
	MappingTableController m_mappingControl;

	Point3f **m_worldCoordinate;

	int m_depthTh;
	int frameCount = 0;
	bool makeOn = false;

	Mat m_cropImage;

	Point3d m_userLocation;
};