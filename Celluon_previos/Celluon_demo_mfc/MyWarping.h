#ifndef _MY_WARPING_H_
#define _MY_WARPING_H_

#include <opencv2\opencv.hpp>

class MyWarping
{
public:
	MyWarping();
	~MyWarping();

	void CalcTransformMatrix(cv::Mat _src, cv::Point _LTop, cv::Point _RTop, cv::Point _LBottom, cv::Point _RBottom);
	void CalcWarping(cv::Mat _src, cv::Mat& _dst);

	enum{LEFTTOP, RIGHTTOP, RIGHTBOTTOM, LEFTBOTTOM};

protected:
	cv::Point m_point[4];
	cv::Mat m_transformMatrix;
};

#endif