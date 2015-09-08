#include "stdafx.h"
#include "MyWarping.h"

MyWarping::MyWarping()
{

}

MyWarping::~MyWarping()
{

}


void MyWarping::CalcTransformMatrix(cv::Mat _src, cv::Point _LTop, cv::Point _RTop, cv::Point _LBottom, cv::Point _RBottom)
{
	cv::Point2f srcPoint[4], dstPoint[4];
	srcPoint[0] = cv::Point2f(0, 0); // LEFT TOP
	srcPoint[1] = cv::Point2f(_src.cols, 0); // RIGHT TOP
	srcPoint[2] = cv::Point2f(_src.cols-1, _src.rows-1); // RIGHT BOTTOM
	srcPoint[3] = cv::Point2f(0, _src.rows-1); // LEFT BOTTOM

	dstPoint[0] = _LTop;
	dstPoint[1] = _RTop;
	dstPoint[2] = _RBottom;
	dstPoint[3] = _LBottom;

	m_transformMatrix = getPerspectiveTransform(srcPoint, dstPoint);
}

 void MyWarping::CalcWarping(cv::Mat _src, cv::Mat& _dst)
{
	warpPerspective(_src, _dst, m_transformMatrix, _src.size());
}