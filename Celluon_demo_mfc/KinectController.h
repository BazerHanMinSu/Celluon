#pragma once

#include <Kinect.h>
#include "KinectController.h"
#include "CommonHeader.h"

using namespace std;

class KinectController
{
public:
	KinectController();
	~KinectController();

	void UpdateKinectColorFrame();
	void UpdateKinectDepthFrame();

	Mat GetKinectColorFrame();
	Mat GetKinectDepthFrame();
	Mat GetKinectOriginalDepthFrame();
	Mat GetKinectColorDepthFrame();

protected:
	HRESULT Init();
	void Release();

protected:
	HRESULT m_isInitSucessful;
	IKinectSensor* m_pKinectSensor;
	IColorFrameReader* m_pColorFrameReader;
	IDepthFrameReader* m_pDepthFrameReader;
	IColorFrameSource* m_pColorFrameSource;
	IDepthFrameSource* m_pDepthFrameSource;
	ICoordinateMapper* m_pCoordinateMapper;
	Mat* m_matKinectColor;
	Mat* m_matKinectDepth;
	Mat* m_matKinectColorDepth; // Depth mapped Color Image

};
