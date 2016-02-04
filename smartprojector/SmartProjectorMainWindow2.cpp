#include <QtGui>
#include "SmartProjectorMainWindow.h"
#include <string>
//#include "ComboBoxItem.h"
#include <QMessageBox>

#include <cv.h>
//#include <highgui.h>
#include <opencv2\imgproc\imgproc.hpp>


#include <iostream>
#include <signal.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <fstream>

bool sortY(cv::Point a, cv::Point b);
bool sortX(cv::Point a, cv::Point b);



//////////////////////////////////////////////////////////
///   타이머 인터럽트 걸려 주기적으로 호출
//////////////////////////////////////////////////////////
void SmartProjectorMainWindow::timeout()
{
	//if (mIsKinectDeviceInit == false)
	//	this->openDeivce();

	if (listener->hasNewFrame() == false)
		return;

	cv::Mat image;				// 키넥트 RGB이미지
	cv::Mat imageDepth;			// Depth 이미지 
	cv::Mat imageIR;			// IR 이미지
	cv::Mat imageRegisted;  	// Depth + RGB calibration이미지
	cv::Mat userDepth;          // Plane에 정면이 되도록 본 Depth
	cv::Mat userRegistered;     // Plane에 정면이 되도록 본 Depth + RGB calibratrion이미지

	// 워핑 결과 일때
	if (mode == WARP_RESULT)
	{
		// 키넥트 영상 획득
		getKinectImage(image, imageDepth, imageIR, imageRegisted);
		

		// Plane에서 정면이 되도록 본 Depth 
		//cv::remap(imageDepth, userDepth, mHumanDepthMap_x, mHumanDepthMap_y, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0)); // 
		if (mAccFrameNumber < mNumAvgFrame)
		{
			if (mAccFrameNumber == 0)
				mAvgDepth = imageDepth;
			else
				mAvgDepth = mAvgDepth + imageDepth;

			mAccFrameNumber++;
		}

		// Warping 수행
		if (mAccFrameNumber == mNumAvgFrame)
		{
			//DepthImageHoleFilling(imageDepth);			// 홀필링
			warpScreenImage(mPatternImage, imageDepth); //워핑 수행
			//mAvgDepth = mAvgDepth / (float)mNumAvgFrame;
			//warpScreenImage(mPatternImage, mAvgDepth); //워핑 수행
			mAccFrameNumber = 0;
		}
	}


	//프로그램 화면에 영상 보여주는 부
	if (mode == READY)
	{
		// 키넥트 영상 획득
		getKinectImage(image, imageDepth, imageIR, imageRegisted);

		// Plane에서 정면이 되도록 본 Depth 및 Registration 이미지
		cv::remap(imageDepth, userDepth, mHumanDepthMap_x, mHumanDepthMap_y, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
		cv::remap(imageRegisted, userRegistered, mHumanDepthMap_x, mHumanDepthMap_y, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

		// 프로젝터 키넥트 칼리브레이션이 되어 있다면 프로젝션 영역을 그려준다. 
		cv::Mat reg2 = imageRegisted.clone();
		if (mCalbrationDone)
		{
			// 키넥트 좌표상 프로젝션 영역
			detectProjectionArea(imageDepth, imageRegisted);

			// 유저 좌표상 프로젝션 영역
			//detectProjectionAreaUser(userDepth, userRegistered);
		}
		// 홀필링
		//DepthImageHoleFilling(imageDepth);


		// 화면 출력용 이미지 생성
		//imageIR.convertTo(imageIR, CV_8U, 1.0 / 50.0);
		imageDepth.convertTo(imageDepth, CV_8U, 1.0 / 50.0);
		cv::cvtColor(imageDepth, imageDepth, CV_GRAY2RGB);

		
		mCurrentImage = image;
		//mCurrentIRImage = imageIR;
		mCurrentDepthImage = imageDepth;
		mCurrentIRImage = reg2;
		mCurrentCalibrationImage = imageRegisted;


		//rgb 이미지 복사
		QImage qimg((uchar*)mCurrentImage.data, mCurrentImage.cols, mCurrentImage.rows, mCurrentImage.step, QImage::Format_RGB888);
		//IR 이미지 복사
		QImage qimg_depth((uchar*)mCurrentDepthImage.data, mCurrentDepthImage.cols, mCurrentDepthImage.rows, mCurrentDepthImage.step, QImage::Format_RGB888);
		// undistored depth 복사
		QImage qimg_ir((uchar*)mCurrentIRImage.data, mCurrentIRImage.cols, mCurrentIRImage.rows, mCurrentIRImage.step, QImage::Format_RGB888);
		// Depth + RGB calibration 복사
		QImage qimg_ud((uchar*)mCurrentCalibrationImage.data, mCurrentCalibrationImage.cols, mCurrentCalibrationImage.rows, mCurrentCalibrationImage.step, QImage::Format_RGB888);


		mVideoWidget1->SetImage(qimg);
		mVideoWidget2->SetImage(qimg_depth);
		mVideoWidget3->SetImage(qimg_ir);
		mVideoWidget4->SetImage(qimg_ud);

		mVideoWidget1->update();
		mVideoWidget2->update();
		mVideoWidget3->update();
		mVideoWidget4->update();
	}

	if (mode == CALIBRATING)
	{

		// 키넥트 영상 획득
		getKinectImage(image, imageDepth, imageIR, imageRegisted);

		// 칼리브레이션  timeout
		if (mSampleTimeout < 0)
		{
			// 컬러 depth registration image 변환 
			cv::cvtColor(imageRegisted, imageRegisted, CV_BGRA2GRAY);

			// 체크보드 찾기 
			bool calResult = findCheckBoard(imageRegisted);

			// 체스보드 검출 오류 메세지
			if (calResult == false)
			{
				QMessageBox::critical(this, QString::fromLocal8Bit("칼리브레이션 체크보드 검출 오류"), QString::fromLocal8Bit("체크보드 패턴을 검출하지 못했습니다. 다시 시도 합니다."));
				mSampleTimeout = mInterval * 1000;
			}
			else
			{
				// 칼리브레이션 파라미서 추가 
				calculateCalibration(imageDepth);

				// 화면 깜박임을 위한 코드
				mProjectorScreen->showNormal();
				mProjectorScreen->hide();

				// 이터레이션 감소
				mCalIteration--;

				// 다음 칼리브레이션을 위한 체크보드 인덱스 설정
				int curindex = (mMaxIteration - mCalIteration);
				int moveX = curindex % 5;
				int moveY = curindex / 5;

				// 새로운 체크보드 패턴 생성 및 화면에 띄우기 
				cv::Mat out = cv::Mat::zeros(mPatternImage.size(), mPatternImage.type());
				out.setTo(255);
				mPatternImage(cv::Rect(0, 0, 1690, 1190)).copyTo(out(cv::Rect(moveX * 20, moveY * 20, 1690, 1190)));
				mCurrentPatternImage = out;
				QImage qimg((uchar*)mCurrentPatternImage.data, mCurrentPatternImage.cols, mCurrentPatternImage.rows, mCurrentPatternImage.step, QImage::Format_RGB888);
				mProjectorScreen->SetImage(qimg);

				// 새로운 체크보드 패턴 truth값 생성
				mCurrentTruth = mCornerTruth;   // trutu table 복사 
				int ii;
				for (ii = 0; ii < mCurrentTruth.size(); ii++)
				{
					mCurrentTruth[ii].x = mCornerTruth[ii].x + moveX * 20;
					mCurrentTruth[ii].y = mCornerTruth[ii].y + moveY * 20;
				}

				// 칼리브레이션이 완료되면 파라미터 값을 푼다. 
				if (mCalIteration == 0)
				{
					solveCalibration();
					mCalbrationDone = true;
					mode = READY;
				}
				else  // 칼리브레이션이 완료되지 않았으면 체스 보드를 띄워준다.
				{
					mProjectorScreen->showFullScreen();
					mSampleTimeout = mInterval * 1000;
				}
			}
		}
		else
		{
			// 칼리브레이션 time out display
			lcdNumber->display(mSampleTimeout / 1000);
			mSampleTimeout -= mTimer.interval();
		}
	}
}

///////////////////////////////////////
// 워핑을 수행하는 부
// screenImage : 화면에 뿌려주는 이미지
// undistorted : avgDetphImg
////////////////////////////////////////
void SmartProjectorMainWindow::warpScreenImage(cv::Mat &screenImage, cv::Mat &undistorted) 
{
	// 1. Projection Area 검출
	// 2. Area중 최대 Contour 검출
	// 3. 최대 Contour안에서 가장큰 Rect구하기
	// 4. 가장큰 Rect를 Image좌표로 바꿔서 Contour구성하고 화면에 띄워보기 
	// 5. Warping 수행

	// 1. depth to X, Y, Z (world coordinate)
	//libfreenect2::Freenect2Device::IrCameraParams depthCamera;    ///< Depth camera parameters.
	//depthCamera = mKinectDevice->getIrCameraParams();
	//const float cx(depthCamera.cx), cy(depthCamera.cy);
	//const float fx(1 / depthCamera.fx), fy(1 / depthCamera.fy);
	// 1. depth to X, Y, Z (world coordinate)
	libfreenect2::Freenect2Device::IrCameraParams depthCamera;    ///< Depth camera parameters.
	depthCamera = mKinectDevice->getIrCameraParams();				//IR camera params
	const float cx(depthCamera.cx), cy(depthCamera.cy);
	const float fx(1 / depthCamera.fx), fy(1 / depthCamera.fy);
	float* undistorted_data = (float *)undistorted.data;  //depthImg 데이터 연결

	int i, j;
	float x, y, z;
	float xi, yi;

	float c1 = mProjectorImageWidth / 2.0f / tan(mProjectorFovX / 2.0f * 0.0174533f);
	float c2 = mProjectorImageHeight / 2.0f / tan(mProjectorFovY / 2.0f * 0.0174533f);
	float halfwidth = mProjectorImageWidth / 2.0f;
	float halfhegiht = mProjectorImageHeight / 2.0f;

	//프로젝션 영역 마스크 초기화
	mProjectionAreaMask.setTo(0);

	for (i = 0; i < undistorted.rows; i++)
	{

		float *XYZ_Ptr = mPointXYZ.ptr<float>(i);

		for (j = 0; j < undistorted.cols; j++)
		{
			//cv::Vec3b dimageVec = depthimg.at<cv::Vec3b>(i, j);
			const float depth_val = undistorted_data[512 * i + j];  //위치의 depthValue

			if (isnan(depth_val) || depth_val <= 0.001)
				continue;

			x = (j + 0.5 - cx) * fx * depth_val;
			y = (i + 0.5 - cy) * fy * depth_val;
			z = depth_val;

			//xp = (q1*xk + q2*yk + q3*zk + q4)/(q9*xk + q10*yk + q11*zk + 1)
			xi = c1* (x + mCalibrationValues.at<float>(0, 0) * y + mCalibrationValues.at<float>(0, 1) * z + mCalibrationValues.at<float>(0, 2)) /
				(mCalibrationValues.at<float>(0, 3) * x + mCalibrationValues.at<float>(0, 4) * y + mCalibrationValues.at<float>(0, 5) * z + mCalibrationValues.at<float>(0, 6));

			yi = c2* (x + mCalibrationValues.at<float>(0, 7) * y + mCalibrationValues.at<float>(0, 8) * z + mCalibrationValues.at<float>(0, 9)) /
				(mCalibrationValues.at<float>(0, 10) * x + mCalibrationValues.at<float>(0, 11) * y + mCalibrationValues.at<float>(0, 12) * z + mCalibrationValues.at<float>(0, 13));

			mImage_xPos.at<float>(i, j) = xi + halfwidth;
			mImage_yPos.at<float>(i, j) = yi + halfhegiht;

			if ((xi > -halfwidth) && (xi < halfwidth) && (yi > -halfhegiht) && (yi < halfhegiht))
				mProjectionAreaMask.at<uchar>(i, j) = 255;

			XYZ_Ptr[j * 3] = x;
			XYZ_Ptr[j * 3 + 1] = y;
			XYZ_Ptr[j * 3 + 2] = z;
		}
	}

	if (mAutoPlaneRotationCB->checkState() == Qt::Checked)
	{
		// calculate Plane normal vector 
		float angleX, angleY;
		calculatePlaneNormalVector(angleX, angleY);

		mRotateXSpinBox->setValue(angleX);
		mRotateYSpinBox->setValue(-angleY);
		create_Human_To_depth_Map();
	}

	cv::remap(mProjectionAreaMask, mProjectionAreaMaskUser, mHumanDepthMap_x, mHumanDepthMap_y, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
	cv::remap(mImage_xPos, mImage_xPos, mHumanDepthMap_x, mHumanDepthMap_y, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
	cv::remap(mImage_yPos, mImage_yPos, mHumanDepthMap_x, mHumanDepthMap_y, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

	// 1. Largest Contour 검출
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(mProjectionAreaMaskUser, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));
	cv::Mat contourImage = cv::Mat::zeros(mProjectionAreaMaskUser.size(), CV_8UC3);;

	unsigned int maxSize = 0;
	unsigned int id = 0;
	for (i = 0; i<contours.size(); ++i)
	{
		if (contours.at(i).size() > maxSize)
		{
			maxSize = contours.at(i).size();
			id = i;
		}
	}

	cv::Mat contourMask = cv::Mat::zeros(mProjectionAreaMaskUser.size(), CV_8UC1);
	cv::drawContours(contourMask, contours, id, cv::Scalar(255), -1, 8, hierarchy, 0, cv::Point());

	// 2. Contour안의 최대 사각형 검출
	cv::Rect OptimalRect = mOpRect.findOptimalRect(contourMask);


	std::vector<cv::Point> rectContour;

	//                   1
	//      ------------------------------>
	//      |                             |
	//      |                             |
	//   4  |                             |  2
	//      |                             |
	//      <-----------------------------
	//                   3]

	int xPos;
	int yPos;

	mImage_xPos.at<float>(OptimalRect.x, OptimalRect.y);

	yPos = OptimalRect.y;
	for (i = OptimalRect.x; i < (OptimalRect.x + OptimalRect.width); i++)
	{
		float xValue = mImage_xPos.at<float>(yPos, i);
		float yValue = mImage_yPos.at<float>(yPos, i);
		rectContour.push_back(cv::Point(xValue, yValue));
	}

	xPos = OptimalRect.x + OptimalRect.width;
	for (i = OptimalRect.y; i < (OptimalRect.y + OptimalRect.height); i++)
	{
		float xValue = mImage_xPos.at<float>(i, xPos);
		float yValue = mImage_yPos.at<float>(i, xPos);
		rectContour.push_back(cv::Point(xValue, yValue));
	}

	yPos = OptimalRect.y + OptimalRect.height;
	for (i = (OptimalRect.x + OptimalRect.width - 1); i >= OptimalRect.x; i--)
	{
		float xValue = mImage_xPos.at<float>(yPos, i);
		float yValue = mImage_yPos.at<float>(yPos, i);
		rectContour.push_back(cv::Point(xValue, yValue));
	}

	xPos = OptimalRect.x;
	for (i = (OptimalRect.y + OptimalRect.height - 1); i >= OptimalRect.y; i--)
	{
		float xValue = mImage_xPos.at<float>(i, xPos);
		float yValue = mImage_yPos.at<float>(i, xPos);
		rectContour.push_back(cv::Point(xValue, yValue));
	}

	// for drawing
	std::vector<std::vector<cv::Point> > contours2;
	contours2.push_back(rectContour);


	// 실제 워핑이 일어나는 부분
	// 3. warping 수행
	bool Checkwapring = true;
	if (Checkwapring)
	{
		//cv::Mat warpImg;
		cv::Mat resizedScreenImg;
		if (screenImage.cols != mProjectorImageWidth || screenImage.rows != mProjectorImageHeight)
			cv::resize(screenImage, resizedScreenImg, cv::Size(mProjectorImageWidth, mProjectorImageHeight));
		else
			resizedScreenImg = screenImage.clone();

		mWarpImage = cv::Mat::zeros(resizedScreenImg.size(), resizedScreenImg.type());
		
		if (mCorrectionFlag)
		{
			// homography transform
			if (mHomographyCB->checkState() == Qt::Unchecked)
			{
				//추가함수
				WarpArbitrary(resizedScreenImg, mWarpImage, OptimalRect);
				//원본
				//cv::remap(resizedScreenImg, mWarpImage, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
			}
			else //4점으로 워핑 수행 (평면에서 가능하도록
			{
				cv::Point2f inputQuad[4];
				cv::Point2f outputQuad[4];

				float x1, x2, x3, x4;
				float y1, y2, y3, y4;
				x1 = mImage_xPos.at<float>(OptimalRect.y, OptimalRect.x);
				x2 = mImage_xPos.at<float>(OptimalRect.y, OptimalRect.x + OptimalRect.width - 1);
				x3 = mImage_xPos.at<float>(OptimalRect.y + OptimalRect.height - 1, OptimalRect.x + OptimalRect.width - 1);
				x4 = mImage_xPos.at<float>(OptimalRect.y + +OptimalRect.height - 1, OptimalRect.x);

				y1 = mImage_yPos.at<float>(OptimalRect.y, OptimalRect.x);
				y2 = mImage_yPos.at<float>(OptimalRect.y, OptimalRect.x + OptimalRect.width - 1);
				y3 = mImage_yPos.at<float>(OptimalRect.y + OptimalRect.height - 1, OptimalRect.x + OptimalRect.width - 1);
				y4 = mImage_yPos.at<float>(OptimalRect.y + +OptimalRect.height - 1, OptimalRect.x);

				outputQuad[0] = cv::Point2f(x1, y1);
				outputQuad[1] = cv::Point2f(x2, y2);
				outputQuad[2] = cv::Point2f(x3, y3);
				outputQuad[3] = cv::Point2f(x4, y4);

				inputQuad[0] = cv::Point2f(0, 0);
				inputQuad[1] = cv::Point2f(mProjectorImageWidth - 1, 0);
				inputQuad[2] = cv::Point2f(mProjectorImageWidth - 1, mProjectorImageHeight - 1);
				inputQuad[3] = cv::Point2f(0, mProjectorImageHeight - 1);

				cv::Mat transformMatrix = getPerspectiveTransform(inputQuad, outputQuad);
				cv::warpPerspective(resizedScreenImg, mWarpImage, transformMatrix, mWarpImage.size());
			}
		}
		else
		{
			mWarpImage = resizedScreenImg;
		}
		cv::cvtColor(mWarpImage, mWarpImage, CV_BGRA2RGB);
		cv::drawContours(mWarpImage, contours2, 0, cv::Scalar(255, 0, 0), 2);
		// optimal rect에 해당되는 영역 그려주기 
		//cv::contour
		QImage qimg((uchar*)mWarpImage.data, mWarpImage.cols, mWarpImage.rows, mWarpImage.step, QImage::Format_RGB888);


		mWarpingScreen->SetImage(qimg);
		mWarpingScreen->update();
	}
	//mSecondScreen->SetImage(qimg);
}



void SmartProjectorMainWindow::calculatePlaneNormalVector(float &angleX, float &angleY)
{
	// 1. 프로젝션 영역의 X, Y, Z값 찾기 
	// 2. 프로젝션 영역중 가장큰 contour남기기
	// 3. contour에 해당하는 값의 Normal vector구하기 
	cv::Mat A(3, 3, CV_32F);
	cv::Mat B(3, 3, CV_32F);
	cv::Mat x;

	A.setTo(0);
	B.setTo(0);

	float sumA[3][3] = { { 0, 0, 0 },
	{ 0, 0, 0 },
	{ 0, 0, 0 } };

	float sumB[3] = { 0, 0, 0 };

	int numPt = 0;


	//Least square 연산
	int i, j;
	for (i = 0; i < mProjectionAreaMask.rows; i++)
	{
		uchar* linebuf = mProjectionAreaMask.ptr<uchar>(i);
		float* xyzBuf = mPointXYZ.ptr<float>(i);
		for (j = 0; j < mProjectionAreaMask.cols; j++)
		{

			if (linebuf[j] == 255)
			{
				float x = *(xyzBuf + j * 3);
				float y = *(xyzBuf + j * 3 + 1);
				float z = *(xyzBuf + j * 3 + 2);

				// A
				sumA[0][0] += x * x; 		sumA[0][1] += x * y; 			sumA[0][2] += x;
				sumA[1][0] += x * y; 		sumA[1][1] += y * y; 			sumA[1][2] += y;
				sumA[2][0] += x;			sumA[2][1] += y;

				// B
				sumB[0] += x * z;
				sumB[1] += y * z;
				sumB[2] += z;

				numPt++;
			}

		}
	}

	sumA[2][2] = numPt;

	//Mat A 
	A.at<float>(0, 0) = sumA[0][0];		A.at<float>(0, 1) = sumA[0][1];		A.at<float>(0, 2) = sumA[0][2];
	A.at<float>(1, 0) = sumA[1][0];		A.at<float>(1, 1) = sumA[1][1];		A.at<float>(1, 2) = sumA[1][2];
	A.at<float>(2, 0) = sumA[2][0];		A.at<float>(2, 1) = sumA[2][1];		A.at<float>(2, 2) = sumA[2][2];

	//Mat B
	B.at<float>(0, 0) = sumB[0];		B.at<float>(1, 0) = sumB[1];		B.at<float>(2, 0) = sumB[2];

	cv::solve(A, B, x, cv::DECOMP_SVD);

	//평면 Normal Vector
	float a, b, c;
	a = -x.at<float>(0, 0);
	b = -x.at<float>(1, 0);
	c = 1;

	// 각도 계산
	angleX = atan2(b, c) * 57.2958f;
	angleY = atan2(a, c) * 57.2958f;

	// print trace information
	QString str1 = "Detect Plane Normal Vector";
	QString str2 = "A B C : " + QString::number(a) + " " + QString::number(b) + " " + QString::number(c);
	QString str3 = "Angle X : " + QString::number(angleX);
	QString str4 = "Angle Y : " + QString::number(angleY);
	trace(str1);
	trace(str2);
	trace(str3);
	trace(str4);
}