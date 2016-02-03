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

SmartProjectorMainWindow::SmartProjectorMainWindow() : READY(0), CALIBRATING(1), SHOW_CURRENT_RESULT(2), WARP_RESULT(3), mMaxIteration(4)
{
    // UI 셋업
	setupUi(this);
	
	openDeivce();

	connect(&mTimer, SIGNAL(timeout()), this, SLOT(timeout()));
	connect(mInspectStartBtn, SIGNAL(pressed()), this, SLOT(startInspect()));
	connect(mInspectEndBtn, SIGNAL(pressed()), this, SLOT(endInspect()));
	connect(mKinectCalibrationBtn, SIGNAL(pressed()), this, SLOT(calibrationImage()));
	connect(mCorrection, SIGNAL(pressed()), this, SLOT(correctionScreen()));

	connect(mFovXSpinBox, SIGNAL(valueChanged(double)), this, SLOT(changeHumanFovX(double)));
	connect(mFovYSpinBox, SIGNAL(valueChanged(double)), this, SLOT(changeHumanFovY(double)));

	//connect(mVideoCaptureBtn, SIGNAL(pressed()), this, SLOT(captureImage()));
	//connect(mVideoCaptureBtn, SIGNAL(pressed()), this, SLOT(turnOnCaptureLight()));
	//connect(mVideoCaptureBtn, SIGNAL(released()), this, SLOT(turnOffCaptureLight()));

	//connect(mVideoRecStartBtn, SIGNAL(pressed()), this, SLOT(startVideoRecord()));
	//connect(mVideoRecEndBtn, SIGNAL(pressed()), this, SLOT(endVideoRecord()));
	//connect(mVideoRecStartBtn, SIGNAL(pressed()), this, SLOT(turnOnRecordLight()));
	//connect(mVideoRecEndBtn, SIGNAL(pressed()), this, SLOT(turnOffRecordLight()));


	mTimer.setInterval(100);

	mDefaultImage = cv::Mat(100, 100, CV_8SC3);
	mDefaultImage.setTo(135);


	
	//mProjectorScreen->hide();
	//mProjectorScreen->showFullScreen();
	//InitPreViews();

	//InitStatusBar();
	
	mode = READY;
	mInterval = 3;

	mSampleTimeout = 0;
	//mCalResultTimeout = 0;
	mIsKinectDeviceInit = false;
	mBoardSize = cv::Size(9, 6);
	mSquareSize = 2.8f;
	mMaxScale = 2;
	mCalbrationDone = false;

	mProjectorScreen = new QImageWidget();
	std::string chessPatternPath = QApplication::applicationDirPath().toLocal8Bit() + "/pattern.png";
	mPatternImage = cv::imread(chessPatternPath);

	mWarpingScreen = new QImageWidget();
	mWarpingScreen->hide();
	//QImage qimg((uchar*)mPatternImage.data, mPatternImage.cols, mPatternImage.rows, mPatternImage.step, QImage::Format_RGB888);
	//mProjectorScreen->SetImage(qimg);
	//cv::cvtColor(mPatternImage, mPatternImage, CV_BGR2GRAY);
	//cv::imshow("test",mPatternImage);
	//cv::waitKey(0);

	//findCheckBoardTruth(mPatternImage);

	mCornerTruth.push_back(cv::Point2f(184, 184));
	mCornerTruth.push_back(cv::Point2f(350, 184));
	mCornerTruth.push_back(cv::Point2f(516, 184));
	mCornerTruth.push_back(cv::Point2f(683, 184));
	mCornerTruth.push_back(cv::Point2f(849, 184));
	mCornerTruth.push_back(cv::Point2f(1015, 184));
	mCornerTruth.push_back(cv::Point2f(1182, 184));
	mCornerTruth.push_back(cv::Point2f(1348, 184));
	mCornerTruth.push_back(cv::Point2f(1514, 184));

	mCornerTruth.push_back(cv::Point2f(184, 350));
	mCornerTruth.push_back(cv::Point2f(350, 350));
	mCornerTruth.push_back(cv::Point2f(516, 350));
	mCornerTruth.push_back(cv::Point2f(683, 350));
	mCornerTruth.push_back(cv::Point2f(849, 350));
	mCornerTruth.push_back(cv::Point2f(1015, 350));
	mCornerTruth.push_back(cv::Point2f(1182, 350));
	mCornerTruth.push_back(cv::Point2f(1348, 350));
	mCornerTruth.push_back(cv::Point2f(1514, 350));
	
	mCornerTruth.push_back(cv::Point2f(184, 516));
	mCornerTruth.push_back(cv::Point2f(350, 516));
	mCornerTruth.push_back(cv::Point2f(516, 516));
	mCornerTruth.push_back(cv::Point2f(683, 516));
	mCornerTruth.push_back(cv::Point2f(849, 516));
	mCornerTruth.push_back(cv::Point2f(1015, 516));
	mCornerTruth.push_back(cv::Point2f(1182, 516));
	mCornerTruth.push_back(cv::Point2f(1348, 516));
	mCornerTruth.push_back(cv::Point2f(1514, 516));

	mCornerTruth.push_back(cv::Point2f(184, 683));
	mCornerTruth.push_back(cv::Point2f(350, 683));
	mCornerTruth.push_back(cv::Point2f(516, 683));
	mCornerTruth.push_back(cv::Point2f(683, 683));
	mCornerTruth.push_back(cv::Point2f(849, 683));
	mCornerTruth.push_back(cv::Point2f(1015, 683));
	mCornerTruth.push_back(cv::Point2f(1182, 683));
	mCornerTruth.push_back(cv::Point2f(1348, 683));
	mCornerTruth.push_back(cv::Point2f(1514, 683));

	mCornerTruth.push_back(cv::Point2f(184, 849));
	mCornerTruth.push_back(cv::Point2f(350, 849));
	mCornerTruth.push_back(cv::Point2f(516, 849));
	mCornerTruth.push_back(cv::Point2f(683, 849));
	mCornerTruth.push_back(cv::Point2f(849, 849));
	mCornerTruth.push_back(cv::Point2f(1015, 849));
	mCornerTruth.push_back(cv::Point2f(1182, 849));
	mCornerTruth.push_back(cv::Point2f(1348, 849));
	mCornerTruth.push_back(cv::Point2f(1514, 849));

	mCornerTruth.push_back(cv::Point2f(184, 1015));
	mCornerTruth.push_back(cv::Point2f(350, 1015));
	mCornerTruth.push_back(cv::Point2f(516, 1015));
	mCornerTruth.push_back(cv::Point2f(683, 1015));
	mCornerTruth.push_back(cv::Point2f(849, 1015));
	mCornerTruth.push_back(cv::Point2f(1015, 1015));
	mCornerTruth.push_back(cv::Point2f(1182, 1015));
	mCornerTruth.push_back(cv::Point2f(1348, 1015));
	mCornerTruth.push_back(cv::Point2f(1514, 1015));

	//QRect mainScreenSize = mWarpingScreen->availableGeometry(mWarpingSc->primaryScreen());

	mProjectorImageWidth = 1600;
	mProjectorImageHeight = 1200;

	mProjectorFovX = 31.38;
	mProjectorFovY = 17.96;

	mProjectionAreaMask = cv::Mat(424, 512,CV_8U);
	mImage_xPos = cv::Mat(424, 512, CV_32F);
	mImage_yPos = cv::Mat(424, 512, CV_32F);
	mPointXYZ = cv::Mat(424, 512, CV_32FC3);

	loadCalibrationData();
	create_Human_To_depth_Map();
}

SmartProjectorMainWindow::~SmartProjectorMainWindow()
{	
	mKinectDevice->stop();
	mKinectDevice->close();
}

void SmartProjectorMainWindow::openDeivce()
{
	libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
	
	mKinectDevice = 0;
	mPipeline = 0;

	if (freenect2.enumerateDevices() == 0)
	{
		QMessageBox::critical(this, QString::fromLocal8Bit("Open Device Error"), QString::fromLocal8Bit("no device connected!"));
		return;
		//std::cout << "no device connected!" << std::endl;
		//return -1;
	}

	std::string serial = freenect2.getDefaultDeviceSerialNumber();

	bool viewer_enabled = true;

	mPipeline = new libfreenect2::OpenCLPacketPipeline();
	mKinectDevice = freenect2.openDevice(serial);

	if (mKinectDevice == 0)
	{
		QMessageBox::critical(this, QString::fromLocal8Bit("Open Device Error"), QString::fromLocal8Bit("failure opening device!"));
		return;
	}

}

void SmartProjectorMainWindow::timeout()
{
	//if (mIsKinectDeviceInit == false)
	//	this->openDeivce();

	if (listener->hasNewFrame() == false)
		return;

	if (mode == WARP_RESULT)
	{
		libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
		libfreenect2::FrameMap frames;

		listener->waitForNewFrame(frames);

		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];	// 4byte float으로 데이터 들어옴
		mRegistration->apply(rgb, depth, &undistorted, &registered);
		
		// convert from libfreenect2::Frame to cv::Mat
		cv::Mat imageDepth;
		imageDepth = cv::Mat(undistorted.height, undistorted.width, CV_32F);
		std::copy(undistorted.data, undistorted.data + undistorted.width * undistorted.height * undistorted.bytes_per_pixel, imageDepth.data);
		cv::flip(imageDepth, imageDepth, 1);			// 좌우 반전

		warpScreenImage(mPatternImage, imageDepth);

		listener->release(frames);
	}

	if (mode == READY)
	{
		libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
		libfreenect2::FrameMap frames;

		listener->waitForNewFrame(frames);

		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];			// 4byte float으로 데이터 들어옴
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];	// 4byte float으로 데이터 들어옴
		mRegistration->apply(rgb, depth, &undistorted, &registered);

		cv::Mat image;
		cv::Mat imageDepth;
		cv::Mat imageIR;
		cv::Mat ImageUndistored;

		image = cv::Mat(rgb->height, rgb->width, CV_8UC4);
		imageDepth = cv::Mat(undistorted.height, undistorted.width, CV_32F);
		imageIR = cv::Mat(undistorted.height, undistorted.width, CV_32F);
		ImageUndistored = cv::Mat(registered.height, registered.width, CV_8UC4);

		std::copy(rgb->data, rgb->data + rgb->width * rgb->height * rgb->bytes_per_pixel, image.data);										// RGB 이미지 복사
		std::copy(depth->data, depth->data + depth->width * depth->height * depth->bytes_per_pixel, imageIR.data);											// IR 이미지 복사
		std::copy(undistorted.data, undistorted.data + undistorted.width * undistorted.height * undistorted.bytes_per_pixel, imageDepth.data); // undistored depth 복사
		std::copy(registered.data, registered.data + registered.width * registered.height * registered.bytes_per_pixel, ImageUndistored.data); // Depth + RGB calibration 이미지 복사

		cv::flip(image, image, 1);						// 좌우 반전
		cv::flip(imageDepth, imageDepth, 1);			// 좌우 반전
		cv::flip(imageIR, imageIR, 1);					// 좌우 반전
		cv::flip(ImageUndistored, ImageUndistored, 1);	// 좌우 반전
		
		
		// 화면에 뿌려줄 depth
		//cv::Mat imageDepthCopy;
		//imageDepth.convertTo(imageDepthCopy, CV_8U, 1.0 / 50.0);
		cv::cvtColor(ImageUndistored, ImageUndistored, CV_BGRA2RGB);

		if (mCalbrationDone)
		{
			detectProjectionArea(imageDepth, ImageUndistored);
			//detectOptimalRectFromLargestContour(ImageUndistored);
		}
		imageDepth.convertTo(imageDepth, CV_8U, 1.0 / 50.0);
		imageIR.convertTo(imageIR, CV_8U, 1.0 / 50.0);

		//cv::remap(imageDepth, imageIR, mHumanDepthMap_x, mHumanDepthMap_y, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

		cv::cvtColor(image, image, CV_BGRA2RGB);
		cv::cvtColor(imageIR, imageIR, CV_GRAY2RGB);
		cv::cvtColor(imageDepth, imageDepth, CV_GRAY2RGB);

		//if (mCalbrationDone)
		//{
		//	detectOptimalRectFromLargestContour(imageIR);
		//}
		mCurrentImage = image;
		mCurrentIRImage = imageIR;
		mCurrentDepthImage = imageDepth;
		mCurrentCalibrationImage = ImageUndistored;

		QImage qimg((uchar*)mCurrentImage.data, mCurrentImage.cols, mCurrentImage.rows, mCurrentImage.step, QImage::Format_RGB888);
		QImage qimg_depth((uchar*)mCurrentDepthImage.data, mCurrentDepthImage.cols, mCurrentDepthImage.rows, mCurrentDepthImage.step, QImage::Format_RGB888);
		QImage qimg_ir((uchar*)mCurrentIRImage.data, mCurrentIRImage.cols, mCurrentIRImage.rows, mCurrentIRImage.step, QImage::Format_RGB888);
		QImage qimg_ud((uchar*)mCurrentCalibrationImage.data, mCurrentCalibrationImage.cols, mCurrentCalibrationImage.rows, mCurrentCalibrationImage.step, QImage::Format_RGB888);

		mVideoWidget1->SetImage(qimg);
		mVideoWidget2->SetImage(qimg_depth);
		mVideoWidget3->SetImage(qimg_ir);
		mVideoWidget4->SetImage(qimg_ud);

		//mSecondScreen->SetImage(qimg);

		mVideoWidget1->update();
		mVideoWidget2->update();
		mVideoWidget3->update();
		mVideoWidget4->update();
		//mSecondScreen->update();

		listener->release(frames);
	}

	if (mode == CALIBRATING)
	{
		
		libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
		libfreenect2::FrameMap frames;

		listener->waitForNewFrame(frames);

		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];			// 4byte float으로 데이터 들어옴
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];	// 4byte float으로 데이터 들어옴
		mRegistration->apply(rgb, depth, &undistorted, &registered);


		// 칼리브레이션 검출
		if (mSampleTimeout < 0)
		{
			// 컬러 depth registration image 변환 
			cv::Mat image = cv::Mat(registered.height, registered.width, CV_8UC4);
			std::copy(registered.data, registered.data + registered.width * registered.height * registered.bytes_per_pixel, image.data);
			cv::cvtColor(image, image, CV_BGRA2GRAY);
			cv::flip(image, image, 1);

			cv::Mat undistortedImage = cv::Mat(undistorted.height, undistorted.width, CV_32F);
			std::copy(undistorted.data, undistorted.data + undistorted.width * undistorted.height * undistorted.bytes_per_pixel, undistortedImage.data);
			cv::flip(undistortedImage, undistortedImage, 1);

			// 체크보드 찾기 
			bool calResult = findCheckBoard(image);

			if (calResult == false)
			{
				QMessageBox::critical(this, QString::fromLocal8Bit("칼리브레이션 체크보드 검출 오류"), QString::fromLocal8Bit("체크보드 패턴을 검출하지 못했습니다. 다시 시도 합니다."));
				mSampleTimeout = mInterval * 1000;
			}
			else
			{
				
				calculateCalibration(undistortedImage);
				

				mProjectorScreen->showNormal();
				mProjectorScreen->hide();
				
				mCalIteration--;

				// 다음 칼리브레이션을 위한 체크보드 인덱스 설정
				int curindex = (mMaxIteration - mCalIteration);
				int moveX = curindex % 5;
				int moveY = curindex / 5;

				// 새로운 체크보드 패턴 생성
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


				if (mCalIteration == 0)
				{
					solveCalibration();
					mCalbrationDone = true;
					mode = READY;
				}
				else
				{
					mProjectorScreen->showFullScreen();
					mSampleTimeout = mInterval * 1000;
				}
			}
		}
		else
		{
			lcdNumber->display(mSampleTimeout / 1000);
			mSampleTimeout -= mTimer.interval();

			
		}
		listener->release(frames);
	}
	//mTimer.start();
}

// 검사 시작
void SmartProjectorMainWindow::startInspect()
{
//	mIsInspectStarted = true;
	listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
	mKinectDevice->setColorFrameListener(listener);
	mKinectDevice->setIrAndDepthFrameListener(listener);
	mKinectDevice->start();
	mRegistration = new libfreenect2::Registration(mKinectDevice->getIrCameraParams(), mKinectDevice->getColorCameraParams());
	mTimer.start();

}

// 검사 종료
void SmartProjectorMainWindow::endInspect()
{
	mTimer.stop();
	mKinectDevice->stop();
	mKinectDevice->close();
	delete mRegistration;
	//delete listener;

	QImage qimg((uchar*)mDefaultImage.data, mDefaultImage.cols, mDefaultImage.rows, mDefaultImage.step, QImage::Format_RGB888);
	mVideoWidget1->SetImage(qimg);
	mVideoWidget2->SetImage(qimg);
	mVideoWidget3->SetImage(qimg);
	mVideoWidget4->SetImage(qimg);
	//mSecondScreen->SetImage(qimg);
	mVideoWidget1->update();
	mVideoWidget2->update();
	mVideoWidget3->update();
	mVideoWidget4->update();
	//mSecondScreen->update();
}

// 영상 캡쳐
void SmartProjectorMainWindow::calibrationImage()
{
	mSampleTimeout = mInterval * 1000;
	mCalIteration = mMaxIteration;
	int featureSize = mBoardSize.width * mBoardSize.height;
	mLeftHand = cv::Mat(featureSize * 2 * mMaxIteration, 14, CV_32F);
	mRightHand = cv::Mat(featureSize * 2 * mMaxIteration, 1, CV_32F);

	mCurrentTruth = mCornerTruth;   // trutu table 복사 
	mPatternImage.copyTo(mCurrentPatternImage);
	QImage qimg((uchar*)mPatternImage.data, mPatternImage.cols, mPatternImage.rows, mPatternImage.step, QImage::Format_RGB888);
	mProjectorScreen->SetImage(qimg);
	mAddedPointNum = 0;
	mode = CALIBRATING;
	mProjectorScreen->showFullScreen();
}



bool SmartProjectorMainWindow::findCheckBoard(cv::Mat &inputImage, bool drawConners)
{
	// 체크보드 패턴 찾은 경우 
	bool found = false;
	mFoundCorners.clear();

	for (int scale = 1; scale <= mMaxScale; scale++)
	{
		cv::Mat timg;

		// 스케일에 따른 이미지 생성
		if (scale == 1)
			timg = inputImage;
		else
			cv::resize(inputImage, timg, cv::Size(), scale, scale);

		// 체크보드 찾기 
		found = findChessboardCorners(timg, mBoardSize, mFoundCorners,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

		// 체크 보드 찾았으면
		if (found)
		{
			if (scale > 1)
			{
				// 스케일 조절
				cv::Mat cornersMat(mFoundCorners);
				cornersMat *= 1. / scale;

			}
			break;
		}
	}

	if (found == false)
		return false;

	if (drawConners)
	{
		cv::cvtColor(inputImage, inputImage, CV_GRAY2BGR);
		// 체크 보드 코너를 그리기
		drawChessboardCorners(inputImage, mBoardSize, mFoundCorners, true);
	}
	return true;

}

bool SmartProjectorMainWindow::findCheckBoardTruth(cv::Mat &inputImage, bool drawConners)
{
	// 체크보드 패턴 찾은 경우 
	bool found = false;
	mFoundCorners.clear();

	for (int scale = 1; scale <= mMaxScale; scale++)
	{
		cv::Mat timg;

		// 스케일에 따른 이미지 생성
		if (scale == 1)
			timg = inputImage;
		else
			cv::resize(inputImage, timg, cv::Size(), scale, scale);

		// 체크보드 찾기 
		found = findChessboardCorners(timg, mBoardSize, mFoundCorners,0 );

		// 체크 보드 찾았으면
		if (found)
		{
			if (scale > 1)
			{
				// 스케일 조절
				cv::Mat cornersMat(mFoundCorners);
				cornersMat *= 1. / scale;

			}
			break;
		}
	}

	if (found == false)
		return false;

	if (drawConners)
	{
		cv::cvtColor(inputImage, inputImage, CV_GRAY2BGR);
		// 체크 보드 코너를 그리기
		drawChessboardCorners(inputImage, mBoardSize, mFoundCorners, true);
	}
	return true;

}

void SmartProjectorMainWindow::calculateCalibration(const cv::Mat& undistorted)
{
	int i;

	libfreenect2::Freenect2Device::IrCameraParams depthCamera;    ///< Depth camera parameters.
	depthCamera = mKinectDevice->getIrCameraParams();
	const float cx(depthCamera.cx), cy(depthCamera.cy);
	const float fx(1 / depthCamera.fx), fy(1 / depthCamera.fy);
	float* undistorted_data = (float *)undistorted.data;

	//const float bad_point = std::numeric_limits<float>::quiet_NaN();

	cv::Mat A;
	cv::Mat b;
	cv::Mat x;

	//A = cv::Mat()
	std::vector<float> x_vector;
	std::vector<float> y_vector;
	std::vector<float> z_vector;

	std::vector<float> x_pi;
	std::vector<float> y_pi;

	for (i = 0; i < mFoundCorners.size(); i++)
	{
		int r, c;
		r = std::roundf(mFoundCorners[i].y);
		c = std::roundf(mFoundCorners[i].x);

		float x, y, z;
		const float depth_val = undistorted_data[512 * r + c];

		if (isnan(depth_val) || depth_val <= 0.001)
			continue;

		x = (c + 0.5 - cx) * fx * depth_val;
		y = (r + 0.5 - cy) * fy * depth_val;
		z = depth_val;

		x_vector.push_back(x);
		y_vector.push_back(y);
		z_vector.push_back(z);

		x = mCurrentTruth[i].x *(mProjectorImageWidth / 1830.0f) - (mProjectorImageWidth / 2.0f);
		y = mCurrentTruth[i].y *(mProjectorImageHeight / 1330.0f) - (mProjectorImageHeight / 2.0f);

		x_pi.push_back(x);
		y_pi.push_back(y);
	}
	float c1 = mProjectorImageWidth / 2.0f / tan(mProjectorFovX / 2.0f * 0.0174533f);
	float c2 = mProjectorImageHeight / 2.0f / tan(mProjectorFovY / 2.0f * 0.0174533f);
	//int matindex = (mMaxIteration - mCalIteration) * 96;
	for (i = 0; i < x_vector.size(); i++)
	{
		float m1 = x_pi[i] / c1;
		float m2 = y_pi[i] / c2;

		mLeftHand.at<float>(mAddedPointNum, 0) = -y_vector[i];
		mLeftHand.at<float>(mAddedPointNum, 1) = -z_vector[i];
		mLeftHand.at<float>(mAddedPointNum, 2) = -1;
		mLeftHand.at<float>(mAddedPointNum, 3) = m1 * x_vector[i];
		mLeftHand.at<float>(mAddedPointNum, 4) = m1 * y_vector[i];
		mLeftHand.at<float>(mAddedPointNum, 5) = m1 * z_vector[i];
		mLeftHand.at<float>(mAddedPointNum, 6) = m1;
		mLeftHand.at<float>(mAddedPointNum, 7) = 0;
		mLeftHand.at<float>(mAddedPointNum, 8) = 0;
		mLeftHand.at<float>(mAddedPointNum, 9) = 0;
		mLeftHand.at<float>(mAddedPointNum, 10) = 0;
		mLeftHand.at<float>(mAddedPointNum, 11) = 0;
		mLeftHand.at<float>(mAddedPointNum, 12) = 0;
		mLeftHand.at<float>(mAddedPointNum, 13) = 0;

		mRightHand.at<float>(mAddedPointNum, 0) = x_vector[i];
		mAddedPointNum++;

		mLeftHand.at<float>(mAddedPointNum, 0) = 0;
		mLeftHand.at<float>(mAddedPointNum, 1) = 0;
		mLeftHand.at<float>(mAddedPointNum, 2) = 0;
		mLeftHand.at<float>(mAddedPointNum, 3) = 0;
		mLeftHand.at<float>(mAddedPointNum, 4) = 0;
		mLeftHand.at<float>(mAddedPointNum, 5) = 0;
		mLeftHand.at<float>(mAddedPointNum, 6) = 0;
		mLeftHand.at<float>(mAddedPointNum, 7) = -y_vector[i];
		mLeftHand.at<float>(mAddedPointNum, 8) = -z_vector[i];
		mLeftHand.at<float>(mAddedPointNum, 9) = -1;
		mLeftHand.at<float>(mAddedPointNum, 10) = m2 * x_vector[i];
		mLeftHand.at<float>(mAddedPointNum, 11) = m2 * y_vector[i];
		mLeftHand.at<float>(mAddedPointNum, 12) = m2 * z_vector[i];
		mLeftHand.at<float>(mAddedPointNum, 13) = m2;

		mRightHand.at<float>(mAddedPointNum, 0) = x_vector[i];
		mAddedPointNum++;
	}


}

void SmartProjectorMainWindow::loadCalibrationData()
{
	// calibration data path 
	std::string calDataPath = QApplication::applicationDirPath().toLocal8Bit() + "/lvalues.txt";
	std::ifstream infile;
	infile.open(calDataPath);

	if (infile.is_open() == false)
	{
		mCalbrationDone = false;
		return;

	}

	mCalibrationValues = cv::Mat(1, 14, CV_32F);
	int i;
	for (i = 0; i < 14; i++)
	{
		float data;
		infile >> data;
		mCalibrationValues.at<float>(0, i) = data;
	}
	infile.close();

	mCalbrationDone = true;
}
void SmartProjectorMainWindow::solveCalibration()
{
	// 일부 잘못된 뎁스 때문에 다 차지 않았을 경우 
	//if (mAddedPointNum != (54 * 25 * 2))
	//{
	//	cv::Mat newLeft, newRight;
	//	newLeft = cv::Mat(mAddedPointNum, 14, CV_32F);
	//	newRight = cv::Mat(mAddedPointNum, 1, CV_32F);

	//	mLeftHand(cv::Rect(0, 0, 14, mAddedPointNum)).copyTo(newLeft(cv::Rect(0, 0, 14, mAddedPointNum)));
	//	mRightHand(cv::Rect(0, 0, 1, mAddedPointNum)).copyTo(newRight(cv::Rect(0, 0, 1, mAddedPointNum)));
	//	mLeftHand = newLeft;
	//	mRightHand = newRight;
	//}

	cv::Mat qvalues;
	cv::solve(mLeftHand, mRightHand, qvalues, cv::DECOMP_SVD);

	std::string calPath = QApplication::applicationDirPath().toLocal8Bit() + "/lvalues.txt";

	std::ofstream outfile;
	outfile.open(calPath);

	int i;
	for (i = 0; i < 14; i++)
		outfile << qvalues.at<float>(i, 0) << std::endl;

	mCalibrationValues = qvalues;
	mCalibrationValues = mCalibrationValues.t();
	outfile.close();

}

void SmartProjectorMainWindow::detectProjectionArea(cv::Mat& undistorted, cv::Mat& depthimg)
{
	// 1. depth to X, Y, Z (world coordinate)
	libfreenect2::Freenect2Device::IrCameraParams depthCamera;    ///< Depth camera parameters.
	depthCamera = mKinectDevice->getIrCameraParams();
	const float cx(depthCamera.cx), cy(depthCamera.cy);
	const float fx(1 / depthCamera.fx), fy(1 / depthCamera.fy);
	float* undistorted_data = (float *)undistorted.data;

	int i, j;
	float x, y, z;
	float xi, yi;

	float c1 = mProjectorImageWidth / 2.0f / tan(mProjectorFovX / 2.0f * 0.0174533f);
	float c2 = mProjectorImageHeight / 2.0f / tan(mProjectorFovY / 2.0f * 0.0174533f);
	float halfwidth = mProjectorImageWidth / 2.0f;
	float halfhegiht = mProjectorImageHeight / 2.0f;

	mProjectionAreaMask.setTo(0);

	for (i = 0; i < undistorted.rows; i++)
	{
		for (j = 0; j < undistorted.cols; j++)
		{
			float *XYZ_Ptr = mPointXYZ.ptr<float>(i);

			cv::Vec3b dimageVec = depthimg.at<cv::Vec3b>(i, j);
			const float depth_val = undistorted_data[512 * i + j];

			if (isnan(depth_val) || depth_val <= 0.001)
				continue;

			x = (j + 0.5 - cx) * fx * depth_val;
			y = (i + 0.5 - cy) * fy * depth_val;
			z = depth_val;

			//xp = (q1*xk + q2*yk + q3*zk + q4)/(q9*xk + q10*yk + q11*zk + 1)
			xi =c1* ( x + mCalibrationValues.at<float>(0, 0) * y + mCalibrationValues.at<float>(0, 1) * z + mCalibrationValues.at<float>(0, 2)) /
				(mCalibrationValues.at<float>(0, 3) * x + mCalibrationValues.at<float>(0, 4) * y + mCalibrationValues.at<float>(0, 5) * z + mCalibrationValues.at<float>(0, 6));

			yi = c2* (x + mCalibrationValues.at<float>(0, 7) * y + mCalibrationValues.at<float>(0, 8) * z + mCalibrationValues.at<float>(0, 9)) /
				(mCalibrationValues.at<float>(0, 10) * x + mCalibrationValues.at<float>(0, 11) * y + mCalibrationValues.at<float>(0, 12) * z + mCalibrationValues.at<float>(0, 13));

			mImage_xPos.at<float>(i, j) = xi + halfwidth;
			mImage_yPos.at<float>(i, j) = yi + halfhegiht;

			if ((xi > -halfwidth) && (xi < halfwidth) && (yi > -halfhegiht) && (yi < halfhegiht))
			{
				dimageVec[0] = 255;
				dimageVec[1] = dimageVec[1] / 2;
				dimageVec[2] = dimageVec[2] / 2;
				depthimg.at<cv::Vec3b>(i, j) = dimageVec;

				mProjectionAreaMask.at<uchar>(i, j) = 255;
			}

			XYZ_Ptr[j * 3] = x;
			XYZ_Ptr[j * 3+1] = y;
			XYZ_Ptr[j * 3+2] = z;
		}
	}


}


void SmartProjectorMainWindow::detectOptimalRectFromLargestContour(cv::Mat& depthimg)
{
	// 1. Largest Contour 검출
	// 2. Contour안의 최대 사각형 검출
	// now extract the outer contour
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(mProjectionAreaMask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));

	std::cout << "found contours: " << contours.size() << std::endl;

	cv::Mat contourImage = cv::Mat::zeros(mProjectionAreaMask.size(), CV_8UC3);;

	unsigned int maxSize = 0;
	unsigned int id = 0;
	int i, j;
	for (int i = 0; i<contours.size(); ++i)
	{
		if (contours.at(i).size() > maxSize)
		{
			maxSize = contours.at(i).size();
			id = i;
		}
	}

	std::cout << "chosen id: " << id << std::endl;
	std::cout << "max size: " << maxSize << std::endl;

	cv::Mat contourMask = cv::Mat::zeros(mProjectionAreaMask.size(), CV_8UC1);
	cv::drawContours(contourMask, contours, id, cv::Scalar(255), -1, 8, hierarchy, 0, cv::Point());
	cv::drawContours(depthimg, contours, id, cv::Scalar(255,0,0), 2, 8, hierarchy, 0, cv::Point());

	cv::Rect OptimalRect = mOpRect.findOptimalRect(contourMask);
	cv::rectangle(depthimg, OptimalRect, cv::Scalar(0, 0, 255), 2);

	//cv::Mat 
	//cv::Mat maskedImage;
	//input.copyTo(maskedImage);
	//for (unsigned int y = 0; y<maskedImage.rows; ++y)
	//	for (unsigned int x = 0; x<maskedImage.cols; ++x)
	//	{
	//		maskedImage.at<cv::Vec3b>(y, x)[2] = 255;
	//	}
	//input.copyTo(maskedImage, mask2);

	//cv::imshow("masked image", maskedImage);
	//cv::imwrite("interiorBoundingBoxResult.png", maskedImage);
	//cv::waitKey(0);

}

// 영상 녹화 시작
void SmartProjectorMainWindow::correctionScreen()
{
	// warp mode 변경
	if (mode != WARP_RESULT)
	{
		mode = WARP_RESULT;
		mWarpingScreen->showFullScreen();
		return;
	}

	// 일반 mode 변경
	if (mode == WARP_RESULT)
	{
		mode = READY;
		mWarpingScreen->showNormal();
		mWarpingScreen->hide();
		return;
	}
}


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
	const float cx = 259.740692f;  // center of depth image
	const float cy = 205.260895f;  // center of depth image

	float humanFovX = mFovXSpinBox->value() *  0.0174533f;  // radian
	float humanFovY = mFovYSpinBox->value() *  0.0174533f;  // radian

	float fxh = 512 / 2 / tan(humanFovX / 2.0f); // fx for human depth image
	float fyh = 424 / 2 / tan(humanFovY / 2.0f); // fy for human depth image
	fxh = 1.0f/  fxh;
	fyh = 1.0f / fyh;
	//float* undistorted_data = (float *)undistorted.data;

	int i, j;
	float x, y, z;
	float xi, yi;

	float c1 = mProjectorImageWidth / 2.0f / tan(mProjectorFovX / 2.0f * 0.0174533f);
	float c2 = mProjectorImageHeight / 2.0f / tan(mProjectorFovY / 2.0f * 0.0174533f);
	float halfwidth = mProjectorImageWidth / 2.0f;
	float halfhegiht = mProjectorImageHeight / 2.0f;

	mProjectionAreaMask.setTo(0);
	cv::Mat humanDepth;
	cv::remap(undistorted, humanDepth, mHumanDepthMap_x, mHumanDepthMap_y, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
	// Calculate ProjectionArea
	for (i = 0; i < humanDepth.rows; i++)
	{
		float* hDepthPtr = humanDepth.ptr<float>(i);
		for (j = 0; j < humanDepth.cols; j++)
		{
			//cv::Vec3b dimageVec = depthimg.at<cv::Vec3b>(i, j);
			const float depth_val = hDepthPtr[j];

			if (isnan(depth_val) || depth_val <= 0.001)
				continue;

			x = (j + 0.5 - cx) * fxh * depth_val;
			y = (i + 0.5 - cy) * fyh * depth_val;
			z = depth_val;

			//xp = (q1*xk + q2*yk + q3*zk + q4)/(q9*xk + q10*yk + q11*zk + 1)
			xi = c1* (x + mCalibrationValues.at<float>(0, 0) * y + mCalibrationValues.at<float>(0, 1) * z + mCalibrationValues.at<float>(0, 2)) /
				(mCalibrationValues.at<float>(0, 3) * x + mCalibrationValues.at<float>(0, 4) * y + mCalibrationValues.at<float>(0, 5) * z + mCalibrationValues.at<float>(0, 6));

			yi = c2* (x + mCalibrationValues.at<float>(0, 7) * y + mCalibrationValues.at<float>(0, 8) * z + mCalibrationValues.at<float>(0, 9)) /
				(mCalibrationValues.at<float>(0, 10) * x + mCalibrationValues.at<float>(0, 11) * y + mCalibrationValues.at<float>(0, 12) * z + mCalibrationValues.at<float>(0, 13));

			mImage_xPos.at<float>(i, j) = xi + halfwidth;
			mImage_yPos.at<float>(i, j) = yi + halfhegiht;

			if ((xi > -halfwidth) && (xi < halfwidth) && (yi > -halfhegiht) && (yi < halfhegiht))
			{
				//dimageVec[0] = 255;
				//dimageVec[1] = dimageVec[1] / 2;
				//dimageVec[2] = dimageVec[2] / 2;
				//depthimg.at<cv::Vec3b>(i, j) = dimageVec;
				mProjectionAreaMask.at<uchar>(i, j) = 255;
			}
		}
	}

	// 1. Largest Contour 검출
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(mProjectionAreaMask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));

	std::cout << "found contours: " << contours.size() << std::endl;

	cv::Mat contourImage = cv::Mat::zeros(mProjectionAreaMask.size(), CV_8UC3);;

	unsigned int maxSize = 0;
	unsigned int id = 0;
	//int i, j;
	for ( i = 0; i<contours.size(); ++i)
	{
		if (contours.at(i).size() > maxSize)
		{
			maxSize = contours.at(i).size();
			id = i;
		}
	}

	std::cout << "chosen id: " << id << std::endl;
	std::cout << "max size: " << maxSize << std::endl;
	cv::Mat contourMask = cv::Mat::zeros(mProjectionAreaMask.size(), CV_8UC1);
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

	yPos = OptimalRect.y;
	for (i = OptimalRect.x; i < (OptimalRect.x+OptimalRect.width); i++)
	{
		float xValue = mImage_xPos.at<float>(yPos, i);
		float yValue = mImage_yPos.at<float>(yPos, i);
		rectContour.push_back(cv::Point(xValue, yValue));
	}

	xPos = OptimalRect.x + OptimalRect.width;
	for (i = OptimalRect.y; i < (OptimalRect.y +OptimalRect.height); i++)
	{
		float xValue = mImage_xPos.at<float>(i, xPos);
		float yValue = mImage_yPos.at<float>(i, xPos);
		rectContour.push_back(cv::Point(xValue, yValue));
	}

	yPos = OptimalRect.y + OptimalRect.height;
	for (i = (OptimalRect.x + OptimalRect.width); i >= OptimalRect.x; i--)
	{
		float xValue = mImage_xPos.at<float>(yPos, i);
		float yValue = mImage_yPos.at<float>(yPos, i);
		rectContour.push_back(cv::Point(xValue, yValue));
	}

	xPos = OptimalRect.x;
	for (i = (OptimalRect.y + OptimalRect.height); i >= OptimalRect.y; i--)
	{
		float xValue = mImage_xPos.at<float>(i, xPos);
		float yValue = mImage_yPos.at<float>(i, xPos);
		rectContour.push_back(cv::Point(xValue, yValue));
	}

	// Draw Rect
	std::vector<std::vector<cv::Point> > contours2;
	contours2.push_back(rectContour);
	//mWarpImage = mPatternImage.clone();
	//cv::resize(mWarpImage, mWarpImage, cv::Size(mProjectorImageWidth, mProjectorImageHeight));
	//cv::drawContours(mWarpImage, contours2, 0, cv::Scalar(255, 0, 0), 2);
	//QImage qimg((uchar*)mWarpImage.data, mWarpImage.cols, mWarpImage.rows, mWarpImage.step, QImage::Format_RGB888);

	//mWarpingScreen->SetImage(qimg);
	//mWarpingScreen->update();


	// 3. warping 수행
	bool Checkwapring = true;
	if (Checkwapring)
	{

		cv::Mat map_x;
		cv::Mat map_y;

		map_x = mImage_xPos(OptimalRect).clone();
		map_y = mImage_yPos(OptimalRect).clone();

		cv::resize(map_x, map_x, cv::Size(mProjectorImageWidth, mProjectorImageHeight));
		cv::resize(map_y, map_y, cv::Size(mProjectorImageWidth, mProjectorImageHeight));

		for (int i = 0; i < map_x.rows; i++)
		{
			float *map_x_Ptr = map_x.ptr<float>(i);
			float *map_y_Ptr = map_y.ptr<float>(i);
			for (int j = 0; j < map_x.cols; j++)
			{
				map_x_Ptr[j] = 2 * j - map_x_Ptr[j];
				map_y_Ptr[j] = 2 * i - map_y_Ptr[j];
			}
		}

		//cv::Mat warpImg;
		if (screenImage.cols != mProjectorImageWidth || screenImage.rows != mProjectorImageHeight)
			cv::resize(screenImage, screenImage, cv::Size(mProjectorImageWidth, mProjectorImageHeight));

		mWarpImage.create(screenImage.size(), screenImage.type());
		cv::remap(mPatternImage, mWarpImage, map_x, map_y, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
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

void SmartProjectorMainWindow::create_Human_To_depth_Map()
{
	const float cx = 259.740692f;  // center of depth image
	const float cy = 205.260895f;  // center of depth image
	
	const float fxk = 365.091095f; // fx for kinect depth image
	const float fyk = 365.091095f; // fy for kinect depth image

	float humanFovX = mFovXSpinBox->value() *  0.0174533f;  // radian
	float humanFovY = mFovYSpinBox->value() *  0.0174533f;  // radian

	float fxh = 512 / 2 / tan(humanFovX / 2.0f); // fx for human depth image
	float fyh = 424 / 2 / tan(humanFovY / 2.0f); // fy for human depth image
	
	int i, j;
	float fxRatio = fxk / fxh;
	float fyRatio = fyk / fyh;

	mHumanDepthMap_x = cv::Mat(424, 512, CV_32F);
	mHumanDepthMap_y = cv::Mat(424, 512, CV_32F);

	for (i = 0; i < 424; i++)
	{
		for (j = 0; j < 512; j++)
		{
			float ixk = (j + 0.5 - cx) * fxRatio + cx - 0.5;
			float iyk = (i + 0.5 - cy) * fyRatio + cy - 0.5;

			mHumanDepthMap_x.at<float>(i, j) = ixk;
			mHumanDepthMap_y.at<float>(i, j) = iyk;
		}
	}

}