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

SmartProjectorMainWindow::SmartProjectorMainWindow() : READY(0), CALIBRATING(1), SHOW_CURRENT_RESULT(2), WARP_RESULT(3), mMaxIteration(25), mNumAvgFrame(10)
{
    // UI 셋업
	setupUi(this);
	
	openDeivce();

	connect(&mTimer, SIGNAL(timeout()), this, SLOT(timeout()));
	connect(mInspectStartBtn, SIGNAL(pressed()), this, SLOT(startKinect()));
	connect(mInspectEndBtn, SIGNAL(pressed()), this, SLOT(endKinect()));
	connect(mKinectCalibrationBtn, SIGNAL(pressed()), this, SLOT(calibrationImage()));
	connect(mCorrection, SIGNAL(pressed()), this, SLOT(correctionScreen()));

	connect(mFovXSpinBox, SIGNAL(valueChanged(double)), this, SLOT(changeHumanSetting(double)));
	connect(mFovYSpinBox, SIGNAL(valueChanged(double)), this, SLOT(changeHumanSetting(double)));
	connect(mRotateXSpinBox, SIGNAL(valueChanged(double)), this, SLOT(changeHumanSetting(double)));
	connect(mRotateYSpinBox, SIGNAL(valueChanged(double)), this, SLOT(changeHumanSetting(double)));
	connect(mRotateZSpinBox, SIGNAL(valueChanged(double)), this, SLOT(changeHumanSetting(double)));

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
	connect(mWarpingScreen, SIGNAL(SigClickTrigger(int)), this, SLOT(setCorrection(int)));

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
	mProjectionAreaMaskUser = cv::Mat(424, 512, CV_8U);

	mImage_xPos = cv::Mat(424, 512, CV_32F);
	mImage_yPos = cv::Mat(424, 512, CV_32F);
//	mImage_xPosUser = cv::Mat(424, 512, CV_32F);
//	mImage_yPosUser = cv::Mat(424, 512, CV_32F);

	mPointXYZ = cv::Mat(424, 512, CV_32FC3);	   // 키넥트 depth의 x,y,z 좌표값
	mPointXYZ_User = cv::Mat(424, 512, CV_32FC3);  // 사용자 depth의 x,y,z 실제 좌표값

	loadCalibrationData();
	create_Human_To_depth_Map();

	mAccFrameNumber = 0;
	mCorrectionFlag = true;
}

SmartProjectorMainWindow::~SmartProjectorMainWindow()
{	
	mKinectDevice->stop();
	mKinectDevice->close();
}

void SmartProjectorMainWindow ::setCorrection(int value)
{
	if (mCorrectionFlag == true)
		mCorrectionFlag = false;
	else
		mCorrectionFlag = true;

}

void SmartProjectorMainWindow::getKinectImage(cv::Mat &image, cv::Mat &imageDepth, cv::Mat& imageIR, cv::Mat& imageRegisted)
{
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
	libfreenect2::FrameMap frames;

	listener->waitForNewFrame(frames);

	libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
	libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];			// 4byte float으로 데이터 들어옴
	libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];	// 4byte float으로 데이터 들어옴
	mRegistration->apply(rgb, depth, &undistorted, &registered);

	image = cv::Mat(rgb->height, rgb->width, CV_8UC4);						// 키넥트 RGB이미지
	imageDepth = cv::Mat(undistorted.height, undistorted.width, CV_32F);	// Depth 이미지 
	imageIR = cv::Mat(undistorted.height, undistorted.width, CV_32F);		// IR 이미지
	imageRegisted = cv::Mat(registered.height, registered.width, CV_8UC4);  // Depth + RGB calibration이미지

	std::copy(rgb->data, rgb->data + rgb->width * rgb->height * rgb->bytes_per_pixel, image.data);											// RGB 이미지 복사
	std::copy(ir->data, ir->data + ir->width * ir->height * ir->bytes_per_pixel, imageIR.data);								// IR 이미지 복사
	std::copy(undistorted.data, undistorted.data + undistorted.width * undistorted.height * undistorted.bytes_per_pixel, imageDepth.data);	// undistored depth 복사
	std::copy(registered.data, registered.data + registered.width * registered.height * registered.bytes_per_pixel, imageRegisted.data);	// Depth + RGB calibration 이미지 복사

	if (mVerticalFlip->checkState() == Qt::Unchecked)
	{
		cv::flip(image, image, 1);						// 좌우 반전
		cv::flip(imageDepth, imageDepth, 1);			// 좌우 반전
		cv::flip(imageIR, imageIR, 1);					// 좌우 반전
		cv::flip(imageRegisted, imageRegisted, 1);	    // 좌우 반전
	}
	else
	{
		cv::flip(image, image, 0);						// 상하 반전
		cv::flip(imageDepth, imageDepth, 0);			// 상하 반전
		cv::flip(imageIR, imageIR, 0);					// 상하 반전
		cv::flip(imageRegisted, imageRegisted, 0);	    // 상하 반전

	}

	// BGRA to RGB
	cv::cvtColor(imageRegisted, imageRegisted, CV_BGRA2RGB);
	cv::cvtColor(image, image, CV_BGRA2RGB);

	// Release
	listener->release(frames);
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
			mAvgDepth = mAvgDepth / (float)mNumAvgFrame;
			warpScreenImage(mPatternImage, imageDepth);
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

// Kinect 시작
void SmartProjectorMainWindow::startKinect()
{
	listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
	mKinectDevice->setColorFrameListener(listener);
	mKinectDevice->setIrAndDepthFrameListener(listener);
	mKinectDevice->start();
	mRegistration = new libfreenect2::Registration(mKinectDevice->getIrCameraParams(), mKinectDevice->getColorCameraParams());
	mTimer.start();
}

// Kinect 종료
void SmartProjectorMainWindow::endKinect()
{
	mTimer.stop();
	mKinectDevice->stop();
	mKinectDevice->close();
	delete mRegistration;
	delete listener;

	QImage qimg((uchar*)mDefaultImage.data, mDefaultImage.cols, mDefaultImage.rows, mDefaultImage.step, QImage::Format_RGB888);
	mVideoWidget1->SetImage(qimg);
	mVideoWidget2->SetImage(qimg);
	mVideoWidget3->SetImage(qimg);
	mVideoWidget4->SetImage(qimg);
	mVideoWidget1->update();
	mVideoWidget2->update();
	mVideoWidget3->update();
	mVideoWidget4->update();
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


//프로젝션 에어리어 찾기
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
		float *XYZ_Ptr = mPointXYZ.ptr<float>(i);

		for (j = 0; j < undistorted.cols; j++)
		{
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

	if (mAutoPlaneRotationCB->checkState() == Qt::Checked)
	{
		// calculate Plane normal vector 
		float angleX, angleY;
		calculatePlaneNormalVector(angleX, angleY);
		
		// set new rotation 
		//if (mVerticalFlip->checkState() == Qt::Unchecked)
		//{
			mRotateXSpinBox->setValue(angleX);
			mRotateYSpinBox->setValue(-angleY);
		//}
		//else
		//{
		//	mRotateXSpinBox->setValue(-angleX);
		//	mRotateYSpinBox->setValue(angleY);
		//}
	}
}


//유저관점에서 프로젝션 에어리어 검출
void SmartProjectorMainWindow::detectProjectionAreaUser(cv::Mat& userDepth, cv::Mat& IRimg)
{
	//float theta1 = mRotateXSpinBox->value();   // x축 회전 값
	//float theta2 = mRotateYSpinBox->value();   // y축 회전 값
	//float theta3 = mRotateZSpinBox->value();   // z축 회전 값

	//theta1 = theta1 * 0.0174533f; // 회전값 radian 변환
	//theta2 = theta2 * 0.0174533f; // 회전값 radian 변환
	//theta3 = theta3 * 0.0174533f; // 회전값 radian 변환

	//float r11, r12, r13, r21, r22, r23, r31, r32, r33;
	//r11 = cos(theta2) * cos(theta3);
	//r12 = cos(theta2) * sin(theta3);
	//r13 = -sin(theta2);

	//r21 = sin(theta1) * sin(theta2) * cos(theta3) - cos(theta1) * sin(theta3);
	//r22 = sin(theta1) * sin(theta2) * sin(theta3) + cos(theta1) * cos(theta3);
	//r23 = sin(theta1) * cos(theta2);

	//r31 = cos(theta1) * sin(theta2) * cos(theta3) + sin(theta1) * sin(theta3);
	//r32 = cos(theta1) * sin(theta2) * sin(theta3) - sin(theta1) * cos(theta3);
	//r33 = cos(theta1) * cos(theta2);

	//// 1. depth to X, Y, Z (world coordinate)
	//const float cx = 259.740692f;  // center of depth image
	//const float cy = 205.260895f;  // center of depth image

	//const float fxk = 365.091095f; // fx for kinect depth image
	//const float fyk = 365.091095f; // fy for kinect depth image

	//float humanFovX = mFovXSpinBox->value() *  0.0174533f;  // radian
	//float humanFovY = mFovYSpinBox->value() *  0.0174533f;  // radian

	//float cxh = (512.0 - 1) / 2;
	//float cyh = (424.0 - 1) / 2;

	//float fxh = 512 / 2 / tan(humanFovX / 2.0f); // fx for human depth image
	//float fyh = 424 / 2 / tan(humanFovY / 2.0f); // fy for human depth image

	////float fxh = fxk;// cx / tan(humanFovX / 2.0f); // fx for human depth image
	////float fyh = fyk;// cy / tan(humanFovY / 2.0f); // fy for human depth image

	//float* user_depth_data = (float *)userDepth.data;

	//int i, j;
	//float x, y, z;
	//float rx, ry, rz;
	//float xi, yi;

	//float c1 = mProjectorImageWidth / 2.0f / tan(mProjectorFovX / 2.0f * 0.0174533f);
	//float c2 = mProjectorImageHeight / 2.0f / tan(mProjectorFovY / 2.0f * 0.0174533f);
	//float halfwidth = mProjectorImageWidth / 2.0f;
	//float halfhegiht = mProjectorImageHeight / 2.0f;

	//mProjectionAreaMaskUser.setTo(0);

	//for (i = 0; i < userDepth.rows; i++)
	//{
	//	for (j = 0; j < userDepth.cols; j++)
	//	{
	//		//float *XYZ_Ptr = mPointXYZ_User.ptr<float>(i);
	//		cv::Vec3b dimageVec = IRimg.at<cv::Vec3b>(i, j);

	//		const float depth_val = user_depth_data[512 * i + j];

	//		if (isnan(depth_val) || depth_val <= 0.001)
	//			continue;

	//		x = (j + 0.5 - cxh) / fxh * depth_val;
	//		y = (i + 0.5 - cyh) / fyh * depth_val;
	//		z = depth_val;

	//		// 역 회전변환 적용
	//		rx = r11 * x + r12 *y + r13 * z;
	//		ry = r21 * x + r22 *y + r23 * z;
	//		rz = r31 * x + r32 *y + r33 * z;

	//		//xp = (q1*xk + q2*yk + q3*zk + q4)/(q9*xk + q10*yk + q11*zk + 1)
	//		xi = c1* (rx + mCalibrationValues.at<float>(0, 0) * ry + mCalibrationValues.at<float>(0, 1) * rz + mCalibrationValues.at<float>(0, 2)) /
	//			(mCalibrationValues.at<float>(0, 3) * rx + mCalibrationValues.at<float>(0, 4) * ry + mCalibrationValues.at<float>(0, 5) * rz + mCalibrationValues.at<float>(0, 6));

	//		yi = c2* (rx + mCalibrationValues.at<float>(0, 7) * ry + mCalibrationValues.at<float>(0, 8) * rz + mCalibrationValues.at<float>(0, 9)) /
	//			(mCalibrationValues.at<float>(0, 10) *rx + mCalibrationValues.at<float>(0, 11) * ry + mCalibrationValues.at<float>(0, 12) * rz + mCalibrationValues.at<float>(0, 13));

	//		mImage_xPosUser.at<float>(i, j) = xi + halfwidth;
	//		mImage_yPosUser.at<float>(i, j) = yi + halfhegiht;

	//		if ((xi > -halfwidth) && (xi < halfwidth) && (yi > -halfhegiht) && (yi < halfhegiht))
	//		{
	//			dimageVec[0] = 255;
	//			dimageVec[1] = dimageVec[1] / 2;
	//			dimageVec[2] = dimageVec[2] / 2;
	//			IRimg.at<cv::Vec3b>(i, j) = dimageVec;

	//			mProjectionAreaMaskUser.at<uchar>(i, j) = 255;
	//		}
	//	}
	//}

	cv::remap(mProjectionAreaMask, mProjectionAreaMaskUser, mHumanDepthMap_x, mHumanDepthMap_y, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	int i;
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


	cv::drawContours(IRimg, contours, id, cv::Scalar(0, 0, 255), 2, 8, hierarchy, 0, cv::Point());
	
	cv::Mat contourMask = cv::Mat::zeros(mProjectionAreaMaskUser.size(), CV_8UC1);
	cv::drawContours(contourMask, contours, id, cv::Scalar(255), -1, 8, hierarchy, 0, cv::Point());
	
	
	// 2. Contour안의 최대 사각형 검출
	cv::Rect OptimalRect = mOpRect.findOptimalRect(contourMask);

	cv::rectangle(IRimg, OptimalRect, cv::Scalar(0, 255, 0), -1);

}


//largestContour에서 optimalrect 찾기
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
	int i;// j;
	for (i = 0; i<contours.size(); ++i)
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


///////////////////////////////////////
// 워핑을 수행하는 부
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

	//프로젝션 영역 마스크
	mProjectionAreaMask.setTo(0);

	for (i = 0; i < undistorted.rows; i++)
	{
		float *XYZ_Ptr = mPointXYZ.ptr<float>(i);

		for (j = 0; j < undistorted.cols; j++)
		{
			//cv::Vec3b dimageVec = depthimg.at<cv::Vec3b>(i, j);
			const float depth_val = undistorted_data[512 * i + j];

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
	for ( i = 0; i<contours.size(); ++i)
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
	for (i = (OptimalRect.x + OptimalRect.width-1); i >= OptimalRect.x; i--)
	{
		float xValue = mImage_xPos.at<float>(yPos, i);
		float yValue = mImage_yPos.at<float>(yPos, i);
		rectContour.push_back(cv::Point(xValue, yValue));
	}

	xPos = OptimalRect.x;
	for (i = (OptimalRect.y + OptimalRect.height-1); i >= OptimalRect.y; i--)
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
		cv::Mat map_x;
		cv::Mat map_y;

		map_x = mImage_xPos(OptimalRect).clone();			//최종 이미지 x좌표
		map_y = mImage_yPos(OptimalRect).clone();			//죄종 이미지 y좌표

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
		cv::Mat resizedScreenImg;
		if (screenImage.cols != mProjectorImageWidth || screenImage.rows != mProjectorImageHeight)
			cv::resize(screenImage, resizedScreenImg, cv::Size(mProjectorImageWidth, mProjectorImageHeight));
		else
			resizedScreenImg = screenImage.clone();

		mWarpImage=  cv::Mat::zeros(resizedScreenImg.size(), resizedScreenImg.type());
		//cv::remap(resizedScreenImg, mWarpImage, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

		if (mCorrectionFlag)
		{
			// homography transform
			if (mHomographyCB->checkState() == Qt::Unchecked)
			{
				cv::remap(resizedScreenImg, mWarpImage, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
			}
			else
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

//사람관점에서의 depth map
void SmartProjectorMainWindow::create_Human_To_depth_Map()
{
	float theta1 = mRotateXSpinBox->value();   // x축 회전 값
	float theta2 = mRotateYSpinBox->value();   // y축 회전 값
	float theta3 = mRotateZSpinBox->value();   // z축 회전 값

	theta1 = theta1 * 0.0174533f; // 회전값 radian 변환
	theta2 = theta2 * 0.0174533f; // 회전값 radian 변환
	theta3 = theta3 * 0.0174533f; // 회전값 radian 변환
	
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;
	r11 = cos(theta2) * cos(theta3);
	r12 = cos(theta2) * sin(theta3);
	r13 = -sin(theta2);

	r21 = sin(theta1) * sin(theta2) * cos(theta3) - cos(theta1) * sin(theta3);
	r22 = sin(theta1) * sin(theta2) * sin(theta3) + cos(theta1) * cos(theta3);
	r23 = sin(theta1) * cos(theta2);

	r31 = cos(theta1) * sin(theta2) * cos(theta3) + sin(theta1) * sin(theta3);
	r32 = cos(theta1) * sin(theta2) * sin(theta3) - sin(theta1) * cos(theta3);
	r33 = cos(theta1) * cos(theta2);

	const float cx = 259.740692f;  // center of depth image
	const float cy = 205.260895f;  // center of depth image
	
	const float fxk = 365.091095f; // fx for kinect depth image
	const float fyk = 365.091095f; // fy for kinect depth image

	float humanFovX = mFovXSpinBox->value() *  0.0174533f;  // radian
	float humanFovY = mFovYSpinBox->value() *  0.0174533f;  // radian

	float cxh = (512.0 - 1) / 2;
	float cyh = (424.0 - 1) / 2;

	float fxh = 512/2 / tan(humanFovX / 2.0f); // fx for human depth image
	float fyh = 424/2 / tan(humanFovY / 2.0f); // fy for human depth image
	
	
	//float fxh = 365.091095f;
	//float fyh = 365.091095f;

	int i, j;
	float fxRatio = fxk / fxh;
	float fyRatio = fyk / fyh;

	mHumanDepthMap_x = cv::Mat(424, 512, CV_32F);
	mHumanDepthMap_y = cv::Mat(424, 512, CV_32F);
	
	r11 = r11 / fxh;
	r12 = r12 / fyh;
	r21 = r21 / fxh;
	r22 = r22 / fyh;
	r31 = r31 / fxh;
	r32 = r32 / fyh;

	//float halfwidth = 512.0f / 2.0f;
	//float halfheight = 424.0f / 2.0f;
	for (i = 0; i < 424; i++)
	{
		for (j = 0; j < 512; j++)
		{
			float ixh = j + 0.5 - cxh;
			float iyh = i + 0.5 - cyh;

			float ixk = ( r11 * ixh + r12 * iyh + r13) /  (r31 *ixh + r32 * iyh + r33) * fxk + cx - 0.5;
			float iyk = ( r21 * ixh + r22 * iyh + r23) /  (r31 *ixh + r32 * iyh + r33) * fyk + cy - 0.5;

			mHumanDepthMap_x.at<float>(i, j) = ixk;
			mHumanDepthMap_y.at<float>(i, j) = iyk;
		}
	}

	// print trace information
	QString str1 = "Human Eye Parameters are changed";
	QString str2 = "FOV_X : " + QString::number(humanFovX* 57.2958f);
	QString str3 = "FOV_Y : " + QString::number(humanFovX* 57.2958f);
	QString str4 = "R_X : " + QString::number(theta1* 57.2958f);
	QString str5 = "R_Y : " + QString::number(theta2* 57.2958f);
	QString str6 = "R_Z : " + QString::number(theta3* 57.2958f);
	trace(str1);
	trace(str2);
	trace(str3);
	trace(str4);
	trace(str5);
	trace(str6);
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
	
	int i, j;
	for (i = 0; i < mProjectionAreaMask.rows; i++)
	{
		uchar* linebuf = mProjectionAreaMask.ptr<uchar>(i);
		float* xyzBuf = mPointXYZ.ptr<float>(i);
		for (j = 0; j < mProjectionAreaMask.cols; j++)
		{

			if (linebuf[j] == 255)
			{
				float x = *(xyzBuf + j*3);
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