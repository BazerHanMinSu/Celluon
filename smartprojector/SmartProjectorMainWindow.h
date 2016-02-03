#ifndef FISHEYE_DEWARPING_MAIN_WINDOWS_H_
#define FISHEYE_DEWARPING_MAIN_WINDOWS_H_


#include "ui_SmartProjectorMainWindow.h"
#include <QMainwindow>
#include <QtGui>

#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>


#include <iostream>
#include <signal.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include "optimalRectDDJ.h"
/// @brief FisheyeDewarping 프로그램의 메인 윈도우 클래스입니다.
///
/// 이 메인 윈도우 상에서 wiget들과 버튼이 생성되고 초기화 됩니다.
class SmartProjectorMainWindow : public QMainWindow, public Ui::CarInspectMainWindow
{
    Q_OBJECT
public:

	/// @brief 생성자RegNewChart()
	///
	///	생성자에서는 Ui를 생성하고  
	/// DB의 연결 정보를 읽어온다. 
	/// 툴바의 버튼에 액션과 관련 함수를 연결(connect)시켜 준다. 
	SmartProjectorMainWindow();

	/// @brief 소멸자
	///
	///	소멸자에서 하는일은 별로 없음..
	~SmartProjectorMainWindow();
    

	//void InitListView();
	//void InitPreViews();
	//void InitStatusBar();
	bool findCheckBoard(cv::Mat &inputImage, bool drawConners = true);
	bool findCheckBoardTruth(cv::Mat &inputImage, bool drawConners = true);

	void calculateCalibration(const cv::Mat& undistorted);
	void solveCalibration();
	void loadCalibrationData();

	void detectProjectionArea(cv::Mat& undistorted, cv::Mat& depthimg);

	void detectOptimalRectFromLargestContour(cv::Mat& depthimg);
	//bool calculateCalibrationParameter(cv::Mat &inputDepth, )
	void warpScreenImage(cv::Mat &screenImage, cv::Mat &undistorted);

public slots:
	void openDeivce();
	void timeout();

	// 검사 시작
	void startInspect();
	
	// 검사 종료
	void endInspect();

	// 칼리브레이션
	void calibrationImage();

	// 보정 수행
	void correctionScreen();
	
	// human Fov 변화
	void changeHumanFovX(double d){
		create_Human_To_depth_Map();
	}

	void changeHumanFovY(double d){
		create_Human_To_depth_Map();
	}
private:
	// 동영상을 보여주기 위한 타이머
	QTimer mTimer; 

	// 현재 영상
	cv::Mat mCurrentImage;
	cv::Mat mCurrentIRImage;
	cv::Mat mCurrentDepthImage;
	cv::Mat mCurrentCalibrationImage;

	// 검사 종료시 보여주는 defalut 영상
	cv::Mat mDefaultImage;

	// 칼리브레이션 프로젝터 스크린
	QImageWidget *mProjectorScreen;
	

	////////////////////////////////////////////////////////////////////////
	// Preview Widgets
	////////////////////////////////////////////////////////////////////////
	QList<QImageWidget*> mListPreviewWidgets;
	QList<cv::Mat>       mListPreviewCvImages;
	QImage               mDefaultPreviewImage;
	int mCaptureNumber;

	////////////////////////////////////////////////////////////////////////
	// 캐넥트 관련 변수
	////////////////////////////////////////////////////////////////////////
	libfreenect2::Freenect2Device *mKinectDevice;
	libfreenect2::PacketPipeline  *mPipeline;

	libfreenect2::SyncMultiFrameListener *listener;
	libfreenect2::FrameMap *frames;
	libfreenect2::Registration* mRegistration;
	libfreenect2::Freenect2 freenect2;
	bool mIsKinectDeviceInit;

	////////////////////////////////////////////////////////////////////////
	// 칼리브레이션 관련 변수
	////////////////////////////////////////////////////////////////////////
	const int READY;				//칼리브레이션 모드 
	const int CALIBRATING;			// 칼리브레이션 수행중
	const int SHOW_CURRENT_RESULT;  // 현재 결과 보기
	const int WARP_RESULT;			// 최종 보정결과보기
	
	int mode;      //칼리브레이션 모드
	int mInterval; // 칼리브레이션 interval

	int mSampleTimeout;                 // 칼리브리에이션 대기 모드
	cv::Size mBoardSize;				// 체스보드 사이즈
	float    mSquareSize;				// 체스보드 사각형 사지으
	int      mMaxScale;					// 체스보드 구할 스케일
	std::vector<cv::Point2f> mFoundCorners;  // 이미지 상에서 찾은 코너
	std::vector<cv::Point2f> mCornerTruth;   // 이미지 좌표상의 코너점
	std::vector<cv::Point2f> mCurrentTruth;   // 이미지 좌표상의 코너점
	
	cv::Mat mPatternImage;					  //  오리지날 패턴 이미지
	cv::Mat mCurrentPatternImage;			  // 현재 패턴 이미지

	bool mCalbrationDone;                    // 칼리브레이션 확인 체크
	cv::Mat mCalibrationValues;              // 칼리브레이션 l값들
	int mCalIteration;                       // 칼리브레이션 현재 반복 횟수
	cv::Mat mLeftHand;                       // 칼리브레이션 Linear Equantion RightSize
	cv::Mat mRightHand;						 // 칼리브레이션 Linear Equantion LeftSize
	const int mMaxIteration;				 // 칼리브레이션 최대 iteration
	int mProjectorImageWidth;				 // 칼리브레이션 Projector 해상도
	int mProjectorImageHeight;				 // 칼리브레이션 Projector 해상도

	int mAddedPointNum;				// 칼리브레이션을 위해서 추가된 점 숫자
	float mProjectorFovX;			// 프로젝터 FOV X
	float mProjectorFovY;			// 프로젝터 FOV Y

	////////////////////////////////////////////////////////////////////////
	// 보정 수행 관련 변수
	////////////////////////////////////////////////////////////////////////
	cv::Mat mPointXYZ;
	cv::Mat mProjectionAreaMask;
	cv::Mat mImage_xPos;
	cv::Mat mImage_yPos;
	optimalRectDDJ mOpRect;
	QImageWidget *mWarpingScreen;  // 보정 된 워핑 스크린
	cv::Mat mWarpImage;
	cv::Mat mHumanDepth;
	void create_Human_To_depth_Map();
	cv::Mat mHumanDepthMap_x;
	cv::Mat mHumanDepthMap_y;
	
};

#endif