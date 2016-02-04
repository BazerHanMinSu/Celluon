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

#include <qscrollbar.h>

/// @brief FisheyeDewarping ���α׷��� ���� ������ Ŭ�����Դϴ�.
///
/// �� ���� ������ �󿡼� wiget��� ��ư�� �����ǰ� �ʱ�ȭ �˴ϴ�.
class SmartProjectorMainWindow : public QMainWindow, public Ui::CarInspectMainWindow
{
    Q_OBJECT
public:

	
	/// @brief ������RegNewChart()
	///
	///	�����ڿ����� Ui�� �����ϰ�  
	/// DB�� ���� ������ �о�´�. 
	/// ������ ��ư�� �׼ǰ� ���� �Լ��� ����(connect)���� �ش�. 
	SmartProjectorMainWindow();

	/// @brief �Ҹ���
	///
	///	�Ҹ��ڿ��� �ϴ����� ���� ����..
	~SmartProjectorMainWindow();
    
	/// trace â�� �޼����� ����. 
	void trace(QString str){
		mTextConsole->appendPlainText(str);
		mTextConsole->verticalScrollBar()->setValue(mTextConsole->verticalScrollBar()->maximum());
	}

	//void InitListView();
	//void InitPreViews();
	//void InitStatusBar();
	bool findCheckBoard(cv::Mat &inputImage, bool drawConners = true);
	bool findCheckBoardTruth(cv::Mat &inputImage, bool drawConners = true);

	void calculateCalibration(const cv::Mat& undistorted);
	void solveCalibration();
	void loadCalibrationData();

	void detectProjectionArea(cv::Mat& undistorted, cv::Mat& depthimg);
	void detectProjectionAreaUser(cv::Mat& userDepth, cv::Mat& IRimg);

	void detectOptimalRectFromLargestContour(cv::Mat& depthimg);
	//bool calculateCalibrationParameter(cv::Mat &inputDepth, )
	void warpScreenImage(cv::Mat &screenImage, cv::Mat &undistorted);
	void calculatePlaneNormalVector(float &angleX, float &angleY);

	
	// get kinect frame
	// image         : 1920x1080 RGB CV_8UC3
	// imageDepth    : 512x424 float CV_32F
	// imageIR       : 512x424 float CV_32F
	// imageRegisted : 512x424 float CV_8UC3
	void getKinectImage(cv::Mat &image, cv::Mat &imageDepth, cv::Mat& imageIR, cv::Mat& imageRegisted);

public:
	//Depth Image Ȧ�ʸ�
	void DepthImageHoleFilling(cv::Mat& imgDepth);
	void WarpArbitrary(cv::Mat& inDepth, cv::Mat& outDepth, cv::Rect OptimalRect);

public slots:
	void openDeivce();
	void timeout();

	// Kinect Play
	void startKinect();
	
	// Kinect stop 
	void endKinect();

	// Į���극�̼�
	void calibrationImage();

	// ���� ����
	void correctionScreen();
	
	// human Fov ��ȭ
	void changeHumanSetting(double d){
		create_Human_To_depth_Map();
	}
	void setCorrection(int value);
private:
	// �������� �����ֱ� ���� Ÿ�̸�
	QTimer mTimer; 

	// ���� Kinect ����
	cv::Mat mCurrentImage;
	cv::Mat mCurrentIRImage;
	cv::Mat mCurrentDepthImage;
	cv::Mat mCurrentCalibrationImage;

	// �˻� ����� �����ִ� defalut ����
	cv::Mat mDefaultImage;

	// Į���극�̼� �������� ��ũ��
	QImageWidget *mProjectorScreen;
	

	////////////////////////////////////////////////////////////////////////
	// Preview Widgets
	////////////////////////////////////////////////////////////////////////
	QList<QImageWidget*> mListPreviewWidgets;
	QList<cv::Mat>       mListPreviewCvImages;
	QImage               mDefaultPreviewImage;
	int mCaptureNumber;

	////////////////////////////////////////////////////////////////////////
	// ĳ��Ʈ ���� ����
	////////////////////////////////////////////////////////////////////////
	libfreenect2::Freenect2Device *mKinectDevice;
	libfreenect2::PacketPipeline  *mPipeline;

	libfreenect2::SyncMultiFrameListener *listener;
	libfreenect2::FrameMap *frames;
	libfreenect2::Registration* mRegistration;
	libfreenect2::Freenect2 freenect2;
	bool mIsKinectDeviceInit;

	////////////////////////////////////////////////////////////////////////
	// Į���극�̼� ���� ����
	////////////////////////////////////////////////////////////////////////
	const int READY;				//Į���극�̼� ��� 
	const int CALIBRATING;			// Į���극�̼� ������
	const int SHOW_CURRENT_RESULT;  // ���� ��� ����
	const int WARP_RESULT;			// ���� �����������
	
	int mode;      //Į���극�̼� ���
	int mInterval; // Į���극�̼� interval

	int mSampleTimeout;                 // Į���긮���̼� ��� ���
	cv::Size mBoardSize;				// ü������ ������
	float    mSquareSize;				// ü������ �簢�� ������
	int      mMaxScale;					// ü������ ���� ������
	std::vector<cv::Point2f> mFoundCorners;  // �̹��� �󿡼� ã�� �ڳ�
	std::vector<cv::Point2f> mCornerTruth;   // �̹��� ��ǥ���� �ڳ���
	std::vector<cv::Point2f> mCurrentTruth;   // �̹��� ��ǥ���� �ڳ���
	
	cv::Mat mPatternImage;					  //  �������� ���� �̹���
	cv::Mat mCurrentPatternImage;			  // ���� ���� �̹���

	bool mCalbrationDone;                    // Į���극�̼� Ȯ�� üũ
	cv::Mat mCalibrationValues;              // Į���극�̼� l����
	int mCalIteration;                       // Į���극�̼� ���� �ݺ� Ƚ��
	cv::Mat mLeftHand;                       // Į���극�̼� Linear Equantion RightSize
	cv::Mat mRightHand;						 // Į���극�̼� Linear Equantion LeftSize
	const int mMaxIteration;				 // Į���극�̼� �ִ� iteration
	int mProjectorImageWidth;				 // Į���극�̼� Projector �ػ�
	int mProjectorImageHeight;				 // Į���극�̼� Projector �ػ�

	int mAddedPointNum;				// Į���극�̼��� ���ؼ� �߰��� �� ����
	float mProjectorFovX;			// �������� FOV X
	float mProjectorFovY;			// �������� FOV Y

	////////////////////////////////////////////////////////////////////////
	// ���� ���� ���� ����
	////////////////////////////////////////////////////////////////////////
	cv::Mat mPointXYZ;					// ���� �޾ƿ� depth �̹����� Kinect ��ǥ�� 3D��
	cv::Mat mPointXYZ_User;				// ���� �޾ƿ� ���� depth �̹����� Kinect ��ǥ�� 3D��

	cv::Mat mProjectionAreaMask;		// �������� ���� mask
	cv::Mat mProjectionAreaMaskUser;	// �������� ���� mask

	cv::Mat mImage_xPos;				// ���� �޾ƿ� depth �̹����� �������� �̹��� x��ǥ��
	cv::Mat mImage_yPos;				// ���� �޾ƿ� depth �̹����� �������� �̹��� y��ǥ��
	//cv::Mat mImage_xPosUser;			// ���� �޾ƿ� ���� depth �̹����� �������� �̹��� x��ǥ��
	//cv::Mat mImage_yPosUser;			// ���� �޾ƿ� ���� depth �̹����� �������� �̹��� y��ǥ��

	optimalRectDDJ mOpRect;				// Optimal Rect ����� ���� ��ü
	QImageWidget *mWarpingScreen;		// ���� �� ���� ��ũ��

	cv::Mat mWarpImage;					// ���� ����
	//cv::Mat mHumanDepth;				// ���� Depth ����

	void create_Human_To_depth_Map();   // human Depth to kinect Depth ��ȯ �� ���� �Լ�
	cv::Mat mHumanDepthMap_x;			// human Depth mapping table
	cv::Mat mHumanDepthMap_y;			// human Depth mapping table
	
	cv::Mat mAvgDepth;					// ������ ��� depth��
	int mNumAvgFrame;					// ��� frame ����
	int mAccFrameNumber;				// ���� ������ frame ����
	
	bool mPlaneRotationCorrection;
	bool mCorrectionFlag;
};

#endif