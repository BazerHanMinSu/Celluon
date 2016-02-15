#include <cv.h>
//#include <highgui.h>
#include <opencv2\imgproc\imgproc.hpp>
#include <cmath>
using namespace cv;


/*

class MyWarping
{


public:
	MyWarping();
	~MyWarping();

	void InitValue1();
	void InitValue2();

	//아비터리 워핑 진행
	void ProcessMakingArbitraryMappingTable(Mat cropImage);
	//평면 워핑 진행
	void ProcessMakingPlaneMappingTable(Mat cropImage);
	//투사면 정면 워핑진행
	void ProcessMakingFrontPlaneMappingTable(Mat cropImage);


	void detectProjectionArea(cv::Mat& depthimg);

	//Pixel 좌표계를 이용하여 WorldCoordinate생성
	void TransPixelToWorldCoordiante(Mat inImg);
	//유저기준의 투사플레인 생성
	void CreateWorldPlaneFromUser();
	//유저기준의 VirtualScreen 생성
	void CreateVirtualScreenCoordinate();
	//가상평면 회전
	void RotateVirtualScreen();
	//투사면 정면으로의 회정
	void RotateFrontScreen();

	//매핑테이블 만들기
	void CreateMappingTable();
	//곡면 워핑진행
	void RunArbitraryWarping();


public:

	Point3f m_vectorFromUserToPlaneCenter;
	Point3f **returnWorldCoordinateForPlane;
	Point3f **m_worldCoordinateForPlane;
	Point3f **m_worldCoordinateFromUser;
	Point3f **m_virtualScreen;				//가상평면
	Point3f **m_virtualScreenRotated;		//회전변환된 가상평면 (회색)	

	//바운더리 관련변수들
	Point3f m_worldCoordinateBoundary[5];
	Point3f m_worldCoordinateBoundaryFromUser[5];
	Point3f m_virtualScreenBoundary[5];				//가상평면 바운더리
	Point3f m_virtualScreenBoundaryRotated[5];		//회전된 점	
	Point2f m_userScreenFoundedBoundary[5];			//유저관점의 평면 바운더리
	Point3f m_frontScreenBoundary[5];				//정면스크린의 바운더리
	Point3f m_rotatedFrontScreenBoundary[5];		//회전변환된 정면스크린의 바운더리
	Point3f m_normalizeFronScreenBoundary[5];		//노멀라이즈된  정면스크린의 바운더리


	//노멀라이즈된 유저스크린과 투사면 스크린
	Point3i **m_normalizeScreen;			//노멀라이즈된 투사면 스크린
	Point2i **m_userScreen;					//노멀라이즈된 유저 스크린
	Point2f **m_fNormalizeProject;			//노멀라이드 프로젝트 스크린

	Table **m_mappingTable;					//매핑테이블

	Point2f m_BoundaryMathedPoint[5];		//매칭된 점정보

public:
	//스크린 가로세로 픽셀
	CPoint2i GetScreenSize();
	void SetScreenSize(CPoint2i inData);

	//유저 위치
	CPoint3i GetUserLocation();
	void SetUserLocation(CPoint3i inData);

	//뎁스카메라 FoV
	CPoint2f GetDepthFoV();
	void SetDepthFoV(CPoint2f inData);

	//프로젝터 FoV
	CPoint2f GetProjectorFoV();
	void SetProjectorFoV(CPoint2f inData);



	///////////////////////////////////////////
	//함수용 private
private:
	void SetProjectorLocation(CPoint3i inData);				//프로젝터 위치 설정



	//////////////////////////////////////////
	//변수용 private
private:
	float m_sizePerPixel;
	float m_boundary[4];

	Rect m_Rect;
	Mat m_Image;


	//외부로부터 입력받을값	
	int m_dx;
	int m_dy;

	//스크린 픽셀수
	CPoint2i m_nPixelOfScreen;
	//뎊스카메라 FoV
	CPoint2f m_fDepthFoV;
	//프로젝터 FoV
	CPoint2f m_fProjectorFoV;
	//뎊스카메라 가로세로 비율
	CPoint2f m_fDepthDistanceRate;
	//유저 위치
	CPoint3i m_userLocation;		    //유저위치
	//프로젝터 위치 (0,0,0)고정
	CPoint3i m_projectorLoaction;	//프로젝터의 위치


public:
	float q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12, q13, q14;

	const float fx = 365.091095;
	const float fy = 365.091095;

	const float cx = 259.740692;
	const float cy = 205.260895;


};


struct CPoint2i{
	int x;
	int y;
	CPoint2i(int val1 = 0, int val2 = 0)
	{
		x = val1;
		y = val2;
	}
};

struct CPoint3i{
	int x;
	int y;
	int z;
	CPoint3i(int val1 = 0, int val2 = 0, int val3 = 0)
	{
		x = val1;
		y = val2;
		z = val3;
	}
};

struct CPoint2f{
	float x;
	float y;
	CPoint2f(float val1 = 0, float val2 = 0)
	{
		x = val1;
		y = val2;
	}
};

struct CPoint3f{
	float x;
	float y;
	float z;
	CPoint3f(float val1 = 0, float val2 = 0, float val3 = 0)
	{
		x = val1;
		y = val2;
		z = val3;
	}
};

struct Table{
	int x;
	int y;
	float dist[4];
};

//가장자리 번호지정
#define TOPLEFT 0
#define TOPRIGHT 1
#define BOTTOMRIGHT 2
#define BOTTOMLEFT 3
#define CENTER 4

#define LEFT_U 0
#define RIGHT_U 1
#define TOP_U 2
#define BOTTOM_U 3

#define KB_ENTER 13
#define KB_ESC 27
#define KB_SPACE 32

//고정 상수
#define INVAILD_VALUE 10000
#define BASE_DEPTH 1000
#define PI 3.141592

//스크린 사이즈 지정
//#define PIXEL_OF_SCREEN_X 641
//#define PIXEL_OF_SCREEN_Y 361

//스크린 사이즈 지정
#define PIXEL_OF_SCREEN_X 1281
#define PIXEL_OF_SCREEN_Y 721

//
#define PIXEL_OF_DEPTH_X 512
#define PIXEL_OF_DEPTH_Y 424

//키넥트 FOV
#define BASE_FOV_X 70.6
#define BASE_FOV_Y 60.0

//키넥트 FoV절반에 대한 tan 값
#define BASE_WIDTH_RATE tan(BASE_FOV_X/2*PI/180.)
#define BASE_HEIGHT_RATE tan(BASE_FOV_Y/2*PI/180.)
//키넥트의 기본 화면크기
#define WIDHT_OF_KINECT BASE_WIDTH_RATE*DEPTH_OF_BASE*2.0
#define HEIGHT_OF_KINECT BASE_HEIGHT_RATE*DEPTH_OF_BASE*2.0



//보정 파라미터들
#define WIDTH_OF_PROJECTOR 920.0
#define HEIGHT_OF_PROJECTOR 520.0
#define DEPTH_OF_BASE 1570.0
#define DIST_OF_KINECT_PROJECTOR 90.0
#define DIST_OF_VERTICAL_KINECT_PROJECTOR 0.0

//보정 위한 연산식
#define WIDTH_OF_KIENCT_PIXEL tan(BASE_FOV_X/2.0*PI/180.)*DEPTH_OF_BASE/PIXEL_OF_DEPTH_X
#define HEIGHT_OF_KIENCT_PIXEL tan(BASE_FOV_Y/2.0*PI/180.)*DEPTH_OF_BASE/PIXEL_OF_DEPTH_Y

*/