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

	//�ƺ��͸� ���� ����
	void ProcessMakingArbitraryMappingTable(Mat cropImage);
	//��� ���� ����
	void ProcessMakingPlaneMappingTable(Mat cropImage);
	//����� ���� ��������
	void ProcessMakingFrontPlaneMappingTable(Mat cropImage);


	void detectProjectionArea(cv::Mat& depthimg);

	//Pixel ��ǥ�踦 �̿��Ͽ� WorldCoordinate����
	void TransPixelToWorldCoordiante(Mat inImg);
	//���������� �����÷��� ����
	void CreateWorldPlaneFromUser();
	//���������� VirtualScreen ����
	void CreateVirtualScreenCoordinate();
	//������� ȸ��
	void RotateVirtualScreen();
	//����� ���������� ȸ��
	void RotateFrontScreen();

	//�������̺� �����
	void CreateMappingTable();
	//��� ��������
	void RunArbitraryWarping();


public:

	Point3f m_vectorFromUserToPlaneCenter;
	Point3f **returnWorldCoordinateForPlane;
	Point3f **m_worldCoordinateForPlane;
	Point3f **m_worldCoordinateFromUser;
	Point3f **m_virtualScreen;				//�������
	Point3f **m_virtualScreenRotated;		//ȸ����ȯ�� ������� (ȸ��)	

	//�ٿ���� ���ú�����
	Point3f m_worldCoordinateBoundary[5];
	Point3f m_worldCoordinateBoundaryFromUser[5];
	Point3f m_virtualScreenBoundary[5];				//������� �ٿ����
	Point3f m_virtualScreenBoundaryRotated[5];		//ȸ���� ��	
	Point2f m_userScreenFoundedBoundary[5];			//���������� ��� �ٿ����
	Point3f m_frontScreenBoundary[5];				//���齺ũ���� �ٿ����
	Point3f m_rotatedFrontScreenBoundary[5];		//ȸ����ȯ�� ���齺ũ���� �ٿ����
	Point3f m_normalizeFronScreenBoundary[5];		//��ֶ������  ���齺ũ���� �ٿ����


	//��ֶ������ ������ũ���� ����� ��ũ��
	Point3i **m_normalizeScreen;			//��ֶ������ ����� ��ũ��
	Point2i **m_userScreen;					//��ֶ������ ���� ��ũ��
	Point2f **m_fNormalizeProject;			//��ֶ��̵� ������Ʈ ��ũ��

	Table **m_mappingTable;					//�������̺�

	Point2f m_BoundaryMathedPoint[5];		//��Ī�� ������

public:
	//��ũ�� ���μ��� �ȼ�
	CPoint2i GetScreenSize();
	void SetScreenSize(CPoint2i inData);

	//���� ��ġ
	CPoint3i GetUserLocation();
	void SetUserLocation(CPoint3i inData);

	//����ī�޶� FoV
	CPoint2f GetDepthFoV();
	void SetDepthFoV(CPoint2f inData);

	//�������� FoV
	CPoint2f GetProjectorFoV();
	void SetProjectorFoV(CPoint2f inData);



	///////////////////////////////////////////
	//�Լ��� private
private:
	void SetProjectorLocation(CPoint3i inData);				//�������� ��ġ ����



	//////////////////////////////////////////
	//������ private
private:
	float m_sizePerPixel;
	float m_boundary[4];

	Rect m_Rect;
	Mat m_Image;


	//�ܺηκ��� �Է¹�����	
	int m_dx;
	int m_dy;

	//��ũ�� �ȼ���
	CPoint2i m_nPixelOfScreen;
	//�X��ī�޶� FoV
	CPoint2f m_fDepthFoV;
	//�������� FoV
	CPoint2f m_fProjectorFoV;
	//�X��ī�޶� ���μ��� ����
	CPoint2f m_fDepthDistanceRate;
	//���� ��ġ
	CPoint3i m_userLocation;		    //������ġ
	//�������� ��ġ (0,0,0)����
	CPoint3i m_projectorLoaction;	//���������� ��ġ


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

//�����ڸ� ��ȣ����
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

//���� ���
#define INVAILD_VALUE 10000
#define BASE_DEPTH 1000
#define PI 3.141592

//��ũ�� ������ ����
//#define PIXEL_OF_SCREEN_X 641
//#define PIXEL_OF_SCREEN_Y 361

//��ũ�� ������ ����
#define PIXEL_OF_SCREEN_X 1281
#define PIXEL_OF_SCREEN_Y 721

//
#define PIXEL_OF_DEPTH_X 512
#define PIXEL_OF_DEPTH_Y 424

//Ű��Ʈ FOV
#define BASE_FOV_X 70.6
#define BASE_FOV_Y 60.0

//Ű��Ʈ FoV���ݿ� ���� tan ��
#define BASE_WIDTH_RATE tan(BASE_FOV_X/2*PI/180.)
#define BASE_HEIGHT_RATE tan(BASE_FOV_Y/2*PI/180.)
//Ű��Ʈ�� �⺻ ȭ��ũ��
#define WIDHT_OF_KINECT BASE_WIDTH_RATE*DEPTH_OF_BASE*2.0
#define HEIGHT_OF_KINECT BASE_HEIGHT_RATE*DEPTH_OF_BASE*2.0



//���� �Ķ���͵�
#define WIDTH_OF_PROJECTOR 920.0
#define HEIGHT_OF_PROJECTOR 520.0
#define DEPTH_OF_BASE 1570.0
#define DIST_OF_KINECT_PROJECTOR 90.0
#define DIST_OF_VERTICAL_KINECT_PROJECTOR 0.0

//���� ���� �����
#define WIDTH_OF_KIENCT_PIXEL tan(BASE_FOV_X/2.0*PI/180.)*DEPTH_OF_BASE/PIXEL_OF_DEPTH_X
#define HEIGHT_OF_KIENCT_PIXEL tan(BASE_FOV_Y/2.0*PI/180.)*DEPTH_OF_BASE/PIXEL_OF_DEPTH_Y

*/