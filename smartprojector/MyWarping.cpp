
/*
#include "MyWarping.h"

#include <math.h>
#include <fstream>

using namespace std;


MyWarping::MyWarping()
{
	InitValue1();
	InitValue2();
}

MyWarping::~MyWarping()
{

	delete[] m_worldCoordinateForPlane;
	delete[] m_worldCoordinateFromUser;
	delete[] m_virtualScreen;
	delete[] m_virtualScreenRotated;
	delete[] m_fNormalizeProject;
	delete[] m_normalizeScreen;
	delete[] m_userScreen;
	delete[] m_mappingTable;
}

void MyWarping::InitValue1()
{
	m_dx = PIXEL_OF_SCREEN_X;
	m_dy = PIXEL_OF_SCREEN_Y;
}

void MyWarping::InitValue2()
{
	int height = PIXEL_OF_SCREEN_Y;
	int width = PIXEL_OF_SCREEN_X;

	//스크린 관련 변수들 동적할당
	m_worldCoordinateForPlane = (Point3f **)malloc(sizeof(Point3f *)* height);
	m_worldCoordinateForPlane[0] = (Point3f *)malloc(sizeof(Point3f)* width*height);
	for (int y = 1; y < height; y++)
		m_worldCoordinateForPlane[y] = m_worldCoordinateForPlane[y - 1] + width;

	m_worldCoordinateFromUser = (Point3f **)malloc(sizeof(Point3f *)* height);
	m_worldCoordinateFromUser[0] = (Point3f *)malloc(sizeof(Point3f)* width*height);
	for (int y = 1; y < height; y++)
		m_worldCoordinateFromUser[y] = m_worldCoordinateFromUser[y - 1] + width;


	m_virtualScreen = (Point3f **)malloc(sizeof(Point3f *)* height);
	m_virtualScreen[0] = (Point3f *)malloc(sizeof(Point3f)* width*height);
	for (int y = 1; y < height; y++)
		m_virtualScreen[y] = m_virtualScreen[y - 1] + width;


	m_virtualScreenRotated = (Point3f **)malloc(sizeof(Point3f *)* height);
	m_virtualScreenRotated[0] = (Point3f *)malloc(sizeof(Point3f)* width*height);
	for (int y = 1; y < height; y++)
		m_virtualScreenRotated[y] = m_virtualScreenRotated[y - 1] + width;


	m_fNormalizeProject = (Point2f **)malloc(sizeof(Point2f *)* height);
	m_fNormalizeProject[0] = (Point2f *)malloc(sizeof(Point2f)* width*height);
	for (int y = 1; y < height; y++)
		m_fNormalizeProject[y] = m_fNormalizeProject[y - 1] + width;


	m_normalizeScreen = (Point3i **)malloc(sizeof(Point3i *)* height);
	m_normalizeScreen[0] = (Point3i *)malloc(sizeof(Point3i)* width*height);
	for (int y = 1; y < height; y++)
		m_normalizeScreen[y] = m_normalizeScreen[y - 1] + width;

	m_userScreen = (Point2i **)malloc(sizeof(Point2i *)* height);
	m_userScreen[0] = (Point2i *)malloc(sizeof(Point2i)* width*height);
	for (int y = 1; y < height; y++)
		m_userScreen[y] = m_userScreen[y - 1] + width;

	m_mappingTable = (Table **)malloc(sizeof(Table *)* height);
	m_mappingTable[0] = (Table *)malloc(sizeof(Table)* width*height);
	for (int y = 1; y < height; y++)
		m_mappingTable[y] = m_mappingTable[y - 1] + width;

	//Table Initialize
	for (int y = 0; y < m_dy; y++)
	{
		for (int x = 0; x < m_dx; x++)
		{
			m_mappingTable[y][x].x = INVAILD_VALUE;
			m_mappingTable[y][x].y = INVAILD_VALUE;
			for (int i = 0; i < 4; i++)
			{
				m_mappingTable[y][x].dist[i] = 0;
			}
		}
	}
}

void MyWarping::ProcessMakingArbitraryMappingTable(Mat depthImg)
{
	//센터와 바운더리의 월드코디네이터 생성	
	TransPixelToWorldCoordiante(depthImg);
	//유저관점에서의 Plane 생성
	CreateWorldPlaneFromUser();
	//유저관점으로 회전
	RotateVirtualScreen();
	//유저평면 노멀라이즈
	CreateVirtualScreenCoordinate();
	//바운더리 영역 찾기
	FindVirtualScreenBoundary();
	//매핑 테이블 만들기
	CreateMappingTable();
}

void MyWarping::SetDepthFoV(CPoint2f inData)
{
	m_fDepthFoV.x = inData.x;
	m_fDepthFoV.y = inData.y;
}


//world coordinate로 변경
void MyWarping::TransPixelToWorldCoordiante(Mat inImg)
{
	//스크린 X * Y 사이즈 지정
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;

	float degree = (float)PI / 180;	//1도
	float baseDetph = 1000;	//기준 거리

	//프로젝트의 FOV
	float fovX = (float)PROJECT_FOV_X / 2.;
	float fovY = (float)PROJECT_FOV_Y / 2.;

	//프로젝트의 FoV에 따른 실제 X, Y의 최대 길이 (가운데점 (0,0) 기준)
	float maxX = baseDetph*tan(fovX * degree);
	float maxY = baseDetph*tan(fovY * degree);

	//1미터 거리에서 1픽셀당 실제 길이
	float sizePerPixel = maxX / ((float)PIXEL_OF_SCREEN_X / 2.);
	float sizePerPixelY = maxY / ((float)PIXEL_OF_SCREEN_Y / 2.);

	//임시용
	m_sizePerPixel = sizePerPixel;

	float depth = 0;
	//World Coordinate 생성

	//센터 월드좌표 생성
	depth = inImg.at<float>(dy / 2, dx / 2);
	m_worldCoordinateBoundary[CENTER].x = 0;
	m_worldCoordinateBoundary[CENTER].y = 0;
	m_worldCoordinateBoundary[CENTER].z = depth;

	//바운더리 월드좌표 생성
	depth = inImg.at<float>(0, 0);
	m_worldCoordinateBoundary[TOPLEFT].x = -maxX*(depth / baseDetph);
	m_worldCoordinateBoundary[TOPLEFT].y = maxY*(depth / baseDetph);
	m_worldCoordinateBoundary[TOPLEFT].z = depth;

	depth = inImg.at<float>(0, dx - 1);
	m_worldCoordinateBoundary[TOPRIGHT].x = maxX*(depth / baseDetph);
	m_worldCoordinateBoundary[TOPRIGHT].y = maxY*(depth / baseDetph);
	m_worldCoordinateBoundary[TOPRIGHT].z = depth;

	depth = inImg.at<float>(dy - 1, dx - 1);
	m_worldCoordinateBoundary[BOTTOMRIGHT].x = maxX*(depth / baseDetph);
	m_worldCoordinateBoundary[BOTTOMRIGHT].y = -maxY*(depth / baseDetph);
	m_worldCoordinateBoundary[BOTTOMRIGHT].z = depth;

	depth = inImg.at<float>(dy - 1, 0);
	m_worldCoordinateBoundary[BOTTOMLEFT].x = -maxX*(depth / baseDetph);
	m_worldCoordinateBoundary[BOTTOMLEFT].y = -maxY*(depth / baseDetph);
	m_worldCoordinateBoundary[BOTTOMLEFT].z = depth;


	//실험용
	//World Coordinate 생성
	for (register int y = 0; y < dy; y++)
		for (register int x = 0; x < dx; x++)
		{
			depth = inImg.at<float>(y, x);
			m_worldCoordinateForPlane[y][x].x = (float)((float)(x - (dx / 2)) * sizePerPixel*(depth / baseDetph));
			m_worldCoordinateForPlane[y][x].y = -(float)((float)(y - (dy / 2)) * sizePerPixelY*(depth / baseDetph));
			m_worldCoordinateForPlane[y][x].z = depth;
		}

	for (int i = 0; i < 5; i++)
	{
		m_frontScreenBoundary[i] = m_worldCoordinateBoundary[i];

	}
}

void MyWarping::CreateWorldPlaneFromUser()
{

	//스크린 X * Y 사이즈 지정
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;
	//
	//1. 유저로부터 화면 중심좌표의 벡터와 거리 구하기 
	//	

	//유저로부터의 투사면의 바운더리의 4개점
	for (int i = 0; i < 5; i++)
	{
		m_worldCoordinateBoundaryFromUser[i].x = m_worldCoordinateBoundary[i].x - m_userLocation.x;
		m_worldCoordinateBoundaryFromUser[i].y = m_worldCoordinateBoundary[i].y - m_userLocation.y;
		m_worldCoordinateBoundaryFromUser[i].z = m_worldCoordinateBoundary[i].z - m_userLocation.z;
	}

	//실험용
	//유저입장에서의 가상평면
	for (register int y = 0; y < dy; y++)
		for (register int x = 0; x < dx; x++)
		{
			m_worldCoordinateFromUser[y][x].x = m_worldCoordinateForPlane[y][x].x - m_userLocation.x;
			m_worldCoordinateFromUser[y][x].y = m_worldCoordinateForPlane[y][x].y - m_userLocation.y;
			m_worldCoordinateFromUser[y][x].z = m_worldCoordinateForPlane[y][x].z - m_userLocation.z;
		}
}


void MyWarping::RotateFrontScreen()
{
	//4개의 영역점
	float x1, y1, z1;
	float x2, y2, z2;
	float x3, y3, z3;
	float x4, y4, z4;
	float x5, y5, z5;

	//2개의 직선 벡터
	float v11, v12, v13;
	float v21, v22, v23;

	float h1, h2, h3;


	//4개의 영역점 대입

	x1 = m_frontScreenBoundary[TOPLEFT].x;
	y1 = m_frontScreenBoundary[TOPLEFT].y;
	z1 = m_frontScreenBoundary[TOPLEFT].z;

	x2 = m_frontScreenBoundary[TOPRIGHT].x;
	y2 = m_frontScreenBoundary[TOPRIGHT].y;
	z2 = m_frontScreenBoundary[TOPRIGHT].z;

	x3 = m_frontScreenBoundary[BOTTOMRIGHT].x;
	y3 = m_frontScreenBoundary[BOTTOMRIGHT].y;
	z3 = m_frontScreenBoundary[BOTTOMRIGHT].z;

	x4 = m_frontScreenBoundary[BOTTOMLEFT].x;
	y4 = m_frontScreenBoundary[BOTTOMLEFT].y;
	z4 = m_frontScreenBoundary[BOTTOMLEFT].z;

	x5 = m_frontScreenBoundary[CENTER].x;
	y5 = m_frontScreenBoundary[CENTER].y;
	z5 = m_frontScreenBoundary[CENTER].z;

	//2개의 직선벡터
	v11 = x2 - x1;	v12 = y2 - y1;	v13 = z2 - z1;
	v21 = x4 - x3;	v22 = y4 - y3;	v23 = z4 - z3;

	v11 /= 10.0;
	v12 /= 10.0;
	v13 /= 10.0;
	v21 /= 10.0;
	v22 /= 10.0;
	v23 /= 10.0;

	//투사면의 벡선벡터
	h1 = v12*v23 - v13*v22;
	h2 = v13*v21 - v11*v23;
	h3 = v11*v22 - v12*v21;

	//회전각 구하기
	float angleX;
	float angleY;
	//float angleX = -cvFastArctan(h1, h3);
	//float angleYFastArctan(h2, h3)+5;
	//float angleY = 0;

	float distUnW = sqrt((h1 *h1) + (h2 *h2) + (h3 *h3));

	float tmp1 = std::atan2f(abs(h2), distUnW);
	//angleX = -tmp1 -0.1;
	angleX = 0;
	if (h1 < 0) angleX = -angleX;
	if (angleX < -(PI / 2)) angleX += PI;
	if (angleX >(PI / 2)) angleX -= PI;
	float tmp2 = std::atan2f(abs(h1), abs(h3));
	angleY = -tmp2*1.1;
	if (h1*h1 <= 0) angleY = -angleY;
	if (angleY < -(PI / 2)) angleY += PI;
	if (angleY >(PI / 2)) angleY -= PI;
	//angleY = 10;

	Point3f ptTemp(0, 0, 0);


	//y축 회전변환	
	for (int i = 0; i < 5; i++)
	{
		ptTemp.x = m_frontScreenBoundary[i].x;
		ptTemp.y = m_frontScreenBoundary[i].y;
		ptTemp.z = m_frontScreenBoundary[i].z;

		m_rotatedFrontScreenBoundary[i].x = ptTemp.z * sin(angleY) + ptTemp.x*cos(angleY);
		m_rotatedFrontScreenBoundary[i].y = ptTemp.y;
		m_rotatedFrontScreenBoundary[i].z = ptTemp.z * cos(angleY) - ptTemp.x*sin(angleY);
	}
	//x축 회전변환	
	for (int i = 0; i < 5; i++)
	{
		ptTemp.x = m_rotatedFrontScreenBoundary[i].x;
		ptTemp.y = m_rotatedFrontScreenBoundary[i].y;
		ptTemp.z = m_rotatedFrontScreenBoundary[i].z;

		m_rotatedFrontScreenBoundary[i].x = ptTemp.x;
		m_rotatedFrontScreenBoundary[i].y = cos(angleX)*ptTemp.y - sin(angleX)*ptTemp.z;
		m_rotatedFrontScreenBoundary[i].z = sin(angleX)*ptTemp.y + cos(angleX)*ptTemp.z;
	}

	//회전된 센터좌표저장
	Point3f center = m_rotatedFrontScreenBoundary[CENTER];

	//키넥트 중심좌표로 평행이동
	for (int i = 0; i < 5; i++)
	{
		ptTemp.x = m_rotatedFrontScreenBoundary[i].x;
		ptTemp.y = m_rotatedFrontScreenBoundary[i].y;
		ptTemp.z = m_rotatedFrontScreenBoundary[i].z;

		m_rotatedFrontScreenBoundary[i].x = ptTemp.x - center.x;
		m_rotatedFrontScreenBoundary[i].y = ptTemp.y - center.y;
	}

	//스크린 X * Y 사이즈 지정
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;


	//실험용
	//회전변환된 virtual coordinater 생성
	for (register int y = 0; y < dy; y++)
		for (register int x = 0; x < dx; x++)
		{
			ptTemp.x = m_worldCoordinateForPlane[y][x].x;
			ptTemp.y = m_worldCoordinateForPlane[y][x].y;
			ptTemp.z = m_worldCoordinateForPlane[y][x].z;
			m_virtualScreenRotated[y][x].x = ptTemp.z * sin(angleY) + ptTemp.x*cos(angleY);
			m_virtualScreenRotated[y][x].y = ptTemp.y;
			m_virtualScreenRotated[y][x].z = ptTemp.z * cos(angleY) - ptTemp.x*sin(angleY);
		}


	for (register int y = 0; y < dy; y++)
		for (register int x = 0; x < dx; x++)
		{
			ptTemp.x = m_virtualScreenRotated[y][x].x;
			ptTemp.y = m_virtualScreenRotated[y][x].y;
			ptTemp.z = m_virtualScreenRotated[y][x].z;
			m_virtualScreenRotated[y][x].x = ptTemp.x;
			m_virtualScreenRotated[y][x].y = cos(angleX)*ptTemp.y - sin(angleX)*ptTemp.z;
			m_virtualScreenRotated[y][x].z = sin(angleX)*ptTemp.y + cos(angleX)*ptTemp.z;
		}


	for (register int y = 0; y < dy; y++)
		for (register int x = 0; x < dx; x++)
		{
			ptTemp.x = m_virtualScreenRotated[y][x].x;
			ptTemp.y = m_virtualScreenRotated[y][x].y;
			ptTemp.z = m_virtualScreenRotated[y][x].z;
			m_virtualScreenRotated[y][x].x = ptTemp.x - center.x;
			m_virtualScreenRotated[y][x].y = ptTemp.y - center.y;
		}

	float dist = 100;
	float depth = 1000;

	//평면의 방정식 구하기
	float a, b, c, d;	//평면 parameter
	float t;			//임의값


	//평면위의 한점
	x1 = 0;
	y1 = 0;
	z1 = 1000;

	//평면의 Normal vector
	a = 0;
	b = 0;
	c = 1;

	//ax +by +cz +d = 0 의 d 구하기
	d = (-x1)*a + (-y1)*b + (-z1)*c;

	//바운더리 노멀라이즈
	for (int i = 0; i < 5; i++)
	{
		x2 = m_rotatedFrontScreenBoundary[i].x;
		y2 = m_rotatedFrontScreenBoundary[i].y;
		z2 = m_rotatedFrontScreenBoundary[i].z;

		t = (-d) / (a*x2 + b*y2 + c*z2);

		m_virtualScreenBoundary[i].x = x2*t;
		m_virtualScreenBoundary[i].y = y2*t;
		m_virtualScreenBoundary[i].z = z2*t;
	}

	//버추얼스크린 노멀라이즈
	for (register int y = 0; y < dy; y++)
		for (register int x = 0; x < dx; x++)
		{
			x2 = m_virtualScreenRotated[y][x].x;
			y2 = m_virtualScreenRotated[y][x].y;
			z2 = m_virtualScreenRotated[y][x].z;

			t = (-d) / (a*x2 + b*y2 + c*z2);

			m_virtualScreen[y][x].x = x2*t;
			m_virtualScreen[y][x].y = y2*t;
			m_virtualScreen[y][x].z = z2*t;
		}
}


void MyWarping::RotateVirtualScreen()
{
	//스크린 X * Y 사이즈 지정
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;
	Point3f ptTemp(0, 0, 0);

	float dist = 100;
	float depth = 1000;

	//평면의 방정식 구하기
	float a, b, c, d;	//평면 parameter
	float x1, y1, z1;
	float x2, y2, z2;
	float t;			//임의값


	//평면위의 한점
	x1 = 0;
	y1 = 0;
	z1 = 1000;

	//평면의 Normal vector
	a = 0;
	b = 0;
	c = 1;

	//ax +by +cz +d = 0 의 d 구하기
	d = (-x1)*a + (-y1)*b + (-z1)*c;

	float angleX, angleY;
	float distUnW;

	distUnW = sqrt(
		(m_userLocation.x * m_userLocation.x) +
		((m_worldCoordinateBoundary[CENTER].z - m_userLocation.z) *
		(m_worldCoordinateBoundary[CENTER].z - m_userLocation.z))
		);


	//y축 회전변환	
	for (int i = 0; i < 5; i++)
	{
		ptTemp.x = m_worldCoordinateBoundaryFromUser[i].x;
		ptTemp.y = m_worldCoordinateBoundaryFromUser[i].y;
		ptTemp.z = m_worldCoordinateBoundaryFromUser[i].z;

		m_virtualScreenBoundaryRotated[i].x = ptTemp.z * sin(angleY) + ptTemp.x*cos(angleY);
		m_virtualScreenBoundaryRotated[i].y = ptTemp.y;
		m_virtualScreenBoundaryRotated[i].z = ptTemp.z * cos(angleY) - ptTemp.x*sin(angleY);
	}
	//x축 회전변환	
	for (int i = 0; i < 5; i++)
	{
		ptTemp.x = m_virtualScreenBoundaryRotated[i].x;
		ptTemp.y = m_virtualScreenBoundaryRotated[i].y;
		ptTemp.z = m_virtualScreenBoundaryRotated[i].z;

		m_virtualScreenBoundaryRotated[i].x = ptTemp.x;
		m_virtualScreenBoundaryRotated[i].y = cos(angleX)*ptTemp.y - sin(angleX)*ptTemp.z;
		m_virtualScreenBoundaryRotated[i].z = sin(angleX)*ptTemp.y + cos(angleX)*ptTemp.z;
	}


	//실험용
	//회전변환된 virtual coordinater 생성
	for (register int y = 0; y < dy; y++)
		for (register int x = 0; x < dx; x++)
		{
			ptTemp.x = m_worldCoordinateFromUser[y][x].x;
			ptTemp.y = m_worldCoordinateFromUser[y][x].y;
			ptTemp.z = m_worldCoordinateFromUser[y][x].z;
			m_virtualScreenRotated[y][x].x = ptTemp.z * sin(angleY) + ptTemp.x*cos(angleY);
			m_virtualScreenRotated[y][x].y = ptTemp.y;
			m_virtualScreenRotated[y][x].z = ptTemp.z * cos(angleY) - ptTemp.x*sin(angleY);
		}


	for (register int y = 0; y < dy; y++)
		for (register int x = 0; x < dx; x++)
		{
			ptTemp.x = m_virtualScreenRotated[y][x].x;
			ptTemp.y = m_virtualScreenRotated[y][x].y;
			ptTemp.z = m_virtualScreenRotated[y][x].z;
			m_virtualScreenRotated[y][x].x = ptTemp.x;
			m_virtualScreenRotated[y][x].y = cos(angleX)*ptTemp.y - sin(angleX)*ptTemp.z;
			m_virtualScreenRotated[y][x].z = sin(angleX)*ptTemp.y + cos(angleX)*ptTemp.z;
		}

	//버추얼스크린 노멀라이즈
	for (register int y = 0; y < dy; y++)
		for (register int x = 0; x < dx; x++)
		{
			x2 = m_virtualScreenRotated[y][x].x;
			y2 = m_virtualScreenRotated[y][x].y;
			z2 = m_virtualScreenRotated[y][x].z;

			t = (-d) / (a*x2 + b*y2 + c*z2);

			m_virtualScreen[y][x].x = x2*t;
			m_virtualScreen[y][x].y = y2*t;
			m_virtualScreen[y][x].z = z2*t;
		}}



void MyWarping::CreateVirtualScreenCoordinate()
{
	//스크린 X * Y 사이즈 지정
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;
	float dist = 100;

	float depth = 1000;

	//평면의 방정식 구하기
	float a, b, c, d;	//평면 parameter
	float x1, y1, z1;	//평면위의 한점
	float x2, y2, z2;	//직선방향 벡터
	float t;			//임의값


	//평면위의 한점
	x1 = 0;
	y1 = 0;
	z1 = 1000;

	//평면의 Normal vector
	a = 0;
	b = 0;
	c = 1;

	//ax +by +cz +d = 0 의 d 구하기
	d = (-x1)*a + (-y1)*b + (-z1)*c;

	//바운더리 노멀라이즈
	for (int i = 0; i < 5; i++)
	{
		x2 = m_virtualScreenBoundaryRotated[i].x;
		y2 = m_virtualScreenBoundaryRotated[i].y;
		z2 = m_virtualScreenBoundaryRotated[i].z;

		t = (-d) / (a*x2 + b*y2 + c*z2);

		m_virtualScreenBoundary[i].x = x2*t;
		m_virtualScreenBoundary[i].y = y2*t;
		m_virtualScreenBoundary[i].z = z2*t;
	}

	//버추얼스크린 노멀라이즈
	for (register int y = 0; y < dy; y++)
		for (register int x = 0; x < dx; x++)
		{
			x2 = m_virtualScreenRotated[y][x].x;
			y2 = m_virtualScreenRotated[y][x].y;
			z2 = m_virtualScreenRotated[y][x].z;

			t = (-d) / (a*x2 + b*y2 + c*z2);

			m_virtualScreen[y][x].x = x2*t;
			m_virtualScreen[y][x].y = y2*t;
			m_virtualScreen[y][x].z = z2*t;
		}
}

void MyWarping::CreateMappingTable()
{
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;

	Rect userScreenRect = m_Rect;

	Point2f ptMovingValue;
	float fScale;

	float tmpX = 0;
	float tmpY = 0;

	//평행이동 값
	ptMovingValue.x = (float)-userScreenRect.x;
	ptMovingValue.y = (float)userScreenRect.y;

	fScale = (float)dx / (float)userScreenRect.width;

	for (int y = 0; y < dy; y++)
	{
		for (int x = 0; x < dx; x++)
		{
			//모든 좌표 평행이동
			tmpX = m_virtualScreen[y][x].x + ptMovingValue.x;
			tmpY = m_virtualScreen[y][x].y + ptMovingValue.y;

			//해상도에 맞춰 스케일링
			m_fNormalizeProject[y][x].x = tmpX*fScale;
			m_fNormalizeProject[y][x].y = tmpY*fScale;
			if (m_fNormalizeProject[y][x].x > PIXEL_OF_SCREEN_X ||
				m_fNormalizeProject[y][x].x < 0)
			{
				m_fNormalizeProject[y][x].x = INVAILD_VALUE;
				m_fNormalizeProject[y][x].y = INVAILD_VALUE;
			}

			if (m_fNormalizeProject[y][x].y > PIXEL_OF_SCREEN_Y ||
				m_fNormalizeProject[y][x].y < 0)
			{
				m_fNormalizeProject[y][x].x = INVAILD_VALUE;
				m_fNormalizeProject[y][x].y = INVAILD_VALUE;
			}
		}
	}
	float distX;
	float distY;
	float threshold = 0.05;
	int val = 2;

	for (int y = 0; y < dy; y++)
	{
		for (int x = 0; x < dx; x++)
		{
			if (m_fNormalizeProject[y][x].x == INVAILD_VALUE)
			{
				continue;
			}
			else{
				m_mappingTable[y][x].x = (int)floor(m_fNormalizeProject[dy - y - 1][x].x);
				if (m_mappingTable[y][x].x > PIXEL_OF_SCREEN_X - val) m_mappingTable[y][x].x = PIXEL_OF_SCREEN_X - val;
				m_mappingTable[y][x].y = (int)floor(m_fNormalizeProject[dy - y - 1][x].y);
				if (m_mappingTable[y][x].y > PIXEL_OF_SCREEN_Y - val) m_mappingTable[y][x].y = PIXEL_OF_SCREEN_Y - val;

				distX = m_fNormalizeProject[y][x].x - floor(m_fNormalizeProject[y][x].x);
				distY = m_fNormalizeProject[y][x].x - floor(m_fNormalizeProject[y][x].x);
				//TOPLEFT에 가까운경우
				if (distX < threshold && distY < threshold)
				{
					m_mappingTable[y][x].dist[TOPLEFT] = 1;
					m_mappingTable[y][x].dist[TOPRIGHT] = 0;
					m_mappingTable[y][x].dist[BOTTOMRIGHT] = 0;
					m_mappingTable[y][x].dist[BOTTOMLEFT] = 0;
				}
				//TOPRIGHT에 가까운경우
				else if (distX >(1 - threshold) && distY < threshold)
				{
					m_mappingTable[y][x].dist[TOPLEFT] = 0;
					m_mappingTable[y][x].dist[TOPRIGHT] = 1;
					m_mappingTable[y][x].dist[BOTTOMRIGHT] = 0;
					m_mappingTable[y][x].dist[BOTTOMLEFT] = 0;
				}
				//BOTTOMRIGHT에 가까운경우
				else if (distX >(1 - threshold) && distY > (1 - threshold))
				{
					m_mappingTable[y][x].dist[TOPLEFT] = 0;
					m_mappingTable[y][x].dist[TOPRIGHT] = 0;
					m_mappingTable[y][x].dist[BOTTOMRIGHT] = 1;
					m_mappingTable[y][x].dist[BOTTOMLEFT] = 0;
				}
				//BOTTOMLEFT에 가까운 경우
				else if (distX < threshold && distY >(1 - threshold))
				{
					m_mappingTable[y][x].dist[TOPLEFT] = 0;
					m_mappingTable[y][x].dist[TOPRIGHT] = 0;
					m_mappingTable[y][x].dist[BOTTOMRIGHT] = 0;
					m_mappingTable[y][x].dist[BOTTOMLEFT] = 1;
				}
				else  //그 이외의 경우
				{
					m_mappingTable[y][x].dist[TOPLEFT] =
						(1 - distX) * (1 - distY);
					m_mappingTable[y][x].dist[TOPRIGHT] =
						(1 - distY) - m_mappingTable[y][x].dist[TOPLEFT];
					m_mappingTable[y][x].dist[BOTTOMRIGHT] =
						(distX)* (distY);
					m_mappingTable[y][x].dist[BOTTOMLEFT] =
						1 - (m_mappingTable[y][x].dist[TOPLEFT]
						+ m_mappingTable[y][x].dist[TOPRIGHT]
						+ m_mappingTable[y][x].dist[BOTTOMRIGHT]);
				}
			}
		}
	}
}



//warping진행
void MyWarping::RunArbitraryWarping()
{


	//스크린 X * Y 사이즈 지정
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;

	//테스트에 사용할 이미지
	//cv::Mat img = imread("original.bmp");
	cv::Mat img("vertical2.png");
	resize(img, img, Size(dx, dy));

	//결과 이미지 셋팅
	Mat resultImg;
	resultImg.create(Size(dx, dy - 1), CV_8UC3);
	memset(resultImg.data, 0, sizeof(uchar)*resultImg.cols*resultImg.rows * 3);


	uchar* src = img.data;
	uchar* dst = resultImg.data;
	float sumR, sumG, sumB, dist;
	int tempY, tempX;
	for (int y = 0; y < dy - 1; y++)
	{
		for (int x = 0; x < dx; x++)
		{
			if (m_mappingTable[y][x].x == INVAILD_VALUE)
			{
				dst++;
				dst++;
				dst++;
				continue;
			}
			else
			{
				sumR = 0;
				sumG = 0;
				sumB = 0;
				for (int c = 0; c < 4; c++)
				{
					tempX = m_mappingTable[y][x].x;
					tempY = m_mappingTable[y][x].y;
					dist = m_mappingTable[y][x].dist[c];
					sumB += (*(src + (tempY + (c / 2)) *resultImg.cols * 3 + (tempX + (c % 2)) * 3 + 0))*dist;
					sumG += (*(src + (tempY + (c / 2)) *resultImg.cols * 3 + (tempX + (c % 2)) * 3 + 1))*dist;
					sumR += (*(src + (tempY + (c / 2)) *resultImg.cols * 3 + (tempX + (c % 2)) * 3 + 2))*dist;
				}
				if (sumB > 255) sumB = 255;
				if (sumG > 255) sumG = 255;
				if (sumR > 255) sumR = 255;

				*(dst++) = (uchar)sumB;
				*(dst++) = (uchar)sumG;
				*(dst++) = (uchar)sumR;
			}
		}
	}
}


*/