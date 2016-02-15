
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

	//��ũ�� ���� ������ �����Ҵ�
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
	//���Ϳ� �ٿ������ �����ڵ������ ����	
	TransPixelToWorldCoordiante(depthImg);
	//�������������� Plane ����
	CreateWorldPlaneFromUser();
	//������������ ȸ��
	RotateVirtualScreen();
	//������� ��ֶ�����
	CreateVirtualScreenCoordinate();
	//�ٿ���� ���� ã��
	FindVirtualScreenBoundary();
	//���� ���̺� �����
	CreateMappingTable();
}

void MyWarping::SetDepthFoV(CPoint2f inData)
{
	m_fDepthFoV.x = inData.x;
	m_fDepthFoV.y = inData.y;
}


//world coordinate�� ����
void MyWarping::TransPixelToWorldCoordiante(Mat inImg)
{
	//��ũ�� X * Y ������ ����
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;

	float degree = (float)PI / 180;	//1��
	float baseDetph = 1000;	//���� �Ÿ�

	//������Ʈ�� FOV
	float fovX = (float)PROJECT_FOV_X / 2.;
	float fovY = (float)PROJECT_FOV_Y / 2.;

	//������Ʈ�� FoV�� ���� ���� X, Y�� �ִ� ���� (����� (0,0) ����)
	float maxX = baseDetph*tan(fovX * degree);
	float maxY = baseDetph*tan(fovY * degree);

	//1���� �Ÿ����� 1�ȼ��� ���� ����
	float sizePerPixel = maxX / ((float)PIXEL_OF_SCREEN_X / 2.);
	float sizePerPixelY = maxY / ((float)PIXEL_OF_SCREEN_Y / 2.);

	//�ӽÿ�
	m_sizePerPixel = sizePerPixel;

	float depth = 0;
	//World Coordinate ����

	//���� ������ǥ ����
	depth = inImg.at<float>(dy / 2, dx / 2);
	m_worldCoordinateBoundary[CENTER].x = 0;
	m_worldCoordinateBoundary[CENTER].y = 0;
	m_worldCoordinateBoundary[CENTER].z = depth;

	//�ٿ���� ������ǥ ����
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


	//�����
	//World Coordinate ����
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

	//��ũ�� X * Y ������ ����
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;
	//
	//1. �����κ��� ȭ�� �߽���ǥ�� ���Ϳ� �Ÿ� ���ϱ� 
	//	

	//�����κ����� ������� �ٿ������ 4����
	for (int i = 0; i < 5; i++)
	{
		m_worldCoordinateBoundaryFromUser[i].x = m_worldCoordinateBoundary[i].x - m_userLocation.x;
		m_worldCoordinateBoundaryFromUser[i].y = m_worldCoordinateBoundary[i].y - m_userLocation.y;
		m_worldCoordinateBoundaryFromUser[i].z = m_worldCoordinateBoundary[i].z - m_userLocation.z;
	}

	//�����
	//�������忡���� �������
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
	//4���� ������
	float x1, y1, z1;
	float x2, y2, z2;
	float x3, y3, z3;
	float x4, y4, z4;
	float x5, y5, z5;

	//2���� ���� ����
	float v11, v12, v13;
	float v21, v22, v23;

	float h1, h2, h3;


	//4���� ������ ����

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

	//2���� ��������
	v11 = x2 - x1;	v12 = y2 - y1;	v13 = z2 - z1;
	v21 = x4 - x3;	v22 = y4 - y3;	v23 = z4 - z3;

	v11 /= 10.0;
	v12 /= 10.0;
	v13 /= 10.0;
	v21 /= 10.0;
	v22 /= 10.0;
	v23 /= 10.0;

	//������� ��������
	h1 = v12*v23 - v13*v22;
	h2 = v13*v21 - v11*v23;
	h3 = v11*v22 - v12*v21;

	//ȸ���� ���ϱ�
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


	//y�� ȸ����ȯ	
	for (int i = 0; i < 5; i++)
	{
		ptTemp.x = m_frontScreenBoundary[i].x;
		ptTemp.y = m_frontScreenBoundary[i].y;
		ptTemp.z = m_frontScreenBoundary[i].z;

		m_rotatedFrontScreenBoundary[i].x = ptTemp.z * sin(angleY) + ptTemp.x*cos(angleY);
		m_rotatedFrontScreenBoundary[i].y = ptTemp.y;
		m_rotatedFrontScreenBoundary[i].z = ptTemp.z * cos(angleY) - ptTemp.x*sin(angleY);
	}
	//x�� ȸ����ȯ	
	for (int i = 0; i < 5; i++)
	{
		ptTemp.x = m_rotatedFrontScreenBoundary[i].x;
		ptTemp.y = m_rotatedFrontScreenBoundary[i].y;
		ptTemp.z = m_rotatedFrontScreenBoundary[i].z;

		m_rotatedFrontScreenBoundary[i].x = ptTemp.x;
		m_rotatedFrontScreenBoundary[i].y = cos(angleX)*ptTemp.y - sin(angleX)*ptTemp.z;
		m_rotatedFrontScreenBoundary[i].z = sin(angleX)*ptTemp.y + cos(angleX)*ptTemp.z;
	}

	//ȸ���� ������ǥ����
	Point3f center = m_rotatedFrontScreenBoundary[CENTER];

	//Ű��Ʈ �߽���ǥ�� �����̵�
	for (int i = 0; i < 5; i++)
	{
		ptTemp.x = m_rotatedFrontScreenBoundary[i].x;
		ptTemp.y = m_rotatedFrontScreenBoundary[i].y;
		ptTemp.z = m_rotatedFrontScreenBoundary[i].z;

		m_rotatedFrontScreenBoundary[i].x = ptTemp.x - center.x;
		m_rotatedFrontScreenBoundary[i].y = ptTemp.y - center.y;
	}

	//��ũ�� X * Y ������ ����
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;


	//�����
	//ȸ����ȯ�� virtual coordinater ����
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

	//����� ������ ���ϱ�
	float a, b, c, d;	//��� parameter
	float t;			//���ǰ�


	//������� ����
	x1 = 0;
	y1 = 0;
	z1 = 1000;

	//����� Normal vector
	a = 0;
	b = 0;
	c = 1;

	//ax +by +cz +d = 0 �� d ���ϱ�
	d = (-x1)*a + (-y1)*b + (-z1)*c;

	//�ٿ���� ��ֶ�����
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

	//���߾�ũ�� ��ֶ�����
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
	//��ũ�� X * Y ������ ����
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;
	Point3f ptTemp(0, 0, 0);

	float dist = 100;
	float depth = 1000;

	//����� ������ ���ϱ�
	float a, b, c, d;	//��� parameter
	float x1, y1, z1;
	float x2, y2, z2;
	float t;			//���ǰ�


	//������� ����
	x1 = 0;
	y1 = 0;
	z1 = 1000;

	//����� Normal vector
	a = 0;
	b = 0;
	c = 1;

	//ax +by +cz +d = 0 �� d ���ϱ�
	d = (-x1)*a + (-y1)*b + (-z1)*c;

	float angleX, angleY;
	float distUnW;

	distUnW = sqrt(
		(m_userLocation.x * m_userLocation.x) +
		((m_worldCoordinateBoundary[CENTER].z - m_userLocation.z) *
		(m_worldCoordinateBoundary[CENTER].z - m_userLocation.z))
		);


	//y�� ȸ����ȯ	
	for (int i = 0; i < 5; i++)
	{
		ptTemp.x = m_worldCoordinateBoundaryFromUser[i].x;
		ptTemp.y = m_worldCoordinateBoundaryFromUser[i].y;
		ptTemp.z = m_worldCoordinateBoundaryFromUser[i].z;

		m_virtualScreenBoundaryRotated[i].x = ptTemp.z * sin(angleY) + ptTemp.x*cos(angleY);
		m_virtualScreenBoundaryRotated[i].y = ptTemp.y;
		m_virtualScreenBoundaryRotated[i].z = ptTemp.z * cos(angleY) - ptTemp.x*sin(angleY);
	}
	//x�� ȸ����ȯ	
	for (int i = 0; i < 5; i++)
	{
		ptTemp.x = m_virtualScreenBoundaryRotated[i].x;
		ptTemp.y = m_virtualScreenBoundaryRotated[i].y;
		ptTemp.z = m_virtualScreenBoundaryRotated[i].z;

		m_virtualScreenBoundaryRotated[i].x = ptTemp.x;
		m_virtualScreenBoundaryRotated[i].y = cos(angleX)*ptTemp.y - sin(angleX)*ptTemp.z;
		m_virtualScreenBoundaryRotated[i].z = sin(angleX)*ptTemp.y + cos(angleX)*ptTemp.z;
	}


	//�����
	//ȸ����ȯ�� virtual coordinater ����
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

	//���߾�ũ�� ��ֶ�����
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
	//��ũ�� X * Y ������ ����
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;
	float dist = 100;

	float depth = 1000;

	//����� ������ ���ϱ�
	float a, b, c, d;	//��� parameter
	float x1, y1, z1;	//������� ����
	float x2, y2, z2;	//�������� ����
	float t;			//���ǰ�


	//������� ����
	x1 = 0;
	y1 = 0;
	z1 = 1000;

	//����� Normal vector
	a = 0;
	b = 0;
	c = 1;

	//ax +by +cz +d = 0 �� d ���ϱ�
	d = (-x1)*a + (-y1)*b + (-z1)*c;

	//�ٿ���� ��ֶ�����
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

	//���߾�ũ�� ��ֶ�����
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

	//�����̵� ��
	ptMovingValue.x = (float)-userScreenRect.x;
	ptMovingValue.y = (float)userScreenRect.y;

	fScale = (float)dx / (float)userScreenRect.width;

	for (int y = 0; y < dy; y++)
	{
		for (int x = 0; x < dx; x++)
		{
			//��� ��ǥ �����̵�
			tmpX = m_virtualScreen[y][x].x + ptMovingValue.x;
			tmpY = m_virtualScreen[y][x].y + ptMovingValue.y;

			//�ػ󵵿� ���� �����ϸ�
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
				//TOPLEFT�� �������
				if (distX < threshold && distY < threshold)
				{
					m_mappingTable[y][x].dist[TOPLEFT] = 1;
					m_mappingTable[y][x].dist[TOPRIGHT] = 0;
					m_mappingTable[y][x].dist[BOTTOMRIGHT] = 0;
					m_mappingTable[y][x].dist[BOTTOMLEFT] = 0;
				}
				//TOPRIGHT�� �������
				else if (distX >(1 - threshold) && distY < threshold)
				{
					m_mappingTable[y][x].dist[TOPLEFT] = 0;
					m_mappingTable[y][x].dist[TOPRIGHT] = 1;
					m_mappingTable[y][x].dist[BOTTOMRIGHT] = 0;
					m_mappingTable[y][x].dist[BOTTOMLEFT] = 0;
				}
				//BOTTOMRIGHT�� �������
				else if (distX >(1 - threshold) && distY > (1 - threshold))
				{
					m_mappingTable[y][x].dist[TOPLEFT] = 0;
					m_mappingTable[y][x].dist[TOPRIGHT] = 0;
					m_mappingTable[y][x].dist[BOTTOMRIGHT] = 1;
					m_mappingTable[y][x].dist[BOTTOMLEFT] = 0;
				}
				//BOTTOMLEFT�� ����� ���
				else if (distX < threshold && distY >(1 - threshold))
				{
					m_mappingTable[y][x].dist[TOPLEFT] = 0;
					m_mappingTable[y][x].dist[TOPRIGHT] = 0;
					m_mappingTable[y][x].dist[BOTTOMRIGHT] = 0;
					m_mappingTable[y][x].dist[BOTTOMLEFT] = 1;
				}
				else  //�� �̿��� ���
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



//warping����
void MyWarping::RunArbitraryWarping()
{


	//��ũ�� X * Y ������ ����
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;

	//�׽�Ʈ�� ����� �̹���
	//cv::Mat img = imread("original.bmp");
	cv::Mat img("vertical2.png");
	resize(img, img, Size(dx, dy));

	//��� �̹��� ����
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