#include "stdafx.h"
#include "MappingTableController.h"
#include <math.h>
#include "optimalRect.h"

MappingTableController::MappingTableController()
{
	InitValue();
}

MappingTableController::~MappingTableController()
{
	delete[] m_worldCoordinateForPlane;
	delete[] m_worldCoordinateFromUser;
	delete[] m_virtualScreen;
	delete[] m_virtualScreenRotated;
	delete[] tempCoordinate;
	delete[] m_matchedPoint;
}

void MappingTableController::InitValue()
{
	//임시로 셋팅
	SetProjectorLocation(cv::Point3d(0, 0, 0));
	//SetUserLocation(cv::Point3d(USER_X, USER_Y, USER_Z));

	int height = PIXEL_OF_SCREEN_Y;
	int width = PIXEL_OF_SCREEN_X;
	m_worldCoordinateForPlane = (Point3f **)malloc(sizeof(Point3f *)* height);
	m_worldCoordinateForPlane[0] = (Point3f *)malloc(sizeof(Point3f) * width*height);
	for (int y = 1; y < height; y++)
		m_worldCoordinateForPlane[y] = m_worldCoordinateForPlane[y - 1] + width;

	m_worldCoordinateFromUser = (Point3f **)malloc(sizeof(Point3f *)* height);
	m_worldCoordinateFromUser[0] = (Point3f *)malloc(sizeof(Point3f) * width*height);
	for (int y = 1; y < height; y++)
		m_worldCoordinateFromUser[y] = m_worldCoordinateFromUser[y - 1] + width;


	m_virtualScreen = (Point3f **)malloc(sizeof(Point3f *)* height);
	m_virtualScreen[0] = (Point3f *)malloc(sizeof(Point3f) * width*height);
	for (int y = 1; y < height; y++)
		m_virtualScreen[y] = m_virtualScreen[y - 1] + width;

	m_virtualScreenRotated = (Point3f **)malloc(sizeof(Point3f *)* height);
	m_virtualScreenRotated[0] = (Point3f *)malloc(sizeof(Point3f) * width*height);
	for (int y = 1; y < height; y++)
		m_virtualScreenRotated[y] = m_virtualScreenRotated[y - 1] + width;


	tempCoordinate = (Point2f **)malloc(sizeof(Point2f *)* height);
	tempCoordinate[0] = (Point2f *)malloc(sizeof(Point2f) * width*height);
	for (int y = 1; y < height; y++)
		tempCoordinate[y] = tempCoordinate[y - 1] + width;


	m_matchedPoint = (MappingPos **)malloc(sizeof(MappingPos *)* height);
	m_matchedPoint[0] = (MappingPos *)malloc(sizeof(MappingPos) * width*height);
	for (int y = 1; y < height; y++)
		m_matchedPoint[y] = m_matchedPoint[y - 1] + width;

	m_bMarkingPos = (bool **)malloc(sizeof(bool *)* height);
	m_bMarkingPos[0] = (bool *)malloc(sizeof(bool) * width*height);
	for (int y = 1; y < height; y++)
		m_bMarkingPos[y] = m_bMarkingPos[y - 1] + width;


	m_mappingTable = (Point2f **)malloc(sizeof(Point2f *)* height);
	m_mappingTable[0] = (Point2f *)malloc(sizeof(Point2f) * width*height);
	for (int y = 1; y < height; y++)
		m_mappingTable[y] = m_mappingTable[y - 1] + width;




}

//임시 Depth영상 생성
cv::Mat MappingTableController::TemporaryDepthImage()
{
	Mat img1;
	//스크린 X * Y 사이즈 지정
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;

	img1.create(cvSize(dx, dy), CV_8UC3);

	for (register int y = 0; y < dy; y++)
		for (register int x = 0; x < dx; x++)
		{
			img1.at<short>(y, x) = 1000 + x;
		}	
	return img1;
}

//User의 위치 지정
void MappingTableController::SetUserLocation(Point3d inData)
{
	m_userLocation.x = inData.x;
	m_userLocation.y = inData.y;
	m_userLocation.z = inData.z;

}

//프로젝터의 위치 지정
void MappingTableController::SetProjectorLocation(Point3d inData)
{
	m_projectorLoaction.x = inData.x;
	m_projectorLoaction.y = inData.y;
	m_projectorLoaction.z = inData.z;
}

//world coordinate로 변경
void MappingTableController::TransPixelToWorldCoordiante(Mat inImg)
{
	//스크린 X * Y 사이즈 지정
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;

	double degree = PI / 180;	//1도
	double baseDetph = 1000;	//기준 거리

	//프로젝트의 FOV
	double fovX = PROJECT_FOV_X / 2;
	double fovY = PROJECT_FOV_Y / 2;

	//프로젝트의 FoV에 따른 실제 X, Y의 최대 길이 (가운데점 (0,0) 기준)
	double maxX = baseDetph*tan(fovX * degree);
	double maxY = baseDetph*tan(fovY * degree);

	//1미터 거리에서 1픽셀당 실제 길이
	double sizePerPixel = maxX / (PIXEL_OF_SCREEN_X/2);

	//임시용
	m_sizePerPixel = sizePerPixel;

	double depth = 0;
	//World Coordinate 생성

	//센터 월드좌표 생성
	depth = inImg.at<float>(dy/2, dx/2);
	m_worldCoordinateBoundary[CENTER].x = 0;
	m_worldCoordinateBoundary[CENTER].y = 0;
	m_worldCoordinateBoundary[CENTER].z = depth;

	//바운더리 월드좌표 생성
	depth = inImg.at<float>(0, 0);
	m_worldCoordinateBoundary[TOPLEFT].x = -maxX*(depth / baseDetph);
	m_worldCoordinateBoundary[TOPLEFT].y = maxY*(depth / baseDetph);
	m_worldCoordinateBoundary[TOPLEFT].z = depth;

	depth = inImg.at<float>(0, dx-1);
	m_worldCoordinateBoundary[TOPRIGHT].x = maxX*(depth / baseDetph);
	m_worldCoordinateBoundary[TOPRIGHT].y = maxY*(depth / baseDetph);
	m_worldCoordinateBoundary[TOPRIGHT].z = depth;

	depth = inImg.at<float>(dy-1, dx-1);
	m_worldCoordinateBoundary[BOTTOMRIGHT].x = maxX*(depth / baseDetph);
	m_worldCoordinateBoundary[BOTTOMRIGHT].y = -maxY*(depth / baseDetph);
	m_worldCoordinateBoundary[BOTTOMRIGHT].z = depth;

	depth = inImg.at<float>(dy-1, 0);
	m_worldCoordinateBoundary[BOTTOMLEFT].x = -maxX*(depth / baseDetph);
	m_worldCoordinateBoundary[BOTTOMLEFT].y = -maxY*(depth / baseDetph);
	m_worldCoordinateBoundary[BOTTOMLEFT].z = depth;


	//실험용
	//World Coordinate 생성
	for (register int y = 0; y < dy; y++)
		for (register int x = 0; x < dx; x++)
		{
			depth = inImg.at<float>(y, x);
			m_worldCoordinateForPlane[y][x].x = (double)((x - (dx / 2)) * sizePerPixel*(depth / baseDetph));
			m_worldCoordinateForPlane[y][x].y = -(double)((y - (dy / 2)) * sizePerPixel*(depth / baseDetph));
			m_worldCoordinateForPlane[y][x].z = depth;
		}


		cvWaitKey(1);
}

/*

//world coordinate로 변경
void MappingTableController::TransPixelToWorldCoordiante(Mat inImg, Point3f** worldCoordinateForPlane)
{
	//스크린 X * Y 사이즈 지정
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;

	double degree = PI / 180;	//1도
	double baseDetph = 1000;	//기준 거리

	//기준 FOV
	double fovX = BASE_FOV_X / 2;
	double fovY = BASE_FOV_Y / 2;

	//기준 FoV에 따른 실제 X, Y의 최대 길이 (가운데점 (0,0) 기준)
	double maxX = baseDetph*tan(fovX * degree);

	//1미터 거리에서 1픽셀당 실제 길이
	double sizePerPixel = maxX / (256.) * (160./1920.);
	
	//임시용
	m_sizePerPixel = sizePerPixel;

	double depth = 0;
	//World Coordinate 생성
	for (register int y = 0; y < dy; y++)
		for (register int x = 0; x < dx; x++)
		{
			depth = inImg.at<float>(y, x);
			m_worldCoordinateForPlane[y][x].x = (double)((x - (dx / 2)) * sizePerPixel*(depth / baseDetph));
			m_worldCoordinateForPlane[y][x].y = (double)((y - (dy / 2)) * sizePerPixel*(depth / baseDetph));
			m_worldCoordinateForPlane[y][x].z = depth;
		}
	worldCoordinateForPlane = m_worldCoordinateForPlane;
}
*/

void MappingTableController::CreateWorldPlaneFromUser()
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



void MappingTableController::RotateVirtualScreen()
{
	//스크린 X * Y 사이즈 지정
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;
	Point3f ptTemp(0, 0, 0);


	double angleX, angleY, angleZ;
	double distUnW;

	distUnW = sqrt(
		(m_userLocation.x * m_userLocation.x) +
		((m_worldCoordinateBoundary[CENTER].z - m_userLocation.z) *
		(m_worldCoordinateBoundary[CENTER].z - m_userLocation.z))
		);	


	//프로젝터 투사면 유저간 각도 구하기
	angleX = -cvFastArctan(m_userLocation.y, distUnW);
	//angleX = -cvFastArctan(m_worldCoordinateBoundary[CENTER].z - m_userLocation.z, m_userLocation.y);
	angleY = cvFastArctan(m_userLocation.x, m_worldCoordinateBoundary[CENTER].z - m_userLocation.z);
	angleX = angleX / 180. * PI;
	angleY = angleY / 180. * PI;

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



       	cv::waitKey();
	
}


void MappingTableController::CreateVirtualScreenCoordinate()
{
	//스크린 X * Y 사이즈 지정
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;
	double dist = 100;

	double depth = 1000;

	//평면의 방정식 구하기
	double a, b, c, d;	//평면 parameter
	double x1, y1, z1;	//평면위의 한점
	double x2, y2, z2;	//직선방향 벡터
	double t;			//임의값


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
	cvWaitKey(1);
}




//바운더리 점 찾기
void MappingTableController::FindVirtualScreenBoundary()
{
	//스크린 X * Y 사이즈 지정
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;
	
	double minDistX = 10000;
	double minDistY = 10000;
	double baseDist;

	for (int x = 0; x < dx; x++)
	{
		if (minDistY > abs(m_virtualScreenRotated[0][x].y))
		{
			minDistY = abs(m_virtualScreenRotated[0][x].y);
		}
		if (minDistY > abs(m_virtualScreenRotated[dy - 1][x].y))
		{
			minDistY = abs(m_virtualScreenRotated[dy - 1][x].y);
		}
	}

	for (int y = 0; y < dy; y++)
	{
		if (minDistX > abs(m_virtualScreenRotated[y][0].x))
		{
			minDistX = abs(m_virtualScreenRotated[y][0].x);
		}
		if (minDistX > abs(m_virtualScreenRotated[0][dx - 1].x))
		{
			minDistX = abs(m_virtualScreenRotated[0][dx - 1].x);
		}
	}


	if (minDistX >= minDistY*(16.0 / 9.0))
		baseDist = minDistY*(16.0 / 9.0);
	else
		baseDist = minDistX;

	double distPerPixel = baseDist*2 / (double)dx;

	m_boundary[LEFT_U] = -(PIXEL_OF_SCREEN_X / 2)*distPerPixel;
	m_boundary[RIGHT_U] = (PIXEL_OF_SCREEN_X / 2)*distPerPixel;
	m_boundary[TOP_U] = -(PIXEL_OF_SCREEN_Y / 2)*distPerPixel;
	m_boundary[BOTTOM_U] = (PIXEL_OF_SCREEN_Y / 2)*distPerPixel;



	for (int y = 0; y < dy; y++)
		for (int x = 0; x < dx; x++)
		{
			tempCoordinate[y][x].x = (x - PIXEL_OF_SCREEN_X/2)*distPerPixel;
			tempCoordinate[y][x].y = (y - PIXEL_OF_SCREEN_Y/2)*distPerPixel;			
		}
}



void MappingTableController::MatchingBoundaryMappingPoint()
{
	//스크린 X * Y 사이즈 지정
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;
	//Making Mapping Table

	double tempDist;
	double range;

	float val = 0;

	for (int i = 0; i < 4; i++)
	{
		val = max(val, abs(m_virtualScreenBoundary[i].x));
		val = max(val, abs(m_virtualScreenBoundary[i].y));
	}

	optimalRect opt;

	for (int i = 0; i < 4; i++)
		opt.addPoint(Point2f(m_virtualScreenBoundary[i].x+val, m_virtualScreenBoundary[i].y+val));

	cv::Rect rect = opt.computeLargestRectangle();
	//임시로 4개의 유저관점 평면 바운더리 지정
	m_userScreenFoundedBoundary[BOTTOMLEFT].x = rect.x;
	m_userScreenFoundedBoundary[BOTTOMLEFT].y = rect.y;

	m_userScreenFoundedBoundary[BOTTOMRIGHT].x = rect.x + rect.width;
	m_userScreenFoundedBoundary[BOTTOMRIGHT].y = rect.y;

	m_userScreenFoundedBoundary[TOPRIGHT].x = rect.x + rect.width;
	m_userScreenFoundedBoundary[TOPRIGHT].y = rect.y + rect.height;

	m_userScreenFoundedBoundary[TOPLEFT].x = rect.x;
	m_userScreenFoundedBoundary[TOPLEFT].y = rect.y + rect.height;

	m_userScreenFoundedBoundary[CENTER].x = 0;
	m_userScreenFoundedBoundary[CENTER].y = 0;

	for (int i = 0; i < 4; i++)
	{
		m_userScreenFoundedBoundary[i].x -= val;
		m_userScreenFoundedBoundary[i].y -= val;
	}


	//가장가까운점 찾기
	for (int i = 0; i < 5; i++)
	{
		range = 100;
		for (int y = 0; y < dy; y++)
		{
			for (int x = 0; x < dx; x++)
			{
				//좌표간 거리
				tempDist = sqrt(
					(m_virtualScreen[y][x].x - m_userScreenFoundedBoundary[i].x) *
					(m_virtualScreen[y][x].x - m_userScreenFoundedBoundary[i].x) +
					(m_virtualScreen[y][x].y - m_userScreenFoundedBoundary[i].y) *
					(m_virtualScreen[y][x].y - m_userScreenFoundedBoundary[i].y)
					);

				//설정한 범위내에 점인지 확인
				if (range > tempDist)
				{
					m_BoundaryMathedPoint[i].x = x; //x좌표 대입
					m_BoundaryMathedPoint[i].y = y; //y좌표 대입
					range = tempDist;
				}//if
			}//for x			
		}//for y
	}
	cvWaitKey(1);
}



//mappingTable
void MappingTableController::MatchingMappingPoint()
{
	//스크린 X * Y 사이즈 지정
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;
	//Making Mapping Table
	
	double tempDist;
	double storeDist;

	int numberOfMarking = 0;
	int trueCount = 0;

	
	int endY = dy;
	int endX = dx;
	double range = abs(tempCoordinate[0][1].x - tempCoordinate[0][0].x)*1;
	int count = 0;

	int pxDist = 10;
	
	

	Point2d preScreenPos(0, 0); //가까운 점 찾기의 시작점
	


	int tmpX = 0;
	int tmpY = 0;
	/*
	for (int y = 0; y < dy; y++)
	{
		for (int x = 0; x < dx; x++)
		{			
			tempDist = sqrt(
				(tempCoordinate[0][0].x - m_virtualScreenRotated[y][x].x) *
				(tempCoordinate[0][0].x - m_virtualScreenRotated[y][x].x) +
				(tempCoordinate[0][0].y - m_virtualScreenRotated[y][x].y) *
				(tempCoordinate[0][0].y - m_virtualScreenRotated[y][x].y)
				);
			//설정한 범위내에 점인지 확인
			if (range > tempDist)
			{
				tmpX = x;
				tmpY = y;
				//검색범위 지정
				preScreenPos.x = x - pxDist;
				preScreenPos.y = y - pxDist;
				endX = x + pxDist;
				endY = y + pxDist;
				if (preScreenPos.x < 0) preScreenPos.x = 0;
				if (endX > PIXEL_OF_SCREEN_X - 1) endX = PIXEL_OF_SCREEN_X - 1;
				if (preScreenPos.y < 0) preScreenPos.x = 0;
				if (endY > PIXEL_OF_SCREEN_Y - 1) endY = PIXEL_OF_SCREEN_Y - 1;
				break;
			}//if
		}
	}
	
	*/

	std::cout << tempCoordinate[0][0].x << ", " << tempCoordinate[0][0].y << std::endl;
	std::cout << tmpX  << ", " << tmpY << std::endl;
	std::cout << m_virtualScreenRotated[tmpY][tmpX].x << ", " << m_virtualScreenRotated[tmpY][tmpX].y << std::endl;
	


	preScreenPos.x = 0;
	preScreenPos.y = 0;
	endX = dx;
	endY = dy;


	for (int y = 0; y < dy; y++)
	{
		for (int x = 0; x < dx; x++)
		{
			count = 0;
			if (m_bMarkingPos[y][x])
			{
				numberOfMarking++;
				for (int yy = preScreenPos.y; yy < endY; yy++)
				{
					for (int xx = preScreenPos.x; xx < endX; xx++)
					{
						//좌표간 거리
						tempDist = sqrt(
							(m_virtualScreenRotated[y][x].x - tempCoordinate[yy][xx].x) *
							(m_virtualScreenRotated[y][x].x - tempCoordinate[yy][xx].x) +
							(m_virtualScreenRotated[y][x].y - tempCoordinate[yy][xx].y) *
							(m_virtualScreenRotated[y][x].y - tempCoordinate[yy][xx].y) 							
							);

						//설정한 범위내에 점인지 확인
						if (range > tempDist)
						{
							m_matchedPoint[y][x].relatedPos[count].x = xx; //x좌표 대입
							m_matchedPoint[y][x].relatedPos[count].y = yy; //y좌표 대입
							m_matchedPoint[y][x].relatedPos[count].dist = tempDist; //거리대입
							count++;	//카운트 증가
							
						}//if
					}//for xx			
				}//for yy		
				//검색범위 지정
				if (count > 0)
				{
					preScreenPos.x = m_matchedPoint[y][x].relatedPos[0].x - pxDist;
					preScreenPos.y = m_matchedPoint[y][x].relatedPos[0].y - pxDist;
					endX = m_matchedPoint[y][x].relatedPos[0].x + pxDist;
					endY = m_matchedPoint[y][x].relatedPos[0].y + pxDist;
					trueCount++;
				}

				if (preScreenPos.x < 0) 
					preScreenPos.x = 0;
				if (endX > PIXEL_OF_SCREEN_X - 1) 
					endX = PIXEL_OF_SCREEN_X - 1;
				if (preScreenPos.y < 0)
					preScreenPos.y = 0;
				if (endY > PIXEL_OF_SCREEN_Y - 1) 
					endY = PIXEL_OF_SCREEN_Y - 1;
			}
			m_matchedPoint[y][x].count = count;//카운트값 대입
		}//for x
		preScreenPos.x = 0;
		endX = dx;

		std::cout <<  y << std::endl;
	}//for y
	std::cout << trueCount << std::endl;
}


void MappingTableController::CreateMappingTable()
{
	double sum = 0;


}

void MappingTableController::RunWarping()
{
	//스크린 X * Y 사이즈 지정
	int dx = PIXEL_OF_SCREEN_X;
	int dy = PIXEL_OF_SCREEN_Y;


	Mat img = imread("Original.bmp");
	resize(img, img, Size(dx, dy));
	printf("channel : %d\n", img.channels());
	Mat resultImg;
	resultImg.create(Size(dx, dy), CV_8UC3);
	memset(resultImg.data, 0, sizeof(uchar)*resultImg.cols*resultImg.rows * 3);

	double sumDist;

	uchar* src = img.data;
	uchar* dst = resultImg.data;
	double sumR, sumG, sumB, dist;
	int tempY, tempX;
	for (int y = 0; y < dy; y++)
	{
		for (int x = 0; x < dx; x++)
		{
			sumDist = 0;
			sumR = 0; 
			sumG = 0; 
			sumB = 0;
			for (int c = 0; c < m_matchedPoint[y][x].count; c++)
			{
				tempY = m_matchedPoint[y][x].relatedPos[c].y;
				tempX = m_matchedPoint[y][x].relatedPos[c].x;
				dist = m_matchedPoint[y][x].relatedPos[c].dist;
				sumDist += dist;
				sumB += (*(src + tempY*resultImg.cols * 3 + tempX * 3 + 0))*dist;
				sumG += (*(src + tempY*resultImg.cols * 3 + tempX * 3 + 1))*dist;
				sumR += (*(src + tempY*resultImg.cols * 3 + tempX * 3 + 2))*dist;
			}
			*(dst++) = sumB / sumDist;
			*(dst++) = sumG / sumDist;
			*(dst++) = sumR / sumDist;
			
		}
	}

	imshow("original", img);
	imshow("resultImg", resultImg);
	imwrite("resultImg.png", resultImg);
	waitKey();
}






Point3f** MappingTableController::returnWorldCoordinateForPlane()
{
	return m_worldCoordinateForPlane;
}

/*
void MappingTableController::Matching(int _maskSize)
{
	Point3f** _gray = m_virtualScreenRotated;
	Point2f** _red = tempCoordinate;
	int H, W;
	double dist;

	// [0][0] 위치를 먼저 찾는다.
	double min = 1000000;
	int minH, minW;
	int prev_minH, prev_minW;
	minH = minW = 0;
	for (int h = 0; h < MAXHEIGHT; h++)
	{
		for (int w = 0; w < MAXWIDTH; w++)
		{
			dist = Distance(_red[0][0], _gray[h][w]);
			if (dist < min)
			{
				min = dist;
				minH = h;
				minW = w;
			}
		}
	}
	prev_minH = minH;
	prev_minW = minW;

	int maskSize = _maskSize;
	for (int h = 0; h < MAXHEIGHT; h++)
	{
		if (h > 1)
		{
			prev_minH = m_mappingTable[h - 1][0].y;
			prev_minW = m_mappingTable[h - 1][0].x;
		}
		for (int w = 0; w < MAXWIDTH; w++)
		{
			min = 1000000;
			minH = minW = 0;

			for (int hh = prev_minH - maskSize; hh < prev_minH + maskSize; hh++)
			{
				for (int ww = prev_minW - maskSize; ww < prev_minW + maskSize; ww++)
				{
					if (hh < 0) continue;
					if (hh >= MAXHEIGHT) continue;
					if (ww < 0) continue;
					if (ww >= MAXWIDTH) continue;
					dist = Distance(_red[h][w], _gray[hh][ww]);
					if (dist < min)
					{
						min = dist;
						minH = hh;
						minW = ww;
					}
				}
			}
			prev_minH = minH;
			prev_minW = minW;
			m_mappingTable[h][w].x = minW;
			m_mappingTable[h][w].y = minH;
		}
		printf("Calc : [%d]\n", h);
	}
}

void MappingTableController::Mapping()
{
	Mat img = imread("Original.bmp");
	resize(img, img, Size(MAXWIDTH, MAXHEIGHT));
	printf("channel : %d\n", img.channels());
	Mat resultImg;
	resultImg.create(Size(MAXWIDTH, MAXHEIGHT), CV_8UC3);
	memset(resultImg.data, 0, sizeof(uchar)*resultImg.cols*resultImg.rows * 3);

	uchar* src = img.data;
	uchar* dst = resultImg.data;
	int tempH, tempW;
	for (int h = 0; h < MAXHEIGHT; h++)
	{
		for (int w = 0; w < MAXWIDTH; w++)
		{
			tempH = m_mappingTable[h][w].y;
			tempW = m_mappingTable[h][w].x;
			*(dst + tempH*resultImg.cols * 3 + tempW * 3 + 0) = *(src++);
			*(dst + tempH*resultImg.cols * 3 + tempW * 3 + 1) = *(src++);
			*(dst + tempH*resultImg.cols * 3 + tempW * 3 + 2) = *(src++);
		}
	}

	imshow("resultImg", resultImg);
	imwrite("resultImg.png", resultImg);
	waitKey();
}
*/