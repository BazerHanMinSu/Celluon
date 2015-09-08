#include "stdafx.h"
#include "Application.h"

Application::Application()
{
	frameCount = 0;
	
	int height = PIXEL_OF_SCREEN_Y;
	int width = PIXEL_OF_SCREEN_X;
	m_worldCoordinate = (Point3f **)malloc(sizeof(Point3f *)* height);
	m_worldCoordinate[0] = (Point3f *)malloc(sizeof(Point3f) * width*height);
	for (int y = 1; y < height; y++)
		m_worldCoordinate[y] = m_worldCoordinate[y - 1] + width;

	m_userLocation.x = USER_X;
	m_userLocation.y = USER_Y;
	m_userLocation.z = USER_Z;
	
}
Application::~Application()
{

}

void Application::Run()
{



	Mat medianDepthImg;

	/*
	while (1)
	{
		Mat depthFrame;		
		int width, height;

		m_kinect.UpdateKinectDepthFrame();
		depthFrame = m_kinect.GetKinectOriginalDepthFrame();		

		height = depthFrame.rows;
		width = depthFrame.cols;		
		
		Mat resultImg = Mat::zeros(height, width, CV_16UC1);
		
		ushort *src, *dst;
		int h, w;

		m_kinect.UpdateKinectDepthFrame();
		depthFrame = m_kinect.GetKinectOriginalDepthFrame();

		
		src = (ushort*)depthFrame.data;		
		dst = (ushort*)resultImg.data;
		for (h = 0; h < height; h++)
		{
			for (w = 0; w < width; w++)
			{
				dst[h*width + w] = src[h*width + w] * 20;
			}		
		}
		int centerX = width / 2;
		int centerY = height / 2;
			for (int y = -5; y <= 5; y++)
			{
				for (int x = -5; x <= 5; x++)
				{
					dst[(centerY + y)*width + (centerX + x)] = 60000;
				}
			}

		imshow("resultImg", resultImg);
		cvWaitKey(1);
	}
	*/

	GetMedianDepthFrame(medianDepthImg, 30, 3);     // median filtering 된 depth 영상 획득
	CropDepthImage(medianDepthImg, m_cropImage);	//이미지 크랍
	m_mappingControl.SetUserLocation(m_userLocation);
	//센터와 바운더리의 월드코디네이터 생성
	m_mappingControl.TransPixelToWorldCoordiante(m_cropImage);
	//m_mappingControl.TransPixelToWorldCoordiante(m_cropImage, m_worldCoordinate);
	m_mappingControl.CreateWorldPlaneFromUser();	

	//유저관점으로 회전
	m_mappingControl.RotateVirtualScreen();
	//유저평면 노멀라이즈
	m_mappingControl.CreateVirtualScreenCoordinate();
	//워핑 픽셀 위치 찾기
	m_mappingControl.MatchingBoundaryMappingPoint();
	m_BoundaryMathedPoint = m_mappingControl.m_BoundaryMathedPoint;
	
	//m_mappingControl.FindVirtualScreenBoundary();
	//m_mappingControl.MakringPos();
	//m_mappingControl.MatchingMappingPoint();
	//m_mappingControl.RunWarping();





	//m_mappingControl.Matching();
	//m_mappingControl.Mapping();
	


	//OutFileOfPoints();
	//MakeObjFile(m_cropImage, "cropMedian.obj");
	//MakeObjFile(m_mappingControl.m_worldCoordinateForPlane, PIXEL_OF_SCREEN_X, PIXEL_OF_SCREEN_Y, "point3f_world.obj");	
	//MakeObjFile(m_mappingControl.m_worldCoordinateFromUser, PIXEL_OF_SCREEN_X, PIXEL_OF_SCREEN_Y, "worldCoordinateFromUser.obj");	
	//MakeObjFile(m_mappingControl.m_virtualScreen, PIXEL_OF_SCREEN_X, PIXEL_OF_SCREEN_Y, "virtualScreen_BeforeRotate.obj");
	//MakeObjFile(m_mappingControl.m_virtualScreenRotated, PIXEL_OF_SCREEN_X, PIXEL_OF_SCREEN_Y, "virtualScreen_AfterRotate.obj");
	//MakeObjFile(m_mappingControl.tempCoordinate, PIXEL_OF_SCREEN_X, PIXEL_OF_SCREEN_Y, "tempCoordinate.obj");
	//MakeObjFile(m_mappingControl.m_bMarkingPos, PIXEL_OF_SCREEN_X, PIXEL_OF_SCREEN_Y, "InnerRegion.obj");
	//MakeObjFile(m_mappingControl.m_matchedPoint, PIXEL_OF_SCREEN_X, PIXEL_OF_SCREEN_Y, "mappingTable.obj");


	//MakeObjFile(medianDepthImg, "medianOBJ.obj");		
	//imwrite("medianDepthImg.bmp", medianDepthImg);
	




	//m_kinect.UpdateKinectDepthFrame();
	//MeanDepthFrame(30);


	/*
	while (1)
	{
	m_kinect.UpdateKinectDepthFrame();
	imshow("Depth", m_kinect.GetKinectDepthFrame());
	waitKey(1);
	}
	*/
}



void Application::MakeObjFile(Mat _depthImg, string _filePath)
{
	//_depthImg = imread("4.tif", CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
	ofstream fout;
	fout.open(_filePath);
	float* src = (float*)_depthImg.data;
	double temp;

	//int halfW = _depthImg.cols / 2;
	//int halfH = _depthImg.rows / 2;

	int halfW = 0;
	int halfH = 0;

	for (int h = 0; h < _depthImg.rows; h++)
	{
		for (int w = 0; w < _depthImg.cols; w++)
		{
			temp = *(src++);
			temp = (temp > 4500) ? 0 : temp;
			if (!temp) continue;
			//fout << "v " << (w - halfW) << " " << (h - halfH) << " " << -(temp) << " " <<
			//	temp / 4500. << " " << temp / 4500. << " " << temp / 4500. << endl;
			fout << "v " << w << " " << h << " " << temp << " " <<endl;
		}
		printf("%d / %d\n", h, _depthImg.rows);
	}
	fout.close();
}

void Application::MakeObjFile(Point3f** realPosArr, int width, int height, string _filePath)
{
	//_depthImg = imread("4.tif", CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
	ofstream fout;
	fout.open(_filePath);
	double temp;
	for (int h = 0; h < height; h++)
	{
		for (int w = 0; w < width; w++)
		{
			if (realPosArr[h][w].z == 0)
				continue;
			if (realPosArr[h][w].z > 4500)
				continue;
			/*
					fout << "v " << realPosArr[h][w].x << " " << realPosArr[h][w].y << " "
						<< realPosArr[h][w].z << " " << realPosArr[h][w].z / 4500. << " "
						<< realPosArr[h][w].z / 4500. << " " << realPosArr[h][w].z / 4500. << endl;
			*/
			fout << "v " << realPosArr[h][w].x << " " << realPosArr[h][w].y << " "
				<< realPosArr[h][w].z << endl;
		}
		printf("%d / %d\n", h, height);
	}
	fout.close();
}

void Application::MakeObjFile(Point2f** realPosArr, int width, int height, string _filePath)
{
	ofstream fout;
	fout.open(_filePath);
	double temp;
	for (int h = 0; h < height; h++)
	{
		for (int w = 0; w < width; w++)
		{
			fout << "v " << realPosArr[h][w].x << " " << realPosArr[h][w].y << " " << 1000 << " "
				<<255 <<" "<< 0 <<" "<< 0 << endl;
		}
		printf("%d / %d\n", h, height);
	}
	fout.close();
}

void Application::MakeObjFile(bool** realPosArr, int width, int height, string _filePath)
{	
	ofstream fout;
	fout.open(_filePath);
	double temp;
	for (int h = 0; h < height; h++)
	{
		for (int w = 0; w < width; w++)
		{
			if (realPosArr[h][w])
			{
				fout << "v " << w << " " << h << " " << 1000 << " " << endl;
			}
		}
		printf("%d / %d\n", h, height);
	}
	fout.close();
}


void Application::MakeObjFile(MappingPos** realPosArr, int width, int height, string _filePath)
{	
	ofstream fout;
	fout.open(_filePath);
	double temp;
	for (int h = 0; h < height; h++)
	{
		for (int w = 0; w < width; w++)
		{
			for (int c = 0; c < realPosArr[h][w].count; c++)
			{
				fout << "v "
					<< realPosArr[h][w].relatedPos[c].x << " "
					<< realPosArr[h][w].relatedPos[c].y << " "
					<< realPosArr[h][w].relatedPos[c].dist * 100
					<< endl;
			}
			
		}
		printf("%d / %d\n", h, height);
	}
	fout.close();
}



//Crop Image 만들기
void Application::CropDepthImage(Mat inImg, Mat &cropImg)
{
	int inHeight = inImg.rows;
	int inWidth = inImg.cols;

	//크랍할 X,Y의 픽셀 지정
	int outWidth = inWidth * (PROJECT_WIDTH_RATE / BASE_WIDTH_RATE);
	int outHeight = outWidth * (9. / 16.);


	Mat tempImg = Mat::zeros(outHeight, outWidth, CV_32FC1);

	for (int y = 0; y < outHeight; y++)
		for (int x = 0; x < outWidth; x++)		
		{
			tempImg.at<float>(y, x) = inImg.at<ushort>(inHeight / 2 - outHeight / 2 + y, inWidth / 2 - outWidth / 2 + x);
		}

	tempImg.copyTo(cropImg);
	
	//크랍한 이미지 평면의 3차원 obj
	MakeObjFile(tempImg, "CropTempImg.obj");


	//이미지 Resize
	resize(tempImg, cropImg, cvSize(PIXEL_OF_SCREEN_X, PIXEL_OF_SCREEN_Y), INTER_LINEAR);
}












void Application::OutFileOfPoints()
{

	ofstream fout1, fout2;

	fout1.open("gray.txt");
	fout2.open("red.txt");

	for (int h = 0; h < 1081; h++)
	{
		for (int w = 0; w < 1921; w++)
		{
			fout1 << m_mappingControl.m_virtualScreenRotated[h][w].x << " " << m_mappingControl.m_virtualScreenRotated[h][w].y << endl;
			fout2 << m_mappingControl.tempCoordinate[h][w].x << " " << m_mappingControl.tempCoordinate[h][w].y << endl;
		}
	}

	fout1.close();
	fout2.close();

}

void Application::GetMedianDepthFrame(Mat& dstImg, int length, int maskSize)
{
	Mat depthFrame;
	int ***accDepthFrame;
	int width, height;
	int dump = maskSize / 2;

	m_kinect.UpdateKinectDepthFrame();
	depthFrame = m_kinect.GetKinectOriginalDepthFrame();

	accDepthFrame = new int **[30];

	height = depthFrame.rows;
	width = depthFrame.cols;

	// alloc
	for (int i = 0; i < length; i++)
	{
		accDepthFrame[i] = new int *[height];

		for (int j = 0; j < depthFrame.rows; j++)
		{
			accDepthFrame[i][j] = new int[width];
			memset(accDepthFrame[i][j], 0, sizeof(int)*width);
		}
	}

	// accumulate
	int index = 0;
	DWORD start;
	DWORD end;
	vector<ushort> tempVector;
	Mat resultImg = Mat::zeros(height, width, CV_16UC1);
	Mat resultImg_median = Mat::zeros(height, width, CV_16UC1);
	ushort *src, *dst;
	int h, w, len;

	// length 개의 프레임을 누적한다.
	for (int i = 0; i < length; i++)
	{
		m_kinect.UpdateKinectDepthFrame();
		depthFrame = m_kinect.GetKinectOriginalDepthFrame();

		src = (ushort*)depthFrame.data;
#pragma omp parallel for private(h, w)
		for (h = 0; h < height; h++)
		{
			for (w = 0; w < width; w++)
				accDepthFrame[i][h][w] = src[h*width + w];
		}
	}

	// median filter이기 때문에 ...
	dst = (ushort*)resultImg.data;
#pragma omp parallel for private(tempVector) private(h, w, len)
	for (h = 0; h < height; h++)
	{
		//printf("%d\n", h);
		for (w = 0; w < width; w++)
		{
			// 벡터에 누적된 픽셀 값을 넣는다.
			for (len = 0; len < length; len++)
				tempVector.push_back(accDepthFrame[len][h][w]);

			// 벡터를 정렬한다.
			sort(tempVector.begin(), tempVector.end());
			// 벡터에서 중간값을 뽑는다.
			dst[h*width + w] = tempVector[length / 2];// 65535 - (tempVector[length / 2]);
			tempVector.clear();
		}
	}


	// 영상을 mask size x mask size 크기로 medain filtering한다.
#pragma omp parallel for private(tempVector, h, w)
	for (h = dump; h < height - dump; h++)
	{
		//printf("%d\n", h);
		for (w = dump; w < width - dump; w++)
		{
			for (int i = -dump; i <= dump; i++)
				for (int j = -dump; j <= dump; j++)
					tempVector.push_back(resultImg.at<ushort>(h + i, w + j));

			sort(tempVector.begin(), tempVector.end());
			resultImg_median.at<ushort>(h, w) = tempVector[maskSize*maskSize / 2];
			tempVector.clear();
		}
	}


	// free
	for (int i = 0; i < length; i++)
	{
		for (int j = 0; j < height; j++)
			delete[] accDepthFrame[i][j];

		delete[] accDepthFrame[i];
	}

	delete[] accDepthFrame;

	resultImg_median.copyTo(dstImg);
}


void Application::MeanDepthFrame(int length)
{
	Mat depthFrame;
	int ***accDepthFrame;
	int width, height;

	m_kinect.UpdateKinectDepthFrame();
	depthFrame = m_kinect.GetKinectDepthFrame();

	accDepthFrame = new int **[30];

	height = depthFrame.rows;
	width = depthFrame.cols;

	// alloc
	for (int i = 0; i < length; i++)
	{
		accDepthFrame[i] = new int *[height];

		for (int j = 0; j < depthFrame.rows; j++)
		{
			accDepthFrame[i][j] = new int[width];
			memset(accDepthFrame[i][j], 0, sizeof(int)*width);
		}
	}

	// accumulate
	int index = 0;
	DWORD start;
	DWORD end;
	vector<ushort> tempVector;
	Mat resultImg = Mat::zeros(height, width, CV_16UC1);
	Mat resultImg2 = Mat::zeros(height, width, CV_8UC1);
	ushort *src, *dst;
	uchar *dst2;
	int h, w;
	while (true)
	{
		printf("%d\n", index);
		m_kinect.UpdateKinectDepthFrame();
		depthFrame = m_kinect.GetKinectOriginalDepthFrame();

		src = (ushort*)depthFrame.data;
#pragma omp parallel for private(h, w)
		for (h = 0; h < height; h++)
		{
			for (w = 0; w < width; w++)
				accDepthFrame[index % 30][h][w] = src[h*width + w];
		}
		index++;

		if (index >= 30)
		{
			start = GetTickCount();
			int temp;

			dst = (ushort*)resultImg.data;
			dst2 = (uchar*)resultImg2.data;
#pragma omp parallel for private(temp) private(h, w)
			for (h = 0; h < height; h++)
			{
				for (w = 0; w < width; w++)
				{
					temp = 0;
					for (int len = 0; len < length; len++)
						temp += accDepthFrame[len][h][w];

					dst[h*width + w] = (65535 - (temp / length));
					dst2[h*width + w] = (temp / length) % 255;
					if (m_depthTh > dst[h*width + w] || dst[h*width + w] == 65535)
						dst[h*width + w] = 0;
				}
			}

			end = GetTickCount();

			//printf("\n실행시간 : %lf \n", (end - start) / (double)1000);

			imshow("accDepth", resultImg);
			imshow("oriDepth", depthFrame);
			imshow("accDepth%256", resultImg2);
			char a = waitKey(1);
			
			if (a == 'w')
				m_depthTh = (m_depthTh + 1000 > 65535) ? 65535 : m_depthTh + 1000;
			else if (a == 's')
				m_depthTh = (m_depthTh - 1000 < 0) ? 0 : m_depthTh - 1000;
			printf("%d\n", m_depthTh);
		}
	}

	// free
	for (int i = 0; i < length; i++)
	{
		for (int j = 0; j < height; j++)
			delete[] accDepthFrame[i][j];

		delete[] accDepthFrame[i];
	}

	delete[] accDepthFrame;
}

void Application::SetUserLocation(double _x, double _y, double _z)
{
	m_userLocation.x = _x;
	m_userLocation.y = _y;
	m_userLocation.z = _z;
}