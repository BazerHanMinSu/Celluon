#pragma once

#include <opencv2\opencv.hpp>

using namespace cv;


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

//스크린 사이즈 지정
#define PIXEL_OF_SCREEN_X 641
#define PIXEL_OF_SCREEN_Y 361

#define PIXEL_OF_DEPTH_X 512
#define PIXEL_OF_DEPTH_Y 424

//키넥트 FOV
#define BASE_FOV_X 70.6
#define BASE_FOV_Y 60

//키넥트 FOV절반의 Tan값
#define BASE_WIDTH_RATE 0.71
#define BASE_HEIGHT_RATE 0.5773

//프로젝트 FOV
//#define PROJECT_FOV_X 21.8
//#define PROJECT_FOV_Y 12.26

//임시 테스트용
#define PROJECT_FOV_X 43.6
#define PROJECT_FOV_Y 24.52

//프로젝트 FoV절반의 Tan값
#define PROJECT_WIDTH_RATE 0.4
#define PROJECT_HEIGHT_RATE 0.225




//유저 위치 지정
#define USER_X 1000 // 1000은 1m
#define USER_Y 0
#define USER_Z 0

#define BASE_DEPTH 1000

#define PI 3.141592

//매핑될 점과 거리저장
struct distPos
{
	int x;
	int y;
	double dist;
};

//매핑될 포인트의 연결될 점
struct MappingPos{
	distPos relatedPos[16];
	int count = 0;
};
