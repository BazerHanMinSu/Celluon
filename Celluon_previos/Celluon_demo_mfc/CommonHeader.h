#pragma once

#include <opencv2\opencv.hpp>

using namespace cv;


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

//��ũ�� ������ ����
#define PIXEL_OF_SCREEN_X 641
#define PIXEL_OF_SCREEN_Y 361

#define PIXEL_OF_DEPTH_X 512
#define PIXEL_OF_DEPTH_Y 424

//Ű��Ʈ FOV
#define BASE_FOV_X 70.6
#define BASE_FOV_Y 60

//Ű��Ʈ FOV������ Tan��
#define BASE_WIDTH_RATE 0.71
#define BASE_HEIGHT_RATE 0.5773

//������Ʈ FOV
//#define PROJECT_FOV_X 21.8
//#define PROJECT_FOV_Y 12.26

//�ӽ� �׽�Ʈ��
#define PROJECT_FOV_X 43.6
#define PROJECT_FOV_Y 24.52

//������Ʈ FoV������ Tan��
#define PROJECT_WIDTH_RATE 0.4
#define PROJECT_HEIGHT_RATE 0.225




//���� ��ġ ����
#define USER_X 1000 // 1000�� 1m
#define USER_Y 0
#define USER_Z 0

#define BASE_DEPTH 1000

#define PI 3.141592

//���ε� ���� �Ÿ�����
struct distPos
{
	int x;
	int y;
	double dist;
};

//���ε� ����Ʈ�� ����� ��
struct MappingPos{
	distPos relatedPos[16];
	int count = 0;
};
