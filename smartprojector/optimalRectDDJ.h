#ifndef OPTIMAL_RECT_DDJ_H_
#define OPTIMAL_RECT_DDJ_H_

//#include <assert.h>
//#include <stdio.h>
//#include <stdlib.h>

#include <cv.h>

typedef struct {
	int one;
	int two;
} Pair;

class optimalRectDDJ {

public:
	optimalRectDDJ(){

		M = 0;
		N = 0;
		best_ll.one = 0;
		best_ll.two = 0;

		best_ur.one = 0;
		best_ur.two = 0;

		best_area = 0;

		c = NULL;
		s = NULL;
		top = 0;
	}
	~optimalRectDDJ(){};

	void push(int a, int b) {
		s[top].one = a;
		s[top].two = b;
		++top;
	}

	void pop(int *a, int *b) {
		--top;
		*a = s[top].one;
		*b = s[top].two;
	}

	// �̹��� ������ ���� �о� ���� 
	void update_cache(cv::Mat mask, int rowIndex);


	// �Է����� ���̳ʸ� �̹����� ���� �Ѵ�. 
	// 0�� �ƴѰ��� ä���� �������� �����Ѵ�. 
	cv::Rect findOptimalRect(cv::Mat input);

protected:

	int M;
	int N;

	Pair best_ll; //= { 0, 0 };
	Pair best_ur; //= { -1, -1 };
	int best_area;// = 0;


	int *c; /* Cache */
	Pair *s; /* Stack */
	int top; /* Top of stack */
};


#endif