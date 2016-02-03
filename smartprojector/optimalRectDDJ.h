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

	// 이미지 데이터 한줄 읽어 오기 
	void update_cache(cv::Mat mask, int rowIndex);


	// 입력으로 바이너리 이미지가 들어가야 한다. 
	// 0이 아닌값을 채워진 영역으로 간주한다. 
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