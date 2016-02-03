#include "optimalRectDDJ.h"

void optimalRectDDJ::update_cache(cv::Mat mask, int rowIndex) {
//	int m;
	char b;

	int i;
	for (i = 0; i<M; i++) {
		b = mask.at<uchar>(rowIndex, i);

		if (b == 0) {
			c[i] = 0;
		}
		else { ++c[i]; }
	}

	//fprintf(stderr, "\n");
}

cv::Rect optimalRectDDJ::findOptimalRect(cv::Mat input)
{
	int m, n;
	//scanf("%d %d", &M, &N);           // 이미지 사이즈 M : withd N : height 
	
	best_ll.one = 0;
	best_ll.two = 0;

	best_ur.one = 0;
	best_ur.two = 0;

	best_area = 0;

	c = NULL;
	s = NULL;
	top = 0;

	M = input.cols;   // M : image Width
	N = input.rows;   // N : image height

	//fprintf(stderr, "Reading %dx%d array (1 row == %d elements)\n", M, N, M);


	c = new int[M + 1]; /* Cache */
	s = new Pair[M + 1];/* stack */

	// 캐쉬 스택 초기화 
	for (m = 0; m != M + 1; ++m) { c[m] = s[m].one = s[m].two = 0; }

	/* Main algorithm: */
	for (n = 0; n != N; ++n) {      // height for

		int open_width = 0;

		update_cache(input, n); // row 한줄을 읽어 온다. 

		for (m = 0; m != M + 1; ++m) {  // row for

			if (c[m]>open_width) { /* Open new rectangle? */

				push(m, open_width);
				open_width = c[m];
			}
			else /* "else" optional here */

				if (c[m]<open_width) { /* Close rectangle(s)? */
					int m0, w0, area;
					do {
						pop(&m0, &w0);
						area = open_width*(m - m0);
						if (area>best_area) {
							best_area = area;
							best_ll.one = m0; best_ll.two = n;
							best_ur.one = m - 1; best_ur.two = n - open_width + 1;
						}
						open_width = w0;
					} while (c[m]<open_width);
					open_width = c[m];
					if (open_width != 0) {
						push(m0, w0);
					}
				}
		}
	}

	delete[]c;
	delete[]s;

	int left, right, top, bottom;
	if (best_ll.one > best_ur.one)
	{
		left = best_ur.one;
		right = best_ll.one;
	}
	else
	{
		left = best_ll.one;
		right = best_ur.one;
	}
	if (best_ll.two > best_ur.two)
	{
		top = best_ur.two;
		bottom = best_ll.two;
	}
	else
	{
		top = best_ll.two;
		bottom = best_ur.two;
	}

	cv::Rect optimalRect(left, top, right - left, bottom - top);
	return optimalRect;
}