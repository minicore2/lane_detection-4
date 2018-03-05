#ifndef SCAN_WINDOW
#define SCAN_WINDOW

#include <assert.h>
#include <vector>
#include "detect.h"

using namespace cv;
using namespace std;

int scan_window_float(Mat img, Point_list& points, int win_row, int win_col, float sum_thresh, float diff_thresh, int step_row, int step_col);

#endif
