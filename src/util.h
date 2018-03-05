#ifndef UTIL
#define UTIL

#include "detect.h"

int draw_lines(Mat& img, Line_list& lines, bool reverse, Point offset, int draw_height);
bool line_filter(Mat& color_img, Line& line, Mat cluster_centers, bool reverse, Point offset, int detect_height, int detect_range, float detect_thresh_x, float detect_thresh_y, int color_distance_thresh);

#endif
