#ifndef HOUGH
#define HOUGH

#include "detect.h"
#include <math.h>

int hough_transform(Point_list& list, Line_list& lines, float theta_mesh, float rho_mesh, float k_thresh, float rho_max, int hough_thresh);
Line get_line_from_points(Point p1, Point p2);

#endif
