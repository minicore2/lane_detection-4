#ifndef TRACKER
#define TRACKER

#include "detect.h"

Line lstsq(Point_list& points);
int self_complete_line(Line& line);
int merge_lines(Line_list& input_lines, Line_list& output_lines, float rho_thresh, float theta_thresh);
int relocate_line(Line_list&, Point_list&, Line_list&, float, int);
float get_point2line_distance(Point& point, Line& line);
int filter_and_recog_line(Line_list& , Point_list& , Line_list& , float, int, int);


#endif

