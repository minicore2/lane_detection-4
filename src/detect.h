#ifndef DETECT
#define DETECT

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/flann/flann.hpp"
#include <iostream>
#include <time.h>

#define API_OK 1
#define API_NG -1

#define PI 3.141592654

using namespace std;
using namespace cv;

//type define
typedef vector<Point> Point_list;

typedef struct{
    float k;
    float b;
    float rho;
    float theta;
} Line;


typedef struct{
    float thresh_min;
    float thresh_max;
} Thresh;

typedef vector<Line> Line_list;

typedef enum lineyype{
	FULL_LINE = 0,
	DOTTED_LINE,
} LineType;

#endif
