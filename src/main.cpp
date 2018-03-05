#include <fstream>
#include <math.h>
#include <pthread.h>
#include <sys/time.h>
#include "hough.h"
#include "tracker.h"
#include "detect.h"
#include "scan_window.h"
#include "util.h"
#include "cluster.h"

#define DEBUG_MODE 0
#define RESIZED_H 720
#define RESIZED_V 405
#define BGR_CLUSTER 0

VideoCapture cap;
//pthread_t thread_1, thread_2, thread_3;
int hough_thresh = 12; //hough point number threshold

Line_list detect_lines(Mat& color_img, Rect detect_area){
    clock_t t1 = clock();
    Mat img, dst;
    resize(color_img, color_img, Size(RESIZED_H, RESIZED_V)); 
    cvtColor(color_img, img, COLOR_BGR2GRAY);
    Mat crop(img, detect_area);
    Sobel(crop, dst, CV_8U, 1, 1, 3);
    GaussianBlur(dst, dst, Size(3,3),0);
    int thresh_min = 20;
    int thresh_max = 100;
    for(int ii=0; ii<crop.rows; ii++){
        for(int ij=0; ij<crop.cols; ij++){
            unsigned char tmp_pix = dst.at<unsigned char>(ii,ij);
            if (tmp_pix < thresh_max && tmp_pix >=thresh_min)
                dst.at<unsigned char>(ii, ij) = 1;
            else
                dst.at<unsigned char>(ii, ij) = 0;
        }
    }
    Point_list hough_points;
    int ret = scan_window_float(dst, hough_points, 15, 30, 0.05, 0.15, 2, 4);
    Line_list lines;
    float rho_max = sqrt(pow(dst.rows, 2.0) + pow(dst.cols, 2.0));
    // thresh number of points for a hough line
    // eg. hough_thresh = 10 means the line must have at least 10 belonging points
    ret = hough_transform(hough_points, lines, PI/72.0, 5.0, 0.3, rho_max, hough_thresh);
    //merge lines with similar theta and rho
    Line_list merged_lines;
    merge_lines(lines, merged_lines, 20.0, PI/24);
#if DEBUG_MODE
    printf ("get %ld lines\n", lines.size());
    for(int kk=0; kk<lines.size(); kk++)
        cout<<lines[kk].theta*180/PI<<" "<<lines[kk].rho<<endl;
    printf ("get %ld lines after merging\n", merged_lines.size());
    for(int jk=0; jk<merged_lines.size(); jk++)
        cout<<merged_lines[jk].theta*180/PI<<" "<<merged_lines[jk].rho<<endl;
#endif /* END DEBUG_MODE*/
    Line_list _lines;
    //merge_lines(relocated_lines, _lines, 10.0, PI/36);
	Line_list recog_lines;
	filter_and_recog_line(lines, hough_points, recog_lines, 10.0, 7, 40);
    merge_lines(recog_lines, _lines, 20.0, PI/24);
    // relocate lines
    Line_list relocated_lines;
    relocate_line(_lines, hough_points, relocated_lines, 20.0, 5);


#if DEBUG_MODE
    printf ("after relocation\n");
    for(int km=0; km<_lines.size(); km++)
        cout<<_lines[km].theta*180/PI<<" "<<_lines[km].rho<<endl;
#endif /* END DEBUG_MODE*/

#if BGR_CLUSTER
    //cluster BGR image
    Mat color_crop(color_img, Rect(0, 324, RESIZED_H, 80));
    resize(color_crop, color_crop, Size(90, 10));
    Mat centers = cluster(color_crop);
    //  lines filter
    Point offset(img.cols/2, img.rows-dst.rows);
    for(int mm=0; mm < _lines.size(); mm++){
        bool flag = line_filter(color_img, _lines[mm], centers, true, offset, dst.rows, 5, 0.3, 0.5, 100);
        if (!flag)
            _lines.erase(_lines.begin() + mm);
    }
    //draw_lines(color_img, _lines, true, offset, dst.rows);
#endif /*END BGR_CLUSTER*/

#if DEBUG_MODE
    Mat dot(dst.rows, dst.cols, CV_8UC3, Scalar(0,0,0));
    for (int ii=0; ii<dst.rows; ii++){
        for (int ij=0; ij<dst.cols; ij++){
            if (dst.at<unsigned char>(ii, ij) == 1)
                dot.at<Vec3b>(ii, ij)[2] = 255;
        }
    }

    ofstream point_file("points.py");
    point_file<<"points=[";

    for (Point_list::iterator it = hough_points.begin(); it != hough_points.end(); it++){
        float x= it->x;
        float y= it->y;
        point_file<<x<<", "<<y<<", ";
        circle(dot, Point(it->x+dst.cols/2,dst.rows-it->y-1), 1, cv::Scalar(0,200,0), -1, CV_AA);
    }
    point_file<<"]";
#endif /* END DEBUG_MODE*/

#if DEBUG_MODE
    imshow("detect image", color_img);
    imshow("sobel result", dst*255);
    imshow("dot", dot);
	imwrite("./sobel.jpg", dot);
#endif /* END DEBUG_MODE*/
    cout<<"time: "<<double(clock()-t1)/CLOCKS_PER_SEC<<endl;
    return merged_lines;
}

int main(int argc, char* argv[]){
    //read
    Mat color_img;
    if (argc <= 2){
        if (argc == 1)
            color_img = imread("../road.png");
        else if (argc == 2)
            color_img = imread(argv[1]);
        //cluster BGR image
        Rect detect_area(0, 244, RESIZED_H, 160);
        Line_list _lines = detect_lines(color_img, detect_area);
        
        //draw result
        Point offset(detect_area.width/2, detect_area.y);
        draw_lines(color_img, _lines, true, offset, detect_area.height);
        imshow("result", color_img);
		//imwrite("result.jpg",color_img);
        waitKey(0);
        return 0;
    }
    else{
        if (0 == strcmp(argv[1], "video")){
            cap.open(argv[2]);
            timespec req = {0, 30*1000000};
            if (!cap.isOpened())
                return -1;
            while (1){
                Mat frame;
                cap >> frame;
                Rect detect_area(0, 122, RESIZED_H, 160);
                resize(frame, frame, Size(RESIZED_H, RESIZED_V));
                //Line_list _lines = detect_lines(frame, detect_area);
                //Point offset(detect_area.width/2, detect_area.y);
                //draw_lines(frame, _lines, true, offset, detect_area.height);
                imshow("result", frame);
                int key = waitKey(1);
                if (key  == 27)
                    break;
            }
        }
        else
            return -1;
    }
}

