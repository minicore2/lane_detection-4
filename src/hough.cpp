#include "hough.h"

/* apply hough transformation*/
int hough_transform(Point_list& list, Line_list& lines, float theta_mesh, float rho_mesh, float k_thresh, float rho_max, int hough_thresh){
    int theta_mesh_num = 2*PI/theta_mesh+1;
    int rho_mesh_num = rho_max/rho_mesh+1;
    Mat result_mat(theta_mesh_num, rho_mesh_num, CV_16SC1, Scalar(0));
    for (int ii = 0; ii < list.size(); ii++){
        for (int ij = ii+1; ij < list.size(); ij++){
            Point p1 = list[ii];
            Point p2 = list[ij];
            Line line = get_line_from_points(p1, p2);
            if (line.rho > rho_max)
                line.rho = rho_max;
            // line k
            float k, k_;
            k_ = tan(line.theta);
            if (k_ != 0)
                k = 1/k_;
            else
                k = 10000.0;
            if (abs(k) >= k_thresh){
                int x = line.theta/theta_mesh;
                int y = line.rho/rho_mesh;
                if (int(result_mat.at<int>(x, y)/2) < hough_thresh*(hough_thresh-1)/2){
                    result_mat.at<int>(x, y) += 2;
                    if (x > 1)
                        result_mat.at<int>(x-1, y) += 1;
                    if (y > 1)
                        result_mat.at<int>(x, y-1) += 1;
                    if (x < result_mat.rows - 1)
                        result_mat.at<int>(x+1, y) += 1;
                    if (y < result_mat.cols - 1)
                        result_mat.at<int>(x, y+1) += 1;
                }

                else if (int(result_mat.at<int>(x, y)/2) == hough_thresh*(hough_thresh-1)/2){
                    Line hough_line;
                    hough_line.theta = x*theta_mesh;
                    hough_line.rho   = y*rho_mesh;
                    k_ = tan(line.theta);
                    if (k_ != 0)
                        hough_line.k = 1/k_;
                    else
                        hough_line.k = 10000.0;
                    if (hough_line.theta == 0)
                        hough_line.b = -hough_line.k*hough_line.rho;
                    else if (hough_line.theta == PI)
                        hough_line.b = hough_line.k*hough_line.rho;
                    else 
                        hough_line.b = hough_line.rho/sin(hough_line.theta);
                    lines.push_back(hough_line);
                    result_mat.at<int>(x, y) += 2;
                }
            }
        }
    }
    return API_OK;
}

/* calculate line from 2 given points*/
Line get_line_from_points(Point p1, Point p2){
    float theta, rho;
    Line line;
    if (p1.x == p2.x){
        if (p1.x >= 0){
            line.theta = 0;
            line.rho = abs(p1.x);
            line.k = 100000.0;
            line.b = p1.x;
            return line;
        }
        else{
            line.theta = PI;
            line.rho = abs(p1.x);
            line.k = 100000.0;
            line.b = p1.x;
            return line;
        }
    }
    //get k
    float k = float(p1.y-p2.y)/(p1.x-p2.x);
    float b = p1.y - p1.x*k;
    float a = -b/k;
    line.rho = abs(b)/sqrt(1+k*k);
    line.b = b;
    line.k = k;
    if (a <= 0)
        line.theta = atan(-1.0/k) + PI;
    else if (a > 0 && b > 0)
        line.theta = atan(-1.0/k);
    else
        line.theta = atan(-1.0/k) + PI*2;
    return line;
}



