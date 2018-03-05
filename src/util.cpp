#include "util.h"
#include "hough.h"

#define DEBUG_MODE 0

/* draw line in image                   */
/* first reverse/none-reverse           */
/* then add offset                      */
int draw_lines(Mat& img, Line_list& lines, bool reverse, Point offset, int draw_height){
    for (Line_list::iterator it = lines.begin(); it != lines.end(); it++){
        int tmp_x = it->rho*cos(it->theta);
        int tmp_y = it->rho*sin(it->theta);
        float k_ = tan(it->theta);
        float k;
        if (k_ != 0)
            k = -1.0/tan(it->theta);
        else
            k = 10000.0;
        //cout<<it->theta/PI*180<<" "<<it->rho<<endl;
        //calc draw points
        Point plot_p1, plot_p2;
        if (reverse){
            plot_p1.y = offset.y;
            plot_p2.y = offset.y + draw_height - 1;
        }
        else{
            plot_p1.y = offset.y + draw_height - 1;
            plot_p2.y = offset.y;
        }
        plot_p1.x = (draw_height-1-tmp_y)/k + tmp_x + offset.x;
        plot_p2.x = -tmp_y/k + tmp_x + offset.x;
        //draw lines
        line(img, plot_p1, plot_p2, Scalar(255,0,0), 1, 4); 
    }
}

bool line_filter(Mat& color_img, Line& line, Mat cluster_centers, bool reverse, Point offset, int detect_height, int detect_range, float detect_thresh_x, float detect_thresh_y, int color_distance_thresh){
    assert (cluster_centers.rows == 2);
    // get line color and choose the line and background color
    // cluster 1
    float cluster_b_1 = cluster_centers.at<float>(0, 0); 
    float cluster_g_1 = cluster_centers.at<float>(0, 1); 
    float cluster_r_1 = cluster_centers.at<float>(0, 2); 
    // cluster 2
    float cluster_b_2 = cluster_centers.at<float>(1, 0); 
    float cluster_g_2 = cluster_centers.at<float>(1, 1); 
    float cluster_r_2 = cluster_centers.at<float>(1, 2); 
    //get grayscale value
    int grayscale_1 = 0.299*cluster_r_1 + 0.578*cluster_g_1 + 0.114*cluster_b_1;
    int grayscale_2 = 0.299*cluster_r_2 + 0.578*cluster_g_2 + 0.114*cluster_b_2;
    // higher grayscale one as line, the other as background
    int line_label = grayscale_1 > grayscale_2? 1: 2;
#if DEBUG_MODE
    cout<<"line label: "<<line_label<<endl;
#endif
    
    // reset color distance threshold
    color_distance_thresh = pow(color_distance_thresh, 2.0);

    // reset line
    //y = k*x + b -> x=(y-b/)k
    Point p1, p2;
    if (reverse){
        p1.x = -line.b/line.k + offset.x;
        p1.y = offset.y + detect_height-1;
        p2.x = (detect_height - line.b)/line.k + offset.x;
        p2.y = offset.y;
    }
    else{
        p1.x = (detect_height - line.b)/line.k + offset.x;
        p1.y = offset.y + detect_height-1;
        p2.x = -line.b/line.k + offset.x;
        p2.y = offset.y;
    }
    Line tmp_line = get_line_from_points(p1, p2);
    if (tmp_line.k == 0)
        return false;
    else{
        //count line labeling pix along the line
        int count = 0;
        for(int y_=color_img.rows-1; y_>=color_img.rows-detect_height; y_--){
            int x = round((y_ - tmp_line.b)/tmp_line.k);
            int detect_start = (x - detect_range) >= 0? x - detect_range: 0;
            int detect_end = (x + detect_range) < color_img.cols? x + detect_range + 1: color_img.cols;
            int tmp_count = 0;
            for (int x_ = detect_start; x_ < detect_end; x_++){
                //color image
                int b = color_img.at<Vec3b>(y_, x_)[0];
                int g = color_img.at<Vec3b>(y_, x_)[1];
                int r = color_img.at<Vec3b>(y_, x_)[2];

                double dist_sq_1 = pow(cluster_b_1-b, 2.0) + pow(cluster_g_1-g, 2.0) + pow(cluster_r_1-r, 2.0);
                double dist_sq_2 = pow(cluster_b_2-b, 2.0) + pow(cluster_g_2-g, 2.0) + pow(cluster_r_2-r, 2.0);
                //get pix label
                int label;
                if ((dist_sq_1 < dist_sq_2) && (dist_sq_1 < color_distance_thresh))
                    label = 1;
                else if ((dist_sq_1 >= dist_sq_2) && (dist_sq_2 < color_distance_thresh))
                    label = 2;
                else
                    label = -1; // too much distance so not counted
                // judge
                if (label == line_label)
                    tmp_count++;
            }
            // x direction threshold
            if (tmp_count > detect_thresh_x*(detect_end-detect_start))
                count++;
        }
        cout<<"y_count_rate: " <<float(count)/detect_height<< endl;
        if (count > detect_thresh_y*detect_height)
            return true;
        else
            return false;
    }
}
