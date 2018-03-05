#include "scan_window.h"

/* apply scan window*/
int scan_window_float(Mat img, Point_list& hough_points, int win_row, int win_col, float sum_thresh, float diff_thresh, int step_row, int step_col){
    assert (win_col%2 == 0); //window have to be divided into 2 parts
    int cols = img.cols;
    int rows = img.rows;
    int nms_gap_thresh = 5;
    // vector store Points
    
    for (int ij=0; ij<rows-win_row+1; ij+=step_row){
        int nms_gap;
        float nms_min = 100000.0;
        float nms_min_index = 0.0;
		int continous_point_num = 0;
        for (int jj=0; jj<cols-win_col+1; jj+=step_col){
            Mat left(img, Rect(jj, ij, win_col/2, win_row));
            Mat right(img, Rect(jj+win_col/2, ij, win_col/2, win_row));
			/*get sum of left and right mat*/
            int left_sum = sum(left)[0];
            int right_sum = sum(right)[0];
            int left_right_sum = left_sum + right_sum;
			/*diff_ration defined here*/
            float diff_ratio = abs(left_sum - right_sum)/float(left_right_sum);
			int start_pos;
            if ((left_right_sum > sum_thresh*win_col*win_row) && (diff_ratio < diff_thresh)){
				if (continous_point_num == 0)
					start_pos = jj;
                nms_gap = 0;
				continous_point_num++;

                //nms, not used temproarily
                if (diff_ratio < nms_min){
                    nms_min_index = jj + win_col/2 + 0.5;
                    nms_min = diff_ratio;
                    }
                }
            else{
                nms_gap++;
                if ((nms_gap > nms_gap_thresh && continous_point_num > 0) || (jj+win_col == cols)){
                    nms_min = 100000.0;
                    if (nms_min_index > 0){
						nms_min_index = start_pos + continous_point_num/2 + win_col/2; //newly added, algorithm changed
                        hough_points.push_back(Point(nms_min_index-cols/2, rows - ij - win_row/2 + 1)); //coordinate transfer(zero point is in the middle of the lowest line)
					}
                    nms_min_index = -1.0;
					continous_point_num = 0;
                    }
                }
            }
        }
        cout<<"get "<<hough_points.size()<<" points"<<endl;
        return API_OK;
    }

