#include "tracker.h"

/* self complete k and b given rho and theta  */
int self_complete_line(Line& line){
    float k_ = tan(line.theta);
    if (k_ != 0)
        line.k = -1/k_;
    else
        line.k = 10000.0;
    if (line.theta == 0)
        line.b = -line.k*line.rho;
    else 
        line.b = line.rho/sin(line.theta);

    return API_OK;
}

/* calculate distance from point to line*/
float get_point2line_distance(Point& point, Line& line){
    float k = line.k;
    int x = point.x;
    int y = point.y;
    float distance = abs(x*k - y +line.b)/sqrt(1+k*k);
    return distance;
}

/* merge all lines have similar rho and theta, not very good but temporarily used*/
int merge_lines(Line_list& input_lines, Line_list& output_lines, float rho_thresh, float theta_thresh){
    if (input_lines.size() == 0){
        fprintf(stderr, "no line input!\n");
		return API_NG;
	}
    else{
        //output_lines stores new line cluster
        output_lines.push_back(input_lines[0]);
        int* label = (int*)calloc(input_lines.size(), sizeof(int));
        label[0] = 1;
        int* label_count = (int*)calloc(input_lines.size(), sizeof(int));
        label_count[0] = 1;
        for (int ii = 1; ii < input_lines.size(); ii++){
            bool add_flag = true;
            int ij;
            for (ij = 0; ij < output_lines.size(); ij++){
                float theta_diff = abs(output_lines[ij].theta - input_lines[ii].theta);
                float rho_diff = abs(output_lines[ij].rho - input_lines[ii].rho);
                if ((theta_diff <= theta_thresh) && (rho_diff <= rho_thresh)){
                    //recallocate line cluster
                    output_lines[ij].theta = (output_lines[ij].theta*label_count[ij] + input_lines[ii].theta)/(label_count[ij]+1);
                    output_lines[ij].rho = (output_lines[ij].rho*label_count[ij] + input_lines[ii].rho)/(label_count[ij]+1);
                    //add label count
                    label_count[ij] += 1;
                    add_flag = false;
                    break;
                }
            }
            if (add_flag){
                output_lines.push_back(input_lines[ii]);
                label_count[ij] = 1;
            }
        }
    }
    return API_OK;
}

/* apply least square method */
/* for this you can refer to wiki page*/
Line lstsq(Point_list& points){
    int n = points.size();
    assert (n>=3);
    // calc sum of x and y
    int sigma_x = 0;
    int sigma_y = 0;
    int sigma_xy = 0;
    int sigma_xx = 0;
    for (Point_list::iterator it_p = points.begin(); it_p != points.end(); it_p++){
        sigma_x += it_p->x;
        sigma_y += it_p->y;
        sigma_xy += (it_p->x)*(it_p->y);
        sigma_xx += (it_p->x)*(it_p->x);
    }
    float k = float(n*sigma_xy-sigma_x*sigma_y)/(n*sigma_xx-pow(sigma_x,2.0));
    float b = float(sigma_xx*sigma_y-sigma_xy*sigma_x)/(n*sigma_xx-pow(sigma_x,2.0));
    //y = a*x+b
    Line line_;
    line_.rho = abs(b)/sqrt(1+k*k);
    line_.k = k;
    line_.b = b;
    float a = -b/k;
    if (a <= 0)
        line_.theta = atan(-1.0/k) + PI;
    else if (a > 0 && b > 0)
        line_.theta = atan(-1.0/k);
    else
        line_.theta = atan(-1.0/k) + PI*2;
    return line_;
}

/*relocate lines according to the surronding points*/
int relocate_line(Line_list& lines, Point_list& points, Line_list& output_lines, float distance_thresh, int line_num_thresh){
    for (Line_list::iterator it = lines.begin(); it != lines.end(); it++){
        Point_list tmp_list;
        for (Point_list::iterator it_p = points.begin(); it_p != points.end(); it_p++){
            Line line = *it;
            self_complete_line(line);
            float distance = get_point2line_distance(*it_p, line);
			/*get all points near line*/
            if (distance < distance_thresh)
                tmp_list.push_back(*it_p);
        }
		/* relocate line using least square*/
        if (tmp_list.size() > line_num_thresh)
            output_lines.push_back(lstsq(tmp_list));
    }
    return API_OK;
}

int filter_and_recog_line(Line_list& lines, Point_list& points, Line_list& output_lines, float distance_thresh, int continuous_line_gap_thresh, int continuous_line_thresh){
	/*here same as relocate_line*/
    for (Line_list::iterator it = lines.begin(); it != lines.end(); it++){
        vector<int> continuous_line_recorder; //record number of continuous points
		vector<int> tmp_line_recorder; //record last y coord in points
		int tmp_continuous_num = 0;
        Line line = *it;
        for (Point_list::iterator it_p = points.begin(); it_p != points.end(); it_p++){
            self_complete_line(line);
            float distance = get_point2line_distance(*it_p, line);
            if (distance < distance_thresh){	// points near line
				if (tmp_line_recorder.size() == 0){
					tmp_continuous_num++ ;
				}
				else if (abs(it_p->y - tmp_line_recorder.back()) <= continuous_line_gap_thresh
						&& abs(it_p->y - tmp_line_recorder.back()) > 0){
					tmp_continuous_num += abs(it_p->y - tmp_line_recorder.back());
				}
				else if (abs(it_p->y - tmp_line_recorder.back()) > continuous_line_gap_thresh){
					/* break */
					continuous_line_recorder.push_back(tmp_continuous_num); // record continuous number
					tmp_line_recorder.clear(); //clear tmp vector
					tmp_continuous_num = 0; //count from zero
				}
				tmp_line_recorder.push_back(it_p->y);
			}
        }
		if (tmp_continuous_num > continuous_line_gap_thresh)
				continuous_line_recorder.push_back(tmp_continuous_num); // record continuous number
		for (int iter=0; iter<continuous_line_recorder.size(); iter++){
			if (continuous_line_recorder[iter] > continuous_line_thresh){
				output_lines.push_back(line);
				break;
			}
		}
    }
}
