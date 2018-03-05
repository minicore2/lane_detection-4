#include "cluster.h"
#include "detect.h"

/*use kmeans to get the white line luminance*/
/* sometimes perform not very well so unused temporarily*/
Mat cluster(Mat& img){
    //assert(img.depth() == CV_8UC3);
    //prepare data
    Mat samples(img.rows*img.cols,3,CV_32FC1);
    //img.convertTo(img,CV_32FC3);
    for (int ii = 0; ii < img.rows; ii++){
        for (int ij = 0; ij < img.cols; ij++){
            for (int jj = 0; jj < 3; jj++){
                samples.at<float>(ii*img.cols+ij, jj) = img.at<Vec3b>(ii,ij)[jj];
            }
        }
    }
    Mat centers;
    int n_clusters = 2;
    Mat clusters = Mat::zeros(samples.rows, 1, CV_32SC1);
    kmeans(samples, n_clusters, clusters, cvTermCriteria(CV_TERMCRIT_EPS|CV_TERMCRIT_ITER, 10, 1.0), 1, KMEANS_PP_CENTERS, centers);
    Mat result(img.rows, img.cols, CV_8UC1, Scalar(0));
    for(int jk = 0; jk<clusters.rows; jk++)
        result.at<unsigned char>(jk/img.cols, jk%img.cols) = clusters.at<int>(jk, 0)*255;
#if DEBUG_MODE
    //for (int kk =0; kk < img.rows; kk++)
    //    cout<< img.at<float>(kk, 0) <<endl;
    imshow("cluster result", result);
#endif
    return centers;
}
    
