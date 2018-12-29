#ifndef INCLUDE_FILES_H
#define INCLUDE_FILES_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace std;
cv::Mat angleTOmat(cv::Mat src,int angle1,int angle2);

cv::Mat  warp_change(cv::Mat src,int an_x,int an_y);
cv::Mat rotate2D_change(cv::Mat image,cv::Point center,double angle,float scale);

cv::Mat  warp_PerspectiveTrans(cv::Mat src,int an_x,int an_y);

cv::Mat SURF_test(cv::Mat image01,cv::Mat image02,int Threshold);
#endif // INCLUDE_FILES_H
