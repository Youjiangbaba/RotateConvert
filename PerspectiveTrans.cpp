//
// Created by jiang on 18-12-28.
//
// 不用opencv的变换函数

#include "include_files.h"
#include <math.h>

extern int out_width,out_height;
cv::Mat Rot_Perspective(cv::Mat src,int angle1,int angle2,int angle3)
{
    float anx,any,anz;
    cout << "anx="<<angle1<<"         any="<<angle2<<"         anz="<<angle3<<endl;
    anx = 3.14*angle1/180;   //x边动，y中轴不变
    any = 3.14*angle2/180;


    vector<cv::Point2f> corners(4),corners_trans(4);
    corners[0] = cv::Point2f(0,0);
    corners[1] = cv::Point2f(src.cols-1,0);
    corners[2] = cv::Point2f(src.cols-1,src.rows-1);
    corners[3] = cv::Point2f(0,src.rows-1);

    int dis_x,dis_y,dis_xx1,dis_yy1,dis_xx2,dis_yy2;
    dis_x = 0.5*src.rows*cos(any)*sin(any);
    dis_y = 0.5*src.rows*cos(any)*cos(any);
    dis_xx1 = 0.5*(src.cols + 2*dis_x)*cos(anx)*cos(anx);
    dis_yy1 = 0.5*(src.cols + 2*dis_x)*cos(anx)*sin(anx);
    dis_xx2 = 0.5*(src.cols - 2*dis_x)*cos(anx)*cos(anx);
    dis_yy2 = 0.5*(src.cols - 2*dis_x)*cos(anx)*sin(anx);

    corners_trans[0].x = 0.5*src.cols - dis_xx2;
    corners_trans[0].y = (0.5*src.rows - dis_y) - dis_yy2;

    corners_trans[1].x =0.5*src.cols + dis_xx2;
    corners_trans[1].y = (0.5*src.rows - dis_y) + dis_yy2;

    corners_trans[2].x = 0.5*src.cols + dis_xx1;
    corners_trans[2].y = src.rows - (dis_yy1 +  (0.5*src.rows - dis_y));

    corners_trans[3].x = 0.5*src.cols - dis_xx1;
    corners_trans[3].y = src.rows + (dis_yy1 -  (0.5*src.rows - dis_y));



    //这里增加旋转角度（中心点旋转：外切矩形中心） ：和透视一起算旋转后四点
    if(abs(angle3) >= 5)
    {
        anz = 3.14*angle3/180;
        cout << "z "<< anz<<endl;
        double an[4];
        float r[4];
        cv::Point2f rot_center = cv::Point2f(0.5*src.cols,0.5*src.rows);
        for(int i = 0;i <4;i++){
            //以旋转中心为原点,即center平移到现原点
            corners_trans[i] -= rot_center;
            //极坐标，这里以y顺时针旋转
            an[i] = atan2(corners_trans[i].x,corners_trans[i].y);
            r[i] = sqrt((corners_trans[i].y*corners_trans[i].y)+(corners_trans[i].x*corners_trans[i].x));
            cout << an[i] <<endl;

            //算旋转之后的角度
            an[i] += anz;
            //算旋转之后的x,y坐标
            corners_trans[i].x = r[i]*sin(an[i]);
            corners_trans[i].y = r[i]*cos(an[i]);
            //还原到原理的原点
            corners_trans[i] += rot_center;
            //cout<<"dada"<<corners_trans[i]<<endl;
        }
        cout<< "rot::"<<endl;
        cout << corners_trans <<endl;

    }

    //求正外切矩形
    cv::Point2f change_point = cv::Point2f(0,0);
    int max_x = src.cols,min_x = 0,max_y = src.rows,min_y = 0;
    int max_x_i = -1,min_x_i = -1,max_y_i = -1,min_y_i = -1;
    for(int i = 0;i<4;i++){
        if(corners_trans[i].x >= max_x){
            max_x = corners_trans[i].x;
            max_x_i = i;
        }
        else if(corners_trans[i].x <= min_x){
            min_x = corners_trans[i].x;
            min_x_i = i;
        }
        if(corners_trans[i].y >= max_y){
            max_y = corners_trans[i].y;
            max_y_i = i;
        }
        else if(corners_trans[i].y <= min_y){
            min_y = corners_trans[i].y;
            min_y_i = i;
        }
    }
    if(min_y_i >= 0) {
        change_point.y = -min_y;
    }
    if(min_x_i >= 0) {
        change_point.x = -min_x;
    }
    out_width = max_x - min_x;
    out_height = max_y - min_y;
    for(int i = 0;i < 4; i ++){
        corners_trans[i] += change_point;
    }



    cout << corners_trans <<endl;


    //调用opencv四点函数得出矩阵 getPerspectiveTransform(const Point2f src[], const Point2f dst[])
    cv::Mat mat33 = getPerspectiveTransform(corners,corners_trans);
    //cout << mat33 <<endl;
    return  mat33;
}
//透视变换，opencv输入矩阵为3*3
cv::Mat  Rot_PerspectiveTrans(cv::Mat src,int an_x,int an_y,int anz)
{
    double  anglex,angley;
    cv::Mat warp_mat( 3, 3, CV_32FC1 );
    cv::Mat warp_dst;

    anglex = an_x/20;
    angley = an_y/20;
    //warp_mat = angleTOmat(src,anglex,angley);

    warp_mat = Rot_Perspective(src,anglex,angley,anz);

    warpPerspective(src,warp_dst,warp_mat,cv::Size(out_width,out_height));
    return warp_dst;
//CvMat* cvGetPerspectiveTransform(const CvPoint2D32f* src, const CvPoint2D32f* dst, CvMat* map_matrix)
}