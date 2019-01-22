/*
 * 1、学会透视变换矩阵的用法与推导（4点法；其他？）
 * 2、如何输入角度，输出矩阵
 * 3、重写opencv透视变换函数，图片输出的大小根据参数实际变换
 * 4、矩阵变换：平移、缩放、透视
 * 5、键盘控制变换操作
 */ 

#include "include_files.h"


/*//偷懒了，用了opencv方法(四点法)得出矩阵，并保存
* 1研究：矩阵+几何得出关系
* 2神经网络记录数据并得出关系
   0  1
   3  2
*
* 局限性：不是绕中轴旋转；左上角不变
*        向外旋转放大会溢出屏幕
*        平移、缩放不涉及
*/
cv::Mat angleTOmat(cv::Mat src,int angle1,int angle2)
{
    float tanx0 = 0.5*src.cols/src.rows;  //绕x旋转固定轴
    float tany0 = 0.5*src.rows/src.cols;  //绕y旋转固定轴

    float anx1,any1;
    anx1 = 3.14*angle1/180;
    any1 = 3.14*angle2/180;

    vector<cv::Point2f> corners(4),corners_trans(4);
    corners[0] = cv::Point2f(0,0);
    corners[1] = cv::Point2f(src.cols-1,0);
    corners[2] = cv::Point2f(src.cols-1,src.rows-1);
    corners[3] = cv::Point2f(0,src.rows-1);

    corners_trans[0] = cv::Point2f(0,0);
    corners_trans[1].x = 1.0*src.cols*tany0/(tany0+tan(anx1)) - 1;
    corners_trans[1].y = corners_trans[1].x*tan(anx1);
    corners_trans[3].y = 1.0*src.rows*tanx0/(tanx0+tan(any1)) - 1;
    corners_trans[3].x = corners_trans[3].y * tan(any1);

    //直线交点
    float k1,b1;      //下斜率 （绕y）
    float k2,b2;      //右斜率 （绕x)
    cv::Point2f pointX2;                //只绕x旋转，angle2 = 0;
    pointX2.y = 1.0*src.rows*tanx0/(tanx0+tan(any1))-1;
    pointX2.x = src.cols - pointX2.y*tan(any1) - 1;
    k1 = 1.0*(pointX2.y - 0)/(pointX2.x - src.cols);
    b1 = corners_trans[1].y - k1*corners_trans[1].x;
    //cout<<"                              y= " << k1 <<"x+ "<<b1 <<endl;

    cv::Point2f pointY2;
    pointY2.x = 1.0*src.cols*tany0/(tany0+tan(anx1)) - 1;
    pointY2.y = src.rows - pointY2.x*tan(anx1) -1;
    k2 = 1.0*(pointY2.y - src.rows)/(pointY2.x - 0);
    b2 = corners_trans[3].y - k2*corners_trans[3].x;
    //cout<<"                              y= " << k2 <<"x + "<<b2 <<endl;

    corners_trans[2].x = 1.0*(b2 - b1)/(k1 - k2);
    if(k1 == 0)
        corners_trans[2].y = k1*corners_trans[2].x + b1;
    else if(k2 == 0)
        corners_trans[2].y = k2*corners_trans[2].x + b2;
    else
        corners_trans[2].y = k1*corners_trans[2].x + b1;

    cout << corners_trans<<endl;

    //调用opencv四点函数得出矩阵 getPerspectiveTransform(const Point2f src[], const Point2f dst[])
    cv::Mat mat33 = getPerspectiveTransform(corners,corners_trans);
    cout << mat33 <<endl;
    return  mat33;
}

float max_(float a, float b){
    return (a>b)?a:b;
}
float min_(float a, float b){
    return (a<b)?a:b;
}

int out_width = 0,out_height = 0;

cv::Mat angleTOmatbyMID(cv::Mat src,int angle1,int angle2)
{
    float anx,any;
    cout << "anx="<<angle1<<"         any="<<angle2<<endl;
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

    //求外切矩形 左x比小、右x比大；上y比小，下y比大。

    cv::Point2f change_point = cv::Point2f(0,0);

    if(corners_trans[0].y == corners_trans[1].y)   // anx = 0   上下平行
    {
        out_height = corners_trans[3].y - corners_trans[0].y;
        if(corners_trans[0].x > corners_trans[3].x){
            cout << "下大"<<endl;
            out_width = corners_trans[2].x - corners_trans[3].x;
            change_point.x = 0 - corners_trans[3].x;
            change_point.y = out_height  - corners_trans[3].y;
        }

        else{
            cout << "上大"<<endl;
            out_width = corners_trans[1].x - corners_trans[0].x;
            change_point.x = 0 - corners_trans[0].x;
            change_point.y = 0 - corners_trans[0].y;
        }
        corners_trans[0] += change_point;
        corners_trans[1] += change_point;
        corners_trans[2] +=change_point;
        corners_trans[3] += change_point;

    }
    else if(corners_trans[0].x == corners_trans[3].x)  // any = 0   左右平行
    {
        out_width = corners_trans[1].x - corners_trans[0].x;
        if(corners_trans[0].y > corners_trans[1].y){
            cout << "右大"<<endl;
            out_height = corners_trans[2].y - corners_trans[1].y;
            change_point.x = out_width - corners_trans[1].x;
            change_point.y = 0  - corners_trans[1].y;
        }

        else{
            cout << "左大"<<endl;
            out_height = corners_trans[3].y - corners_trans[0].y;
            change_point.x = 0 - corners_trans[0].x;
            change_point.y = 0 - corners_trans[0].y;
        }
        corners_trans[0] += change_point;
        corners_trans[1] += change_point;
        corners_trans[2] += change_point;
        corners_trans[3] += change_point;
    }
    else if(corners_trans[0].x < corners_trans[3].x && corners_trans[0].y < corners_trans[1].y)  //左上 0
    {
        cout << "左上"<<endl;
        out_width = corners_trans[1].x - corners_trans[0].x;
        out_height = corners_trans[3].y - corners_trans[0].y;

        change_point.x = 0 - corners_trans[0].x;
        change_point.y = 0 - corners_trans[0].y;

        corners_trans[1] += change_point;
        corners_trans[2] += change_point;
        corners_trans[3] += change_point;
        corners_trans[0] += change_point;

    }
    else if(corners_trans[1].x > corners_trans[2].x && corners_trans[1].y < corners_trans[0].y)  //右上 1
    {
        cout << "右上"<<endl;
        out_width = corners_trans[1].x - corners_trans[0].x;
        out_height = corners_trans[2].y - corners_trans[1].y;

        change_point.x = out_width  - corners_trans[1].x;
        change_point.y = 0 - corners_trans[1].y;

        corners_trans[0] += change_point;
        corners_trans[1] += change_point;
        corners_trans[2] += change_point;
        corners_trans[3] += change_point;
    }
    else if(corners_trans[2].x > corners_trans[1].x && corners_trans[2].y > corners_trans[3].y)  //右下 2
    {
        cout << "右下"<<endl;
        out_width = corners_trans[2].x - corners_trans[3].x;
        out_height = corners_trans[2].y - corners_trans[1].y;

        change_point.x = out_width   - corners_trans[2].x;
        change_point.y = out_height  - corners_trans[2].y;

        corners_trans[0] += change_point;
        corners_trans[1] += change_point;
        corners_trans[2] += change_point;
        corners_trans[3] += change_point;
    }
    else if(corners_trans[3].x < corners_trans[0].x && corners_trans[3].y > corners_trans[2].y)  //左下 3
    {
        cout << "左下"<<endl;
        out_width = corners_trans[2].x - corners_trans[3].x;
        out_height = corners_trans[3].y - corners_trans[0].y;

        change_point.x = 0  - corners_trans[3].x;
        change_point.y = out_height  - corners_trans[3].y;

        corners_trans[0] += change_point;
        corners_trans[1] += change_point;
        corners_trans[2] += change_point;
        corners_trans[3] += change_point;
    }

    cout << corners_trans <<endl;
    /*
    //顺时针为正（左大、下大）
//    if(anx >= 0 && any >= 0) // 左下大
//    {

//        corners_trans[0].x = 0.5*src.cols - dis_xx2;
//        corners_trans[0].y = (0.5*src.rows - dis_y) - dis_yy2;

//        corners_trans[1].x =0.5*src.cols + dis_xx2;
//        corners_trans[1].y = (0.5*src.rows - dis_y) + dis_yy2;

//        corners_trans[2].x = 0.5*src.cols + dis_xx1;;
//        corners_trans[2].y = src.rows - (dis_yy1 +  (0.5*src.rows - dis_y));

//        corners_trans[3].x = 0.5*src.cols - dis_xx1;
//        corners_trans[3].y = src.rows + (dis_yy1 -  (0.5*src.rows - dis_y));
//    }

//    else if(anx > 0 && any < 0) // 左上大
//    {

//        corners_trans[0].x = 0.5*src.cols - dis_xx1;
//        corners_trans[0].y = (0.5*src.rows - dis_y) - dis_yy1;

//        corners_trans[1].x =0.5*src.cols + dis_xx1;
//        corners_trans[1].y = (0.5*src.rows - dis_y) + dis_yy1;

//        corners_trans[2].x = 0.5*src.cols - dis_xx2;;
//        corners_trans[2].y = src.rows + (dis_yy2 -  (0.5*src.rows - dis_y));

//        corners_trans[3].x = 0.5*src.cols + dis_xx2;
//        corners_trans[3].y = src.rows - (dis_yy2 +  (0.5*src.rows - dis_y));

//    }
//    else if(anx < 0 && any < 0) // 右上大
//    {


//        corners_trans[0].x = 0.5*src.cols - dis_xx1;
//        corners_trans[0].y = (0.5*src.rows - dis_y) + dis_yy1;

//        corners_trans[1].x =0.5*src.cols + dis_xx1;
//        corners_trans[1].y = (0.5*src.rows - dis_y) - dis_yy1;

//        corners_trans[2].x = 0.5*src.cols + dis_xx2;;
//        corners_trans[2].y = src.rows + (dis_yy2 -  (0.5*src.rows - dis_y));

//        corners_trans[3].x = 0.5*src.cols - dis_xx2;
//        corners_trans[3].y = src.rows - (dis_yy2 +  (0.5*src.rows - dis_y));

//    }
//    else                     //右下大
//    {
//        dis_x = 0.5*src.rows*cos(any)*sin(any);
//        dis_y = 0.5*src.rows*cos(any)*cos(any);
//        dis_xx1 = 0.5*(src.cols + 2*dis_x)*cos(anx)*cos(anx);
//        dis_yy1 = 0.5*(src.cols + 2*dis_x)*cos(anx)*sin(anx);
//        dis_xx2 = 0.5*(src.cols - 2*dis_x)*cos(anx)*sin(anx);
//        dis_yy1 = 0.5*(src.cols - 2*dis_x)*cos(anx)*sin(anx);

//        corners_trans[0].x = 0.5*src.cols + dis_xx1;
//        corners_trans[0].y = (0.5*src.rows - dis_y) + dis_yy1;

//        corners_trans[1].x =0.5*src.cols - dis_xx1;
//        corners_trans[1].y = (0.5*src.rows - dis_y) - dis_yy1;

//        corners_trans[2].x = 0.5*src.cols + dis_xx2;
//        corners_trans[2].y = src.rows + (dis_yy2 -  (0.5*src.rows - dis_y));

//        corners_trans[3].x = 0.5*src.cols - dis_xx2;
//        corners_trans[3].y = src.rows - (dis_yy2 +  (0.5*src.rows - dis_y));

//    }
*/
    //这里增加旋转角度（中心点旋转：外切矩形中心）在这里算旋转和完成矩阵后算差别不大；思考：和透视一起算旋转后四点

    //调用opencv四点函数得出矩阵 getPerspectiveTransform(const Point2f src[], const Point2f dst[])
    cv::Mat mat33 = getPerspectiveTransform(corners,corners_trans);
    cout << mat33 <<endl;
    return  mat33;
}

cv::Mat inv_mat_formal(cv::Mat mat)
{
    cv::Mat mat_t = mat.t();
    cv::Mat mat_inv = mat_t.inv();
    return mat_inv;
}

cv::Mat Formal_PerspectiveTrans(cv::Mat src,int an_x,int an_y)
{
    cv::Mat warp_mat( 3, 3, CV_32FC1 );
    cv::Mat warp_dst;
    warp_mat = angleTOmatbyMID(src,an_x,an_y);
    warp_mat = inv_mat_formal(warp_mat);                  //逆矩阵

    return warp_dst;
}

/*      0   0   0
 *      0   0   0
 *      0   0   0
 */
void warp33_init(cv::Mat  m)
{
    for(int i = 0;i<3;i++)
        for(int j = 0;j <3;j++){
            m.at<float>(j,i) = 0;
        }
    m.at<float>(0,0) = 1;
    m.at<float>(1,1) = 1;
    m.at<float>(2,2) = 1;
}


//透视变换，opencv输入矩阵为3*3
cv::Mat  warp_PerspectiveTrans(cv::Mat src,int an_x,int an_y)
{
    double  anglex,angley;

//    anglex = 3.14*an_x/180/20;
//    angley = 3.14*an_y/180/20;
    cv::Mat warp_mat( 3, 3, CV_32FC1 );
    cv::Mat warp_dst;

    anglex = an_x/20;
    angley = an_y/20;
    //warp_mat = angleTOmat(src,anglex,angley);

    warp_mat = angleTOmatbyMID(src,anglex,angley);
//    vector<cv::Point2f> ponits, points_trans;
//        for(int i=0;i<src.rows;i++){
//            for(int j=0;j<src.cols;j++){
//                ponits.push_back(cv::Point2f(j,i));
//            }
//        }
//
//    perspectiveTransform( ponits, points_trans, warp_mat);


    warpPerspective(src,warp_dst,warp_mat,cv::Size(out_width,out_height));
    return warp_dst;
//CvMat* cvGetPerspectiveTransform(const CvPoint2D32f* src, const CvPoint2D32f* dst, CvMat* map_matrix)
}

/*      1   0(x)  0
 *   (y)0   1     0
 *
 */
void warp23_init(cv::Mat  m)
{
    m.at<float>(0,0) = 1;                 //(y,x)
    m.at<float>(1,0) = 0;            // x
    m.at<float>(0,1) = 0;
    m.at<float>(1,1) = 1;
    m.at<float>(0,2) = 0;
    m.at<float>(1,2) = 0;
    m.at<float>(2,0) = 0;
}

//用投影方法 3d旋转转为2d  仿射变换
 /*
 *
 *                  ^ y
 *                  |
 *                  |
 *                  |
 *                  --------> x
 *
 */
cv::Mat  warp_change(cv::Mat src,int an_x,int an_y)
{
    double  anglex,angley;
    anglex = 3.14*an_x/180;
    angley = 3.14*an_y/180;
    cv::Mat warp_mat( 2, 3, CV_32FC1 );
    cv::Mat warp_dst;

    warp23_init(warp_mat);

        //有毒！！！长的一边总会超出范围
//    float resize_h = src.rows/(tan(anglex)*src.cols+ src.rows);
//    float resize_w = 1 - tan(angley)*(src.rows/src.cols);

    float resize_h = src.rows/(tan(anglex)*src.cols+ src.rows);
    float resize_w = src.cols/(tan(angley)*src.rows+ src.cols);
    //int Translation_x  = 0.5*(1-resize_x)*src.cols;
    //int Translation_y  = 0.5*(1-resize_y)*src.rows;
    std::cout << resize_h << "   w:"<<resize_w<<std::endl;

    warp_mat.at<float>(1,0) = sin(anglex);     //绕y旋转.对应高缩小
    warp_mat.at<float>(0,1) = sin(angley);     //绕x旋转.对应宽缩小
    warp_mat.at<float>(0,0) = resize_w;    //宽缩小(原点左上角) 这里是比例
    warp_mat.at<float>(1,1) = resize_h;     //高缩小

    //warp_mat.at<float>(0,2) = Translation_x;     //+向右平移  这里是真实坐标(左上角从0,0开始)
    //warp_mat.at<float>(1,2) = Translation_y;     //+向下平移




    warpAffine(src, warp_dst, warp_mat, warp_dst.size());

    return warp_dst;
}

cv::Mat rotate2D_change(cv::Mat image,cv::Point center,double angle,float scale)
{
//    cv::Mat rot_mat( 2, 3, CV_32FC1 );
//    cv::Mat result1;
//    rot_mat = cv::getRotationMatrix2D( center, angle, scale );
//    warpAffine( image, result1, rot_mat, image.size() );
//    cv::imshow("result1",result1);

    cv::Mat rot( 2, 3, CV_32FC1 );
    cv::Mat result;
    rot = cv::getRotationMatrix2D(center, angle, 1);                     //求旋转矩阵
    cv::Rect bbox = cv::RotatedRect(center, image.size(), angle).boundingRect();  //求新的外切矩形
    rot.at<double>(0, 2) += bbox.width / 2.0 - center.x;
    rot.at<double>(1, 2) += bbox.height / 2.0 - center.y;
    cv::warpAffine(image, result,rot,bbox.size(),1,0,cv::Scalar(0,0,0));  //原图像旋转
    return result;

}
