#include "include_files.h"
#include  "ChangeDefine.h"

#define ROTATE 1
#define SPLICE 0


int angleX,angleY;
int nums_x,nums_y;
cv::Mat src;



/*回调函数*/
void track_PerspectiveTrans(int,void*)
{
    cv::Mat src1;
    src1 = warp_PerspectiveTrans(src,nums_x,nums_y);
    cv::imshow("Perspective",src1);
}

void track_warpAffine(int,void*)
{
    cv::Mat src1;
    src1 = warp_change(src,angleX,angleY);
    cv::imshow("xy",src1);
}

void on_mouse(int EVENT, int x, int y, int flags, void* userdata)
{
    float radios[2];
    static int last[2] = {0,0};
    static float last_rad[2] = {0.0,0.0};
    if(EVENT == CV_EVENT_LBUTTONDOWN){
        radios[0] = x/572.0;
        radios[1] = y/440.0;
        cout << x <<","<<y<<"       "<<radios[0]<<";"<<radios[1]<<endl;
        cout << 1.0*last_rad[0]/radios[0] <<";" << 1.0*last_rad[1]/radios[1] << "    "<< 1.0*(y-last[1])/(x-last[0])<<endl;
        for(int i = 0 ;i < 2; i ++){
            last_rad[i] = radios[i];
            last[0] = x;
            last[1] = y;
        }
    }
}

extern cv::Mat  Rot_PerspectiveTrans(cv::Mat src,int an_x,int an_y,int anz);

int main()
{

#if ROTATE
    cv::Mat src1,src2,src_warpPerspective;
    src = cv::imread("/home/jiang/图片/haimianbaobao.png");   //   /home/jiang/图片/3d1.jpeg/home/jiang/图片/standard1.png//
    cv::Point center;

    cv::namedWindow("Perspective",cv::WINDOW_NORMAL);

    nums_x = 0;
    nums_y = 0;
    angleX = 0;
    angleY = 0;
    float angleR = 0;
    float radio = 1.0;
    cv::Mat rot( 2, 3, CV_32FC1 );       //旋转
    cv::Mat rot_src;

//    cv::setMouseCallback("Perspective", on_mouse,&src_warpPerspective);
//    //创建一个滑动条，名字为Low:255，主窗口为edge detection，最大值为255，value为thresh_low，回调函数为canny_track
//    cv::createTrackbar("x: 2000", "Perspective", &nums_x, 1800, track_PerspectiveTrans);
//    cv::createTrackbar("y: 2000", "Perspective", &nums_y, 1800, track_PerspectiveTrans);
//    cv::createTrackbar("x: 90", "xy", &angleX, 90, track_warpAffine);
//    cv::createTrackbar("y: 90", "xy", &angleY, 90, track_warpAffine);

    //cv::imshow("src",src);

    //仿射
//    cv::namedWindow("xy",cv::WINDOW_NORMAL);
//    src1 = warp_change(src,0,0);
//    cv::imshow("xy",src1);

    //透视

//    src_warpPerspective = warp_PerspectiveTrans(src,0,0);
    src_warpPerspective = Rot_PerspectiveTrans(src,0,0,0);
    cv::imshow("Perspective",src_warpPerspective);

    //旋转
    //center = cv::Point(src1.cols/2,src1.rows/2);
    //src2 = rotate2D_change(src1,center,-20,1.);
    //cv::imshow("result",src2);

    cv::Mat big_window(1280,1280,CV_8UC3);
    cv::Mat WindowRoi(src_warpPerspective.cols,src_warpPerspective.rows,CV_8UC3);
    cv::Mat clear(big_window.cols,big_window.rows,CV_8UC3,cv::Scalar(0,0,0));

    int key = 0;
    int translation_x,translation_y;
    translation_x = 0;
    translation_y = 0;
    int flag_key = 0;
    while(1){
        key = cv::waitKey(10);
        if(key > 0)                  //无按键的时候为 -1
            cout << key << endl;      // 左上右下  65361 - 65364       44 46 < >
        else
            continue;
        if(key == 27)
            break;
        switch (key) {
            big_window = clear.clone();
            case 65365:          //放大 pgup
                radio += plus_RADIO;
                if(radio >= plus_MAX){
                    cout << "max , 请缩小" << endl;
                    radio = plus_MAX;
                }break;
            case 65366:          //缩小 pgdown
                radio -= reduce_RADIO;
                if(radio <= reduce_MIN){
                    cout << "max , 请缩小" << endl;
                    radio = reduce_MIN;
                }break;

            case ',':        // <
                angleR -= rotate_ANGLE;
                if(angleR <= -rotate_MAX)
                {
                    cout << "向左旋转极限 按 > 向右" <<endl;
                    angleR = -rotate_MAX;
                }break;
            case '.':      // >
                angleR += rotate_ANGLE;
                if(angleR >= rotate_MAX)
                {
                    cout << "向右旋转极限 按 < 向左" <<endl;
                    angleR = rotate_MAX;
                }break;
            case 'w':

                translation_y -= parallel_STEP;
                if(translation_y  <= 0){
                    cout << "向上平移到顶 按 s 向下" <<endl;
                    translation_y = 0;
                }break;
            case 's':
                translation_y += parallel_STEP;
                if(translation_y + src_warpPerspective.rows > big_window.rows){
                    cout << "向下平移到底 按 w 向下" <<endl;
                    translation_y = big_window.rows - src_warpPerspective.rows -1;
                }break;
            case 'a':
                translation_x -= parallel_STEP;
                if(translation_x  < 0){
                    cout << "向左平移到边界 按 d 向右" <<endl;
                    translation_x = 0;
                }break;
            case 'd':
                translation_x += parallel_STEP;
                if(translation_x  + src_warpPerspective.cols > big_window.cols){
                    cout << "向右平移到边界 按 a 向右" <<endl;
                    translation_x =  big_window.cols - src_warpPerspective.cols;
                }break;

            case 65362:
                            flag_key++;
                            cout << flag_key <<endl;
                nums_y += warp_ANGLE;
                if(nums_y >= warp_MAX){
                    nums_y = warp_MAX;
                    cout << "y轴到极限，请按方向键 下" << endl;
                }break;
            case 65363:
                nums_x += warp_ANGLE;
                if(nums_x >= warp_MAX){
                    nums_x = warp_MAX;
                    cout << "x轴向右到极限，请按方向键 左" << endl;
                }break;
            case 65361:
                nums_x -= warp_ANGLE;
                if(nums_x <= -warp_MAX){
                    nums_x = -warp_MAX;
                    cout << "x轴向左到极限，请按方向键 右" << endl;
                }break;
            case 65364:
                nums_y -= warp_ANGLE;
                if(nums_y <= -warp_MAX){
                    nums_y = -warp_MAX;
                    cout << "y轴向下到极限，请按方向键 上" << endl;
                }break;
            }
//            //进行旋转变换
//            center = cv::Point(0.5*src.cols,0.5*src.rows);
//            rot_src = rotate2D_change(src,center,angleR,1.0);
//
//            //进行透视变换
//            src_warpPerspective = warp_PerspectiveTrans(rot_src,nums_x,nums_y);

            src_warpPerspective = Rot_PerspectiveTrans(src,nums_x,nums_y,angleR);
            //进行放大缩小变换
            cv::resize(src_warpPerspective,src_warpPerspective,cv::Size(radio*src_warpPerspective.cols,radio*src_warpPerspective.rows));

            WindowRoi = big_window(cv::Rect(translation_x,translation_y,src_warpPerspective.cols,src_warpPerspective.rows));
            src_warpPerspective.copyTo(WindowRoi);
            cv::imshow("a window",big_window);

            //clear
            clear.copyTo(big_window);
    }
#endif

#if SPLICE

    std::string img1 = "/home/jiang/Repositories/FaceDeal_Demo1.18/eyes/028.jpg";
    std::string img2 = "/home/jiang/Repositories/FaceDeal_Demo1.18/eyes/066.jpg";
    cv::Mat image1 = cv::imread(img1);// /home/jiang/图片/摄像头/2018-12-11-165320_1.jpg
    cv::Mat image2 = cv::imread(img2);// /home/jiang/图片/摄像头/2018-12-11-165320_3.jpg

    SURF_test(image1,image2,10);
    cv::waitKey(0);
#endif

    return 0;
}



