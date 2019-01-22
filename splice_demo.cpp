#include "include_files.h"
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>

cv::Mat SURF_test(cv::Mat image01,cv::Mat image02,int Threshold)
{
    cv::Mat image1,image2;
    cv::Mat img_match;
    cvtColor(image01, image1, CV_RGB2GRAY);
    cvtColor(image02, image2, CV_RGB2GRAY);

    //提取特征点
    cv::SurfFeatureDetector surfDetector(Threshold);  // 海塞矩阵阈值，在这里调整精度，值越大点越少，越精准
    vector<cv::KeyPoint> keyPoint1, keyPoint2;
    surfDetector.detect(image1, keyPoint1);
    surfDetector.detect(image2, keyPoint2);

    //特征点描述，为下边的特征点匹配做准备
    cv::SurfDescriptorExtractor SurfDescriptor;
    cv::Mat imageDesc1, imageDesc2;
    SurfDescriptor.compute(image1, keyPoint1, imageDesc1);
    SurfDescriptor.compute(image2, keyPoint2, imageDesc2);
    

//    //归一化并显示出来描述子
//	cv::Mat imageDescShow1;
//	cv::Mat imageDescShow2;
//	normalize(imageDesc1,imageDescShow1,0,255,CV_MINMAX);
//	normalize(imageDesc2,imageDescShow2,0,255,CV_MINMAX);
//	convertScaleAbs(imageDescShow1,imageDescShow1);
//	convertScaleAbs(imageDescShow2,imageDescShow2);
//    imshow("描述子1",imageDescShow1);
//	imshow("描述子2",imageDescShow2);

    //获得匹配特征点
     cv::FlannBasedMatcher matcher;
     vector<cv::DMatch> matchePoints;
     
     vector<cv::Mat> train_desc(1, imageDesc1);
     matcher.add(train_desc);
     matcher.train();
//     matcher.knnMatch(imageDesc2, matchePoints, 2);
//     matcher.knnMatch(imageDesc2,matchePoints,2);
     matcher.match(imageDesc1, imageDesc2, matchePoints, cv::Mat());

     cout << "total match points: " << matchePoints.size() << endl;

     vector<cv::DMatch> GoodmatchePoints;
     //Lowe's algorithm  提取最优配对
     for(size_t i = 0;i < (matchePoints.size() - 2);i++){
         if ((matchePoints[i].distance < 0.6 * matchePoints[i+1].distance)/*&&(matchePoints[i].distance < 0.6 * matchePoints[i+2].distance)*/)
         {
             GoodmatchePoints.push_back(matchePoints[i]);
         }
     }

    vector<cv::DMatch> ERRmatchePoints,OKmatchePoints;
    sort(GoodmatchePoints.begin(),GoodmatchePoints.end());  // 按距离从小到大排序,//提取强特征点 ,获取排在前N个的最优匹配结果
//    for(size_t i = (GoodmatchePoints.size()-1);i > 6;i--){
//          ERRmatchePoints.push_back(GoodmatchePoints[i]);
//    }
//    for(size_t i = 0;i < 6;i++){
//          OKmatchePoints.push_back(GoodmatchePoints[i]);
//    }

    cout << "good match points: " << GoodmatchePoints.size() << endl;
    drawMatches(image01, keyPoint1, image02, keyPoint2, GoodmatchePoints, img_match);
    imshow("match",img_match);

    cv::Mat ERRresult,OKresult;
    drawMatches(image01, keyPoint1, image02, keyPoint2, ERRmatchePoints, ERRresult);
    imshow("err",ERRresult);
    //drawMatches(image01, keyPoint1, image02, keyPoint2, OKmatchePoints, OKresult);
    drawMatches(image01, keyPoint1, image02, keyPoint2, matchePoints, OKresult);
    imshow("ok",OKresult);
    return img_match;
}
