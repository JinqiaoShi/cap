//
// Created by jiamingsuen on 8/10/16.
//

#ifndef DJI_VISION_DETECTION_DETECTSQUARES_H
#define DJI_VISION_DETECTION_DETECTSQUARES_H
#include <iostream>
#include <math.h>
#include <string.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "geometry_msgs/Point.h"

using namespace cv;
using namespace std;
class DetectShapes {

public:
    static void findSquares(const Mat& image, vector<vector<Point> >& squares);
    static void drawSquares(Mat& image, const vector<vector<Point> >& squares);
    static geometry_msgs::Point squaresPnP(vector<cv::Point> rect, std::vector<Point3f> objectPoints);
    static vector<cv::Point3f> Generate3DPointsq1();
    static vector<cv::Point3f> Generate3DPointsq2();
    static vector<cv::Point3f> Generate3DPointsq3();
    static vector<cv::Point2f> Generate2DPoints();

private:
    static double angle( Point pt1, Point pt2, Point pt0 );
    static bool isBox(Mat img,std::vector<cv::Point> rect);
    static int graylevel(Mat image);



};
#endif //DJI_VISION_DETECTION_DETECTSQUARES_H
