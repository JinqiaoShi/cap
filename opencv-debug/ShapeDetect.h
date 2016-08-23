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

using namespace cv;
using namespace std;
class ShapeDetect {

public:
    static void findSquares( const Mat& image, vector<vector<Point> >& squares );
    static Mat drawSquares( Mat& image, const vector<vector<Point> >& squares );
    static Mat imgMorphing(Mat& imgHSV_Morphed, int kerOpenSize = 8, int kerCloseSize = 3);
    static vector<vector<Point>> detectNineSquare(Mat& imgOrig, int hsvRange[2], int kerOpenSize = 8, int kerCloseSize = 3);
    static vector<vector<Point>> detectDoll(Mat& imgOrig, int hsvRange[6], int kerOpenSize = 4, int kerCloseSize = 3);
    static bool checkBbox(vector<vector<Point>> bbox);
    static vector<vector<Point>> drawContoursROI(Mat imgBinary, Mat imgOriginal, int minArea = 1000);
    static int findRectInfo(std::vector<cv::Point> rect);






private:
    static double angle( Point pt1, Point pt2, Point pt0 );
    static bool isBox(Mat img,std::vector<cv::Point> rect);
    static int graylevel(Mat image);



};
#endif //DJI_VISION_DETECTION_DETECTSQUARES_H
