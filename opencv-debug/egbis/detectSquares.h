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
class DetectShapes {

public:
    static void findSquares( const Mat& image, vector<vector<Point> >& squares );
    static Mat drawSquares( Mat& image, const vector<vector<Point> >& squares );

private:
    static double angle( Point pt1, Point pt2, Point pt0 );


};
#endif //DJI_VISION_DETECTION_DETECTSQUARES_H
