#include <iostream>
//#include "orbMatching.h"
//#include "ShapeDetect.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;
#define SHOW_IMAGE
//#define SHOW_IMAGE_ORB


vector<vector<Point>> drawContoursROI(Mat imgBinary, Mat imgOriginal, int minArea) {
    vector<vector<Point>> contours;
    vector<Vec4i> he;
    findContours(imgBinary, contours, he, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    vector<vector<Point>> contoursROI;
    for (int i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area > minArea) {
//                drawContours(imgOrig,contours,i,Scalar(0,0,255),5);
            RotatedRect box;
            Point2f vertex[4];
            box = minAreaRect(contours[i]);
            box.points(vertex);

            vector<Point> ptr;
            for (int j = 0; j < 4; j++) {

                line(imgOriginal, vertex[j], vertex[(j + 1) % 4], Scalar(0, 255, 0), 5);
                ptr.push_back(vertex[j]);
            }

            contoursROI.push_back(ptr);
        }

    }

    return contoursROI;
}

int main() {




    Mat  imgOrig, imgOrigLarge, imgOrigThresholdedGrey, imgOrigLedBinary;

    vector<vector<Point> > squares, roi;

    Mat imgHippo = imread("~/Downloads/RM-SC2016/Hippo_LED.jpg");
    VideoCapture capture("/home/robot/catkin_ws/src/video_stream_opencv/test2.mp4");
    capture.set(CV_CAP_PROP_POS_FRAMES, 1000);

    Size imgSize(640, 360);

    for (;;) {

        double time0 = static_cast<double>(getTickCount());
        capture >> imgOrigLarge;

        resize(imgOrigLarge, imgOrig, imgSize);


        Mat imgOrigThresholded;
        threshold(imgOrig, imgOrigThresholded, 185, 255, CV_THRESH_BINARY);


        cvtColor(imgOrigThresholded, imgOrigThresholdedGrey, CV_RGB2GRAY);

        inRange(imgOrigThresholdedGrey, 166, 255, imgOrigLedBinary);
        vector<vector<Point>> bbox;
        bbox = drawContoursROI(imgOrigLedBinary, imgOrig,1000);


//调试用的  不用管
        if (bbox.empty()) {
            cout << "┑(￣Д ￣)┍" << endl;
            //TODO: Implement a mechanism to retrieve last good result of box detection when no box is found.
        }
        else {
            for(int i = 1; i < bbox.size(); i++) {

                cout << bbox[i] << endl;
                cout << "SIZE = " << bbox.size() << endl;

                for (int j = 1; j < bbox[i].size(); j++) {
                    if (bbox[i][j].x < imgSize.width && bbox[i][j].y < imgSize.height) {


                        //This is outputting tuple of all detected bundling box.
                    }
                }
            }
        }




#ifdef SHOW_IMAGE
        imshow("Origin", imgOrig);
       // imshow("Grey", imgOrigThresholdedGrey);
       // imshow("Binary", imgOrigLedBinary);
       // imshow("Thresholded", imgOrigThresholded);
#endif



/****************LED Detection END*************/





    if (waitKey(50) >= 0) break;
//        cout << ">> Loop Time = " << getTickCount() - time0 << "ms" << endl;
    cout << ">> FPS = " << getTickFrequency() / (getTickCount() - time0) << endl;
//        cout << "tr = " << tr <<endl;


}
    return 0;
}
