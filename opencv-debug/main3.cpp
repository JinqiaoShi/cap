#include <iostream>
//#include "orbMatching.h"
//#include "ShapeDetect.h"
#include <iostream>
#include <math.h>
#include <string.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

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
    Mat img_ROI(imgOriginal.rows, imgOriginal.cols, CV_8UC3, Scalar(0, 0, 0));

    for (int i = 0; i < contoursROI.size(); i++) {
        drawContours(img_ROI, contoursROI, i, Scalar::all(255), -1);
    }
    return contoursROI;
}


Mat imgMorphing(Mat &imgHSV_Morphed, int kerOpenSize, int kerCloseSize) {


    Mat kernelOpen = getStructuringElement(0, Size(kerOpenSize, kerOpenSize));
    morphologyEx(imgHSV_Morphed, imgHSV_Morphed, MORPH_OPEN, kernelOpen, Point(-1, -1), 2);
    Mat kernelClose = getStructuringElement(0, Size(kerCloseSize, kerCloseSize));

    morphologyEx(imgHSV_Morphed, imgHSV_Morphed, MORPH_CLOSE, kernelClose, Point(-1, -1), 2);
    return imgHSV_Morphed;
}

///Input: 1. imgMat 2. hsvRange for box you want to detect 3. Morphing Openning kernel size 4. Closing kernel size.
vector<vector<Point>> detectNineSquare(Mat &imgOrig, int hsvRange[2], int kerOpenSize, int kerCloseSize) {
    //TODO: Write param handling different color choice and color range.
///Prepare img Mat.
    Mat imgHSV, imgHSV_H, imgHSV_thresholded, imgHSV_Morphed;
    vector<vector<Point>> result;
    vector<Mat> imgHSV_split;
    cvtColor(imgOrig, imgHSV, CV_BGR2HSV);
    split(imgHSV, imgHSV_split);
    imgHSV_H = imgHSV_split[1];


///Filter img with color range.
    //TODO: Add exposure adjustment process.
    inRange(imgHSV_H, hsvRange[0], hsvRange[1], imgHSV_thresholded);

    imgHSV_thresholded.copyTo(imgHSV_Morphed);
    imgMorphing(imgHSV_Morphed, kerOpenSize, kerCloseSize);

    Mat imgContoured;
    imgHSV_Morphed.copyTo(imgContoured);
    Mat imgWithContour;
    imgOrig.copyTo(imgWithContour);
    result = drawContoursROI(imgContoured, imgWithContour,1000);




    //        imshow("HSV", imgHSV);
imshow("HSV_Channel2", imgHSV_H);
//imshow("HSV_Channel2_Morphed", imgHSV_Morphed);
//imshow("HSV_Channel2_thresholded", imgHSV_thresholded);
imshow("Final", imgWithContour);

    return result;
}


int main() {

    vector<vector<Point> > squares;
    Mat imgOrig, imgOrigLarge, imgHSV, imgHSV_H, imgHSV_thresholded, imgHSV_Morphed;
    VideoCapture capture("/home/robot/catkin_ws/src/video_stream_opencv/test1.mp4");
    Size imgSize(640, 360);
    capture.set(CV_CAP_PROP_POS_FRAMES, 120);
    system("v4l2-ctl -c exposure_absolute=2");



    for (;;) {

        double time0 = static_cast<double>(getTickCount());
        capture >> imgOrigLarge;

        resize(imgOrigLarge, imgOrig, imgSize);


        int blueRange[2] = {170, 255};  //BLUE
        int yellowRange[2] = {0, 50};  //YELLOW








     squares=detectNineSquare(imgOrig,blueRange,4,2);

     if (!squares.empty()) {

         for(int j = 1; j < squares.size(); j++){



std::vector<cv::Point> rect;
cv::Point max(0,0),min(2000,2000);
             int x[4]= {0},y[4]= {0},xx[4]= {0},z1= 0,z2= 0,z3= 0,z4= 0;
            // int maxX = 0,maxY = 0,minX = 2000,minY = 2000,z=0;
             //get the rect points
             for(int i=0;i<4;i++){
                 x[i] = squares[j][i].x;
                 xx[i] = squares[j][i].x;



             }





sort(x,x+4);

for(int i=0;i<4;i++){

 if(x[3]==xx[i]){z1=i;}
 if(x[2]==xx[i]){z2=i;}
 if(x[1]==xx[i]){z3=i;}
 if(x[0]==xx[i]){z4=i;}


}
cout<<" no sort"<<xx[z1]<<"  "<<xx[z2]<<endl;
cout<<"sort"<<x[3]<<"  "<<x[2]<<endl;

if(squares[j][z1].y>squares[j][z2].y) { rect.push_back(squares[j][z1]);rect.push_back(squares[j][z2]);}
else  { rect.push_back(squares[j][z2]);rect.push_back(squares[j][z1]);}

for(int i=0;i<4;i++){

 if(abs(rect[1].y-squares[j][i].y)<20){rect.push_back(squares[j][i]);}

 if(abs(rect[0].y-squares[j][i].y)<20){rect.push_back(squares[j][i]);}


}

                    //  cout<<squares[0][1]<<"                size"<<squares.size()<<endl;
               circle( imgOrig, rect[0], 10, Scalar(0,0,255), -1, 8);//red
                              circle( imgOrig, rect[1], 10, Scalar(255,0,0), -1, 8);//blue
                              circle( imgOrig, rect[2], 10, Scalar(0,255,255), -1, 8);//red
                                             circle( imgOrig, rect[3], 10, Scalar(255,255,0), -1, 8);//blue

        imshow("imgOrig",imgOrig);
        //detectNineSquare(imgOrig,y

      //detect_result.header.stamp=ros::Time::now();
      //detect_result.point=det_sh.squaresPnP(squares[j]);
      //cout<<squares[i]<<endl;

     }
 }

imshow("imgOrig",imgOrig);
//detectNineSquare(imgOrig,yellowRange,4,2);












    if (waitKey(50) >= 0) break;
//        cout << ">> Loop Time = " << getTickCount() - time0 << "ms" << endl;
    cout << ">> FPS = " << getTickFrequency() / (getTickCount() - time0) << endl;
//        cout << "tr = " << tr <<endl;


}
    return 0;
}
