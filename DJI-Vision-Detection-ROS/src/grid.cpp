#include <iostream>
#include"image_converter.h"
/****ros***/
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/CameraInfo.h"
#include <iostream>
#include <math.h>
#include <string.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "detectSquares.h"
using namespace std;
using namespace cv;

#define SHOW_IMAGE
#define DEBUG
//#define SHOW_IMAGE_ORB

/****global***/
sensor_msgs::CameraInfo camera_info;
sensor_msgs::Image image_raw;
Mat imgOriginal;
 int tr=220;
 int tr1=220;
 bool flag=false;
vector<vector<Point>> drawContoursROI(Mat imgBinary, Mat imgOriginal, int minArea) {
    vector<vector<Point>> contours;
    vector<Vec4i> he;
    findContours(imgBinary, contours, he, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    vector<vector<Point>> contoursROI;
    for (int i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        //cout<<area<<endl;
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
    result = drawContoursROI(imgContoured, imgWithContour,8000);




    //        imshow("HSV", imgHSV);
//imshow("HSV_Channel2", imgHSV_H);
//imshow("HSV_Channel2_Morphed", imgHSV_Morphed);
//imshow("HSV_Channel2_thresholded", imgHSV_thresholded);
imshow("Final", imgWithContour);

    return result;
}



void image_callback(const sensor_msgs::Image::ConstPtr msg){

ros_to_cv(&imgOriginal,msg);

}


void mission_callback(const geometry_msgs::PointStamped::ConstPtr msg){

if(msg->header.frame_id=="2") flag=true;
else flag =false;

}



int main(int argc, char **argv){

    ros::init(argc,argv,"image_process");
    ros::NodeHandle n;
    ros::Subscriber image_raw_sub = n.subscribe("/image_raw",1, image_callback);
    ros::Publisher vision_detect_result = n.advertise<geometry_msgs::PointStamped>("vision_position", 1);
    ros::Subscriber mission = n.subscribe("/task_status",1, mission_callback);
    Mat imgHippo = imread("~/Downloads/RM-SC2016/Hippo_LED.jpg");

    Size imgSize(640, 480);

    Mat map1,map2;
    Mat CM = Mat(3, 3, CV_32FC1);
    Mat D;
    FileStorage fs2("/home/robot/catkin_ws/src/DJI-Vision-Detection-ROS/m100_detect_camera.yml",FileStorage::READ);
            fs2["camera_matrix"]>>CM;
            fs2["distortion_coefficients"]>>D;
            fs2.release();
            cout<<"CM: "<<CM<<endl<<"D: "<<D<<endl;
            initUndistortRectifyMap(CM,D,Mat(),Mat(),imgSize,CV_32FC1,map1,map2);


#ifdef DEBUG

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    cvCreateTrackbar("LowH", "Control", &tr, 255); //Hue (0 - 179)
        cvCreateTrackbar("LowHss", "Control", &tr1, 255); //Hue (0 - 179)

#endif

    Mat  imgOrig;
    DetectShapes det_sh;
    vector<vector<Point> > squares;
        vector<vector<Point> > squares2;

    geometry_msgs::PointStamped detect_result;

    int yellowRange[2] = {0, 50};  //YELLOW
    detect_result.header.frame_id="oct";
cout<<"1111111111111111111111111"<<endl;
std::vector<cv::Point3f> objectPoints = det_sh.Generate3DPointsq2();//4区盒子

    while(n.ok()){
        double time0 = static_cast<double>(getTickCount());
        //capture >> imgOrig;
squares2.clear();
vector<Point> rect;
int blueRange[2] = {146, 255};  //BLUE
        if(!imgOriginal.empty() )
        {
            remap(imgOriginal,imgOrig,map1,map2,CV_INTER_LINEAR);
            //threshold(imgOriginal, imgOriginal, tr, 255, CV_THRESH_BINARY);
            //resize(imgOriginal, imgOrig, imgSize);

//threshold(imgOrig, imgOrig, tr1, 255, CV_THRESH_BINARY);
        squares=detectNineSquare(imgOrig,blueRange,4,2);
//        int xmin=1000,z=1;
//        for(int i = 1; i < squares.size(); i++){

//            if(squares[i][3].x<xmin){ xmin=squares[i][3].x;z=i;}

//        }

//        cout<<z<<endl;
//        squares2.clear();
       //if(!squares.empty()) squares2.push_back(squares[z]);
        det_sh.drawSquares(imgOrig, squares);

        //drawContoursROI(imgOrig);

        if (!squares.empty()) {

           //for(int j = 0; j < squares.size(); j++){

          int j=0;
          if(squares.size()>=2) j=1;


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
// cout<<" no sort "<<  j<<" "<<xx[z1]<<"  "<<xx[z2]<<endl;
 //cout<<"sort "<<x[3]<<"  "<<x[2]<<endl;

if(squares[j][z1].y>squares[j][z2].y || squares[j][z1].y==squares[j][z2].y) { rect.push_back(squares[j][z1]);rect.push_back(squares[j][z2]);}
else  { rect.push_back(squares[j][z2]);rect.push_back(squares[j][z1]);}

if(squares[j][z3].y<squares[j][z4].y || squares[j][z3].y==squares[j][z4].y) { rect.push_back(squares[j][z3]);rect.push_back(squares[j][z4]);}
else  { rect.push_back(squares[j][z4]);rect.push_back(squares[j][z3]);}

//cout<<squares[j][z1]<<" "<<squares[j][z2]<<endl;
   if(powf((rect[0].x - rect[1].x),2) + powf((rect[0].y - rect[1].y),2)>30) {

//for(int i=0;i<4;i++){

//    if(abs(rect[1].y-squares[j][i].y)<20){rect.push_back(squares[j][i]);}

//    if(abs(rect[0].y-squares[j][i].y)<20){rect.push_back(squares[j][i]);}


//}

                       //  cout<<squares[0][1]<<"                size"<<squares.size()<<endl;
   cout<<rect.size()<<endl;

                  circle( imgOrig, rect[0], 10, Scalar(0,0,255), -1, 8);//red
                                 circle( imgOrig, rect[1], 10, Scalar(255,0,0), -1, 8);//blue
                                 circle( imgOrig, rect[2], 10, Scalar(0,255,255), -1, 8);//red
                                                circle( imgOrig, rect[3], 10, Scalar(255,255,0), -1, 8);//blue





           imshow("imgOrig",imgOrig);
           //detectNineSquare(imgOrig,y

//squares2.push_back(rect);
         //cout<<squares[i]<<endl;
}

           detect_result.header.stamp=ros::Time::now();
           detect_result.point=det_sh.squaresPnP(rect,objectPoints);
    }


}
vision_detect_result.publish(detect_result);
        waitKey(30);

      //  cout << ">> FPS = " << getTickFrequency() / (getTickCount() - time0) << endl;
       ros::spinOnce();
    }

    return 0;
}
