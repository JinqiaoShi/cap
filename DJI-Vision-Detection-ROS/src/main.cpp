#include <iostream>
#include "orbMatching.h"
#include "detectSquares.h"
#include"image_converter.h"
/****ros***/
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/UInt8.h"

using namespace std;
using namespace cv;

#define SHOW_IMAGE
#define DEBUG
//#define SHOW_IMAGE_ORB

/****global***/
sensor_msgs::CameraInfo camera_info;
sensor_msgs::Image image_raw;
Mat imgOriginal;
int tr;
Mat drawContoursROI(Mat& imgBinary) {
    vector<vector<Point>> contours;
    vector<Vec4i> he;
    Mat imgCandidate=imgBinary;
    findContours(imgBinary, contours, he, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    vector<vector<Point>> contoursROI;
    for (int i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area > 10000) {
            drawContours(imgCandidate,contours,i,Scalar(0,0,255),5);
            RotatedRect box;
            Point2f vertex[4];
            box = minAreaRect(contours[i]);
            box.points(vertex);
            vector<Point> ptr;
            for (int j = 0; j < 4; j++) {
                line(imgCandidate, vertex[j], vertex[(j + 1) % 4], Scalar(0, 255, 0), 5);
                ptr.push_back(vertex[j]);
            }
            contoursROI.push_back(ptr);
        }

    }
    Mat img_ROI(imgOriginal.rows, imgOriginal.cols, CV_8UC3, Scalar(0, 0, 0));

    for (int i = 0; i < contoursROI.size(); i++) {
        drawContours(img_ROI, contoursROI, i, Scalar::all(255), -1);
    }
//    cout << contoursROI[1] << endl;
    return imgOriginal;
}






void image_callback(const sensor_msgs::Image::ConstPtr msg){

ros_to_cv(&imgOriginal,msg);

}




int main(int argc, char **argv){

    ros::init(argc,argv,"image_process");
    ros::NodeHandle n;
    ros::Subscriber image_raw_sub = n.subscribe("/image_raw",1, image_callback);
    ros::Publisher vision_detect_result = n.advertise<geometry_msgs::PointStamped>("/vision_position", 1);
    ros::Publisher vision_ready = n.advertise<std_msgs::UInt8>("/vision_ready", 1);
    Mat imgHippo = imread("~/Downloads/RM-SC2016/Hippo_LED.jpg");

    std_msgs::UInt8 vision_ready_msg;
    //************************undistort*********************************


    Mat  imgOrig;
    DetectShapes det_sh;
    vector<vector<Point> > squares;
        vector<vector<Point> > squares2;
    Size imgSize(640, 360);
    geometry_msgs::PointStamped detect_result;


     Mat imagekk;
    detect_result.header.frame_id="oct";
    Mat map1,map2;
    Mat CM = Mat(3, 3, CV_32FC1);
    Mat D;
    FileStorage fs2("/home/robot/catkin_ws/src/DJI-Vision-Detection-ROS/m100_detect_camera.yml",FileStorage::READ);
            fs2["camera_matrix"]>>CM;
            fs2["distortion_coefficients"]>>D;
            fs2.release();
            cout<<"CM: "<<CM<<endl<<"D: "<<D<<endl;
            initUndistortRectifyMap(CM,D,Mat(),Mat(),imgSize,CV_32FC1,map1,map2);
std::vector<cv::Point3f> objectPoints = det_sh.Generate3DPointsq1();//4区盒子
#ifdef DEBUG
    int tr=220;
    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    cvCreateTrackbar("LowH", "Control", &tr, 255); //Hue (0 - 179)

#endif

    while(n.ok()){
        double time0 = static_cast<double>(getTickCount());
        //capture >> imgOrig;



        if(!imgOriginal.empty())
        {
            remap(imgOriginal,imagekk,map1,map2,CV_INTER_LINEAR);

            threshold(imagekk, imgOriginal, tr, 255, CV_THRESH_BINARY);
            resize(imgOriginal, imgOrig, imgSize);


        det_sh.findSquares(imgOrig, squares);
        int xmin=1000,z=1;
        for(int i = 1; i < squares.size(); i++){

            if(squares[i][3].x<xmin){ xmin=squares[i][3].x;z=i;}

        }

        cout<<z<<endl;
        squares2.clear();
       if(!squares.empty()) squares2.push_back(squares[z]);
        det_sh.drawSquares(imgOrig, squares2);

        //drawContoursROI(imgOrig);

        if (!squares.empty()) {

            for(int i = 1; i < squares.size(); i++){

         detect_result.header.stamp=ros::Time::now();
         detect_result.point=det_sh.squaresPnP(squares[i],objectPoints);
         vision_ready_msg.data = 1;
         vision_ready.publish(vision_ready_msg);
         cout<<squares[i]<<endl;

        }}
        else
        {
            vision_ready_msg.data = 0;
            vision_ready.publish(vision_ready_msg);

        }


}
vision_detect_result.publish(detect_result);
        if (waitKey(50) >= 0) break;

      //  cout << ">> FPS = " << getTickFrequency() / (getTickCount() - time0) << endl;
       ros::spinOnce();
    }

    return 0;
}
