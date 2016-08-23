#include <iostream>
//#include "orbMatching.h"
//#include "detectSquares.h"
#include"image_converter.h"
/****ros***/
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8.h"

#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

using namespace std;
using namespace cv;


/*********************
 *
 * 参数 格子面积、长宽比、平均灰度值
 *
 *
 * *********************/
#define SHOW_IMAGE
#define DEBUG
//#define SHOW_IMAGE_ORB

/****global***/
sensor_msgs::CameraInfo camera_info;
sensor_msgs::Image image_raw;
Mat imgOriginal;
int tr,flag;








void image_callback(const sensor_msgs::Image::ConstPtr msg){

ros_to_cv(&imgOriginal,msg);

}


void flag_callback(const std_msgs::Int8::ConstPtr msg){

flag=msg->data;

}



int main(int argc, char **argv){

    ros::init(argc,argv,"image_process");
    ros::NodeHandle n;
    ros::Subscriber image_raw_sub = n.subscribe("/camera/camera_detection",1, image_callback);
    ros::Subscriber flag_sub = n.subscribe("/position_flag",1, flag_callback);
    ros::Publisher vision_detect_result = n.advertise<geometry_msgs::PointStamped>("vision_detect_result", 1);
    ros::Publisher vision_ready = n.advertise<std_msgs::UInt8>("/vision_ready", 1);
   // Mat imgHippo = imread("~/Downloads/RM-SC2016/Hippo_LED.jpg");
    Mat CM = Mat(3, 3, CV_32FC1);
    Mat D;

    //************************undistort*********************************
    Mat map1,map2;

    Mat  imgOrig;


    Size imgSize(640, 480);
    geometry_msgs::PointStamped detect_result;
    std_msgs::UInt8 vision_ready_msg;
ros::Rate loop_rate(20);
     Mat imagekk;




     double t; //timing variable

     //load training image
     Mat object = imread ("/home/robot/catkin_ws/src/DJI-Vision-Detection-ROS/1111.png", CV_LOAD_IMAGE_GRAYSCALE);
     if (!object.data){
         cout<<"Can't open image";
         return -1;
     }
     namedWindow("Good Matches", CV_WINDOW_AUTOSIZE);
 //namedWindow( "imageName" , CV_WINDOW_AUTOSIZE );
 //cvCreateTrackbar("tr","imageName", &tr, 255);
     //SURF Detector, and descriptor parameters
     int minHess=3000;
     vector<KeyPoint> kpObject;
     Mat desObject;


    // Size imgSize(640, 360);

     //SURF Detector, and descriptor parameters, match object initialization
     minHess=2000;
     SurfFeatureDetector detector(minHess);
     detector.detect(object, kpObject);
     SurfDescriptorExtractor extractor;
     extractor.compute(object, kpObject, desObject);
     FlannBasedMatcher matcher;

     vector<Point2f> obj_corners(4);
     obj_corners[0] = cvPoint(0,0);
     obj_corners[1] = cvPoint( object.cols, 0 );
     obj_corners[2] = cvPoint( object.cols, object.rows );
     obj_corners[3] = cvPoint( 0, object.rows );


     double frameCount = 0;
     float thresholdMatchingNN=0.9;
     unsigned int thresholdGoodMatches=1;




    detect_result.header.frame_id="oct";
    FileStorage fs2("/home/robot/catkin_ws/src/DJI-Vision-Detection-ROS/m100_detect_camera.yml",FileStorage::READ);
            fs2["camera_matrix"]>>CM;
            fs2["distortion_coefficients"]>>D;
            fs2.release();
            cout<<"CM: "<<CM<<endl<<"D: "<<D<<endl;
            initUndistortRectifyMap(CM,D,Mat(),Mat(),imgSize,CV_32FC1,map1,map2);

#ifdef DEBUG
    int tr=220;
    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    cvCreateTrackbar("LowH", "Control", &tr, 255); //Hue (0 - 179)

#endif

    while(n.ok()){




//        if(!imgOriginal.empty() && flag==4)
//        {


        if(!imgOriginal.empty())
        {

            remap(imgOriginal,imgOriginal,map1,map2,CV_INTER_LINEAR);
            resize(imgOriginal, imgOrig, imgSize);



//threshold(imgOrig, imgOrig, tr, 255, CV_THRESH_BINARY);


                Mat image;


                cvtColor(imgOrig, image, CV_RGB2GRAY);


imshow("dfsdfsdf",image);
                Mat des_image, img_matches, H;
                vector<KeyPoint> kp_image;
                vector<vector<DMatch > > matches;
                vector<DMatch > good_matches;
                vector<Point2f> obj;
                vector<Point2f> scene;
                vector<Point2f> scene_corners(4);

                detector.detect( image, kp_image );
                extractor.compute( image, kp_image, des_image );
                matcher.knnMatch(desObject, des_image, matches, 2);

                                            for(int i = 0; i < min(des_image.rows-1,(int) matches.size()); i++) //THIS LOOP IS SENSITIVE TO SEGFAULTS
                                            {
                                                if((matches[i][0].distance < thresholdMatchingNN*(matches[i][1].distance)) && ((int) matches[i].size()<=2 && (int) matches[i].size()>0))
                                                {
                                                    good_matches.push_back(matches[i][0]);
                                                }
                                            }



                                            if (good_matches.size() >= thresholdGoodMatches)
                                            {


                                                for(unsigned int i = 0; i < good_matches.size(); i++ )
                                                {
                                                    //Get the keypoints from the good matches
                                                    obj.push_back( kpObject[ good_matches[i].queryIdx ].pt );
                                                    scene.push_back( kp_image[ good_matches[i].trainIdx ].pt );
                                                }

                                                vector<unsigned char> inlierMask(good_matches.size());
                                                vector<DMatch> ransac_matches;
                                                H = findHomography( obj, scene, CV_RANSAC,3,inlierMask);
                                                for(int i=0;i<inlierMask.size();i++)
                                                {
                                                    if(inlierMask[i])
                                                    {
                                                        ransac_matches.push_back(good_matches[i]);
                                                    }
                                                }

                                                for(unsigned int i = 0; i < ransac_matches.size(); i++ )
                                                {
                                                    //Get the keypoints from the good matches
                                                    obj.push_back( kpObject[ ransac_matches[i].queryIdx ].pt );
                                                    scene.push_back( kp_image[ ransac_matches[i].trainIdx ].pt );

                                                }


                                                drawMatches( object, kpObject, image, kp_image, ransac_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
                                           //cout<<ransac_matches.size()<<endl;
                                                 H = findHomography( obj, scene, CV_RANSAC);
                                                perspectiveTransform( obj_corners, scene_corners, H);

                                                double dx1 = scene_corners[1].x - scene_corners[0].x;
                                                double dy1 = scene_corners[1].y - scene_corners[0].y;
                                                double dx2 = scene_corners[2].x - scene_corners[0].x;
                                                double dy2 = scene_corners[2].y - scene_corners[0].y;
                                                double ratio;//边长平方的比
                                                ratio = (dx1 * dx1 + dy1 * dy1) / (dx2 * dx2 + dy2 * dy2);
                                                cout<<ratio<<"          "<<fabs(contourArea(Mat(scene_corners)))<<endl;
                                                if (ratio <2 && ransac_matches.size()>4  && fabs(contourArea(Mat(scene_corners))) > 30000) {//根据边长平方的比过小或过大提前淘汰这个四边形，如果淘汰过多，调整此比例数字
                                                    //      Log("ratio\n");
                                                    line( img_matches, scene_corners[0] + Point2f( object.cols, 0), scene_corners[1] + Point2f( object.cols, 0), Scalar(0, 255, 0), 4 );
                                                    line( img_matches, scene_corners[1] + Point2f( object.cols, 0), scene_corners[2] + Point2f( object.cols, 0), Scalar( 0, 255, 0), 4 );
                                                    line( img_matches, scene_corners[2] + Point2f( object.cols, 0), scene_corners[3] + Point2f( object.cols, 0), Scalar( 0, 255, 0), 4 );
                                                    line( img_matches, scene_corners[3] + Point2f( object.cols, 0), scene_corners[0] + Point2f( object.cols, 0), Scalar( 0, 255, 0), 4 );
                                                vision_ready_msg.data=1;
                                                }
                                            else{ vision_ready_msg.data=0;}

                                            }

                                           if(!img_matches.empty()) imshow( "Good Matches", img_matches );
                                           cvWaitKey(10);










}





vision_detect_result.publish(detect_result);
vision_ready.publish(vision_ready_msg);



       ros::spinOnce();
       loop_rate.sleep();
    }

    return 0;
}
