/*
 * EEL6562 Project
 * Real Time Object Recognition using SURF
 *
 *  Created on: Nov 15, 2013
 *      Author: Frank
 */

//Include statements
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

//Name spaces used
using namespace cv;
using namespace std;





int main()
{
    //turn performance analysis functions on if testing = true

    double t; //timing variable

    //load training image
    Mat object = imread ("/home/robot/catkin_ws/src/video_stream_opencv/222.png", CV_LOAD_IMAGE_GRAYSCALE);
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


    Size imgSize(640, 360);

    //SURF Detector, and descriptor parameters, match object initialization
    minHess=2000;
    SurfFeatureDetector detector(minHess);
    detector.detect(object, kpObject);
    SurfDescriptorExtractor extractor;
    extractor.compute(object, kpObject, desObject);
    FlannBasedMatcher matcher;

    //Initialize video and display window
    //VideoCapture cap(1);  //camera 1 is webcam
    VideoCapture capture("/home/robot/catkin_ws/src/video_stream_opencv/test2.mp4");
   // if (!cap.isOpened()) return -1;
    capture.set(CV_CAP_PROP_POS_FRAMES, 100);
    //Object corner points for plotting box
    vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( object.cols, 0 );
    obj_corners[2] = cvPoint( object.cols, object.rows );
    obj_corners[3] = cvPoint( 0, object.rows );

    //video loop
    char escapeKey='k';
    double frameCount = 0;
    float thresholdMatchingNN=0.7;
    unsigned int thresholdGoodMatches=20;


    for (;;){
        //thresholdGoodMatches=thresholdGoodMatchesV[j];
        //thresholdGoodMatches=8;
        //cout<<thresholdGoodMatches<<endl;

    if(true)
    {
        t = (double)getTickCount();
    }

    while (escapeKey != 'q')
    {double time0 = static_cast<double>(getTickCount());
        frameCount++;
        Mat frame;
        Mat image;
        capture>>frame;
        resize(frame, frame, imgSize);


        cvtColor(frame, image, CV_RGB2GRAY);
        //threshold(image,image,tr,255,CV_THRESH_BINARY);

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

                                    //if (good_matches.size()<1)
                                    //	good_matches.resize(0,cv::DMatch);

                                    //Draw only "good" matches


                                    if (good_matches.size() >= thresholdGoodMatches)
                                    {

                                        //Display that the object is found
                                        //putText(img_matches, "Object Found", cvPoint(10,50),FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0,0,250), 1, CV_AA);
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
                                   cout<<ransac_matches.size()<<endl;
                                         H = findHomography( obj, scene, CV_RANSAC);
                                        perspectiveTransform( obj_corners, scene_corners, H);

                                        double dx1 = scene_corners[1].x - scene_corners[0].x;
                                        double dy1 = scene_corners[1].y - scene_corners[0].y;
                                        double dx2 = scene_corners[2].x - scene_corners[0].x;
                                        double dy2 = scene_corners[2].y - scene_corners[0].y;
                                        double ratio;//边长平方的比
                                        ratio = (dx1 * dx1 + dy1 * dy1) / (dx2 * dx2 + dy2 * dy2);
                                        if (ratio < 0.8 || 1.2 < ratio && ransac_matches.size()>10) {//根据边长平方的比过小或过大提前淘汰这个四边形，如果淘汰过多，调整此比例数字
                                            //      Log("ratio\n");
                                            line( img_matches, scene_corners[0] + Point2f( object.cols, 0), scene_corners[1] + Point2f( object.cols, 0), Scalar(0, 255, 0), 4 );
                                            line( img_matches, scene_corners[1] + Point2f( object.cols, 0), scene_corners[2] + Point2f( object.cols, 0), Scalar( 0, 255, 0), 4 );
                                            line( img_matches, scene_corners[2] + Point2f( object.cols, 0), scene_corners[3] + Point2f( object.cols, 0), Scalar( 0, 255, 0), 4 );
                                            line( img_matches, scene_corners[3] + Point2f( object.cols, 0), scene_corners[0] + Point2f( object.cols, 0), Scalar( 0, 255, 0), 4 );
                                        }
                                        cout<<ratio<<endl;
                                       // Draw lines between the corners (the mapped object in the scene image )

                                    }
                                   // else
                                   // {
                                      //  putText(img_matches, "", cvPoint(10,50), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0,0,250), 1, CV_AA);
                                   // }
//
                                    //Show detected matches
                                   if(!img_matches.empty()) imshow( "Good Matches", img_matches );
                                    escapeKey=cvWaitKey(1);

                                    //imwrite("C:/School/Image Processing/bookIP3.jpg", img_matches);

                                    //if(frameCount>10)
                                        //escapeKey='q';

cout << ">> FPS = " << getTickFrequency() / (getTickCount() - time0) << endl;
    }

    //average frames per second
    if(true)
    {
        t = ((double)getTickCount() - t)/getTickFrequency();
        cout<<t<<" "<<frameCount/t<<endl;
        cvWaitKey(0);
    }

    frameCount=0;
    escapeKey='a';
    }

    //Release camera and exit
    // qcap.release();
    return 0;
}
