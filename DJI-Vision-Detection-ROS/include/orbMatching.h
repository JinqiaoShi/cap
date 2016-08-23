//
// Created by jiamingsuen on 8/7/16.
//

#ifndef DJI_VISION_DETECTION_ORBMATCHING_H
#define DJI_VISION_DETECTION_ORBMATCHING_H
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
class FeatureExtraction{

public: cv::Mat orbMatching(cv::Mat img_Template, cv::Mat img);

};

#endif //DJI_VISION_DETECTION_ORBMATCHING_H
