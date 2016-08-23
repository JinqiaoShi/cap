

//
// Created by shijinqiao on 8/12/16.
//

#ifndef ENTROPY_H
#define ENTROPY_H
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Entropy{

//public: cv::Mat drawHist(cv::Mat hist,int bins,int height,Scalar rgb);
public: cv::Mat Hist(const cv::Mat& src);
public: float calEntropy(const cv::Mat& hist,int threshold);
//public: cv::Mat MaxEntropy(cv::Mat img1, Mat hist);
};

#endif // ENTROPY_H
