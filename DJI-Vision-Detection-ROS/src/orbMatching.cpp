#include "orbMatching.h"


using namespace std;
using namespace cv;

Mat FeatureExtraction::orbMatching(Mat imgTemplate, Mat img) {
    Mat grayImage;
    cvtColor(imgTemplate, grayImage, CV_BGR2GRAY);

    imshow("Grey", grayImage);
    OrbFeatureDetector featureDetector;
    vector<KeyPoint> keyPoints;
    Mat descriptors;

    featureDetector.detect(grayImage, keyPoints);


    OrbDescriptorExtractor featureExtractor;
    featureExtractor.compute(grayImage, keyPoints, descriptors);


    flann::Index flannIndex(descriptors, flann::LshIndexParams(5, 10, 2), cvflann::FLANN_DIST_HAMMING);


    Mat captureImage_gray;

    double time0 = static_cast<double>(getTickCount());

    cvtColor(img, captureImage_gray, CV_BGR2GRAY);

    vector<KeyPoint> captureKeyPoints;
    Mat captureDescription;

    featureDetector.detect(captureImage_gray, captureKeyPoints);
    featureExtractor.compute(captureImage_gray, captureKeyPoints, captureDescription);

    Mat matchIndex(captureDescription.rows, 2, CV_32SC1), matchDistance(captureDescription.rows, 2, CV_32FC1);
    flannIndex.knnSearch(captureDescription, matchIndex, matchDistance, 2, flann::SearchParams());

    vector<DMatch> goodMatches;
    for (int i = 0; i < matchDistance.rows; i++) {
        if (matchDistance.at<float>(i, 0) < 0.9 * matchDistance.at<float>(i, 1)) {
            DMatch dmatches(i, matchIndex.at<int>(i, 0), matchDistance.at<float>(i, 0));
            goodMatches.push_back(dmatches);
        }
    }

    Mat resultImage;
    drawMatches(img, captureKeyPoints, imgTemplate, keyPoints, goodMatches, resultImage);

    cout << ">> FPS = " << getTickFrequency() / (getTickCount() - time0) << endl;
    return resultImage;
}


