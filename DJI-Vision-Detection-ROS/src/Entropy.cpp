#include"Entropy.h"
using namespace std;
using namespace cv;
Mat drawHist(Mat hist,int bins,int height,Scalar rgb)
{
    double maxVal=0;
    minMaxLoc(hist,0,&maxVal,0,0);
    int scale=1;
    Mat histImg = Mat::zeros(height, bins, CV_8UC3);
    float *binVal = hist.ptr<float>(0);
    for (int i=0; i<bins; i++)
    {
        int intensity = cvRound(binVal[i]*height/maxVal);
        rectangle(histImg,Point(i*scale,0),
            Point((i+1)*scale,(intensity)),rgb,CV_FILLED);
    }
    flip(histImg,histImg,0);
    return histImg;
}
//计算直方图;
Mat Hist(const Mat& src)
{
    Mat hist;
    int bins=256;
    int histSize[] = {bins};
    float range[] = {0,256};
    const float* ranges[] = {range};
    int channels[] = {0};
    calcHist(&src,1,channels,Mat(),hist,1,histSize,ranges,true,false);
    Mat histImg = drawHist(hist,bins,200,Scalar(255,0,0));
    imshow("histRGB",histImg);
    return hist;
}
//计算当前熵;
float calEntropy(const Mat& hist,int threshold)
{
    float total_back=0,total_object=0;
    float entropy_back=0,entropy_object=0;
    float entropy = 0;
    int i=0;

    const float* hist_p = (float*) hist.ptr<float>(0);
    for (i=0; i<threshold; i++)
    {
        total_back += hist_p[i];
    }
    total_object=total-total_back;

    //背景熵;
    for (i=0; i<threshold; i++)
    {
     if(hist_p[i]==0)
         continue;
        float percentage = hist_p[i]/total_back;
        entropy_back += -percentage * logf(percentage); // 能量的定义公式
    }
    //前景熵;
    for (i=threshold; i<hist.cols; i++)
    {
      if(hist_p[i]==0)
      {
          continue;
      }
        float percentage = hist_p[i]/total_object;
        entropy_object += -percentage * logf(percentage); // 能量的定义公式；
    }

    entropy = entropy_object+entropy_back;
    return entropy;
}

Mat MaxEntropy(Mat img1, Mat hist)
{
    total = sum(hist)[0];
    float MaxEntropyValue = 0.0, MaxEntropyThreshold=0.0;
    float tmp;
    for (int i=0; i<hist.cols; i++)
    {
        tmp = calEntropy(hist,i);
        if(tmp>MaxEntropyValue)
        {
            MaxEntropyValue = tmp;
            MaxEntropyThreshold = i;
        }
    }
    threshold(img1,img1,tr,255,CV_THRESH_BINARY);
    imshow("thresholdImg",img1);
    imwrite("D:/thresholdImg.png",img1);
    cout<<"MaxEntropyThreshold"<<MaxEntropyThreshold<<endl;
    cout<<"MaxEntropyValue"<<MaxEntropyValue<<endl;
    return img1;
}


Mat imgMorphing(Mat& imgHSV_Morphed, int kerOpenSize = 8, int kerCloseSize = 3) {


    Mat kernelOpen = getStructuringElement(0, Size(kerOpenSize, kerOpenSize));
    morphologyEx(imgHSV_Morphed, imgHSV_Morphed, MORPH_OPEN, kernelOpen, Point(-1, -1), 2);
    Mat kernelClose = getStructuringElement(0, Size(kerCloseSize, kerCloseSize));

    morphologyEx(imgHSV_Morphed, imgHSV_Morphed, MORPH_CLOSE, kernelClose, Point(-1, -1), 2);
    return imgHSV_Morphed;
}
