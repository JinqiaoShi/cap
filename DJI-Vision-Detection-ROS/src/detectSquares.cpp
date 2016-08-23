#include "detectSquares.h"

using namespace cv;
using namespace std;


int thresh = 50, N = 11;
const char *wndname = "Square Detection Demo";

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2

/*******光流追踪******** /

    Mat gray;  //当前灰度图
    Mat gray_prev;  //之前的灰度图
    vector<Point2f> points[2];//前后两帧的特征点
    vector<Point2f> initial;//初始特征点
    Point2f po;
    vector<Point2f> features;//检测到的特征
    int max_count; //要跟踪特征的最大数目
    double qlevel; //特征检测的指标
    double minDist;//特征点之间最小容忍距离
    vector<uchar> status; //特征跟踪状态
    vector<float> err; //跟踪时的错误




    void process(Mat frame,Mat &output,std::vector<cv::Point> rect){
        //得到灰度图
        cvtColor (frame,gray,CV_BGR2GRAY);
        frame.copyTo (output);
        //特征点太少了，重新检测特征点

for(int i=0;i<4;i++)
{
    po.x=rect[i].x;
        po.y=rect[i].y;
    points[0].push_back(po);
}
            //插入检测到的特征点



        //第一帧
        if(gray_prev.empty ()){
                gray.copyTo (gray_prev);
             initial.insert (initial.end (),points[0].begin (),points[0].end ());
        }
        //根据前后两帧灰度图估计前一帧特征点在当前帧的位置
        //默认窗口是15*15
        calcOpticalFlowPyrLK (
                gray_prev,//前一帧灰度图
                gray,//当前帧灰度图
                points[0],//前一帧特征点位置
                points[1],//当前帧特征点位置
                status,//特征点被成功跟踪的标志
                err);//前一帧特征点点小区域和当前特征点小区域间的差，根据差的大小可删除那些运动变化剧烈的点



        size_t i,k;
        for(i = k = 0; i < points[1].size(); i++ )
        {
            if( !status[i] )
                continue;
            points[1][k++] = points[1][i];
            circle( output, points[1][i], 3, Scalar(0,0,255), -1, 8);

        }


        int kk = 0;
        //去除那些未移动的特征点

        points[1].resize (kk);
        initial.resize (kk);
        //标记被跟踪的特征点
       // handleTrackedPoint (output);
        //为下一帧跟踪初始化特征点集和灰度图像
        std::swap(points[1],points[0]);
        cv::swap(gray_prev,gray);
    }*/

   geometry_msgs::Point DetectShapes::squaresPnP(vector<cv::Point> rect,std::vector<Point3f> objectPoints)
   {

      vector<cv::Point2f> imagePoints;
      geometry_msgs::Point tvec_po;
      Point2f po2;
       for(int i=0;i<4;i++)
       {
           po2.x=rect[i].x;
               po2.y=rect[i].y;
           imagePoints.push_back(po2);
       }

        // std::vector<cv::Point3f> objectPoints = Generate3DPointsq1();


         cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);
         cv::setIdentity(cameraMatrix);

         cameraMatrix.at<double>(0) = 459.521240;
         cameraMatrix.at<double>(2) = 319.417322;
         cameraMatrix.at<double>(4) = 460.468918;
         cameraMatrix.at<double>(5) = 213.467184;

         //std::cout << "Initial cameraMatrix: " << cameraMatrix << std::endl;

         cv::Mat distCoeffs(4,1,cv::DataType<double>::type);
         distCoeffs.at<double>(0) = -0.373503;
         distCoeffs.at<double>(1) = 0.129433;
         distCoeffs.at<double>(2) = -0.000842;
         distCoeffs.at<double>(3) = -0.002650  ;

         cv::Mat rvec(3,1,cv::DataType<double>::type);
         cv::Mat tvec(3,1,cv::DataType<double>::type);


         cv::solvePnP(objectPoints, Mat(imagePoints), cameraMatrix, distCoeffs, rvec, tvec);


         tvec_po.x=-tvec.at<double>(1);
         tvec_po.y=tvec.at<double>(0);
         tvec_po.z=tvec.at<double>(2);

         //std::cout << "tvec: " << tvec << std::endl;
         //std::cout << "tvec_po: " << tvec_po << std::endl;


        return tvec_po;

   }
    vector<cv::Point2f> DetectShapes::Generate2DPoints()
    {
      std::vector<cv::Point2f> points;

      float x,y;

      x=282;y=274;
      points.push_back(cv::Point2f(x,y));

      x=397;y=227;
      points.push_back(cv::Point2f(x,y));

      x=577;y=271;
      points.push_back(cv::Point2f(x,y));

      x=462;y=318;
      points.push_back(cv::Point2f(x,y));



      for(unsigned int i = 0; i < points.size(); ++i)
        {
       // std::cout << points[i] << std::endl;
        }

      return points;
    }


    vector<cv::Point3f> DetectShapes::Generate3DPointsq1()//四区盒子
    {
      std::vector<cv::Point3f> points;


      float x,y,z;

      x=0.166; y=-0.166;z=0;
      points.push_back(cv::Point3f(x,y,z));

      x=0.166;y=0.166;z=0;
      points.push_back(cv::Point3f(x,y,z));

      x=-0.166;y=0.166;z=0;
      points.push_back(cv::Point3f(x,y,z));

      x=-0.166;y=-0.166;z=0;
      points.push_back(cv::Point3f(x,y,z));



      for(unsigned int i = 0; i < points.size(); ++i)
        {
        //std::cout << points[i] << std::endl;
        }

      return points;
    }

    vector<cv::Point3f> DetectShapes::Generate3DPointsq2()
    {
      std::vector<cv::Point3f> points;


      float x,y,z;

      x=0.240; y=0.240;z=0;
      points.push_back(cv::Point3f(x,y,z));

      x=0.240;y=0;z=0;
      points.push_back(cv::Point3f(x,y,z));

      x=0;y=0;z=0;
      points.push_back(cv::Point3f(x,y,z));

      x=0;y=0.240;z=0;
      points.push_back(cv::Point3f(x,y,z));



      for(unsigned int i = 0; i < points.size(); ++i)
        {
        //std::cout << points[i] << std::endl;
        }

      return points;
    }


    vector<cv::Point3f> DetectShapes::Generate3DPointsq3()
    {
      std::vector<cv::Point3f> points;


      float x,y,z;

      x=.5;y=.5;z=0;
      points.push_back(cv::Point3f(x,y,z));

      x=.5;y=.5;z=.5;
      points.push_back(cv::Point3f(x,y,z));

      x=-.5;y=.5;z=.5;
      points.push_back(cv::Point3f(x,y,z));

      x=-.5;y=.5;z=0;
      points.push_back(cv::Point3f(x,y,z));



      for(unsigned int i = 0; i < points.size(); ++i)
        {
        //std::cout << points[i] << std::endl;
        }

      return points;
    }




int DetectShapes::graylevel(Mat image)//求取圆形区域内的平均灰度值
{
    int graysum = 0, n = 0;

    for(int i = 0; i <= image.rows; ++i)//访问矩形框内的像素值
    {
        uchar* data = image.ptr<uchar>(i);
        for(int j = 0; j <= image.cols; ++j)
        {

                ++n;
                graysum += (int)data[j];

        }
    }



    return(graysum / n);
}

bool DetectShapes::isBox(Mat img,std::vector<cv::Point> rect)
{
    const int channels[1]={0};

    const int histSize[1]={256};

    float hranges[2]={0,255};

    const float* ranges[1]={hranges};

    MatND hist;




    int x[4]= {0},y[4]= {0};
    int maxX = 0,maxY = 0,minX = 2000,minY = 2000;
    //get the rect points
    for(int i=0;i<4;i++){
        x[i] = rect[i].x;
        y[i] = rect[i].y;
        if(maxX<x[i])
            maxX = x[i];
        if(maxY<y[i])
            maxY = y[i];
        if(minX>x[i])
            minX = x[i];
        if(minY>y[i])
            minY = y[i];
    }

    Rect rectROI(minX,minY,maxX - minX,maxY - minY);
    Mat srcROI=img(rectROI);

    Mat imgHSV, imgHSV_H;
    vector<Mat> imgHSV_split;
    cvtColor(srcROI, imgHSV, 0);
    split(imgHSV, imgHSV_split);
   imgHSV_H = imgHSV_split[2];
   inRange(imgHSV_H,33,255,imgHSV_H);//


   //cout<<"graylevel"<<graylevel(imgHSV_H)<<endl;

imshow( "srcROI" , imgHSV_H );
   if(graylevel(imgHSV_H)>60) return 0;
   else return 1;


}


double DetectShapes::angle(Point pt1, Point pt2, Point pt0) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    double ratio;//边长平方的比
    ratio = (dx1 * dx1 + dy1 * dy1) / (dx2 * dx2 + dy2 * dy2);
    if (ratio < 0.8 || 1.5 < ratio) {//根据边长平方的比过小或过大提前淘汰这个四边形，如果淘汰过多，调整此比例数字
        //      Log("ratio\n");
       // return 1.0;//根据边长平方的比过小或过大提前淘汰这个四边形
    }
    //TODO: Filter the boxes with size, calculated from UAV height.

    return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}



// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
void DetectShapes::findSquares(const Mat &image, vector <vector<Point>> &squares) {
    squares.clear();

    Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, Size(image.cols / 2, image.rows / 2));
    pyrUp(pyr, timg, image.size());
    vector <vector<Point>> contours;

    // find squares in every color plane of the image
    for (int c = 0; c < 3; c++) {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        for (int l = 0; l < N; l++) {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if (l == 0) {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 0, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, Mat(), Point(-1, -1));
            } else {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l + 1) * 255 / N;
            }

            // find contours and store them all as a list
            findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

            vector <Point> approx;

            // test each contour
            for (size_t i = 0; i < contours.size(); i++) {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.02, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if (approx.size() == 4 && fabs(contourArea(Mat(approx))) < 16000&&
                    fabs(contourArea(Mat(approx))) > 2000 &&
                    isContourConvex(Mat(approx))) {
                    double maxCosine = 0;

                    for (int j = 2; j < 5; j++) {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence


                    if (maxCosine < 0.35 && isBox(image,approx))
                    {

                        squares.push_back(approx);

                          cout<<squares.size()<<endl;
                        }
                }
            }
        }
    }
}


// the function draws all the squares in the image
void DetectShapes::drawSquares(Mat &image, const vector <vector<Point>> &squares) {
    for (size_t i = 0; i < squares.size(); i++) {
        const Point *p = &squares[i][0];
        int n = (int) squares[i].size();
            polylines(image, &p, &n, 1, true, Scalar(0, 255, 0), 3, CV_AA);
    }

    imshow(wndname, image);
}


