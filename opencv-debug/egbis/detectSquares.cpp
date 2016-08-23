#include "detectSquares.h"

using namespace cv;
using namespace std;


int thresh = 50, N = 11;
const char* wndname = "Square Detection Demo";
// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
 double DetectShapes::angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
double ratio;//边长平方的比
    ratio = (dx1 * dx1 + dy1 * dy1) / (dx2 * dx2 + dy2 * dy2);
    if (ratio < 0.4 || 1.2 < ratio) {//根据边长平方的比过小或过大提前淘汰这个四边形，如果淘汰过多，调整此比例数字
        //      Log("ratio\n");
        return 1.0;//根据边长平方的比过小或过大提前淘汰这个四边形
    }

    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
 void DetectShapes::findSquares( const Mat& image, vector<vector<Point> >& squares )
{
    squares.clear();

    Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    vector<vector<Point> > contours;

    // find squares in every color plane of the image
    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 0, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, Mat(), Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }

            // find contours and store them all as a list
            findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

            vector<Point> approx;

            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.009, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 &&
                    fabs(contourArea(Mat(approx))) > 1000 &&
                    isContourConvex(Mat(approx)) )
                {
                    double maxCosine = 0;

                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < 0.28 )
                        squares.push_back(approx);
                }
            }
        }
    }
}

 int* findRectInfo(std::vector<cv::Point> rect)
 {
     int rectInfo[4] = {0};
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
     rectInfo[0] = minY;
     rectInfo[1] = minX;
     rectInfo[2] = maxY - minY;
     rectInfo[3] = maxX - minX;
     return rectInfo;
 }

// the function draws all the squares in the image
 Mat DetectShapes::drawSquares( Mat& image, const vector<vector<Point> >& squares )
{
     cv::Mat roi1;
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, CV_AA);
        int rect[4],*tmp;
        tmp = findRectInfo(squares[i]);
        for(int j=0;j<4;j++)
            rect[j] = *(tmp+j);
        //if(rect[3]<10 || rect[2]<10) rect[3]=
        cv::Rect roi(rect[1],rect[0],rect[3],rect[2]);

        //image(roi).copyTo(roi1); // copy the region rect1 from the image to roi1
        //imshow("1", roi1);
        cout<<rect[2]<<"， "<<rect[4]<<endl;
        return roi1;
    }


//cout<<squares.size()<<endl;


    imshow(wndname, image);
}




