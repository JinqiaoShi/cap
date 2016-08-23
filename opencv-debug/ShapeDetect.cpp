#include "ShapeDetect.h"

using namespace cv;
using namespace std;

#define SHOW_IMAGE

int thresh = 50, N = 11;
const char *wndname = "Square Detection Demo";

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2

int ShapeDetect::graylevel(Mat image)//求取圆形区域内的平均灰度值
{
    int graysum = 0, n = 0;

    for (int i = 0; i <= image.rows; ++i)//访问矩形框内的像素值
    {
        uchar *data = image.ptr<uchar>(i);
        for (int j = 0; j <= image.cols; ++j) {

            ++n;
            graysum += (int) data[j];

        }
    }


    return (graysum / n);
}

bool ShapeDetect::isBox(Mat img, std::vector<cv::Point> rect) {
    const int channels[1] = {0};

    const int histSize[1] = {256};

    float hranges[2] = {0, 255};

    const float *ranges[1] = {hranges};

    MatND hist;


    int x[4] = {0}, y[4] = {0};
    int maxX = 0, maxY = 0, minX = 2000, minY = 2000;
    //get the rect points
    for (int i = 0; i < 4; i++) {
        x[i] = rect[i].x;
        y[i] = rect[i].y;
        if (maxX < x[i])
            maxX = x[i];
        if (maxY < y[i])
            maxY = y[i];
        if (minX > x[i])
            minX = x[i];
        if (minY > y[i])
            minY = y[i];
    }

    Rect rectROI(minX, minY, maxX - minX, maxY - minY);
    Mat srcROI = img(rectROI);

    Mat imgHSV, imgHSV_H;
    vector<Mat> imgHSV_split;
    cvtColor(srcROI, imgHSV, 0);
    split(imgHSV, imgHSV_split);
    imgHSV_H = imgHSV_split[2];
    inRange(imgHSV_H, 33, 255, imgHSV_H);//

//   cout<<"graylevel"<<graylevel(imgHSV_H)<<endl;

    imshow("srcROI", imgHSV_H);
    if (graylevel(imgHSV_H) > 30) return 0;
    else return 1;


}


double ShapeDetect::angle(Point pt1, Point pt2, Point pt0) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    double ratio;//边长平方的比
    ratio = (dx1 * dx1 + dy1 * dy1) / (dx2 * dx2 + dy2 * dy2);
    if (ratio < 0.6 || 1.8 < ratio) {//根据边长平方的比过小或过大提前淘汰这个四边形，如果淘汰过多，调整此比例数字
        //      Log("ratio\n");
        return 1.0;//根据边长平方的比过小或过大提前淘汰这个四边形
    }
    //TODO: Filter the boxes with size, calculated from UAV height.

    return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}


// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
void ShapeDetect::findSquares(const Mat &image, vector<vector<Point>> &squares) {
    squares.clear();

    Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, Size(image.cols / 2, image.rows / 2));
    pyrUp(pyr, timg, image.size());
    vector<vector<Point>> contours;

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

            vector<Point> approx;

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
                if (approx.size() == 4 && fabs(contourArea(Mat(approx))) < 12000 &&
                    fabs(contourArea(Mat(approx))) > 2500 &&
                    isContourConvex(Mat(approx))) {
                    double maxCosine = 0;

                    for (int j = 2; j < 5; j++) {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                        maxCosine = MAX(maxCosine, cosine);
//                        for  (int k = 1; k < approx.size(); k++){
//                            cout << "POINT = " << k << ", " << approx[k] << endl;
//                        }
//                        approx[2] = approx[2] * 1.2;
//                        approx[4] = approx[4] * 1.2;
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if (maxCosine < 0.35 && isBox(image, approx))
                        squares.push_back(approx);
                }
            }
        }
    }
}


// the function draws all the squares in the image
Mat ShapeDetect::drawSquares(Mat &image, const vector<vector<Point> > &squares) {
    cv::Mat roi1;
    for (size_t i = 0; i < squares.size(); i++) {
        const Point *p = &squares[i][0];
        int n = (int) squares[i].size();
        polylines(image, &p, &n, 1, true, Scalar(0, 255, 0), 3, CV_AA);
        int rect[4], *tmp;
        tmp = findRectInfo(squares[i]);
        for (int j = 0; j < 4; j++)
            rect[j] = *(tmp + j);
        //if(rect[3]<10 || rect[2]<10) rect[3]=
        cv::Rect roi(rect[1], rect[0], rect[3], rect[2]);

        //image(roi).copyTo(roi1); // copy the region rect1 from the image to roi1
        //imshow("1", roi1);
        cout << rect[2] << "， " << rect[4] << endl;
        return roi1;
    }
}

int ShapeDetect::findRectInfo(std::vector<cv::Point> rect) {
    int rectInfo[3] = {0};
    int x[4] = {0}, y[4] = {0};
    int maxX = 0, maxY = 0, minX = 2000, minY = 2000;
    //get the rect points
    for (int i = 0; i < 4; i++) {
        x[i] = rect[i].x;
        y[i] = rect[i].y;
        if (maxX < x[i])
            maxX = x[i];
        if (maxY < y[i])
            maxY = y[i];
        if (minX > x[i])
            minX = x[i];
        if (minY > y[i])
            minY = y[i];
    }
    rectInfo[0] = minY;
    rectInfo[1] = minX;
    rectInfo[2] = maxY - minY;
    rectInfo[3] = maxX - minX;
    return rectInfo[3];
}


bool ShapeDetect::checkBbox(vector<vector<Point>> bbox) {
    Size imgSize(640, 360);
    if (bbox.empty()) {
        cout << "┑(￣Д ￣)┍" << endl;
//TODO: Implement a mechanism to retrieve last good result of box detection when no box is found.
    } else {
        for (int i = 0; i < bbox.size(); i++) {

            cout << bbox[i] << endl;
            cout << "SIZE = " << bbox.size() << endl;

            for (int j = 0; j < bbox[i].size(); j++) {
                if (bbox[i][j].x < imgSize.width && bbox[i][j].y < imgSize.height) {


//This is outputting tuple of all detected bundling box.
                }
            }
        }
    }
    return 0;
}


vector<vector<Point>> ShapeDetect::drawContoursROI(Mat imgBinary, Mat imgOriginal, int minArea) {
    vector<vector<Point>> contours;
    vector<Vec4i> he;
    findContours(imgBinary, contours, he, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    vector<vector<Point>> contoursROI;
    for (int i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
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


Mat ShapeDetect::imgMorphing(Mat &imgHSV_Morphed, int kerOpenSize, int kerCloseSize) {


    Mat kernelOpen = getStructuringElement(0, Size(kerOpenSize, kerOpenSize));
    morphologyEx(imgHSV_Morphed, imgHSV_Morphed, MORPH_OPEN, kernelOpen, Point(-1, -1), 2);
    Mat kernelClose = getStructuringElement(0, Size(kerCloseSize, kerCloseSize));

    morphologyEx(imgHSV_Morphed, imgHSV_Morphed, MORPH_CLOSE, kernelClose, Point(-1, -1), 2);
    return imgHSV_Morphed;
}


///Input: 1. imgMat 2. hsvRange for box you want to detect 3. Morphing Openning kernel size 4. Closing kernel size.
vector<vector<Point>> ShapeDetect::detectNineSquare(Mat &imgOrig, int hsvRange[2], int kerOpenSize, int kerCloseSize) {
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
    result = drawContoursROI(imgContoured, imgWithContour);
    return result;


#ifdef SHOW_IMAGE
    //        imshow("HSV", imgHSV);
imshow("HSV_Channel2", imgHSV_H);
imshow("HSV_Channel2_Morphed", imgHSV_Morphed);
imshow("HSV_Channel2_thresholded", imgHSV_thresholded);
imshow("Final", imgWithContour);
#endif

}


vector<vector<Point>> ShapeDetect::detectDoll(Mat &imgOrig, int hsvRange[6], int kerOpenSize, int kerCloseSize) {
    //TODO: Write param handling different color choice and color range.
///Prepare img Mat.
    Mat imgHSV, imgHSV_H, imgHSV_thresholded, imgHSV_Morphed;
    vector<vector<Point>> result;
    cvtColor(imgOrig, imgHSV, CV_BGR2HSV);


///Filter img with color range.
    //TODO: Add exposure adjustment process.
    inRange(imgHSV, Scalar(hsvRange[0], hsvRange[1], hsvRange[2]), Scalar(hsvRange[3], hsvRange[4], hsvRange[5]),
            imgHSV_thresholded); //Threshold the image

    imgHSV_thresholded.copyTo(imgHSV_Morphed);
    imgMorphing(imgHSV_Morphed, kerOpenSize, kerCloseSize);

    Mat imgContoured;
    imgHSV_Morphed.copyTo(imgContoured);
    Mat imgWithContour;
    imgOrig.copyTo(imgWithContour);
    result = drawContoursROI(imgContoured, imgWithContour, 2000);
//    checkBbox(result);


#ifdef SHOW_IMAGE
    //        imshow("HSV", imgHSV);
imshow("HSV_Channel2_Morphed", imgHSV_Morphed);
imshow("HSV_Channel2_thresholded", imgHSV_thresholded);
imshow("Final", imgWithContour);
#endif

    return result;
}



