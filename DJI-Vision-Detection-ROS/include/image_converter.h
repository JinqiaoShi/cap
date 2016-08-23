
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>


	void ros_to_cv(cv::Mat* cvimg, const sensor_msgs::Image::ConstPtr& img);

	void cv_to_ros(const cv::Mat& cvimg, sensor_msgs::ImagePtr& img, int bw);



