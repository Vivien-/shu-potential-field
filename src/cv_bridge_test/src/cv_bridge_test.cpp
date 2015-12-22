#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>

using namespace cv;
using namespace std;

static const string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ardrone/bottom/image_raw", 1, 
			       &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
      {
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
    catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
      }

    Mat bgr_image = cv_ptr->image.clone();
    Mat orig_image = cv_ptr->image.clone();

    medianBlur(bgr_image, bgr_image, 3);

    // Convert input image to HSV
    Mat hsv_image;
    cvtColor(bgr_image, hsv_image, COLOR_BGR2HSV);

    // Threshold the HSV image, keep only the red pixels
    Mat lower_red_hue_range;
    Mat upper_red_hue_range;
    inRange(hsv_image, Scalar(0, 100, 100), Scalar(10, 255, 255), lower_red_hue_range);
    inRange(hsv_image, Scalar(160, 100, 100), Scalar(179, 255, 255), upper_red_hue_range);

    // Combine the above two images
    Mat red_hue_image;
    addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);

    GaussianBlur(red_hue_image, red_hue_image, Size(9, 9), 2, 2);

    // Use the Hough transform to detect circles in the combined threshold image
    vector<Vec3f> circles;
    HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 20, 0, 0);

    // Loop over all detected circles and outline them on the original image
    if(circles.size() == 0) 
      cout<<"No circles"<<endl;
    for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
      Point center(round(circles[current_circle][0]), round(circles[current_circle][1]));
      int radius = round(circles[current_circle][2]);

      circle(orig_image, center, radius, Scalar(0, 255, 0), 5);
    }

    // Show images
    imshow("Detected red circles on the input image", orig_image);




    // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   circle(cv_ptr->image, Point(50, 50), 10, CV_RGB(255,0,0));

    // // Update GUI Window
    waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
