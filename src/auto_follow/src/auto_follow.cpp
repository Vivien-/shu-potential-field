#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3.h>
#include <ardrone_autonomy/Navdata.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <string>
#include <cmath>

using namespace cv;
using namespace std;

int n = 0;
const bool PRINT_DATAS = true;
const bool SIMULATION = false;

const string OPENCV_WINDOW = "Image window";
const float  DISTANCE_TO_GOAL = 1.0;
const float  CIRCLE_RADIUS = 0.12;
const float  CAMERA_HFOV = 92;
const int    RADIUS_GOAL = 55;

typedef struct Obstacle {
  float radius; // rayon
  float security; // security distance
  Point position;
  float cautiousness;
}Obstacle;

class AutoFollow {
public:
  AutoFollow();
  AutoFollow(vector<Obstacle> obstacles);
  ~AutoFollow(){
    destroyWindow(OPENCV_WINDOW);
  }
  
private:
  ros::NodeHandle _nh;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  /** Publisher **/
  ros::Publisher _vel_pub;
  ros::Publisher _land;
  ros::Publisher _takeoff;

  /** Subscriber **/
  ros::Subscriber _joy_sub;
  
  bool _hasRedCircle;
  bool _searchRedCircle;
  bool _hasRedGoal;
  bool _stop;
  bool _automatic_control;
  vector<Obstacle> _obstacles;
  int _currentRadius;
  
  void setVelCmd(const sensor_msgs::Joy::ConstPtr &joy, geometry_msgs::Twist &msg);
  void joyCallBack(const sensor_msgs::Joy::ConstPtr &joy);
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  void sendNextVel(Point center, Mat orig_image);
  void stop();



  int lower_lowH;
  int lower_highH;
  int lower_lowS ;
  int lower_highS ;
  int lower_lowV;
  int lower_highV;

  int upper_lowH ;
  int upper_highH;
  int upper_lowS;
  int upper_highS;
  int upper_lowV;
  int upper_highV;
};

AutoFollow::AutoFollow(vector<Obstacle> obstacles): it_(_nh){
  image_sub_ = it_.subscribe("/ardrone/front/image_raw", 1, &AutoFollow::imageCb, this);
  image_pub_ = it_.advertise("/image_converter/output_video", 1);
  namedWindow(OPENCV_WINDOW);
  
  _searchRedCircle = true;
  _hasRedCircle = false;
  _hasRedGoal = false;
  _stop = false;
  _automatic_control = false;
  _currentRadius = 0;
 
  _vel_pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  _joy_sub = _nh.subscribe<sensor_msgs::Joy>("/joy", 100, &AutoFollow::joyCallBack, this);
  _land = _nh.advertise<std_msgs::Empty>("/ardrone/land", 1);
  _takeoff = _nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 100);

   lower_lowH = 84;
   lower_highH = 0;
   lower_lowS = 100;
   lower_highS = 255;
   lower_lowV = 100;
   lower_highV = 255;

   upper_lowH = 160;
   upper_highH = 179;
   upper_lowS = 159;
   upper_highS = 255;
   upper_lowV = 100;
   upper_highV = 255;
}

void AutoFollow::setVelCmd(const sensor_msgs::Joy::ConstPtr &joy, geometry_msgs::Twist &msgCmd){
  msgCmd.linear.x = joy->axes[1] > -0.2 && joy->axes[1] < 0.2 ? 0 : joy->axes[1] / 10;
  msgCmd.linear.y = joy->axes[0] > -0.2 && joy->axes[0] < 0.2 ? 0 : joy->axes[0] / 10;
  msgCmd.linear.z = joy->axes[2] > -0.2 && joy->axes[2] < 0.2 ? 0 : joy->axes[2] / 5;
  printf("msgCmd.linear.x: %f\n", msgCmd.linear.x);
}

void AutoFollow::imageCb(const sensor_msgs::ImageConstPtr& msg) {
  if(_searchRedCircle){
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    //Create trackbars in "Control" window
    cvCreateTrackbar("lower_lowH", "Control", &lower_lowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("lower_lowS", "Control", &lower_lowS, 255);
    cvCreateTrackbar("lower_lowV", "Control", &lower_lowV, 255);

    cvCreateTrackbar("lower_highH", "Control", &lower_highH, 179);
    cvCreateTrackbar("lower_highS", "Control", &lower_highS, 255);
    cvCreateTrackbar("lower_highV", "Control", &lower_highV, 255);

    cvCreateTrackbar("upper_lowH", "Control", &upper_lowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("upper_lowS", "Control", &upper_lowS, 255);
    cvCreateTrackbar("upper_lowV", "Control", &upper_lowV, 255);

    cvCreateTrackbar("upper_highH", "Control", &upper_highH, 179);
    cvCreateTrackbar("upper_highS", "Control", &upper_highS, 255);
    cvCreateTrackbar("upper_highV", "Control", &upper_highV, 255);

    Mat bgr_image = cv_ptr->image.clone();
    Mat orig_image = cv_ptr->image.clone();
    medianBlur(bgr_image, bgr_image, 3);
    Mat hsv_image;
    cvtColor(bgr_image, hsv_image, COLOR_BGR2HSV); // Convert input image to HSV
    Mat lower_red_hue_range;
    Mat upper_red_hue_range;
    inRange(hsv_image, Scalar(lower_lowH, lower_lowS, lower_lowV), Scalar(lower_highH, lower_highS, lower_highV), lower_red_hue_range);    // Threshold the HSV image, keep only the red pixels
    inRange(hsv_image, Scalar(upper_lowH, upper_lowS, upper_lowV), Scalar(upper_highH, upper_highS, upper_highV), upper_red_hue_range);
    Mat red_hue_image;
    addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);  
    // Combine the above two images
    GaussianBlur(red_hue_image, red_hue_image, Size(9, 9), 2, 2);
    vector<Vec3f> circles;
    HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 20, 0, 0); // Use the Hough transform to detect circles in the combined threshold image

    _hasRedCircle = circles.size() > 0;

    int pixelCircleRadius = 0;
    int trueCircleId = 0;

    for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
      Point center(round(circles[current_circle][0]), round(circles[current_circle][1]));
      int radius = round(circles[current_circle][2]);
      if(radius >= pixelCircleRadius){
	pixelCircleRadius = radius;
	trueCircleId = current_circle;
      }
    }
    float distToCenter = 0;
    geometry_msgs::Vector3 goal;

    if(_hasRedCircle){
      Point center(round(circles[trueCircleId][0]), round(circles[trueCircleId][1]));
      circle(orig_image, center, pixelCircleRadius, Scalar(255, 0, 0), 5);
      _currentRadius = pixelCircleRadius;
      circle(orig_image, center, 5, Scalar(0, 255, 0), 5);
      sendNextVel(center, orig_image);
    } else {
      Point center(orig_image.cols/2, orig_image.rows/2);
      _currentRadius = RADIUS_GOAL;
      sendNextVel(center, orig_image);
    }

    imshow("Detected red circles on the input image", orig_image);
    waitKey(3); // Update GUI Window
  }
  //printf("radius_goal:%d\t_current_radius:%d\n",RADIUS_GOAL,_currentRadius);
}

void AutoFollow::stop() {
  geometry_msgs::Twist nextVel;
  nextVel.angular.x = 0;
  nextVel.linear.y = 0;
  nextVel.linear.z = - 1;
  _vel_pub.publish(nextVel);
}

void AutoFollow::joyCallBack(const sensor_msgs::Joy::ConstPtr &joy){
  geometry_msgs::Twist msgCmd;
  std_msgs::Empty landCmd, takeoffCmd;

  if(joy->buttons[2]){
    _automatic_control = !_automatic_control;
    string flight_type = _automatic_control ? "Automatic flight" : "Manual flight";
    cout<<"Now "<<flight_type<<endl;
  }

  if(!_automatic_control){
    if(joy->buttons[0])
      _takeoff.publish(takeoffCmd);
    if(joy->buttons[1])
      _land.publish(landCmd);
    else {
      setVelCmd(joy, msgCmd);  
      _vel_pub.publish(msgCmd);
    }
  }
}

void AutoFollow::sendNextVel(Point center, Mat orig_image) {
  if(_automatic_control){
    float distToCenter = sqrt((center.x - orig_image.cols/2)*(center.x - orig_image.cols/2) + (center.y - orig_image.rows/2)*(center.y - orig_image.rows/2));
    int size_pixels =  orig_image.cols;
    float size_meters = tan((CAMERA_HFOV / 2) * M_PI / 180) * 1.2 * 2; 
    float conversion = size_meters / size_pixels;
    geometry_msgs::Twist dummy;
    geometry_msgs::Twist &nextVel(dummy);
    nextVel.angular.z = (orig_image.cols/2 - center.x) * conversion;
    if(distToCenter * conversion < 0.1){
      printf("Too close, distance of %f cm\n", distToCenter * conversion);
      nextVel.angular.z = 0.0;
    }
    
    int approximation = 10;
    if(abs(_currentRadius - RADIUS_GOAL) < approximation)
      nextVel.linear.x = 0;
    else {
      if(RADIUS_GOAL - _currentRadius < 0)
	nextVel.linear.x = -0.15;
      else
	nextVel.linear.x = 0.13;
    }
    
    if(center.y > orig_image.rows/2 + orig_image.rows/8)
      nextVel.linear.z = - 1;
    if(center.y < orig_image.rows/2 - orig_image.rows/8)
      nextVel.linear.z = 1;

    printf("next.x=%f\n",nextVel.linear.x);
   
    nextVel.linear.z = 0;
    _vel_pub.publish(nextVel);
  }
}

int main(int argc, char **argv){
  int pos = 3;
  ros::init(argc, argv, "potential_field");
  vector<Obstacle> obstacles;

  AutoFollow jc(obstacles);
  ros::spin();
  return 0;
}
