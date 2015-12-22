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
const bool SIMULATION = true;

const string OPENCV_WINDOW = "Image window";
const float  DISTANCE_TO_GOAL = 1.0;
const float  CIRCLE_RADIUS = 0.5;
const float  CAMERA_HFOV = 92;
const float  EPSILON_VALUE = 0.5;

typedef struct Obstacle {
  float radius; // rayon
  float security; // security distance
  Point position;
  float cautiousness;
}Obstacle;

void setVectorValue(float x, float y, float z, geometry_msgs::Vector3 &vec){
  vec.x = x; vec.y = y; vec.z = z;
}

bool isBetween(Point a, Point b, Obstacle o) {
  Point c(o.position);
  float epsilon = o.radius * sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
  float crossproduct = (c.y - a.y) * (b.x - a.x) - (c.x - a.x) * (b.y - a.y);
  if (abs(crossproduct) > epsilon)
    return false;
  float dotproduct = (c.x - a.x) * (b.x - a.x) + (c.y - a.y) * (b.y - a.y);
  if(dotproduct < 0)
    return false;
  
  float squaredlengthba = (b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y);
  if(dotproduct > squaredlengthba)
    return false;
  return true;
}

class PotentialField {
public:
  PotentialField();
  PotentialField(vector<geometry_msgs::Vector3> goals, vector<Obstacle> obstacles);
  ~PotentialField(){
    destroyWindow(OPENCV_WINDOW);
  }
  
private:
  ros::NodeHandle _nh;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  geometry_msgs::Vector3 _localPos;
  geometry_msgs::Vector3 _goal;

  /** Publisher **/
  ros::Publisher _vel_pub;
  ros::Publisher _land;
  ros::Publisher _takeoff;

  /** Subscriber **/
  ros::Subscriber _pos_sub;
  ros::Subscriber _joy_sub;
  
  float prevAngularTarget;
  float _yaw;
  float _epsilon;
  bool _hasRedCircle;
  bool _searchRedCircle;
  bool _hasRedGoal;
  bool _stop;
  bool _automatic_control;
  vector<geometry_msgs::Vector3> _goals;
  vector<Obstacle> _obstacles;
  int _goalId;
  
  void setVelCmd(const sensor_msgs::Joy::ConstPtr &joy, geometry_msgs::Twist &msg);
  void joyCallBack(const sensor_msgs::Joy::ConstPtr &joy);
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  void updatePosition(const tf2_msgs::TFMessage &data);
  void updateEnvironment(geometry_msgs::Twist &nextVel);
  void setPotentialByPos(int i, int j);
  void sendNextVel();
  void stop();
};

PotentialField::PotentialField(): it_(_nh) {
  image_sub_ = it_.subscribe("/ardrone/bottom/image_raw", 1, &PotentialField::imageCb, this);
  image_pub_ = it_.advertise("/image_converter/output_video", 1);
  namedWindow(OPENCV_WINDOW);
  
  _localPos.x = _localPos.y = _localPos.z = _yaw = prevAngularTarget = 0;
  _epsilon = EPSILON_VALUE;
  _searchRedCircle = false;
  _hasRedCircle = false;
  _hasRedGoal = false;
  _stop = false;
  _automatic_control = false;

  _pos_sub = _nh.subscribe("/tf", 2, &PotentialField::updatePosition, this);
  _vel_pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  _joy_sub = _nh.subscribe<sensor_msgs::Joy>("/joy", 100, &PotentialField::joyCallBack, this);
  _land = _nh.advertise<std_msgs::Empty>("/ardrone/land", 1);
  _takeoff = _nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 100);
}

PotentialField::PotentialField(vector<geometry_msgs::Vector3> goals, vector<Obstacle> obstacles): _goals(goals), it_(_nh), _obstacles(obstacles){
  image_sub_ = it_.subscribe("/ardrone/bottom/image_raw", 1, &PotentialField::imageCb, this);
  image_pub_ = it_.advertise("/image_converter/output_video", 1);
  namedWindow(OPENCV_WINDOW);
  
  _localPos.x = _localPos.y = _localPos.z = _yaw = prevAngularTarget = _goalId = 0;
  _epsilon = EPSILON_VALUE;
  _searchRedCircle = false;
  _hasRedCircle = false;
  _hasRedGoal = false;
  _stop = false;
  _automatic_control = false;
  if(_goals.size())
    _goal = goals[0];
 
  _pos_sub = _nh.subscribe("/tf", 2, &PotentialField::updatePosition, this);
  _vel_pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  _joy_sub = _nh.subscribe<sensor_msgs::Joy>("/joy", 100, &PotentialField::joyCallBack, this);
  _land = _nh.advertise<std_msgs::Empty>("/ardrone/land", 1);
  _takeoff = _nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 100);
}

void PotentialField::setVelCmd(const sensor_msgs::Joy::ConstPtr &joy, geometry_msgs::Twist &msgCmd){
  msgCmd.linear.x = joy->axes[1] > -0.2 && joy->axes[1] < 0.2 ? 0 : joy->axes[1];
  msgCmd.linear.y = joy->axes[0] > -0.2 && joy->axes[0] < 0.2 ? 0 : joy->axes[0];
  msgCmd.linear.z = joy->axes[2] > -0.2 && joy->axes[2] < 0.2 ? 0 : joy->axes[2];
}

void PotentialField::imageCb(const sensor_msgs::ImageConstPtr& msg) {
  if(_searchRedCircle){
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    Mat bgr_image = cv_ptr->image.clone();
    Mat orig_image = cv_ptr->image.clone();
    medianBlur(bgr_image, bgr_image, 3);
    Mat hsv_image;
    cvtColor(bgr_image, hsv_image, COLOR_BGR2HSV); // Convert input image to HSV
    Mat lower_red_hue_range;
    Mat upper_red_hue_range;
    inRange(hsv_image, Scalar(0, 100, 100), Scalar(10, 255, 255), lower_red_hue_range); // Threshold the HSV image, keep only the red pixels
    inRange(hsv_image, Scalar(160, 100, 100), Scalar(179, 255, 255), upper_red_hue_range);
    Mat red_hue_image;
    addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);    // Combine the above two images
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

    int size_pixels =  orig_image.cols;
    float size_meters = tan((CAMERA_HFOV / 2) * M_PI / 180) * _localPos.z * 2; // verif ok
    float conversion = size_meters / size_pixels;
    
    if(_hasRedCircle){
      Point center(round(circles[trueCircleId][0]), round(circles[trueCircleId][1]));
      circle(orig_image, center, pixelCircleRadius, Scalar(216, 176, 123), 5);
      circle(orig_image, center, 5, Scalar(0, 255, 0), 5);
      distToCenter = sqrt((center.x - orig_image.cols/2)*(center.x - orig_image.cols/2) + (center.y - orig_image.rows/2)*(center.y - orig_image.rows/2));
      float approx = 0.3;
      if(distToCenter * conversion < approx){
	_stop = true;
	_goals.clear();
	stop();
	_goal.x = _localPos.x;
	_goal.y = _localPos.y;
	std_msgs::Empty landCmd;
	_land.publish(landCmd);
	ROS_INFO("--- Landing in position (%f, %f) with distance To Center %f---\n", _localPos.x, _localPos.y, distToCenter * conversion); 
	return;
      }
    }

    if(_hasRedCircle && !_hasRedGoal){
      _hasRedGoal = true;
      pixelCircleRadius = round(circles[trueCircleId][2]);
      Point trueCircleCenter(round(circles[trueCircleId][0]), round(circles[trueCircleId][1]));
      
      float diffX = (trueCircleCenter.x - orig_image.cols / 2);
      float diffY = (orig_image.rows / 2 - trueCircleCenter.y);

      goal.x = _localPos.x + (diffX * cos(_yaw * M_PI/180 - M_PI/2) + diffY * sin(_yaw * M_PI/180 + M_PI/2)) * conversion;
      goal.y = _localPos.y + (-diffX * sin(_yaw * M_PI/180 + M_PI/2) + diffY * cos(_yaw * M_PI/180 - M_PI/2)) * conversion;
      _goals.clear();
      _goal = goal;
      _goals.push_back(goal);
      _goalId = 0;
    }
    _goal.z = goal.z;
    imshow("Detected red circles on the input image", orig_image);
    waitKey(3); // Update GUI Window
  }
}

void PotentialField::stop() {
  geometry_msgs::Twist nextVel;
  nextVel.angular.x = 0;
  nextVel.linear.y = 0;
  nextVel.linear.z = - _localPos.z;
  _vel_pub.publish(nextVel);
}

void PotentialField::joyCallBack(const sensor_msgs::Joy::ConstPtr &joy){
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

void PotentialField::updatePosition(const tf2_msgs::TFMessage &data) {
  if(SIMULATION){
    if(data.transforms.size() == 3){
      _localPos.x = data.transforms[0].transform.translation.x;
      _localPos.y = data.transforms[0].transform.translation.y;
      _localPos.z = data.transforms[1].transform.translation.z;
      _yaw = tf::getYaw(data.transforms[0].transform.rotation) * 180/M_PI;
    }
  } else {
    if(data.transforms[0].child_frame_id.compare("ardrone_base_link") == 0){
      _localPos.x = data.transforms[0].transform.translation.x;
      _localPos.y = data.transforms[0].transform.translation.y;
      _localPos.z = data.transforms[0].transform.translation.z;
      _yaw = tf::getYaw(data.transforms[0].transform.rotation) * 180/M_PI;
      // printf("x:%f\ty:%f\tz:%f\tangle:%f°\n", _localPos.x, _localPos.y, _localPos.z, _yaw);
      float dist = sqrt((_localPos.x - _goal.x)*(_localPos.x - _goal.x) + (_localPos.y - _goal.y)*(_localPos.y - _goal.y));
      if(PRINT_DATAS){
	if(n%10 == 0)
	  printf("x:%f y:%f distToGoal:%f _yaw:%f\n",_localPos.x, _localPos.y,dist, _yaw);
      }
    }
  }

  if(_automatic_control){
    if(_goals.size() > _goalId)
      sendNextVel();
    else if((_goals.size() <= _goalId || _searchRedCircle) && !_stop) {
      _searchRedCircle = true;
      sendNextVel();
    }
  }
}

void PotentialField::updateEnvironment(geometry_msgs::Twist &nextVel){
  for(int k = 0; k < _obstacles.size(); k++){
    int x = _obstacles[k].position.x;
    int y = _obstacles[k].position.y;
    int s = _obstacles[k].security;
    int r = _obstacles[k].radius;
    float cautiousness = _obstacles[k].cautiousness;

    float d = sqrt((x - _localPos.x)*(x - _localPos.x) + (y - _localPos.y)*(y - _localPos.y));

    int position = (_goal.x - x)*(_localPos.y - y) - (_goal.y - y)*(_localPos.x - x);
    float angle = position < 0 ? M_PI/4 : -M_PI/4;

    float theta = atan2(_localPos.y - y, _localPos.x - x) + angle;
    Point local2DPos(_localPos.x, _localPos.y);
    Point goal2DPos(_goal.x, _goal.y);
    
    if(!isBetween(local2DPos, goal2DPos, _obstacles[k])) {
      cautiousness = 0;
    }

    if(d > s + r) continue; // out of range, do nothing
    else if(d < r) { // inside the object
      nextVel.linear.x += INT_MIN * cos(theta);
      nextVel.linear.y += INT_MIN * sin(theta);
    }
    else if(d >= r && d <= s + r){
      nextVel.linear.x += cautiousness * _epsilon * (s*cautiousness+r-d) * cos(theta);
      nextVel.linear.y += cautiousness * _epsilon * (s*cautiousness+r-d) * sin(theta);
      prevAngularTarget = theta * 180/M_PI;
    }
  }
}

void PotentialField::sendNextVel() {
  float dist = sqrt((_localPos.x - _goal.x)*(_localPos.x - _goal.x) + (_localPos.y - _goal.y)*(_localPos.y - _goal.y));
  float theta = atan2(_goal.y - _localPos.y, _goal.x - _localPos.x);
    
  geometry_msgs::Twist dummy;
  geometry_msgs::Twist &nextVel(dummy);
    
  nextVel.linear.x = _epsilon * dist * cos(theta);
  nextVel.linear.y = _epsilon * dist * sin(theta);
  updateEnvironment(nextVel);
    
  float angularTarget = 0;
  if(nextVel.linear.x != 0){
    angularTarget = atan2(nextVel.linear.y, nextVel.linear.x) * 180/M_PI;
    prevAngularTarget = angularTarget;
  }
  else
    angularTarget = prevAngularTarget;
    
  float delta = atan2(nextVel.linear.y, nextVel.linear.x);
  nextVel.linear.x *= cos(delta);
  nextVel.linear.y *= sin(delta);

  float diff = angularTarget - _yaw;
  nextVel.linear.z = _localPos.z > 1.5 ? -0.5 : _localPos.z < 0.5 ? 0.5 : 0;

  if(fabs(diff) > 2 ){
    if(fabs(diff) < 180)
      nextVel.angular.z = diff;
    else if(diff > 180 || diff < -180)
      nextVel.angular.z = - diff;
    nextVel.linear.x = dist * cos(diff * M_PI/180);
  } else{
    nextVel.angular.z = 0;
    nextVel.linear.y = 0;
    nextVel.linear.x = dist*dist;
  }

  if(_searchRedCircle && !_hasRedCircle && !_hasRedGoal) {
    nextVel.linear.z = 1;
    nextVel.linear.y = 0;
    nextVel.linear.x = 0;
    nextVel.angular.z = 0;
  }

  if(_searchRedCircle && !_hasRedCircle && !_hasRedGoal){
    if(fabs(_yaw) > 5){
      if(fabs(_yaw) < 180)
	nextVel.angular.z = diff;
      else if(_yaw > 180 || _yaw < -180)
	nextVel.angular.z = - _yaw;
    } else{
      nextVel.angular.z = 0;
      nextVel.linear.y = 0;
      nextVel.linear.x = dist;
    }
  }

  float dist_to_goal = _searchRedCircle ? 0.05 :  DISTANCE_TO_GOAL;
  if(dist <= dist_to_goal){
    _hasRedGoal = false;
    if(++_goalId < _goals.size()){
      ROS_INFO("Target %d reached", _goalId);
      _goal = _goals[_goalId];
      if(_searchRedCircle){
	_goals.pop_back();
	_goalId--;
      }
    }
  }
  if(PRINT_DATAS){
    n++;
    if(n%10 == 0)
      printf("x:%f y:%f nextX: %f nextY: %f distToGoal: %f _yaw: %f\n",_localPos.x, _localPos.y, nextVel.linear.x, nextVel.linear.y, dist, _yaw);
  }
  _vel_pub.publish(nextVel);
}

int main(int argc, char **argv){
  int pos = 3;

  ros::init(argc, argv, "potential_field");
  vector<geometry_msgs::Vector3>  goals;
  geometry_msgs::Vector3 goal;
  Obstacle cylinder;
  vector<Obstacle> obstacles;

  goal.z = 1.0;

  /*
   * Old square style
   */
  // for(int i = -pos; i <= pos; i+=2*pos){
  //   for(int j = -pos; j <= pos; j+=2*pos){
  //     goal.x = i; goal.y = j;
  //     goals.push_back(goal);
  //   }
  // }

  /*
   * New square style
   */
  // 
  goal.x = -pos; goal.y = -pos; goals.push_back(goal);
  goal.x = -pos; goal.y = pos; goals.push_back(goal);
  goal.x = pos; goal.y = pos; goals.push_back(goal);
  goal.x = pos; goal.y = -pos; goals.push_back(goal);
  goal.x = 0; goal.y = 0; goals.push_back(goal);
  
  /*
   *  Line style
   */
  // goal.x = 0.5; goal.y = 0.5; goals.push_back(goal);
  // goals.push_back(goal);
  // goal.x = 0; goal.y = 0; goals.push_back(goal);

  /*
   * Obstacle(3,0)
   */
  // cylinder.security = 2;
  // cylinder.radius = 1;
  // cylinder.cautiousness = 2.25;
  // cylinder.position.x = 3;
  // cylinder.position.y = 0;
  // obstacles.push_back(cylinder);
  

  for(int i = -6; i <= 6; i+=12){
    for(int j = -6; j <= 6; j+=12){
      cylinder.security = 3;
      cylinder.radius = 2;
      cylinder.cautiousness = 2.25;
      cylinder.position.x = i;
      cylinder.position.y = j;
      obstacles.push_back(cylinder);
    }
  }
  PotentialField jc(goals, obstacles);
  ros::spin();
  return 0;
}
