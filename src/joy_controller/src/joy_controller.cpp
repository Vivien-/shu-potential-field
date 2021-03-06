#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <ardrone_autonomy/Navdata.h>

class JoyClass {
 public:
  JoyClass();

 private:
  ros::NodeHandle nh;
  ros::Publisher vel_pub;
  ros::Publisher takeoff;
  ros::Publisher land;
  ros::Subscriber joy_sub; 
  ros::Subscriber pos_sub;

  void setVelCmd(const sensor_msgs::Joy::ConstPtr &joy, geometry_msgs::Twist &msg);
  void joyCallBack(const sensor_msgs::Joy::ConstPtr &joy);
  void positionControl(const ardrone_autonomy::Navdata &datas);
};

JoyClass::JoyClass() {
  joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 100, &JoyClass::joyCallBack, this);
  pos_sub = nh.subscribe("/ardrone/navdata", 100, &JoyClass::positionControl, this);

  vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  takeoff = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 100);
  land = nh.advertise<std_msgs::Empty>("/ardrone/land", 100);
}

void JoyClass::setVelCmd(const sensor_msgs::Joy::ConstPtr &joy, geometry_msgs::Twist &msgCmd){
  msgCmd.linear.x = joy->axes[1] > -0.2 && joy->axes[1] < 0.2 ? 0 : joy->axes[1];
  msgCmd.linear.y = joy->axes[0] > -0.2 && joy->axes[0] < 0.2 ? 0 : joy->axes[0];
  msgCmd.linear.z = joy->axes[2] > -0.2 && joy->axes[2] < 0.2 ? 0 : joy->axes[2];
}

void JoyClass::positionControl(const ardrone_autonomy::Navdata &datas){
  //  ROS_INFO("Height: %d\n", datas.altd);
}


void JoyClass::joyCallBack(const sensor_msgs::Joy::ConstPtr &joy){
  for(int i = 0; i < 3; ++i)
    ROS_INFO("axes[%d] = %f\n", i, joy->axes[i]);
  ROS_INFO("\t----------- Msgs: %ud -----------\n", joy->header.seq);

  geometry_msgs::Twist msgCmd;
  std_msgs::Empty landCmd, takeoffCmd;

  if(joy->buttons[0])
    takeoff.publish(takeoffCmd);
  else if(joy->buttons[1])
    land.publish(landCmd);
  
  else {
    setVelCmd(joy, msgCmd);  
    vel_pub.publish(msgCmd);
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "joy_controller");
  JoyClass jc;
  ros::spin();
  return 0;
}
