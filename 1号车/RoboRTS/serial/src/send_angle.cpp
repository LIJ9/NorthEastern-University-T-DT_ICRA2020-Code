#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <std_msgs/Float64MultiArray.h>
double time_init;
double first_angle, first_angle_y,feedback_angle,feedback_angle_y;
double first_angle_ex, first_angle_y_ex,feedback_angle_ex,feedback_angle_y_ex,distance;
 // float out_time;

bool first=true;
void visionCallback(const geometry_msgs::Twist& msg)
{
   ros::Time current_time_now1 = ros::Time::now();
  if(first)
  {
   first_angle=msg.linear.x;
   first_angle_y=msg.linear.y;
   first=0;
  }
   feedback_angle=msg.linear.x-first_angle;
   feedback_angle_y=msg.linear.y-first_angle_y;
  //ROS_INFO_STREAM("msg.linear.x"<<msg.linear.x<<"msg.linear.y"<<msg.linear.y<<"time"<<current_time_now1.toSec()-time_init);
   float out=current_time_now1.toSec()-time_init;
std::ofstream Outfile;
  Outfile.open("/home/tdt/catkin_ws/src/RoboRTS-ros/shoot_feed.txt", std::ios_base::app);
  Outfile<<feedback_angle<<','<<feedback_angle_y<<','<<' '<<' '<<' '<<' '<<' '<< out<<'\n';
  Outfile.close();
}

void mcuCallback(const std_msgs::Float64MultiArray & msg)
{
   ros::Time current_time_now1 = ros::Time::now();
  if(first)
  {
   first_angle_ex=msg.data[0];
   first_angle_y_ex=msg.data[1];
   first=0;
  }
 
   feedback_angle_ex=feedback_angle+first_angle_ex;
   distance=msg.data[5];
   feedback_angle_y_ex=feedback_angle_y+first_angle_y_ex;
  ROS_INFO_STREAM("feedback_angle_ex"<<feedback_angle_ex<<"feedback_angle_y_ex"<< feedback_angle_y_ex<<"time"<<current_time_now1.toSec()-time_init);
   float out=current_time_now1.toSec()-time_init;
std::ofstream Outfile;
  Outfile.open("/home/tdt/catkin_ws/src/RoboRTS-ros/shoot_expect.txt", std::ios_base::app);
  Outfile<<feedback_angle_ex<<' '<<' '<<' '<<' '<<' '<<' '<<feedback_angle_y_ex<<' '<<' '<<' '<<' '<<' '<<' '<<distance<<' '<<' '<<' '<<' '<<' '<<' '<<out<<'\n';
  Outfile.close();
}
int main(int argc,char** argv)
{
	ros::init(argc,argv,"send_angle");//注意节点不要重名
	ros::NodeHandle n;
	  geometry_msgs::Twist gimbal_tdt;
	//ros::Publisher vision_pub = n.advertise<geometry_msgs::Twist>("vision_tomcu", 1000);
	  ros::Subscriber subq_vision=n.subscribe( "vision_tomcu", 1000,mcuCallback);
	 ros::Subscriber sub_vision=n.subscribe( "vision_tonuc", 1000,visionCallback);
time_init=ros::Time::now().toSec();
double x_change,x_in;
double y_change,y_in;
ros::Rate rate(100);
while(ros::ok())
{
  /*double time_now=ros::Time::now().toSec();
  x_change=30*sin(2*(time_now-time_init));
   y_change=15*sin(2*(time_now-time_init));
  x_in=x_change-feedback_angle;
  y_in=y_change-feedback_angle_y;
  //x_last=x_change;
  float out_time=time_now-time_init;
  float out_x=x_change;
  float out_y=y_change;
  //ROS_INFO_STREAM("y_change="<<y_change<<"y_in="<<y_in<<"time="<<out_time);
  std::ofstream Outfile;
  Outfile.open("/home/tdt/catkin_ws/src/RoboRTS-ros/pitch_ex.txt", std::ios_base::app);
  Outfile<<out_y<<' '<<' '<<' '<< out_time<<'\n';
  Outfile.close();
gimbal_tdt.linear.x=0;
gimbal_tdt.linear.y=y_in;
gimbal_tdt.linear.z=1.0;
gimbal_tdt.angular.x=1.0;
vision_pub.publish(gimbal_tdt);
*/
ros::spinOnce();
rate.sleep();
}
return 0;
}

