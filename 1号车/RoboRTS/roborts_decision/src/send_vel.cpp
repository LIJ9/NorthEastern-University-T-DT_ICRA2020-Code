#include<ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <iostream>
#include <std_msgs/Float64MultiArray.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <geometry_msgs/Twist.h>
using namespace std;
ros::Publisher chatter_pub;
geometry_msgs::Twist cmd_vel_simulate;
int i;
int j;
void simulate_vel()
{
  if(i > 2)
  {
    cmd_vel_simulate.angular.z = -cmd_vel_simulate.angular.z;
    i = 0;
  }
  if(j > 4)
  {
    cmd_vel_simulate.linear.y = -cmd_vel_simulate.linear.y;
    j = 0;
  }
  j++;
  i++;
  chatter_pub.publish(cmd_vel_simulate);
}
int main(int argc, char**argv)
{
  ros::init(argc, argv, "send_vel");
  ros::NodeHandle nh;
  chatter_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
  cmd_vel_simulate.linear.y = 0.5;
  cmd_vel_simulate.angular.z = 1.5;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    simulate_vel();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}