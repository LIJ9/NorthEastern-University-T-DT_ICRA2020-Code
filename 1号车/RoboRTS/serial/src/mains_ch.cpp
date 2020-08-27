#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdio>
#include <math.h>
#include "usart_ch.h"
#include <thread>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/types.h>
#include <ros/ros.h>
#include <fcntl.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64MultiArray.h>
using namespace std;
using namespace cv;

class Class_Usart usart;
vector<int> data;
int swing;
void serial_thread();//串口函数
std::mutex serial_mutex;//某个锁
int x_vel=0,y_vel=0,z_vel=0;
int x_ad_vel=0,y_ad_vel=0,z_ad_vel=0;
double secs=0,secs_last;
geometry_msgs::Twist mcu_msgs;
void DicisionToRefereeCallback(const std_msgs::Float64MultiArray::ConstPtr& msg )
{
  //ROS_WARN_STREAM("aaaaaaaaaaaaaaaaaaaaaaaaa");
  int my_position_x= (int)(msg->data[0]*100);
  ROS_WARN_STREAM("my_position_x"<<my_position_x);
  int my_position_y=msg->data[1]*100;
  int enemy_position_x=msg->data[2]*100;
  int enemy_position_y=msg->data[3]*100;
  int bufflet=msg->data[4];
  int bufflet_sign=msg->data[5];
  int buff_sign=msg->data[6];
  int remain_blood=msg->data[7];//remain_blood
  usart.Usart_Send(0xbb,my_position_x,my_position_y,enemy_position_x,enemy_position_y,bufflet,bufflet_sign,buff_sign,remain_blood);//自己——x,自己-y,敌人-x,补弹标志，加成标志，
}
int main(int argc,char** argv)
{
	ros::init(argc,argv,"serial_ch");
	ros::NodeHandle n;
  	// 创建一个Publisher，发布名为chatter的topic，消息类型为geometry_msgs::Twist
	usart.Usart_Init();
 ros::Subscriber communicate= n.subscribe("DecisionToReferee", 2,DicisionToRefereeCallback);//位置？
	ros::spin();
//}
return 0;
}

