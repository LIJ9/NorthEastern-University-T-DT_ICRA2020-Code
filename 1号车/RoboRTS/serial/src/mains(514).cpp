#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdio>
#include <math.h>
#include "usart.h"
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
int add_num;
int rotation;
void serial_thread();//串口函数
std::mutex serial_mutex;//某个锁
int x_vel=0,y_vel=0,z_vel=0;
int x_ad_vel=0,y_ad_vel=0,z_ad_vel=0;
double secs=0,secs_last;
geometry_msgs::Twist mcu_msgs;
int distances=0;

void sendCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  // 将接收到的消息打印出来
 // ROS_INFO("I heard: [%f]", msg->linear.x);
x_vel=msg->linear.x*100;
/*
if(x<0)
{
x=32768-x;//big + data
}
*/
y_vel=-msg->linear.y*100*1.1;/*
if(y<0)
{
y=32768-y;//big + data
}
*/
z_vel=msg->angular.z*100*1.3;

int add_num=msg->angular.x*100*1.3;

 if(x_ad_vel==0)
 {
	usart.fe_Usart_Send(0xfe,x_vel,y_vel,z_vel,0);//头帧，x,y,z速度，跟随，补弹数，
 }
}
void adjustCallback(const geometry_msgs::Twist::ConstPtr& msg_ad)
{
  // 将接收到的消息打印出来
 // ROS_INFO("I heard: [%f]", msg->linear.x);
x_ad_vel=msg_ad->linear.x*100;
/*
if(x<0)
{
x=32768-x;//big + data
}
*/
y_ad_vel=-msg_ad->linear.y*100*1.1;/*
if(y<0)
{
y=32768-y;//big + data
}
*/
z_ad_vel=msg_ad->angular.z*100*1.3;/*
if(z<0)
{
z=32768-z;//big + data
}
*/
//if(secs==0)
//{
//	secs =ros::Time::now().toSec();
 // ROS_INFO("I heard: [%f]", secs);
add_num=msg_ad->angular.y;
//add_num=100;//临时修改
	usart.fe_Usart_Send(0xfe,x_ad_vel,y_ad_vel,z_ad_vel,add_num);
}
void visionCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
int x=msg->linear.x*100;//yaw
int y=msg->linear.y*100;//pitch
int z=msg->linear.z*100;//shoot
int w=msg->angular.x*100;//shoot
rotation =msg->angular.y;//地盘跟随
distances=msg->angular.z;

usart.ff_Usart_Send(0xff,x,y,z,w,distances,rotation,swing);
}
void swingCallback(const std_msgs::Int16::ConstPtr& msg)
{
swing=msg->data;//摇摆
}
void DicisionToRefereeCallback(const std_msgs::Float64MultiArray::ConstPtr& msg )
{
  //ROS_WARN_STREAM("aaaaaaaaaaaaaaa");
  int my_position_x=msg->data[0]*100;
  int my_position_y=msg->data[1]*100;
  int enemy_position_x=msg->data[2]*100;
  int enemy_position_y=msg->data[3]*100;
  int bufflet=msg->data[4]*100;//remain_bullet
  int bufflet_sign=msg->data[5];
  int buff_sign=msg->data[6];
  int remain_blood=msg->data[7]*100;//remain_blood
  

  usart.bb_Usart_Send(0xbb,my_position_x,my_position_y,enemy_position_x,enemy_position_y,bufflet,bufflet_sign,buff_sign);//自己——x,自己-y,敌人-x,补弹标志，加成标志，
}
int main(int argc,char** argv)
{
	ros::init(argc,argv,"serial");
	ros::NodeHandle n;
  	// 创建一个Publisher，发布名为chatter的topic，消息类型为geometry_msgs::Twist
	usart.Usart_Init();
	//std::thread thread1(serial_thread);
	//thread1.detach();
	ros::Subscriber sub = n.subscribe("cmd_vel", 1000,sendCallback);//位置？
	ros::Subscriber sub1 = n.subscribe("cmd_vel1", 1000,adjustCallback);//位置？
	ros::Subscriber sub2_ = n.subscribe("vision_tomcu", 1000,visionCallback);//位置？
ros::Subscriber swing= n.subscribe("Swing_Cmd", 2,swingCallback);//位置？
 ros::Subscriber communicate= n.subscribe("DicisionToReferee", 2,DicisionToRefereeCallback);//位置？

	//ros::Rate rate(9);
	//while(ros::ok())
	//{
	//cout<<"1"<<endl;
	//usart.Usart_Send(1,21,10,1,1);
	ros::spin();
	//rate.sleep();
//}
return 0;
}

