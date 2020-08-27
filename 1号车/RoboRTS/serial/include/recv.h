#ifndef RECV_H
#define RECV_H
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/Int16MultiArray.h"
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#define PI 3.14159265

namespace serial
{
  class Usart_serial
  {
  public:
      Usart_serial();
      void usart_callback(const std_msgs::Int16MultiArray::ConstPtr &usart_msg);
      void Init();
  private:
      ros::NodeHandle nh_;
      ros::Publisher chatter_pub ;                  //返回调用函数的状态   
      ros::Publisher vision_pub ;
      ros::Publisher uwb_pub;    //返回调用函数的状态 
      ros::Publisher referee_pub;
      ros::Publisher communicate_pub;
      ros::Publisher odom_position_pub;
      ros::Publisher area_refereeto_pub;
      ros::Publisher robort_status_pub;
      
      geometry_msgs::Twist mcu_msgs;
      geometry_msgs::Twist uwb_msgs;
      geometry_msgs::Twist vision_msgs;
      std_msgs::Float64MultiArray referee;
      std_msgs::Float64MultiArray area_status;
      std_msgs::Float64MultiArray robort_status;
      std_msgs::Float64MultiArray conmunicate;
      geometry_msgs::PoseStamped odom_position;
      ros::Subscriber usart_sub;
  };
}
#endif // SEC_ODOM_H