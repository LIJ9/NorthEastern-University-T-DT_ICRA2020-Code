// #include "ros/ros.h"
// #include "std_msgs/String.h"
// #include "geometry_msgs/PoseStamped.h"
// /**
//  * This tutorial demonstrates simple receipt of messages over the ROS system.
//  */
// 
// int a,b,c,d,e,f,g;
// 
// void chatterCallback(const std_msgs::String::ConstPtr& msg)
// {
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
// }
// 
// void geometry_chatterCallback(const geometry_msgs::PoseStamped &msg)
// {
//   a = msg.pose.position.x;
// }
// 
// 
// int main(int argc, char **argv)
// {
//   /**
//    * The ros::init() function needs to see argc and argv so that it can perform
//    * any ROS arguments and name remapping that were provided at the command line.
//    * For programmatic remappings you can use a different version of init() which takes
//    * remappings directly, but for most command-line programs, passing argc and argv is
//    * the easiest way to do it.  The third argument to init() is the name of the node.
//    *
//    * You must call one of the versions of ros::init() before using any other
//    * part of the ROS system.
//    */
//   ros::init(argc, argv, "listener");
// 
//   /**
//    * NodeHandle is the main access point to communications with the ROS system.
//    * The first NodeHandle constructed will fully initialize this node, and the last
//    * NodeHandle destructed will close down the node.
//    */
//   ros::NodeHandle n;
// 
//   /**
//    * The subscribe() call is how you tell ROS that you want to receive messages
//    * on a given topic.  This invokes a call to the ROS
//    * master node, which keeps a registry of who is publishing and who
//    * is subscribing.  Messages are passed to a callback function, here
//    * called chatterCallback.  subscribe() returns a Subscriber object that you
//    * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
//    * object go out of scope, this callback will automatically be unsubscribed from
//    * this topic.
//    *
//    * The second parameter to the subscribe() function is the size of the message
//    * queue.  If messages are arriving faster than they are being processed, this
//    * is the number of messages that will be buffered up before beginning to throw
//    * away the oldest ones.
//    */
//   ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
//   
//   
//   ros::Subscriber geometry_sub = n.subscribe("/planthread", 1000, geometry_chatterCallback);
// 
//   /**
//    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
//    * callbacks will be called from within this thread (the main one).  ros::spin()
//    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
//    */
//   ros::spin();
// 
//   return 0;
// }
// 











#include <vector>
#include <thread>
#include <mutex>


#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>


#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>



#include<stdio.h>
#include<math.h>
#include<vector>

#include <iostream>  
#include <fstream>

using namespace std;
using namespace cv;
//校正
  int correct_counter0 = 1;
  int correct_counter1 = 1;
  std::vector<cv::Point> src0_point;
  std::vector<cv::Point> src1_point;
  cv::Point correct_select0;
  cv::Point correct_select1;
  bool finish_flag0 = false;
  bool select_flag0 = false;
  bool finish_flag1 = false;
  bool select_flag1 = false;
  ofstream fout0;
  ofstream fout1;
  cv::Mat src_0;
  cv::Mat src_1;

void src0_onMouse(int event, int x, int y, int flag, void* param)
{
  switch(event)
  {
    case  CV_EVENT_LBUTTONDOWN:
    {
      select_flag0 = true;
      correct_select0.x = x;
      correct_select0.y = y;
      break;
    }

    case  CV_EVENT_LBUTTONUP:
    {
      if(correct_counter0<=4)
      {
	src0_point.push_back(correct_select0);
	correct_counter0++;
      }else
      {
	correct_counter0 = 0;
	finish_flag0 = true;
      }
      select_flag0 = false;
      break;
    }

    case  CV_EVENT_MOUSEMOVE:
    {
      if(select_flag0)
      {
	correct_select0.x = x;
	correct_select0.y = y;
      }
      
      if(correct_counter0 == 5)
	finish_flag0 = true;
      
      break;
    }
    case  CV_EVENT_RBUTTONDOWN:
    {
      select_flag0 = false;
      correct_counter0 = 0;
      select_flag0 = false;
      finish_flag0 = false;
      break;
    }
    default:
      break;
  }
}


void src1_onMouse(int event, int x, int y, int flag, void* param)
{
  switch(event)
  {
    case  CV_EVENT_LBUTTONDOWN:
    {
      select_flag1 = true;
      correct_select1.x = x;
      correct_select1.y = y;
      break;
    }

    case  CV_EVENT_LBUTTONUP:
    {
      if(correct_counter1<=4)
      {
	src1_point.push_back(correct_select1);
	correct_counter1++;
      }else
      {
	correct_counter1 = 0;
	finish_flag1 = true;
      }
      select_flag1 = false;
      break;
    }

    case  CV_EVENT_MOUSEMOVE:
    {
      if(select_flag1)
      {
	correct_select1.x = x;
	correct_select1.y = y;
      }
      
      if(correct_counter0 == 5)
	finish_flag0 = true;
      
      break;
    }
    case  CV_EVENT_RBUTTONDOWN:
    {
      select_flag1 = false;
      correct_counter1 = 0;
      select_flag1 = false;
      finish_flag1 = false;
      break;
    }
    default:
      break;
  }
}

void imageCalllback0(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("Received \n");
  try{
    cv::imshow( "video0", cv_bridge::toCvShare(msg, "bgr8")->image );
    src_0 = cv_bridge::toCvShare(msg, "bgr8")->image;
    if(!src_0.empty())
    {
      if(finish_flag0)
      {
	
	double tmp0_x,tmp0_y,tmp1_x,tmp1_y,tmp2_x,tmp2_y,tmp3_x,tmp3_y;
	tmp0_x = (src0_point[0].y-src0_point[1].y)/(src0_point[0].x-src0_point[1].x);
	tmp0_y = (src0_point[0].x*src0_point[1].y-src0_point[1].x*src0_point[0].y)/(src0_point[0].x-src0_point[1].x);
	
	tmp1_x = (src0_point[0].y-src0_point[2].y)/(src0_point[0].x-src0_point[2].x);
	tmp1_y = (src0_point[0].x*src0_point[2].y-src0_point[2].x*src0_point[0].y)/(src0_point[0].x-src0_point[2].x);
	
	tmp2_x = (src0_point[3].y-src0_point[1].y)/(src0_point[3].x-src0_point[1].x);
	tmp2_y = (src0_point[3].x*src0_point[1].y-src0_point[1].x*src0_point[3].y)/(src0_point[3].x-src0_point[1].x);
	
	tmp3_x = (src0_point[2].y-src0_point[3].y)/(src0_point[2].x-src0_point[3].x);
	tmp3_y = (src0_point[2].x*src0_point[3].y-src0_point[3].x*src0_point[2].y)/(src0_point[2].x-src0_point[3].x);
	
	
	fout0 = ofstream ("/home/tdt/catkin_ws/src/RoboRTS/beginner_tutorials/correct_00.txt");
	fout0<<"Correct result :\n";
 	fout0<<tmp0_x<<"          "<<tmp0_y<<"\n";
 	fout0<<tmp1_x<<"          "<<tmp1_y<<"\n";
 	fout0<<tmp2_x<<"          "<<tmp2_y<<"\n";
 	fout0<<tmp3_x<<"          "<<tmp3_y<<"\n";
	
      }
      
    }
    cv::waitKey(30);
    
  }
  catch( cv_bridge::Exception& e )
  {
    ROS_ERROR( "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str() );
    
  }
  
}


void imageCalllback1(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("Received \n");
  try{
    cv::imshow( "video1", cv_bridge::toCvShare(msg, "bgr8")->image );
    src_1 = cv_bridge::toCvShare(msg, "bgr8")->image;
    if(!src_1.empty())
    {
      if(finish_flag1)
      {
	double tmp0_x,tmp0_y,tmp1_x,tmp1_y,tmp2_x,tmp2_y,tmp3_x,tmp3_y;
	tmp0_x = (src1_point[0].y-src1_point[1].y)/(src1_point[0].x-src1_point[1].x);
	tmp0_y = (src1_point[0].x*src1_point[1].y-src1_point[1].x*src1_point[0].y)/(src1_point[0].x-src1_point[1].x);
	
	tmp1_x = (src1_point[0].y-src1_point[2].y)/(src1_point[0].x-src1_point[2].x);
	tmp1_y = (src1_point[0].x*src1_point[2].y-src1_point[2].x*src1_point[0].y)/(src1_point[0].x-src1_point[2].x);
	
	tmp2_x = (src1_point[3].y-src1_point[1].y)/(src1_point[3].x-src1_point[1].x);
	tmp2_y = (src1_point[3].x*src1_point[1].y-src1_point[1].x*src1_point[3].y)/(src1_point[3].x-src1_point[1].x);
	
	tmp3_x = (src1_point[2].y-src1_point[3].y)/(src1_point[2].x-src1_point[3].x);
	tmp3_y = (src1_point[2].x*src1_point[3].y-src1_point[3].x*src1_point[2].y)/(src1_point[2].x-src1_point[3].x);
	
	
	fout1 = ofstream ("/home/tdt/catkin_ws/src/RoboRTS/beginner_tutorials/correct_01.txt");
	fout1<<"Correct result :\n";
 	fout1<<tmp0_x<<"          "<<tmp0_y<<"\n";
 	fout1<<tmp1_x<<"          "<<tmp1_y<<"\n";
 	fout1<<tmp2_x<<"          "<<tmp2_y<<"\n";
 	fout1<<tmp3_x<<"          "<<tmp3_y<<"\n";
	
      }
      
    }
    cv::waitKey(30);
    
  }
  catch( cv_bridge::Exception& e )
  {
    ROS_ERROR( "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str() );
    
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle n;
  cv::namedWindow("video0");
  cv::namedWindow("video1");
  setMouseCallback("video0",src0_onMouse,0);
  setMouseCallback("video1",src1_onMouse,0);
  
  
  
  
  cv::startWindowThread();
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub0 = it.subscribe( "/back_camera/image_raw", 1, imageCalllback0 );
  image_transport::Subscriber sub1 = it.subscribe( "/back_camera1/image_raw", 1, imageCalllback1 );
  
  
  ros::spin();
  cv::destroyWindow("video0");
  cv::destroyWindow("video1");
  return 0;
}
