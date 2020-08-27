// #include "ros/ros.h"
// #include "std_msgs/String.h"
// #include  "geometry_msgs/PoseStamped.h"
// 
// #include "std_msgs/Float64MultiArray.h"
// 
// #include <sstream>
// 
// #include <iostream>
// #include <cstdlib>
// #include <ctime>
// 
// #define random(x) rand()%(x)
// 
// /**
//  * This tutorial demonstrates simple sending of messages over the ROS system.
//  */
// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "talker");
//   ros::NodeHandle n;
//   ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
//   
//   ros::Publisher global_pub = n.advertise<geometry_msgs::PoseStamped>("/planthread", 1000);
//   
//   ros::Publisher astar_pub = n.advertise<geometry_msgs::PoseStamped>("/Aplan_info", 1000);
//   
//   ros::Publisher local_pub = n.advertise<geometry_msgs::PoseStamped>("/LOOP_info", 1000);
//   
//   ros::Publisher teb_pub = n.advertise<geometry_msgs::PoseStamped>("/cp_vel_info", 1000);
//   
//   ros::Publisher obstacle_pub = n.advertise<std_msgs::Float64MultiArray>("/simulate_area_status_msg", 1000);
//   
//   srand((int)time(0));  // 产生随机种子  把0换成NULL也行
// 
//   ros::Rate loop_rate(10);
//   
//   int stochastic;
//   
//   int count = 0;
//   
//   geometry_msgs::PoseStamped planthread,Aplan,Loop_info,cp_vel_info;
//   std_msgs::Float64MultiArray obstacle_info;
//   while (ros::ok())
//   {
//     std_msgs::String msg;
//     std::stringstream ss;
//     ss << "hello world " << count;
//     msg.data = ss.str();
//     ROS_INFO("%s", msg.data.c_str());
//     chatter_pub.publish(msg);
//     
//     std::cout << random(100)<<std::endl;
//     stochastic = random(100);
//     if(stochastic % 2==0)
//       stochastic = 0;
//     else
//       stochastic = 1;
//     switch(stochastic)
//     {
//       case 0:
//       {
// 	planthread.pose.position.x = 1 ;    //global planner node Get Robot Pose Error"
// 	planthread.pose.position.y= 1;    //global planner node STUCK!!Can not get plan with 5  retries
// 	planthread.pose.position.z= 1;    //global planner node Current goal is not valid!
// 	planthread.pose.orientation.x=1;  //global planner node  Can not get plan for once
// 	planthread.pose.orientation.y=1; //global planner node  The time planning once is beyond the expected time
// 	planthread.pose.orientation.z= 0;
// 	
// 	
// 	//
// 	
// 	
// 	Aplan.pose.position.x = 1;      //A* GP transform error
// 	Aplan.pose.position.y = 1;      //A* map frame transform error
// 	Aplan.pose.position.z = 1;      //A*CHANGED A GOAL
// 	Aplan.pose.orientation.x = 0;
// 	Aplan.pose.orientation.y = 0;
// 	Aplan.pose.orientation.z = 0; 
// 	
// 	//
// 	
// 	
// 	Loop_info.pose.position.x = 1;       //local planner node  algorithm initialize failed
// 	Loop_info.pose.position.y = 1;       //local planner node  stuck vel
// 	Loop_info.pose.position.z = 1;       //local planner node  Can not finish plan with max retries
// 	Loop_info.pose.orientation.x = 1;    //local planner node  Can not get cmd_vel for once
// 	Loop_info.pose.orientation.x = 0; 
// 	Loop_info.pose.orientation.x = 0; 
// 	
// 	
// 	//
// 	
// 	
// 	cp_vel_info.pose.position.x =1;    //TEB  not initialized
// 	cp_vel_info.pose.position.y =1;      //TEB  trajectory is not  feasible
// 	cp_vel_info.pose.position.z =1;      //TEB  oscillating
// 	cp_vel_info.pose.orientation.x =1;    //TEB  plan transform error
// 	cp_vel_info.pose.orientation.y =1;   //TEB  transformed plan is empty
// 	cp_vel_info.pose.orientation.z =1;   //TEB optimal error
// 	cp_vel_info.pose.orientation.w =1;   //TEB can not get the velocity
// 
// 	
// 	//
// 	
// 	
//  	obstacle_info.data.push_back(0);
//  	obstacle_info.data.push_back(1);
//  	obstacle_info.data.push_back(0);
//  	obstacle_info.data.push_back(1);
//  	obstacle_info.data.push_back(0);
//  	obstacle_info.data.push_back(1);
// 
// 
// //         obstacle_info.push_back(0);
// // 	obstacle_info.push_back(1);
// // 	obstacle_info.push_back(0);
// // 	obstacle_info.push_back(1);
// // 	obstacle_info.push_back(0);
// // 	obstacle_info.push_back(1);
// 	break;
//       }
//       case 1:
//       {
// 	planthread.pose.position.x = 0 ;    //global planner node Get Robot Pose NORMAL
// 	planthread.pose.position.y= 0;    //global planner node NOT STUCK!
// 	planthread.pose.position.z= 0;    //global planner node Current goal is valid!
// 	planthread.pose.orientation.x=0;  //global planner node  Can get plan for once
// 	planthread.pose.orientation.y=0; //global planner node  The time planning once is still in the expected time
// 	planthread.pose.orientation.z= 1;
// 	
// 	
// 	//
// 	
// 	
// 	Aplan.pose.position.x = 0;      //A* GP transform NORMAL
// 	Aplan.pose.position.y = 0;      //A* map frame transform NORMAL
// 	Aplan.pose.position.z = 0;      //A* DOESN'T CHANGED A GOAL
// 	Aplan.pose.orientation.x = 1;
// 	Aplan.pose.orientation.y = 1;
// 	Aplan.pose.orientation.z = 1; 
// 	
// 	//
// 	
// 	
// 	Loop_info.pose.position.x = 0;       //local planner node  algorithm initialize SUCCESS
// 	Loop_info.pose.position.y = 0;       //local planner node  NO stuck vel(???)
// 	Loop_info.pose.position.z = 0;       //local planner node  Can finish plan with max retries
// 	Loop_info.pose.orientation.x = 0;    //local planner node  Can get cmd_vel for once
// 	Loop_info.pose.orientation.x = 1; 
// 	Loop_info.pose.orientation.x = 1; 
// 	
// 	
// 	//
// 	
// 	
// 	cp_vel_info.pose.position.x =0;    //TEB  initialized
// 	cp_vel_info.pose.position.y =0;      //TEB  trajectory is feasible
// 	cp_vel_info.pose.position.z =0;      //TEB  Smooth
// 	cp_vel_info.pose.orientation.x =0;    //TEB  plan transform NORMAL
// 	cp_vel_info.pose.orientation.y =0;   //TEB  transformed plan is NOT empty
// 	cp_vel_info.pose.orientation.z =0;   //TEB optimal NORMAL
// 	cp_vel_info.pose.orientation.w =0;   //TEB can get the velocity
// 
// 	
// 	//
// 	
// 	
//  	obstacle_info.data.push_back(1);
//  	obstacle_info.data.push_back(0);
//  	obstacle_info.data.push_back(1);
//  	obstacle_info.data.push_back(0);
//  	obstacle_info.data.push_back(1);
//  	obstacle_info.data.push_back(0);
// 
// 
//         
// 	break;
//       }
//       default:
// 	break;
//   }
// 	
//     global_pub.publish<geometry_msgs::PoseStamped>(planthread);
//     
//     astar_pub.publish<geometry_msgs::PoseStamped>(Aplan);
//     
//     local_pub.publish<geometry_msgs::PoseStamped>(Loop_info);
//     
//     teb_pub.publish<geometry_msgs::PoseStamped>(cp_vel_info);
//     
//     obstacle_pub.publish<std_msgs::Float64MultiArray>(obstacle_info);
//     
//     
//     obstacle_info.data.clear();
// 	
//     ros::spinOnce();
//     loop_rate.sleep();
//     ++count;
//   }
// 
// 
//   return 0;
// }

























#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<stdio.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle n; 
    ros::Time time = ros::Time::now();
    ros::Rate loop_rate(5);
    
     // 定义节点句柄   
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("/camera_name/image_raw0", 1);
    sensor_msgs::ImagePtr msg;
        
    // opencv准备读取视频
    cv::VideoCapture video;
    video.open("/home/tdt/下载/1.mp4");  

    if( !video.isOpened() )
    {
        ROS_INFO("Read Video failed!\n");
        return 0;
    }
    Mat frame;
    int count = 0;
    while(1)
    {
        video >> frame;
        if( frame.empty() )
            break;
        count++;
        
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
        
        ROS_INFO( "read the %dth frame successfully!", count );
        loop_rate.sleep();
        ros::spinOnce();
    }
    video.release();
    
    return 0;
}
