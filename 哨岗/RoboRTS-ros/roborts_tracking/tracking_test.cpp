/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>
#include <chrono>
#include <ros/ros.h>
#include "roborts_msgs/GimbalAngle.h"
#include "std_msgs/Float64MultiArray.h"
#include <geometry_msgs/Twist.h>


#include "tracking_utility.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "KCFcpp/src/kcftracker.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#define HOG 1
#define FIXEDWINDOW 1
#define MULTISCALE 1
#define LAB 1

typedef std::chrono::time_point<std::chrono::high_resolution_clock> timer;
typedef std::chrono::duration<float> duration;

using namespace std;
using namespace cv;

enum selectedstate {
    IDLE,
    INIT,
    ONGOING,
  };
Mat frame;
float a=0,b=0,e=0,d=0;
std::vector<cv::Mat> image_buffer_;
bool memory = 0;

/////////////////////////////////////////////////  Gimbal Control
geometry_msgs::Twist gimbal_tdt;
/////////////////////////////////////////////////
void imageCallback(const sensor_msgs::ImageConstPtr& msg) 
{
  try { 
    frame=cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void chatterCallback1(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if(!msg->data.empty())
  {
  memory=1;
  a=msg->data[0];
  b=msg->data[1];
  e=msg->data[2];
  d=msg->data[3];
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "roborts_tracking_node");
  ros::NodeHandle nh;
  selectedstate state;
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/back_camera/image_raw", 1, imageCallback);
  ros::Subscriber sub1 = nh.subscribe("chatter", 1000, chatterCallback1);
  ros::Rate loop_rate(25);

  const char winName[]="My Camera";
  auto  pub= nh.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 30);
  
  
  /********************************************************************************  Gimbal Control*/
  ros::Publisher vision_pub = nh.advertise<geometry_msgs::Twist>("vision_tomcu", 1000);
  cv::Mat intrinsic_matrix_ = (cv::Mat_<double>(3,3) << 1410.1665141099454, 0, 320, 0, 1410.1665141099454, 240, 0, 0, 1);
  cv::Mat distortion_coeffs_ = (cv::Mat_<double>(5,1) << -4.8215660808079736e-01, -1.8762748885064542e+00, 3.9888099881952204e-03, -4.9441883534484619e-03, 5.8235109932797556e+01);
  /********************************************************************************/
  
  
  ros::Publisher detection_pub = nh.advertise<std_msgs::Float64MultiArray>("return_detection", 1000);
  char message1[100];
  char message2[100];
  Rect roi(0,0,0,0);
  TrackingUtility tu;
  KCFTracker *tracker = NULL;
  
  cv::namedWindow(winName,1);
  
  int i=0;
  bool status=0;
  int img_width = 640;
  int img_height = 480;
  memory=0;//next the while
  cv::Point p1;
  cv::Point p2;
  std_msgs::Float64MultiArray return_detection_msg;
  while (nh.ok())
  {
  if(memory)
  {
    if(state!=ONGOING)
    {
      state=INIT;
    }
  }else
  {
    if(state!=ONGOING)
    {
      state=IDLE;
    }
  }
    char c = cv::waitKey(10);
    if(c==27)
    {
      if(tracker != NULL)
      {
        delete tracker;
        tracker = NULL;
      }
      break; // Quit if ESC is pressed
    }
    if(!frame.empty())//else
    {
      double start = getTickCount();
      int dx = 0;
      int dy = 0;
      int yawRate = 0;
      int pitchRate = 0;
      timer trackerStartTime, trackerFinishTime;
      duration trackerTimeDiff;
      
      roborts_msgs::GimbalAngle gimbal_angle;
      int k = 1920/img_width;
      
      //////////////////////////////////////////////////////////////////// Gimbal Control 
          cv::Mat rvec;
          cv::Mat tvec;
	  std::vector<cv::Point3f> numb_points_;
	  float yaw_output;
	  float pitch_output;
	  std::vector<cv::Point2f> armor;
      ////////////////////////////////////////////////////////////////////
      
      switch(state)
      {
	case IDLE:
	  std::cout<<"Nothing was found"<<std::endl;
	  
	  
	  //////////////////////////////////////////////////////////////////// Gimbal Control 
	  
	  gimbal_tdt.linear.x=0;
	  gimbal_tdt.linear.y=0;
	  gimbal_tdt.linear.z=0;
	  gimbal_tdt.angular.x=1;
	  vision_pub.publish(gimbal_tdt);
	  
	  /////////////////////////////////////////////////////////////////////
	  
	  
	  break;
	case INIT:
	  std::cout<<"Have received the point"<<std::endl;
	  p1.x=a;
	  p1.y=b;
	  p2.x=e;
	  p2.y=d;
	  std::cout<<" a is "<<a<<std::endl;
          std::cout<<" b is "<<b<<std::endl;
          std::cout<<" e is "<<e<<std::endl;
          std::cout<<" d is "<<d<<std::endl;
	  roi = cv::Rect(p1, p2);
	  std::cout<<" roi.tl().x is "<<roi.tl().x<<std::endl;
	  std::cout<<" roi.tl().y is "<<roi.tl().y<<std::endl;
	  std::cout<<" roi.br().x is "<<roi.br().x<<std::endl;
	  std::cout<<" roi.br().y is "<<roi.br().y<<std::endl;
          tracker = new KCFTracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
          tracker->init(roi, frame);
          tu.startTracker();
	  state=ONGOING;
	  break;
	case ONGOING:
	
	  trackerStartTime  = std::chrono::high_resolution_clock::now();
          roi = tracker->update(frame);
	  std::cout<<roi.x<<","<<roi.y<<","<<roi.width<<","<<roi.height<<endl;
          trackerFinishTime = std::chrono::high_resolution_clock::now();
          trackerTimeDiff = trackerFinishTime - trackerStartTime;
          // send gimbal speed command
          dx = (int)(roi.x + roi.width/2  - img_width/2);
          dy = (int)(roi.y + roi.height/2 - img_height/2);
          yawRate   = -dx;
          pitchRate = dy;
	  if(!tracker->spy)
	  {
	    std::cout<<"sssssssssssssssssssssssss"<<std::endl;
	    delete tracker;
            tracker = NULL;
            tu.stopTracker();
	    state=IDLE;
	    roi.x=0;
	    roi.y=0;
	    roi.height=0;
	    roi.width=0;
	    
	    break;
	  }
	  //////////////////////
	  return_detection_msg.data.push_back(roi.tl().x);
          return_detection_msg.data.push_back(roi.tl().y);
          return_detection_msg.data.push_back(roi.br().x);
          return_detection_msg.data.push_back(roi.br().y);
	  detection_pub.publish(return_detection_msg);
	  //////////////////////
          if(abs(yawRate) < 10/k)
          {
            yawRate = 0;
          }
          else if(abs(yawRate)>500/k) {
            yawRate = ((yawRate>0)?1:-1)*500/k;
          }

          if(abs(pitchRate) < 10/k)
          {
            pitchRate = 0;
          }
          else if(abs(pitchRate)>500/k) {
            pitchRate = ((pitchRate>0)?1:-1)*500/k;
          }

          gimbal_angle.pitch_mode = true;
          gimbal_angle.pitch_angle = pitchRate/180.*M_PI/110.*k;

          gimbal_angle.yaw_mode = true;
          gimbal_angle.yaw_angle = yawRate/180.*M_PI/160.*k;
          pub.publish(gimbal_angle);
	  
	  ///////////////////////////////////////////////////////////////////// Gimbal Control 
	  
	numb_points_.emplace_back(cv::Point3f(-3.3, 5.5,  0.0));//-3.3, 5.5,  0.0
        numb_points_.emplace_back(cv::Point3f(3.3,  5.5,  0.0));//3.3,  5.5,  0.0
        numb_points_.emplace_back(cv::Point3f(3.3,  -5.5, 0.0));//3.3,  -5.5, 0.0
        numb_points_.emplace_back(cv::Point3f(-3.3, -5.5, 0.0));//-3.3, -5.5, 0.0
	
	
	armor.emplace_back(roi.tl().x, roi.tl().y);//-3.3, 5.5,  0.0
        armor.emplace_back(cv::Point2f(roi.tl().x+roi.width, roi.tl().y));//3.3,  5.5,  0.0
        armor.emplace_back(cv::Point2f(roi.tl().x+roi.width, roi.tl().y-roi.height));//3.3,  -5.5, 0.0
        armor.emplace_back(cv::Point2f(roi.tl().x, roi.tl().y-roi.height));//-3.3, -5.5, 0.0
	cv::solvePnP(numb_points_,
               armor,
               intrinsic_matrix_,
               distortion_coeffs_,
               rvec,
               tvec);
	yaw_output=atan2(320-(roi.tl().x+(roi.width/2)),intrinsic_matrix_.at<double>(0,0))*57.3;
	pitch_output=-atan2(240-(roi.tl().y+(roi.height/2)),intrinsic_matrix_.at<double>(0,0))*57.3;
	std::cout<<" yaw_output is "<<yaw_output<<std::endl;
	std::cout<<" pitch_output is "<<pitch_output<<std::endl;
        //target_3d = cv::Point3f(tvec);
	  gimbal_tdt.linear.x=yaw_output;
	  gimbal_tdt.linear.y=pitch_output;
	  if(pitch_output<3)
	    gimbal_tdt.linear.z=1;
	  else
	    gimbal_tdt.linear.z=0;
	  gimbal_tdt.angular.x=0;
	  vision_pub.publish(gimbal_tdt);
	  /////////////////////////////////////////////////////////////////////
	  
	  break;
	
	  
	  default:
          break;
      }
      dx = roi.x + roi.width/2  - img_width/2;
      dy = roi.y + roi.height/2 - img_height/2;
      double end = getTickCount();
      double time =(end-start)/getTickFrequency();
      std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
      std::cout<<" time is "<<time<<std::endl;
      std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
      cv::circle(frame, Point(img_width/2, img_height/2), 5, cv::Scalar(255,0,0), 2, 8);
      if(roi.width != 0)
      {
        cv::circle(frame, Point(roi.x + roi.width/2, roi.y + roi.height/2), 3, cv::Scalar(0,0,255), 1, 8);

        cv::line(frame,  Point(img_width/2, img_height/2),
                 Point(roi.x + roi.width/2, roi.y + roi.height/2),
                 cv::Scalar(0,255,255));
      }

      cv::rectangle(frame, roi, cv::Scalar(0,255,0), 1, 8, 0 );
      sprintf(message1,"dx=%04d, dy=%04d",dx, dy);
      putText(frame, message1, Point2f(20,30), FONT_HERSHEY_SIMPLEX, 0.5,  Scalar(0,255,0));
      putText(frame, message2, Point2f(20,60), FONT_HERSHEY_SIMPLEX, 0.5,  Scalar(0,255,0));
      cv::imshow(winName, frame);
    }
    memory=0;
    ros::spinOnce();
  }
  if(tracker)
  {
    delete tracker;
  }
  
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  /*
  auto  pub= nh.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 30);
  const char winName[]="My Camera";
  char message1[100];
  char message2[100];
  Rect roi(0,0,0,0);
  TrackingUtility tu;
  KCFTracker *tracker = NULL;

  cv::namedWindow(winName,1);
  cv::setMouseCallback(winName,TrackingUtility::mouseCallback, (void*)&tu);

  VideoCapture video(0);
  int img_width = 640;
  int img_height = 480;
  video.set(CV_CAP_PROP_FRAME_WIDTH,img_width);
  video.set(CV_CAP_PROP_FRAME_HEIGHT,img_height);
  img_width = video.get(CV_CAP_PROP_FRAME_WIDTH);
  img_height = video.get(CV_CAP_PROP_FRAME_HEIGHT);

  if (!video.isOpened()) {
    cout << "cannot read video!" << endl;
    return -1;
  }
  //可选BOOSTING, MIL, KCF, TLD, MEDIANFLOW, or GOTURN

  int i=0;
  while(ros::ok())
  {
    char c = cv::waitKey(10);
    if(c==27)
    {
      if(tracker != NULL)
      {
        delete tracker;
        tracker = NULL;
      }
      break; // Quit if ESC is pressed
    }
    
    double start = getTickCount();
    
    tu.getKey(c); //Internal states will be updated based on key pressed.

    Mat frame;
    if(video.read(frame)){
      int dx = 0;
      int dy = 0;
      int yawRate = 0;
      int pitchRate = 0;
      timer trackerStartTime, trackerFinishTime;
      duration trackerTimeDiff;

      roborts_msgs::GimbalAngle gimbal_angle;
      int k = 1920/img_width;
      switch(tu.getState())
      {
        case TrackingUtility::STATE_IDLE:
          roi = tu.getROI();
          sprintf(message2, "Please select ROI and press g");
	  std::cout<<"1111111111111111"<<std::endl;
          break;

        case TrackingUtility::STATE_INIT:
          cout << "g pressed, initialize tracker" << endl;
          sprintf(message2, "g pressed, initialize tracker");
          roi = tu.getROI();
          tracker = new KCFTracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
          tracker->init(roi, frame);
          tu.startTracker();
	  std::cout<<"222222222222222222"<<std::endl;
          break;

        case TrackingUtility::STATE_ONGOING:
          trackerStartTime  = std::chrono::high_resolution_clock::now();
          roi = tracker->update(frame);
	  std::cout<<roi.x<<","<<roi.y<<","<<roi.width<<","<<roi.height<<endl;
          trackerFinishTime = std::chrono::high_resolution_clock::now();
          trackerTimeDiff = trackerFinishTime - trackerStartTime;
          sprintf(message2, "Tracking: bounding box update time = %.2f ms\n", trackerTimeDiff.count()*1000.0);

          // send gimbal speed command
          dx = (int)(roi.x + roi.width/2  - img_width/2);
          dy = (int)(roi.y + roi.height/2 - img_height/2);

          yawRate   = -dx;
          pitchRate = dy;
          cout<<"yaw_rate:"<<yawRate<<endl;
          cout<<"pitch_rate:"<<pitchRate<<endl;
	  ///////////////////////////////////////////////////////////
	  if(!tracker->spy)                                        // 
	  {                                                        //
	    std::cout<<"sssssssssssssssssssssssss"<<std::endl;     //
	    tu.getKey('s');                                        //
	    break;                                                 //
	  }                                                        //
	  ///////////////////////////////////////////////////////////
          if(abs(yawRate) < 10/k)
          {
            yawRate = 0;
          }
          else if(abs(yawRate)>500/k) {
            yawRate = ((yawRate>0)?1:-1)*500/k;
          }

          if(abs(pitchRate) < 10/k)
          {
            pitchRate = 0;
          }
          else if(abs(pitchRate)>500/k) {
            pitchRate = ((pitchRate>0)?1:-1)*500/k;
          }

          gimbal_angle.pitch_mode = true;
          gimbal_angle.pitch_angle = pitchRate/180.*M_PI/110.*k;

          gimbal_angle.yaw_mode = true;
          gimbal_angle.yaw_angle = yawRate/180.*M_PI/160.*k;
          pub.publish(gimbal_angle);
	  std::cout<<"33333333333333333333333"<<std::endl;
          break;

        case TrackingUtility::STATE_STOP:
          cout << "s pressed, stop tracker" << endl;
          sprintf(message2, "s pressed, stop tracker");
          delete tracker;
          tracker = NULL;
          tu.stopTracker();
          roi = tu.getROI();
	  std::cout<<"44444444444444444444444"<<std::endl;
          break;

        default:
          break;
      }
      cout<<"state is "<<tu.state<<endl;
      
      dx = roi.x + roi.width/2  - img_width/2;
      dy = roi.y + roi.height/2 - img_height/2;
      
      double end = getTickCount();
      double time =(end-start)/getTickFrequency();
      cout<<"time"<<time<<endl;
      cv::circle(frame, Point(img_width/2, img_height/2), 5, cv::Scalar(255,0,0), 2, 8);
      if(roi.width != 0)
      {
        cv::circle(frame, Point(roi.x + roi.width/2, roi.y + roi.height/2), 3, cv::Scalar(0,0,255), 1, 8);

        cv::line(frame,  Point(img_width/2, img_height/2),
                 Point(roi.x + roi.width/2, roi.y + roi.height/2),
                 cv::Scalar(0,255,255));
      }

      //cv::rectangle(frame, roi, cv::Scalar(0,255,0), 1, 8, 0 );
      cv::rectangle(frame, roi, cv::Scalar(0,255,0), 1, 8, 0 );
      sprintf(message1,"dx=%04d, dy=%04d",dx, dy);
      putText(frame, message1, Point2f(20,30), FONT_HERSHEY_SIMPLEX, 0.5,  Scalar(0,255,0));
      putText(frame, message2, Point2f(20,60), FONT_HERSHEY_SIMPLEX, 0.5,  Scalar(0,255,0));
      cv::imshow(winName, frame);
    }
  }

  if(tracker)
  {
    delete tracker;
  }
  */
  return 0;

}