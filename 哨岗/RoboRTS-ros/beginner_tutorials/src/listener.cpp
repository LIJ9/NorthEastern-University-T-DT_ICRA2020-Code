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
//-- 区域校正\画线点采集
#define num 6

//校正
  int correct_counter0 = 1;
  int correct_counter1 = 1;
  std::vector<cv::Point2d> src0_point;
  std::vector<cv::Point2d> src1_point;
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
      if(correct_counter0<=num)
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
      
      if(correct_counter0 == num+1)
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
      if(correct_counter1<=num)
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
      
      if(correct_counter0 == num+1)
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
  //ROS_INFO("Received \n");
  try{
   // cv::imshow( "video0", 
	  //resize(cv_bridge::toCvShare(msg, "bgr8")->image,cv_bridge::toCvShare(msg, "bgr8")->image, Size(720,640)));
    src_0 = cv_bridge::toCvShare(msg, "bgr8")->image;
	//resize(src_0,src_0,Size(720,540));
	imshow("video0",src_0);
	
    if(!src_0.empty())
    {
      if(finish_flag0)
      {
	
	double tmp0_x,tmp0_y,tmp1_x,tmp1_y,tmp2_x,tmp2_y,tmp3_x,tmp3_y;
	//--func1
	tmp0_x = (src0_point[0].y-src0_point[1].y)/(src0_point[0].x-src0_point[1].x);
	tmp0_y = (src0_point[0].x*src0_point[1].y-src0_point[1].x*src0_point[0].y)/(src0_point[0].x-src0_point[1].x);
	//--func2
	tmp1_x = (src0_point[3].y-src0_point[2].y)/(src0_point[3].x-src0_point[2].x);
	tmp1_y = (src0_point[3].x*src0_point[2].y-src0_point[2].x*src0_point[3].y)/(src0_point[3].x-src0_point[2].x);
	//--func3
	tmp2_x = (src0_point[4].y-src0_point[3].y)/(src0_point[4].x-src0_point[3].x);
	tmp2_y = (src0_point[4].x*src0_point[3].y-src0_point[3].x*src0_point[4].y)/(src0_point[4].x-src0_point[3].x);
	//--func4
	tmp3_x = (src0_point[5].y-src0_point[0].y)/(src0_point[5].x-src0_point[0].x);
	tmp3_y = (src0_point[5].x*src0_point[0].y-src0_point[0].x*src0_point[5].y)/(src0_point[5].x-src0_point[0].x);
	
	
	
	
	fout0 = ofstream ("/home/tdt/catkin_ws/src/RoboRTS-ros/beginner_tutorials/correct_00.txt");
	fout0<<"Correct result :\n";
#if 1
 	fout0<<tmp0_x<<"          "<<tmp0_y<<"\n";
 	fout0<<tmp1_x<<"          "<<tmp1_y<<"\n";
 	fout0<<tmp2_x<<"          "<<tmp2_y<<"\n";
 	fout0<<tmp3_x<<"          "<<tmp3_y<<"\n";
#endif
#if 0
	fout0<<src0_point[0].x*2<<","<<src0_point[0].y*2<<"\n";
	fout0<<src0_point[1].x*2<<","<<src0_point[1].y*2<<"\n";
	fout0<<src0_point[2].x*2<<","<<src0_point[2].y*2<<"\n";
	fout0<<src0_point[3].x*2<<","<<src0_point[3].y*2<<"\n";
	
	fout0<<src0_point[4].x*2<<","<<src0_point[4].y*2<<"\n";
	fout0<<src0_point[5].x*2<<","<<src0_point[5].y*2<<"\n";
	fout0<<src0_point[6].x*2<<","<<src0_point[6].y*2<<"\n";
	fout0<<src0_point[7].x*2<<","<<src0_point[7].y*2<<"\n";
#endif
	
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
    //cv::imshow( "video1", cv_bridge::toCvShare(msg, "bgr8")->image );
    src_1 = cv_bridge::toCvShare(msg, "bgr8")->image;
	//resize(src_1,src_1,Size(720,540));
	imshow("video1",src_1);
    if(!src_1.empty())
    {
      if(finish_flag1)
      {
	double tmp0_x,tmp0_y,tmp1_x,tmp1_y,tmp2_x,tmp2_y,tmp3_x,tmp3_y;
	
	//--func1
	tmp0_x = (src1_point[1].y-src1_point[0].y)/(src1_point[1].x-src1_point[0].x);
	tmp0_y = (src1_point[1].x*src1_point[0].y-src1_point[0].x*src1_point[1].y)/(src1_point[1].x-src1_point[0].x);
	//--func2
	tmp1_x = (src1_point[3].y-src1_point[2].y)/(src1_point[3].x-src1_point[2].x);
	tmp1_y = (src1_point[3].x*src1_point[2].y-src1_point[2].x*src1_point[3].y)/(src1_point[3].x-src1_point[2].x);
	//--func3
	tmp2_x = (src1_point[4].y-src1_point[3].y)/(src1_point[4].x-src1_point[3].x); 
	tmp2_y = (src1_point[4].x*src1_point[3].y-src1_point[3].x*src1_point[4].y)/(src1_point[4].x-src1_point[3].x);
	//--func4
	tmp3_x = (src1_point[5].y-src1_point[0].y)/(src1_point[5].x-src1_point[0].x);
	tmp3_y = (src1_point[5].x*src1_point[0].y-src1_point[0].x*src1_point[5].y)/(src1_point[5].x-src1_point[0].x);
	
	
	
	fout1 = ofstream ("/home/tdt/catkin_ws/src/RoboRTS-ros/beginner_tutorials/correct_01.txt");
	fout1<<"Correct result :\n";
#if 1
 	fout1<<tmp0_x<<"*x+"<<tmp0_y<<"\n";
 	fout1<<tmp1_x<<"*x+"<<tmp1_y<<"\n";
 	fout1<<tmp2_x<<"*x+"<<tmp2_y<<"\n";
 	fout1<<tmp3_x<<"*x+"<<tmp3_y<<"\n";
#endif 
#if 0
	fout1<<src1_point[0].x*2<<","<<src1_point[0].y*2<<"\n";
	fout1<<src1_point[1].x*2<<","<<src1_point[1].y*2<<"\n";
	fout1<<src1_point[2].x*2<<","<<src1_point[2].y*2<<"\n";
	fout1<<src1_point[3].x*2<<","<<src1_point[3].y*2<<"\n";
	
	fout1<<src1_point[4].x*2<<","<<src1_point[4].y*2<<"\n";
	fout1<<src1_point[5].x*2<<","<<src1_point[5].y*2<<"\n";
	fout1<<src1_point[6].x*2<<","<<src1_point[6].y*2<<"\n";
	fout1<<src1_point[7].x*2<<","<<src1_point[7].y*2<<"\n";
#endif
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
#if 1
  cv::namedWindow("video0");
  cv::namedWindow("video1");
#endif
  setMouseCallback("video0",src0_onMouse,0);
  setMouseCallback("video1",src1_onMouse,0);
  
  
  
  
  cv::startWindowThread();
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub0 = it.subscribe( "/back_camera0/image_raw", 1, imageCalllback0 );
  image_transport::Subscriber sub1 = it.subscribe( "/back_camera1/image_raw", 1, imageCalllback1 );
  
  
  ros::spin();
  cv::destroyWindow("video0");
  cv::destroyWindow("video1");
  return 0;
}
