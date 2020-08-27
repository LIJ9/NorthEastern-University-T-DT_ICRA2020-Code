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

#include <cv_bridge/cv_bridge.h>

#include "camera_node.h"

namespace roborts_camera{
//构造函数
CameraNode::CameraNode() 
{  
  //Do GetCameraParam
  camera_num_ = camera_param_.GetCameraParam().size();
  std::cout<<"camera_num_"<<camera_num_<<std::endl; 
  img_pubs_.resize(camera_num_);
  camera_threads_.resize(camera_num_);
  camera_driver_.resize(camera_num_);
  for (unsigned int i = 0; i < camera_num_; i++)
  {     
    
    //Do GetCameraParam
    auto camera_info = camera_param_.GetCameraParam()[i];
    
    nhs_.push_back(ros::NodeHandle(camera_info.camera_name));
    image_transport::ImageTransport it(nhs_.at(i));
    img_pubs_[i] = it.advertiseCamera("image_raw", 1, true);
    //create the selected camera driver
    camera_driver_[i] = roborts_common::AlgorithmFactory<CameraBase,CameraInfo>::CreateAlgorithm(camera_info.camera_type,camera_info);
  }    
  //Do StartThread
  StartThread();
}

void CameraNode::StartThread()
{
  running_ = true;
  for (unsigned int i = 0; i < camera_num_; i++) 
  {    
    //Do Update
    camera_threads_[i] = std::thread(&CameraNode::Update, this, i);
  }
}
void CameraNode::Update(const unsigned int index)
{
  cv::Mat img;
  bool camera_info_send = false;
  int j=0;
  while(running_) 
  {  
     camera_driver_[index]->StartReadCamera(img,index);
    if(!img.empty()) 
    {
      //cv::imshow("camera",img);
      sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
      img_msg->header.frame_id = camera_param_.GetCameraParam()[index].camera_name;
      //std::cout<<camera_param_.GetCameraParam()[index].camera_path<<std::endl;
      img_msg->header.stamp = ros::Time::now();
      camera_param_.GetCameraParam()[index].ros_camera_info->header.stamp = img_msg->header.stamp;
      img_pubs_[index].publish(img_msg, camera_param_.GetCameraParam()[index].ros_camera_info);
    }   
  }
}

//StoptThread() has never used! by Anthony
void CameraNode::StoptThread() 
{
//TODO: To be implemented
}

//When received ctrl C , do this secondly,析构函数
CameraNode::~CameraNode()
{
  running_ = false;
  for (auto &iter: camera_threads_)
  {
    if (iter.joinable())
      iter.join();
  }
}
} //namespace roborts_camera

//When received ctrl C , do this firstly
void SignalHandler(int signal)
{
  if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown())
  {
    ros::shutdown();
  }
}

int main(int argc, char **argv)
{
  signal(SIGINT, SignalHandler);
  signal(SIGTERM,SignalHandler);
  ros::init(argc, argv, "roborts_camera_node", ros::init_options::NoSigintHandler);
  //Do camera_param.cpp-- LoadCameraParam
  roborts_camera::CameraNode camera_test;
  ros::AsyncSpinner async_spinner(2);
  async_spinner.start();
  ros::waitForShutdown();
  return 0;
}
