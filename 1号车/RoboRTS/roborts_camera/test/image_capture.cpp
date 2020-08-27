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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <vector>
#include <thread>
#include <mutex>
#include <string>
#include <sstream>
#include<time.h>

#include "opencv2/opencv.hpp"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

cv::Mat src_img;
void ReceiveImg(const sensor_msgs::ImageConstPtr &msg) {
  src_img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  std::cout<<"111"<<std::endl;
  //std::cout<<" real topic_name is "<<topic_name<<std::endl;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "image_capture");
  ros::NodeHandle nh;
  std::time_t now_time=time(NULL);  
  tm*  t_tm = localtime(&now_time);
  //std::printf("local time is    : %s\n", asctime(t_tm));
  std::string topic_name;
  topic_name = asctime(t_tm);
  cv::VideoWriter writer("/home/tdt/catkin_ws/src/RoboRTS-ros/roborts_debug/"+topic_name+"tdt_2.avi", CV_FOURCC('M', 'J', 'P', 'G'), 250.0, cv::Size(640, 480));
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/video_img", 20, boost::bind(&ReceiveImg, _1));
  while (nh.ok())
  {
    if(!src_img.empty())
      writer.write(src_img);
    ros::spinOnce();
  }
  return 0;
}
