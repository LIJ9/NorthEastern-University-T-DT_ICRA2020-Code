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
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <thread>
#include <csignal>

#include "costmap_interface.h"

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
//#include "laser_geometry/laser_geometry.h"
#include <laser_geometry/laser_geometry.h>

#include <geometry_msgs/PoseStamped.h>
class LaserScanToPointCloud{
 
public:
 
   ros::NodeHandle n_;
   laser_geometry::LaserProjection projector_;
   tf::TransformListener listener_;
   message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
   tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
   ros::Publisher scan_pub_;
   ros::Publisher enemy_position_pub_;
   
 
   LaserScanToPointCloud(ros::NodeHandle n) : 
     n_(n),
     laser_sub_(n_, "scan", 10),
     
     //laser_notifier_(laser_sub_,listener_, "base_link", 10)
     laser_notifier_(laser_sub_,listener_, "base_link", 10)
   {
     laser_notifier_.registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
     laser_notifier_.setTolerance(ros::Duration(0.1));
     scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",1);
     enemy_position_pub_ = n.advertise<geometry_msgs::PoseStamped>("/enemy_laser",1);
   }

   void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
   {
     ros::Rate loop_rate(10);
     sensor_msgs::PointCloud cloud;
     geometry_msgs::PoseStamped enemy_position;
     try
     {
         //projector_.transformLaserScanToPointCloud("base_link",*scan_in, cloud,listener_);
	  projector_.transformLaserScanToPointCloud("map",*scan_in, cloud,listener_);
    }
     catch (tf::TransformException& e)
      {
        std::cout << e.what();
          return;
     }
      for (unsigned int i = 0; i < cloud.points.size(); ++i) {
      ///////////ROS_INFO("size:%d",cloud.points.size());
     double px = cloud.points[i].x, py = cloud.points[i].y, pz = cloud.points[i].z;
     //int x=0,y=0;
     if (px>1&&px<7&&py>1&&py<4){
      if(px>1.0&&px<2.3&&py>1.0&&py<3.0){
	 continue;
      }
      if(px>0.8&&px<2.8&&py>3.3&&py<4.5){
	 continue;
      }
      if(px>4.0&&px<5.5&&py>3.5&&py<5.0){
	 continue;
      }
      if(px>6.0&&px<7.3&&py>2.2&&py<4.2){
	 continue;
      }
      if(px>5.5&&px<7.5&&py>0.5&&py<1.8){
	 continue;
      }
      if(px>3.0&&px<5.2&&py>1.9&&py<3.3){
	 continue;
      }
      if(px>2.9&&px<4.2&&py>0&&py<1.5){
	 continue;
      }
     /*  if((px>1.5&&px<1.8&&py>1.5&&py<2.5)||(px>1.3&&px<2.3&&py>3.8&&py<4)||(px>4.6&&px<4.9&&py>4&&py<5)||
	 (px>6.5&&px<6.8&&py>2.7&&py<3.7)||(px>6.0&&px<7.0&&py>1.0&&py<1.3)||(px>3.6&&px<4.7&&py>2.4&&py<2.7)||(px>3.4&&px<3.7&&py>0&&py<1.0))
	 continue;
	 */
     //将(px,py)坐标值四舍五入后发布
     ROS_INFO("%lf,%lf",px,py);
     //x=round(px);
     //y=round(py);
     
     //ROS_INFO("%d,%d",x,y);
     enemy_position.pose.position.x=px;
     enemy_position.pose.position.y=py;
     enemy_position.pose.orientation.x=0;
     enemy_position.pose.orientation.y=0;
     enemy_position.pose.orientation.z=0;
     enemy_position.pose.orientation.w=1;
     
     }
      }
     // Do something with cloud.
     if(enemy_position.pose.position.x!=0&&enemy_position.pose.position.y!=0){
     enemy_position_pub_.publish(enemy_position);
     loop_rate.sleep();
     }
     scan_pub_.publish(cloud);
     //
 
  }
};

int main(int argc, char** argv){
   
     ros::init(argc, argv, "my_scan_to_cloud");
     ros::NodeHandle n;
     

     LaserScanToPointCloud lstopc(n);
     
     ros::spin();
     //ros::spinOnce();
     
     return 0;
}


/*
void SignalHandler(int signal){
  if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
    ros::shutdown();
  }
}

int main(int argc, char** argv) {

  signal(SIGINT, SignalHandler);
  signal(SIGTERM,SignalHandler);
  ros::init(argc, argv, "test_costmap", ros::init_options::NoSigintHandler);
  tf::TransformListener tf(ros::Duration(10));

  std::string local_map = ros::package::getPath("roborts_costmap") + \
      "/config/costmap_parameter_config_for_local_plan.prototxt";
  roborts_costmap::CostmapInterface costmap_interface("map",
                                                      tf,
                                                      local_map.c_str());

  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  ros::waitForShutdown();
  return 0;
}
*/
