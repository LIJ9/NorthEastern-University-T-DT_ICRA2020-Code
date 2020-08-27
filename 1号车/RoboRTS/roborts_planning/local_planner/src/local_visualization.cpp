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

#include "local_planner/local_visualization.h"

namespace roborts_local_planner {
LocalVisualization::LocalVisualization() : initialized_(false){

}
LocalVisualization::LocalVisualization(ros::NodeHandle &nh, const std::string &visualize_frame) : initialized_(false){
  Initialization(nh, visualize_frame);
}
void LocalVisualization::Initialization(ros::NodeHandle &nh, const std::string &visualize_frame) {
  if (initialized_) {

  }

  visual_frame_ = visualize_frame;
  local_planner_ = nh.advertise<nav_msgs::Path>("trajectory", 1);
  pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("pose", 1);

  initialized_ = true;
}

void LocalVisualization::PublishLocalPlan(const TebVertexConsole& vertex_console) const{

  nav_msgs::Path local_plan;
  local_plan.header.frame_id = visual_frame_;
  local_plan.header.stamp = ros::Time::now();

  for (int i = 0; i <vertex_console.SizePoses(); ++i) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = local_plan.header.frame_id;
    pose_stamped.header.stamp = local_plan.header.stamp;
    pose_stamped.pose.position.x = vertex_console.Pose(i).GetPosition().coeffRef(0);
    pose_stamped.pose.position.y = vertex_console.Pose(i).GetPosition().coeffRef(1);
    pose_stamped.pose.position.z = 0;
    pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(vertex_console.Pose(i).GetTheta());
    
     
    
    float s=pose_stamped.pose.position.x;
    float h=pose_stamped.pose.position.y;
    
    if (s>5 && s <16 && h>63 && h<72)   //F1
    { 
      if(h<-1*s+76){
	//ROS_ERROR("PPPPPPPPPPPPPPPPPPPPPPPPPP");
      pose_stamped.pose.position.x += (h-57)*tan(vertex_console.Pose(i).GetTheta());  //往外移6
      pose_stamped.pose.position.y = 57;
      }
      pose_stamped.pose.position.x = 22;
      pose_stamped.pose.position.y +=(22-s)*tan(vertex_console.Pose(i).GetTheta());
      
    }
    if (pose_stamped.pose.position.x>33 && pose_stamped.pose.position.x <44 && pose_stamped.pose.position.y>34 && pose_stamped.pose.position.y<44)  //F2
    {
      	ROS_ERROR("PPPPPPPPPPPPPPPPPPPPPPPPPP");
     // if (vertex_console.Pose(i).GetTheta()>-1.57&&vertex_console.Pose(i).GetTheta()<1.57||vertex_console.Pose(i).GetTheta()>-3.2&&vertex_console.Pose(i).GetTheta()<){
	
	pose_stamped.pose.position.x += ( pose_stamped.pose.position.y-22)*tan(vertex_console.Pose(i).GetTheta());
	pose_stamped.pose.position.y = 10;
     
    }
//     if (s>75 && s <86 &&  h>87 && h<97)   //F3
//     {
//       pose_stamped.pose.position.x = 23;  //往外移7
//       pose_stamped.pose.position.x = 56;
//     }
//     if (pose_stamped.pose.position.x>5 && pose_stamped.pose.position.x <16 &&  pose_stamped.pose.position.y>63 &&  pose_stamped.pose.position.y<72)
//     {
//       pose_stamped.pose.position.x = 23;  //往外移7
//       pose_stamped.pose.position.x = 56;
//     }
//     if (pose_stamped.pose.position.x>5 && pose_stamped.pose.position.x <16 &&  pose_stamped.pose.position.y>63 &&  pose_stamped.pose.position.y<72)
//     {
//       pose_stamped.pose.position.x = 23;  //往外移7
//       pose_stamped.pose.position.x = 56;
//     }
//     if (pose_stamped.pose.position.x>5 && pose_stamped.pose.position.x <16 &&  pose_stamped.pose.position.y>63 &&  pose_stamped.pose.position.y<72)
//     {
//       pose_stamped.pose.position.x = 23;  //往外移7
//       pose_stamped.pose.position.x = 56;
//     }
    
    //     
    
//        F1.linear.x = 5;//left_low.x
//    F1.linear.y = 63;//left_low.y
//    F1.angular.x = 16;//right_low.x
//    F1.angular.y = 72;//left_up.y
//    
//    F2.linear.x = 33;//left_low.x
//    F2.linear.y = 34;//left_low.y
//    F2.angular.x = 44;//right_low.x
//    F2.angular.y = 44;//left_up.y
//    
//    F3.linear.x = 75;//left_low.x
//    F3.linear.y = 87;//left_low.y
//    F3.angular.x = 86;//right_low.x
//    F3.angular.y = 97;//left_up.y
//    
//    F4.linear.x = 147;//left_low.x
//    F4.linear.y = 30;//left_low.y
//    F4.angular.x = 158;//right_low.x
//    F4.angular.y = 40;//left_up.y
//    
//    F5.linear.x = 119;//left_low.x
//    F5.linear.y = 59;//left_low.y
//    F5.angular.x = 130;//right_low.x
//    F5.angular.y = 68;//left_up.y
//    
//    
//    F6.linear.x = 77;//left_low.x
//    F6.linear.y = 6;//left_low.y
//    F6.angular.x = 88;//right_low.x
//    F6.angular.y = 15;//left_up.y
//   
    
    
    
//     if (a1>32 && a1 <45 && a2>73 && a2<32 ||
// 	 a1>32 && a1<45 && a2>33 && a2<45 ||
// 	a1>74  && a2<87 && a2>86 && a2<98 ||
// 	a1>146 && a1<159 && a2>29 && a2<41 ||
// 	a1>118 && a1<131 && a2>58 && a2<69 ||
// 	a1>76 && a1<89 &&a2>5 && a2<16)
    
    
    local_plan.poses.push_back(pose_stamped);
  }
  local_planner_.publish(local_plan);
}

} // namespace roborts_local_planner

