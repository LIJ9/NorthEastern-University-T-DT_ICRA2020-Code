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
#include "obstacle_layer_setting.pb.h"
#include "obstacle_layer.h"

namespace roborts_costmap {

void ObstacleLayer::OnInitialize() {
  ros::NodeHandle nh;
  ParaObstacleLayer para_obstacle;
  
  ////////////////////////////////
//    ros::NodeHandle h;
// area_sub_ = h.subscribe("simulate_area_status_msg", 1, &ObstacleLayer::areastatusCallback, this);

////
  std::string obstacle_map = ros::package::getPath("roborts_costmap") + \
      "/config/obstacle_layer_config.prototxt";
  roborts_common::ReadProtoFromTextFile(obstacle_map.c_str(), &para_obstacle);
  double observation_keep_time = 0.1, expected_update_rate = 10.0, min_obstacle_height = 0.2, \
 max_obstacle_height = 0.6, obstacle_range = 2.5, raytrace_range = 3.0, transform_tolerance = 0.2;
  observation_keep_time = para_obstacle.observation_keep_time();
  expected_update_rate = para_obstacle.expected_update_rate();
  transform_tolerance = para_obstacle.transform_tolerance();
  max_obstacle_height = para_obstacle.max_obstacle_height();
  min_obstacle_height = para_obstacle.min_obstacle_height();
  obstacle_range = para_obstacle.obstacle_range();
  raytrace_range = para_obstacle.raytrace_range();
  max_obstacle_height_ = max_obstacle_height;
  footprint_clearing_enabled_ = para_obstacle.footprint_clearing_enabled();
  std::string topic_string = "LaserScan", sensor_frame = "laser_frame";
  topic_string = para_obstacle.topic_string();
  sensor_frame = para_obstacle.sensor_frame();

  bool inf_is_valid = false, clearing = false, marking = true;
  inf_is_valid = para_obstacle.inf_is_valid();
  clearing = para_obstacle.clearing();
  marking = para_obstacle.marking();
  rolling_window_ = layered_costmap_->IsRollingWindow();
  bool track_unknown_space = layered_costmap_->IsTrackingUnknown();
// F1.linear.x = 5;//left_low.x
//    F1.linear.y = 51;//left_low.y
//    F1.angular.x = 16;//right_low.x
//    F1.angular.y = 61;//left_up.y
//    
//    F2.linear.x = 33;//left_low.x
//    F2.linear.y = 28;//left_low.y
//    F2.angular.x = 44;//right_low.x
//    F2.angular.y = 38;//left_up.y
//    
//    F3.linear.x = 76;//left_low.x
//    F3.linear.y = 77;//left_low.y
//    F3.angular.x = 87;//right_low.x
//    F3.angular.y = 86;//left_up.y
//    
//    F4.linear.x = 147;//left_low.x
//    F4.linear.y = 29;//left_low.y
//    F4.angular.x = 158;//right_low.x
//    F4.angular.y = 39;//left_up.y
//    
//    F5.linear.x = 119;//left_low.x
//    F5.linear.y = 52;//left_low.y
//    F5.angular.x = 130;//right_low.x
//    F5.angular.y = 62;//left_up.y
//    
//    
//    F6.linear.x = 76;//left_low.x
//    F6.linear.y = 4;//left_low.y
//    F6.angular.x = 87;//right_low.x
//    F6.angular.y = 14;//left_up.y
  
  if (track_unknown_space) {
    default_value_ = NO_INFORMATION;
  } else {
    default_value_ = FREE_SPACE;
  }
  is_current_ = true;
  global_frame_ = layered_costmap_->GetGlobalFrameID();
  ObstacleLayer::MatchSize();
  observation_buffers_.push_back(std::shared_ptr<ObservationBuffer>(new ObservationBuffer(topic_string,
                                                                                            observation_keep_time,
                                                                                            expected_update_rate,
                                                                                            min_obstacle_height,
                                                                                            max_obstacle_height,
                                                                                            obstacle_range,
                                                                                            raytrace_range,
                                                                                            *tf_,
                                                                                            global_frame_,
                                                                                            sensor_frame,
                                                                                            transform_tolerance)));
  if (marking) {
    marking_buffers_.push_back(observation_buffers_.back());
  }
  if (clearing) {
    clearing_buffers_.push_back(observation_buffers_.back());
  } 
  reset_time_ = std::chrono::system_clock::now();
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>
  > sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, topic_string, 50));
  std::shared_ptr<tf::MessageFilter<sensor_msgs::LaserScan>
  > filter(new tf::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50));
  if (inf_is_valid) {
    filter->registerCallback(
        boost::bind(&ObstacleLayer::LaserScanValidInfoCallback, this, _1, observation_buffers_.back()));
  } else {
    filter->registerCallback(
        boost::bind(&ObstacleLayer::LaserScanCallback, this, _1, observation_buffers_.back()));
  }
  observation_subscribers_.push_back(sub);
  observation_notifiers_.push_back(filter);
  observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
  std::vector<std::string> target_frames;
  target_frames.push_back(global_frame_);
  target_frames.push_back(sensor_frame);
  observation_notifiers_.back()->setTargetFrames(target_frames);
  is_enabled_ = true;
}

void ObstacleLayer::LaserScanCallback(const sensor_msgs::LaserScanConstPtr &message,
                                      const std::shared_ptr<ObservationBuffer> &buffer) {
  sensor_msgs::PointCloud2 temp_cloud;
  temp_cloud.header = message->header;
  try {
    projector_.transformLaserScanToPointCloud(temp_cloud.header.frame_id, *message, temp_cloud, *tf_);
  }
  catch (tf::TransformException &ex) {
    projector_.projectLaser(*message, temp_cloud);
  }
  buffer->Lock();
  buffer->BufferCloud(temp_cloud);
  buffer->Unlock();
}

void ObstacleLayer::LaserScanValidInfoCallback(const sensor_msgs::LaserScanConstPtr &raw_message,     //距离等于最大的距离的时候会出现障碍物无法清除
                                               const std::shared_ptr<ObservationBuffer> &buffer) {
  float epsilon = 0.0001, range;
  sensor_msgs::LaserScan message = *raw_message;
  for (size_t i = 0; i < message.ranges.size(); i++) {
    range = message.ranges[i]; 
    if (!std::isfinite(range) && range > 0) {  //  if ((!std::isfinite(range) && (range > 0)) || (range >= message.range_max))
      message.ranges[i] = message.range_max - epsilon;
    }
  }
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message.header;
  try {
    projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("High fidelity enabled, but TF returned a transform exception to frame %s: %s", \
        global_frame_.c_str(), ex.what());
    projector_.projectLaser(message, cloud);
  }
  buffer->Lock();
  buffer->BufferCloud(cloud);
  buffer->Unlock();
}

void ObstacleLayer::UpdateBounds(double robot_x,
                                 double robot_y,
                                 double robot_yaw,
                                 double *min_x,
                                 double *min_y,
                                 double *max_x,
                                 double *max_y) {
  if (rolling_window_) {
    UpdateOrigin(robot_x - GetSizeXWorld() / 2, robot_y - GetSizeYWorld() / 2);
  } else if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - reset_time_) > std::chrono::seconds(2)){
    reset_time_ = std::chrono::system_clock::now();
    ResetMaps();
  }
  if (!is_enabled_) {
    ROS_ERROR("Obstacle layer is not enabled.");
    return;
  }
  UseExtraBounds(min_x, min_y, max_x, max_y);
  bool temp_is_current = true;
  std::vector<Observation> observations, clearing_observations;
  temp_is_current = temp_is_current && GetMarkingObservations(observations);
  temp_is_current = temp_is_current && GetClearingObservations(clearing_observations);
  is_current_ = temp_is_current;

  // raytrace freespace
  for (unsigned int i = 0; i < clearing_observations.size(); ++i) {
    RaytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  }

  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); it++) {
    const Observation obs = *it;
    const pcl::PointCloud<pcl::PointXYZ> &cloud = *(obs.cloud_);
    double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;
    for (unsigned int i = 0; i < cloud.points.size(); ++i) {
      double px = cloud.points[i].x, py = cloud.points[i].y, pz = cloud.points[i].z;

      // if the obstacle is too high or too far away from the robot we won't add it
      if (pz > max_obstacle_height_) {
        continue;
      }
      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y)
          + (pz - obs.origin_.z) * (pz - obs.origin_.z);

      // if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_range) {
        continue;
      }

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my;
      if (!World2Map(px, py, mx, my)) {
        continue;
      }
      unsigned int index = GetIndex(mx, my);
      costmap_[index] = LETHAL_OBSTACLE;

      
      Touch(px, py, min_x, min_y, max_x, max_y);
    }
  }
  UpdateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}
// void ObstacleLayer::areastatusCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) //master_grid
// {
//   area[0]=(int)msg->data[0];
//   area[1]=(int)msg->data[1];
//   area[2]=(int)msg->data[2];
//   area[3]=(int)msg->data[3];
//   area[4]=(int)msg->data[4];
//   area[5]=(int)msg->data[5];   
// }


void ObstacleLayer::UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {
 
//    for(int k=0;k<6;k++)
//     {
//        if(area[k]<3){//0<=area[0]<=2){
//        F[k]=7; //可通过
//        }
//        else
//        {
// 	 F[k]=8;  //不可通过
//        }
// //        ROS_ERROR_STREAM("F["<<k<<"] = "<<F[k]);
//     }
//         /////////////
//   int tmp_suibian = Costmap2D::Zhaojingyu(master_grid);
//   
  
  
//   if(tmp_suibian == 163)//tmp_suibian == 163)
//   {
//   
//     //ROS_ERROR_STREAM("nnnnnnn="<<tmp_suibian);
//     for (auto i = 0; i < 90; i++) //y  90
//   {
//     for (auto j = 0; j < 163; j++) //x
//     {
//        if(i>=F1.linear.y&i<=F1.angular.y&j>=F1.linear.x&j<=F1.angular.x)//&&F[0]==8)
//        {
// 	 if(F[0]==8)
// 	 {
// 	   unsigned int tmp_index = GetIndex(j, i);
//  	costmap_[tmp_index] = LETHAL_OBSTACLE;//255;
// 	
// 	 }
// 	else
// 	  continue;
// 	  
//        }
//    /*    
//        F1.linear.x = 5;//left_low.x
//    F1.linear.y = 51;//left_low.y
//    F1.angular.x = 16;//right_low.x
//     F1.angular.y = 61;//left_up.y*/
//    if(j>15&&j<17&&i>50&&i<63||j>0&&j<17&&i>50&&i<52){
// 	    if(F[0]==8)
// 	 {
// 	  unsigned int tmp_index = GetIndex(j, i);
// 	  costmap_[tmp_index] = INSCRIBED_INFLATED_OBSTACLE; 
// 	   } 
// 	   continue;
// 	}
// 	
//    
//    
   
// 	if(i>=F2.linear.y&i<=F2.angular.y&j>=F2.linear.x&j<=F2.angular.x)//&&)//F[1]==8)
// 	{
// 	  if(F[1]==8)
// 	 {
// 	  unsigned int tmp_index = GetIndex(j, i);
// 	  costmap_[tmp_index] = LETHAL_OBSTACLE;
// 	   
// 	   } 
// 	else
// 	  continue;
// 	}
// //    F2.linear.x = 33;//left_low.x
// //    F2.linear.y = 28;//left_low.y
// //    F2.angular.x = 44;//right_low.x
// //    F2.angular.y = 38;//left_up.y
// 	if(j>32&&j<34&&i>24&&i<39||j>43&&j<45&&i>24&&i<39||j>32&&j<45&&i>24&&i<29){
// 	    if(F[1]==8)
// 	 {
// 	  unsigned int tmp_index = GetIndex(j, i);
// 	  costmap_[tmp_index] = INSCRIBED_INFLATED_OBSTACLE;
// 	   
// 	   } 
// 	   continue;
// 	}
// 	
// 	
// 
// 	
//         if(i>=F3.linear.y&i<=F3.angular.y&j>=F3.linear.x&j<=F3.angular.x)//&&F[2]==8)
//         {
// 	  if(F[2]==8)
// 	 {
// 	  unsigned int tmp_index = GetIndex(j, i);
// 	  costmap_[tmp_index] = LETHAL_OBSTACLE;
// 	 
// 	   }
// 	   
// 	else
// 	  continue;
//         }
//         
//         	   /*
// 	     F3.linear.x = 76;//left_low.x
//    F3.linear.y = 77;//left_low.y
//    F3.angular.x = 87;//right_low.x
//    F3.angular.y = 86;//left_up.y*/
//    if(j>75&&j<77&&i>72&&i<90||j>86&&j<88&&i>72&&i<90){
// 	    if(F[2]==8)
// 	 {
// 	  unsigned int tmp_index = GetIndex(j, i);
// 	  costmap_[tmp_index] = INSCRIBED_INFLATED_OBSTACLE;
// 	   
// 	   } 
// 	   continue;
// 	}
// 
// 	  
//        if(i>=F4.linear.y&i<=F4.angular.y&j>=F4.linear.x&j<=F4.angular.x)//&&F[3]==8)
//        {
// 	if(F[3]==8)
// 	 {
// 	 unsigned int tmp_index = GetIndex(j, i);
// 	costmap_[tmp_index] = LETHAL_OBSTACLE;
// 	   }
// 	else
// 	  continue;
//        }
//        	
// //                  F4.linear.x = 147;//left_low.x
// //    F4.linear.y = 29;//left_low.y
// //    F4.angular.x = 158;//right_low.x
// //    F4.angular.y = 39;//left_up.y
// 
//    if(j>146&&j<148&&i>27&&i<40||j>146&&j<163&&i>38&&i<40){
// 	    if(F[3]==8)
// 	 {
// 	  unsigned int tmp_index = GetIndex(j, i);
// 	  costmap_[tmp_index] = INSCRIBED_INFLATED_OBSTACLE;
// 	   
// 	   } 
// 	   continue;
// 	}
// 	
//       
//        if(i>=F5.linear.y&i<=F5.angular.y&j>=F5.linear.x&j<=F5.angular.x)//&&F[4]==8)
//        {
// 	 if(F[4]==8)
// 	 {
//  	unsigned int tmp_index = GetIndex(j, i);
//  	costmap_[tmp_index] = LETHAL_OBSTACLE;
// 	 }
// 	else
// 	  continue;
//        }
//        ////
// /*        
//    F5.linear.x = 119;//left_low.x
//    F5.linear.y = 52;//left_low.y
//    F5.angular.x = 130;//right_low.x
//    F5.angular.y = 62;//left_up.y*/
//    
// 	if(j>119&&j<130&&i>61&&i<68||j>118&&j<120&&i>50&&i<68||j>129&&j<131&&i>50&&i<68){
// 	    if(F[4]==8)
// 	 {
// 	  unsigned int tmp_index = GetIndex(j, i);
// 	  costmap_[tmp_index] = INSCRIBED_INFLATED_OBSTACLE;
// 	   
// 	   } 
// 	   continue;
// 	}
// // 	
// 	////////
//        if(i>=F6.linear.y&i<=F6.angular.y&j>=F6.linear.x&j<=F6.angular.x)//&&F[5]==8)
//        {
// 	 if(F[5]==8)
// 	 {
//  	unsigned int tmp_index = GetIndex(j, i);
//  	costmap_[tmp_index] = LETHAL_OBSTACLE;
// 	}
// 	else
// 	  continue;
//        }
// //    F6.linear.x = 76;//left_low.x
// //    F6.linear.y = 4;//left_low.y
// //    F6.angular.x = 87;//right_low.x
// //    F6.angular.y = 14;//left_up.y
// 	if(j>75&&j<77&&i>0&&i<16||j>86&&j<88&&i>0&&i<16){
// 	    if(F[5]==8)
// 	 {
// 	  unsigned int tmp_index = GetIndex(j, i);
// 	  costmap_[tmp_index] = INSCRIBED_INFLATED_OBSTACLE;
// 	   
// 	   } 
// 	   continue;
// 	}
//     }
//     
//   }
//   
//       }
  
  if (!is_enabled_) {
    ROS_WARN("Obstacle layer is not enabled");
    return;
  }
  if (footprint_clearing_enabled_) {
    SetConvexRegionCost(transformed_footprint_, FREE_SPACE);
  }
  combination_method_ = 1;
  switch (combination_method_) {
    case 0:  // Overwrite
      UpdateOverwriteByValid(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      UpdateOverwriteByMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

void ObstacleLayer::Activate() {
  for (size_t i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != nullptr) {
      observation_subscribers_[i]->subscribe();
    }
  }
  for (size_t i = 0; i < observation_buffers_.size(); ++i) {
    if (observation_buffers_[i] != nullptr) {
      observation_buffers_[i]->ResetLastUpdated();
    }
  }
}
void ObstacleLayer::Deactivate() {
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != nullptr)
      observation_subscribers_[i]->unsubscribe();
  }
}

void ObstacleLayer::Reset() {
  Deactivate();
  ResetMaps();
  is_current_ = true;
  Activate();
}

bool ObstacleLayer::GetMarkingObservations(std::vector<Observation> &marking_observations) const {
  bool current = true;
  // get the marking observations
  for (size_t i = 0; i < marking_buffers_.size(); ++i) {
    marking_buffers_[i]->Lock();
    marking_buffers_[i]->GetObservations(marking_observations);
    current = marking_buffers_[i]->IsCurrent() && current;
    marking_buffers_[i]->Unlock();
  }
  marking_observations.insert(marking_observations.end(),
                              static_marking_observations_.begin(), static_marking_observations_.end());
  return current;
}

bool ObstacleLayer::GetClearingObservations(std::vector<Observation> &clearing_observations) const {
  bool current = true;
  // get the clearing observations
  for (unsigned int i = 0; i < clearing_buffers_.size(); ++i) {
    clearing_buffers_[i]->Lock();
    clearing_buffers_[i]->GetObservations(clearing_observations);
    current = clearing_buffers_[i]->IsCurrent() && current;
    clearing_buffers_[i]->Unlock();
  }
  clearing_observations.insert(clearing_observations.end(),
                               static_clearing_observations_.begin(), static_clearing_observations_.end());
  return current;
}

void ObstacleLayer::RaytraceFreespace(const Observation &clearing_observation,
                                      double *min_x,
                                      double *min_y,
                                      double *max_x,
                                      double *max_y) {
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  pcl::PointCloud<pcl::PointXYZ> cloud = *(clearing_observation.cloud_);

  // get the map coordinates of the origin of the sensor
  unsigned int x0, y0;
  if (!World2Map(ox, oy, x0, y0)) {
    return;
  }

  // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  double origin_x = origin_x_, origin_y = origin_y_;
  double map_end_x = origin_x + size_x_ * resolution_;
  double map_end_y = origin_y + size_y_ * resolution_;

  Touch(ox, oy, min_x, min_y, max_x, max_y);

  // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
  for (unsigned int i = 0; i < cloud.points.size(); ++i) {
    double wx = cloud.points[i].x;
    double wy = cloud.points[i].y;
   
    //////////////////////////////////////
    unsigned int cell_raytrace_range = World2Cell(clearing_observation.raytrace_range_);
     unsigned int x1, y1;
     
    //////////////////////////////
    
    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the map and scale if necessary
    double a = wx - ox;
    double b = wy - oy;

    // the minimum value to raytrace from is the origin
    if (wx < origin_x) {
      double t = (origin_x - ox) / a;
      wx = origin_x;
      wy = oy + b * t;
    }
    if (wy < origin_y) {
      double t = (origin_y - oy) / b;
      wx = ox + a * t;
      wy = origin_y;
    }

    // the maximum value to raytrace to is the end of the map
    if (wx > map_end_x) {
      double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    if (wy > map_end_y) {
      double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }

    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
//     unsigned int x1, y1;

    // check for legality just in case
    if (!World2Map(wx, wy, x1, y1))    ////// 有意义么?
      continue;

   // unsigned int cell_raytrace_range = World2Cell(clearing_observation.raytrace_range_);
    MarkCell marker(costmap_, FREE_SPACE);
    // and finally... we can execute our trace to clear obstacles along that line
    RaytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);

    UpdateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);
    
    
//   }
    
    
    
  }
}

void ObstacleLayer::UpdateRaytraceBounds(double ox,
                                         double oy,
                                         double wx,
                                         double wy,
                                         double range,
                                         double *min_x,
                                         double *min_y,
                                         double *max_x,
                                         double *max_y) {
  double dx = wx - ox, dy = wy - oy;
  double full_distance = hypot(dx, dy);
  double scale = std::min(1.0, range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  Touch(ex, ey, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::UpdateFootprint(double robot_x,
                                    double robot_y,
                                    double robot_yaw,
                                    double *min_x,
                                    double *min_y,
                                    double *max_x,
                                    double *max_y) {
  if (!footprint_clearing_enabled_)
    return;
  TransformFootprint(robot_x, robot_y, robot_yaw, GetFootprint(), transformed_footprint_);

  for (size_t i = 0; i < transformed_footprint_.size(); i++) {
    Touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

} //namespace roborts_costmap
