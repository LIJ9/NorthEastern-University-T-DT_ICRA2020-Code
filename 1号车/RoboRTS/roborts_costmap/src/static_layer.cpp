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
#include "static_layer_setting.pb.h"
#include "static_layer.h"

namespace roborts_costmap {

void StaticLayer::OnInitialize() {
 
     ros::NodeHandle h;
   area_sub_ = h.subscribe("area_status_msg", 1, &StaticLayer::areastatusCallback, this);/*simulate_*/

  ros::NodeHandle nh;
  is_current_ = true;
  ParaStaticLayer para_static_layer;

  std::string static_map = ros::package::getPath("roborts_costmap") + \
      "/config/static_layer_config.prototxt";
  roborts_common::ReadProtoFromTextFile(static_map.c_str(), &para_static_layer);
  global_frame_ = layered_costmap_-> GetGlobalFrameID();
  first_map_only_ = para_static_layer.first_map_only();
  subscribe_to_updates_ = para_static_layer.subscribe_to_updates();
  track_unknown_space_ = para_static_layer.track_unknown_space();
  use_maximum_ = para_static_layer.use_maximum();
  int temp_threshold = para_static_layer.lethal_threshold();
  lethal_threshold_ = std::max(std::min(100, temp_threshold), 0);
  trinary_costmap_ = para_static_layer.trinary_map();
  unknown_cost_value_ = para_static_layer.unknown_cost_value();
  map_received_ = false;
  bool is_debug_ = para_static_layer.is_debug();
  map_topic_ = para_static_layer.topic_name();
  map_sub_ = nh.subscribe(map_topic_.c_str(), 1, &StaticLayer::InComingMap, this);
  ros::Rate temp_rate(10);
  while(!map_received_) {
    ros::spinOnce();
    temp_rate.sleep();
  }
  staic_layer_x_ = staic_layer_y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  is_enabled_ = true;
  has_updated_data_ = true;
    F1.linear.x = 5;//left_low.x
   F1.linear.y = 63;//left_low.y
   F1.angular.x = 16;//right_low.x
   F1.angular.y = 72;//left_up.y
   
   F2.linear.x = 32;//left_low.x
   F2.linear.y = 32;//left_low.y
   F2.angular.x = 45;//right_low.x
   F2.angular.y = 44;//left_up.y
   
   F3.linear.x = 75;//left_low.x
   F3.linear.y = 90;//left_low.y
   F3.angular.x = 86;//right_low.x
   F3.angular.y = 100;//left_up.y
   
   F4.linear.x = 147;//left_low.x
   F4.linear.y = 30;//left_low.y
   F4.angular.x = 158;//right_low.x
   F4.angular.y = 40;//left_up.y
   
   F5.linear.x = 119;//left_low.x
   F5.linear.y = 60;//left_low.y
   F5.angular.x = 130;//right_low.x
   F5.angular.y = 71;//left_up.y
   
   
   F6.linear.x = 77;//left_low.x
   F6.linear.y = 5;//left_low.y
   F6.angular.x = 88;//right_low.x
   F6.angular.y = 15;//left_up.y
}
void StaticLayer::areastatusCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) //master_grid
{
  area[0]=(int)msg->data[0];
  area[1]=(int)msg->data[1];
  area[2]=(int)msg->data[2];
  area[3]=(int)msg->data[3];
  area[4]=(int)msg->data[4];
  area[5]=(int)msg->data[5];   
}

void StaticLayer::MatchSize() {
  if (!layered_costmap_->IsRolling()) {
    Costmap2D* master = layered_costmap_->GetCostMap();
    ResizeMap(master->GetSizeXCell(), master->GetSizeYCell(), master->GetResolution(),
              master->GetOriginX(), master->GetOriginY());
  }
}
void StaticLayer::InComingMap(const nav_msgs::OccupancyGridConstPtr &new_map) {
  unsigned int temp_index = 0;
  unsigned char value = 0;
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;
  
  
      auto resolution = new_map->info.resolution;
  auto origin_x = new_map->info.origin.position.x;
  auto origin_y = new_map->info.origin.position.y;
  auto master_map = layered_costmap_->GetCostMap();
  if(!layered_costmap_->IsRolling() && (master_map->GetSizeXCell() != size_x || master_map->GetSizeYCell() != size_y ||
      master_map->GetResolution() != resolution || master_map->GetOriginX() != origin_x || master_map->GetOriginY() != origin_y ||
      !layered_costmap_->IsSizeLocked())) {
    layered_costmap_->ResizeMap(size_x, size_y, resolution, origin_x, origin_y, true);
  } else if(size_x_ != size_x || size_y_ != size_y || resolution_ != resolution || origin_x_ != origin_x || origin_y_ != origin_y) {
    ResizeMap(size_x, size_y, resolution, origin_x, origin_y);
  }
  for (auto i = 0; i < size_y; i++) {
    for (auto j = 0; j < size_x; j++) {
      value = new_map->data[temp_index];
      costmap_[temp_index] = InterpretValue(value);
      ++temp_index;
    }
  }
  map_received_ = true;
  
  has_updated_data_ = true;
  map_frame_ = new_map->header.frame_id;
  staic_layer_x_ = staic_layer_y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  if (first_map_only_) {
    map_sub_.shutdown();
  }
}

unsigned char StaticLayer::InterpretValue(unsigned char value) {
  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (!track_unknown_space_ && value == unknown_cost_value_)
    return FREE_SPACE;
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;
  else if (trinary_costmap_)
    return FREE_SPACE;

  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void StaticLayer::Activate() {
  OnInitialize();
}

void StaticLayer::Deactivate() {
//    delete cost_map_;
  //shut down the map topic message subscriber
  map_sub_.shutdown();
}

void StaticLayer::Reset() {
  if(first_map_only_) {
    has_updated_data_ = true;
  } else {
    OnInitialize();
  }
}

void StaticLayer::UpdateBounds(double robot_x,
                               double robot_y,
                               double robot_yaw,
                               double *min_x,
                               double *min_y,
                               double *max_x,
                               double *max_y) { 
  double wx, wy;
  if(!layered_costmap_->IsRollingWindow()) {
    if(!map_received_ || !(has_updated_data_ || has_extra_bounds_)) {
      return;
    }
  }

  //just make sure the value is normal
  UseExtraBounds(min_x, min_y, max_x, max_y);
  Map2World(staic_layer_x_, staic_layer_y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);
  Map2World(staic_layer_x_+ width_, staic_layer_y_ + height_, wx, wy);
  *max_x = std::max(*max_x, wx);
  *max_y = std::max(*max_y, wy);
  has_updated_data_ = false;  
}
void StaticLayer::UpdateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
  
   for(int k=0;k<6;k++)
    {
       if(area[k]<3){//0<=area[0]<=2){
       F[k]=7; 
       }
       else
       {
	 F[k]=8;  
       }
    }
  int tmp_suibian = Costmap2D::Zhaojingyu(master_grid);
  
  
  
  if(tmp_suibian == 161)//tmp_suibian == 163)
  {
  
    //ROS_ERROR_STREAM("nnnnnnn="<<tmp_suibian);
    for (auto i = 0; i < 101; i++) //y  90
  {
    for (auto j = 0; j < 161; j++) //x
    {
       if(i>=F1.linear.y&i<=F1.angular.y&j>=F1.linear.x&j<=F1.angular.x)//&&F[0]==8)
       {
	 if(F[0]==8)
	 {
	   unsigned int tmp_index = GetIndex(j, i);
 	costmap_[tmp_index] = LETHAL_OBSTACLE;//255;
	   
	}
	if(F[0]==7){
	  unsigned int tmp_index = GetIndex(j, i);
 	costmap_[tmp_index] = FREE_SPACE;//255;
       }
       }
   if(j>16&&j<17&&i>62&&i<73||j>0&&j<18&&i>62&&i<63){  //
	    if(F[0]==8)
	 {
	  unsigned int tmp_index = GetIndex(j, i);
	  costmap_[tmp_index] = INSCRIBED_INFLATED_OBSTACLE; 
	   } 
	  	if(F[0]==7){
	  unsigned int tmp_index = GetIndex(j, i);
 	costmap_[tmp_index] = FREE_SPACE;//255;
       }
	}
	
	if(i>=F2.linear.y&i<=F2.angular.y&j>=F2.linear.x&j<=F2.angular.x)//&&)//F[1]==8)
	{
	  if(F[1]==8)
	 {
	  unsigned int tmp_index = GetIndex(j, i);
	  costmap_[tmp_index] = LETHAL_OBSTACLE;
	   
	   } 
	   if(F[1]==7){
	     unsigned int tmp_index = GetIndex(j, i);
	     costmap_[tmp_index] = FREE_SPACE;
	   }
	  }
	if(j>29&&j<47&&i>20&&i<32||j>29&&j<32&&i>20&&i<44||j>44&&j<47&&i>20&&i<44){  //  下 左    右  
	    if(F[1]==8)
	 {
	  unsigned int tmp_index = GetIndex(j, i);
	  costmap_[tmp_index] = INSCRIBED_INFLATED_OBSTACLE;
	   
	   } 
	   if(F[1]==7){
	     unsigned int tmp_index = GetIndex(j, i);
	     costmap_[tmp_index] = FREE_SPACE;
	     
	  }
	}
	
        if(i>=F3.linear.y&&i<=F3.angular.y&&j>=F3.linear.x&&j<=F3.angular.x)//&&F[2]==8)
        {
	  if(F[2]==8)
	 {
	  unsigned int tmp_index = GetIndex(j, i);
	  costmap_[tmp_index] = LETHAL_OBSTACLE;
	  
	   }
	      if(F[2]==7){
	     unsigned int tmp_index = GetIndex(j, i);
	     costmap_[tmp_index] = FREE_SPACE;
	  }
        }
        
   if(j>73&&j<75&&i>85&&i<101||j>85&&j<88&&i>85&&i<99){  //左
	    if(F[2]==8)
	 {
	  unsigned int tmp_index = GetIndex(j, i);
	  costmap_[tmp_index] = INSCRIBED_INFLATED_OBSTACLE;
	   
	   } 
	   if(F[2]==7){
	     unsigned int tmp_index = GetIndex(j, i);
	     costmap_[tmp_index] = FREE_SPACE;
	     
	  }
	}

       if(i>=F4.linear.y&i<=F4.angular.y&j>=F4.linear.x&j<=F4.angular.x)//&&F[3]==8)
       {
	if(F[3]==8)
	 {
	 unsigned int tmp_index = GetIndex(j, i);
	costmap_[tmp_index] = LETHAL_OBSTACLE;
	   }
	
	   if(F[3]==7){
	     unsigned int tmp_index = GetIndex(j, i);
	     costmap_[tmp_index] = FREE_SPACE;
	     
	  }
       }
       	
   if(j>145&&j<148&&i>28&&i<40||j>145&&j<160&&i>40&&i<42){  //左  上
	    if(F[3]==8)
	 {
	  unsigned int tmp_index = GetIndex(j, i);
	  costmap_[tmp_index] = INSCRIBED_INFLATED_OBSTACLE;
	   
	   } 
	     if(F[3]==7){
	     unsigned int tmp_index = GetIndex(j, i);
	     costmap_[tmp_index] = FREE_SPACE;
	     
	  }
	}
	
      
       if(i>=F5.linear.y&i<=F5.angular.y&j>=F5.linear.x&j<=F5.angular.x)//&&F[4]==8)
       {

	 if(F[4]==8)
	 {
 	unsigned int tmp_index = GetIndex(j, i);
 	costmap_[tmp_index] = LETHAL_OBSTACLE;
	 }
	  if(F[4]==7){
	     unsigned int tmp_index = GetIndex(j, i);
	     costmap_[tmp_index] = FREE_SPACE;
	     
	  }
       }
	if(j>116&&j<119&&i>55&&i<80||j>117&&j<132&&i>71&&i<80||j>130&&j<133&&i>55&&i<80){   //左  上 右
	    if(F[4]==8)
	 {
	  unsigned int tmp_index = GetIndex(j, i);
	  costmap_[tmp_index] = INSCRIBED_INFLATED_OBSTACLE;
	   
	   } 
	    if(F[4]==7){
	     unsigned int tmp_index = GetIndex(j, i);
	     costmap_[tmp_index] = FREE_SPACE;
	     
	  }
	}
       if(i>=F6.linear.y&i<=F6.angular.y&j>=F6.linear.x&j<=F6.angular.x)//&&F[5]==8)
       {
	 if(F[5]==8)
	 {
 	unsigned int tmp_index = GetIndex(j, i);
 	costmap_[tmp_index] = LETHAL_OBSTACLE;
	}
	  if(F[5]==7){
	     unsigned int tmp_index = GetIndex(j, i);
	     costmap_[tmp_index] = FREE_SPACE;
	     
	  }
       }
	if(j>75&&j<77&&i>0&&i<18||j>88&&j<90&&i>0&&i<18){  //左
	    if(F[5]==8)
	 {
	  unsigned int tmp_index = GetIndex(j, i);
	  costmap_[tmp_index] = INSCRIBED_INFLATED_OBSTACLE;
	   
	   } 
	   if(F[5]==7){
	     unsigned int tmp_index = GetIndex(j, i);
	     costmap_[tmp_index] = FREE_SPACE;
	  }
	}
      
    }
  }
  }
  
  
  if(!map_received_) {
    return;
  }
  if(!layered_costmap_->IsRollingWindow()) {
    if(!use_maximum_) {
      UpdateOverwriteByAll(master_grid, min_i, min_j, max_i, max_j);
    } else {
      UpdateOverwriteByMax(master_grid, min_i, min_j, max_i, max_j);
    }
  } else {
    unsigned int mx, my;
    double wx, wy;
    tf::StampedTransform temp_transform;
    try {
      tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0), temp_transform);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
    for(auto i = min_i; i < max_i; ++i) {
      for(auto j = min_j; j < max_j; ++j) {
        layered_costmap_->GetCostMap()->Map2World(i, j, wx, wy);
        tf::Point p(wx, wy, 0);
        p = temp_transform(p);
        if(World2Map(p.x(), p.y(), mx, my)){
          if(!use_maximum_) {
            master_grid.SetCost(i, j, GetCost(mx, my));
          }
          else {
            master_grid.SetCost(i, j, std::max(master_grid.GetCost(i, j), GetCost(i, j)));
          }
        }
      }
    }
  }
}

} //namespace roborts_costmap

