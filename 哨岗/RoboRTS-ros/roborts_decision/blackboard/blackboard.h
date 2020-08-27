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
#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>//3-20
#include "roborts_msgs/ArmorDetectionAction.h"

#include "io/io.h"
#include "../proto/decision.pb.h"
#include "costmap/costmap_interface.h"

#include "../robot_information.h"

namespace roborts_decision
{
class Blackboard {
 public:
  typedef std::shared_ptr<Blackboard> Ptr;
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;
  tf::TransformBroadcaster odom_broadcaster;//3-20
  // 使用关键字explicit的类声明, 显示转换 
  explicit Blackboard(const std::string &proto_file_path):
      enemy_detected_(false),
      armor_detection_actionlib_client_("armor_detection_node_action", true)
      {
        tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
        std::string map_path = ros::package::getPath("roborts_costmap") + \
          "/config/costmap_parameter_config_for_decision.prototxt";
        costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,map_path);
        charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();
        costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();
        // Enemy fake pose
        ros::NodeHandle rviz_nh("/move_base_simple");
        enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &Blackboard::GoalCallback, this);
        ros::NodeHandle nh;
        lidar_enemy_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("enemy_laser", 1, &Blackboard::LidarEnemyCallback, this);
        
        roborts_decision::DecisionConfig decision_config;
        roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);
        if (!decision_config.simulate())
        //config配置文件中simulate为true,则！decision_config.simulate()为false,则不执行该段code
        //若不是仿真，装甲识别目标为真
        {
          armor_detection_actionlib_client_.waitForServer();
          ROS_INFO("Armor detection module has been connected!");
          armor_detection_goal_.command = 1;
          armor_detection_actionlib_client_.sendGoal(armor_detection_goal_,
                                                    actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
                                                    actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
                                                    boost::bind(&Blackboard::ArmorDetectionFeedbackCallback, this, _1));
        }
      }
  ~Blackboard() = default;
  void ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback){
    if (feedback->detected)
    {
      enemy_detected_ = true;
      ROS_INFO("Find Enemy!");
      tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
      geometry_msgs::PoseStamped camera_pose_msg, global_pose_msg;
      camera_pose_msg = feedback->enemy_pos;
      //ROS_ERROR_STREAM(" camera_pose_msg is "<<camera_pose_msg);
      //ROS_ERROR_STREAM(" enemy_pose_ is "<<enemy_pose_);
      distance_ = (std::sqrt(camera_pose_msg.pose.position.x * camera_pose_msg.pose.position.x +
						  camera_pose_msg.pose.position.z * camera_pose_msg.pose.position.z))/100;
      //ROS_ERROR_STREAM(" distance_ is "<<distance_);
      //ROS_ERROR_STREAM(" robot_map_pose_yaw is "<<robot_map_pose_yaw);
      double enemy_yaw = atan(camera_pose_msg.pose.position.x / camera_pose_msg.pose.position.z); 
      enemy_pose_.pose.position.x = robot_map_pose_.pose.position.x + \
                                    distance_*cos(robot_map_pose_yaw + Angle_Gimbal_Chassis - enemy_yaw);
      enemy_pose_.pose.position.y = robot_map_pose_.pose.position.y + \
                                    distance_*sin(robot_map_pose_yaw + Angle_Gimbal_Chassis - enemy_yaw);
    }
    else
    {
      enemy_detected_ = false;
    }
  }
  geometry_msgs::PoseStamped GetEnemy() const {
    return enemy_pose_;
  }

  bool IsEnemyDetected() const{
    ROS_INFO("%s: %d", __FUNCTION__, (int)enemy_detected_);
    return enemy_detected_;
  }

  // Goal
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal){
    new_goal_ = true;
    goal_ = *goal;
  }

  geometry_msgs::PoseStamped GetGoal() const {
    return goal_;
  }

  bool IsNewGoal(){
    if(new_goal_){
      new_goal_ =  false;
      return true;
    } else{
      return false;
    }
  }
  
  //Lidar
  void LidarEnemyCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {    
    lidar_enemy_detected_ = true;
    lidar_enemy_pose_ = *msg;
    //exclude PartnerMapPose
    //雷达识别到之后标志位置位，将坐标给地址msg
    auto dx = lidar_enemy_pose_.pose.position.x - PartnerMapPose.pose.position.x;
    auto dy = lidar_enemy_pose_.pose.position.y - PartnerMapPose.pose.position.y;
    if( std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 3 ) 
    {
      lidar_enemy_detected_ = false;
      return;
    } 
    //如果距离过短，lidar_enemy_detected_为假
    
    auto robot_map_pose = GetRobotMapPose();
    lidar_yaw_ = atan2( (lidar_enemy_pose_.pose.position.y - robot_map_pose.pose.position.y), (lidar_enemy_pose_.pose.position.x - robot_map_pose.pose.position.x) );
    //Prevent jitter
    /*static float last_lidar_yaw;
    if( fabs(lidar_yaw_ - last_lidar_yaw) < 0.35 ) //20 degree
    {
      lidar_yaw_ = last_lidar_yaw;
    }
    last_lidar_yaw = lidar_yaw_;*/
    
    auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, lidar_yaw_);
    lidar_enemy_pose_.pose.orientation = quaternion; 
  }
  
  geometry_msgs::PoseStamped LidarGetEnemy() const 
  {
    return lidar_enemy_pose_;
  }
  
  bool IsLidarEnemyDetected() const
  {
    //ROS_INFO("%s: %d", __FUNCTION__, (int)lidar_enemy_detected_);
    ROS_INFO("lidar_enemy_detected_ is : %d", (int)lidar_enemy_detected_);
    //std::cout <<"lidar_enemy_detected_ is : "<<lidar_enemy_detected_<<std::endl;
    return lidar_enemy_detected_;
  }
  
  bool LidarFlagReset()
  {
    lidar_enemy_detected_ = false;
    return lidar_enemy_detected_;
  }
  
  float LidarGetYaw() const 
  {
    return lidar_yaw_;
  }
  //enemy distance form robort camera
  double EnemyDistance()
  {
    return distance_;
  }

  /*通过我的坐标和目标机器人的坐标解算出角度*/
  void CalTargetQuaternion(const geometry_msgs::PoseStamped &mypose, geometry_msgs::PoseStamped &targetpose) 
  {
    auto target_yaw = atan2( (targetpose.pose.position.y - mypose.pose.position.y), (targetpose.pose.position.x - mypose.pose.position.x) );
    //std::cout << "[!!!!!!!!!!!!]lidar target_yaw is : " <<57.3*target_yaw<<std::endl;
    auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, target_yaw);
    targetpose.pose.orientation = quaternion;
  }
  
  const geometry_msgs::PoseStamped GetRobotMapPose() {
    UpdateRobotPose();
    return robot_map_pose_;
  }
//my position
  const std::shared_ptr<CostMap> GetCostMap(){
    return costmap_ptr_;
  }

  const CostMap2D* GetCostMap2D() {
    return costmap_2d_;
  }

  const unsigned char* GetCharMap() {
    return charmap_;
  }

 private:
  void UpdateRobotPose() 
  {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity();

    robot_tf_pose.frame_id_ = "base_link";//头帧
    robot_tf_pose.stamp_ = ros::Time();//时间戳
    try 
    {
      geometry_msgs::PoseStamped robot_pose;//机器人人当前位姿
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);//
      tf_ptr_->transformPose("map", robot_pose, robot_map_pose_);
      tf::quaternionMsgToTF(robot_map_pose_.pose.orientation, quat);
      tf::Matrix3x3(quat).getRPY(robot_map_pose_roll, robot_map_pose_pitch, robot_map_pose_yaw);//进行转换
    }
    catch (tf::LookupException &ex) {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }//地图信息，
  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  //! Enenmy detection
  ros::Subscriber enemy_sub_;
  

  //! Goal info
  geometry_msgs::PoseStamped goal_;
  bool new_goal_;

  //! Enemy info
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  roborts_msgs::ArmorDetectionGoal armor_detection_goal_;
  geometry_msgs::PoseStamped enemy_pose_;
  bool enemy_detected_;

  //! cost map
  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D* costmap_2d_;
  unsigned char* charmap_;

  //! robot map pose
  geometry_msgs::PoseStamped robot_map_pose_;
  
  //lidar
  ros::Subscriber lidar_enemy_sub_;
  geometry_msgs::PoseStamped lidar_enemy_pose_;
  float lidar_yaw_;
  bool lidar_enemy_detected_;
  //enemy distance
  double distance_;
  double robot_map_pose_roll, robot_map_pose_pitch, robot_map_pose_yaw;//定义存储r\p\y的容器
  tf::Quaternion quat;
  
};
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H
