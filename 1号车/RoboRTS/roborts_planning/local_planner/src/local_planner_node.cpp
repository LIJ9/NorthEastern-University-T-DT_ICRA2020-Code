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

#include <csignal>

#include "local_planner/local_planner_node.h"
#include <geometry_msgs/Vector3.h>
namespace roborts_local_planner {

using roborts_common::NodeState;
LocalPlannerNode::LocalPlannerNode() :
    local_planner_nh_("~"),
    as_(local_planner_nh_, "/local_planner_node_action", boost::bind(&LocalPlannerNode::ExcuteCB, this, _1), false),
    initialized_(false), node_state_(roborts_common::NodeState::IDLE),
    node_error_info_(roborts_common::ErrorCode::OK), max_error_(5),
    local_cost_(nullptr), tf_(nullptr) {
     
  if (Init().IsOK()) {
    ROS_INFO("local planner initialize completed.");
  } else {
    ROS_WARN("local planner initialize failed.");
    SetNodeState(NodeState::FAILURE);
  }
  as_.start();
}

LocalPlannerNode::~LocalPlannerNode() {
  StopPlanning();
}

roborts_common::ErrorInfo LocalPlannerNode::Init() {
 
  ROS_INFO("local planner start");
  LocalAlgorithms local_algorithms;
  std::string full_path = ros::package::getPath("roborts_planning") + "/local_planner/config/local_planner.prototxt";
  roborts_common::ReadProtoFromTextFile(full_path.c_str(), &local_algorithms);
  if (&local_algorithms == nullptr) {
    return roborts_common::ErrorInfo(roborts_common::ErrorCode::LP_INITILIZATION_ERROR,
                                   "Cannot load local planner protobuf configuration file.");
  }
  selected_algorithm_ = local_algorithms.selected_algorithm();
  frequency_ = local_algorithms.frequency();
  tf_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

  std::string map_path = ros::package::getPath("roborts_costmap") + \
      "/config/costmap_parameter_config_for_local_plan.prototxt";
  local_cost_ = std::make_shared<roborts_costmap::CostmapInterface>("local_costmap",
                                                                          *tf_,
                                                                          map_path.c_str());
  local_planner_ = roborts_common::AlgorithmFactory<LocalPlannerBase>::CreateAlgorithm(selected_algorithm_);
  if (local_planner_== nullptr) {
    ROS_ERROR("global planner algorithm instance can't be loaded");
    return roborts_common::ErrorInfo(roborts_common::ErrorCode::LP_INITILIZATION_ERROR,
                     "local planner algorithm instance can't be loaded");
  }

  std::string name;
  visual_frame_ = local_cost_->GetGlobalFrameID();
  visual_ = LocalVisualizationPtr(new LocalVisualization(local_planner_nh_, visual_frame_));
  vel_pub_ = local_planner_nh_.advertise<roborts_msgs::TwistAccel>("/cmd_vel_acc", 5);
  pos_sub_ = pos_n.subscribe<geometry_msgs::PoseStamped>("/stuck_pose", 10, &LocalPlannerNode::PosCallback, this);
  nextgoal_sub= nextgoal_h.subscribe<geometry_msgs::PoseStamped>("/nextgoal", 10, &LocalPlannerNode::nextgoalCallback, this);
 bool IsStuck = false;
  return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
}
void LocalPlannerNode::PosCallback(const geometry_msgs::PoseStamped::ConstPtr  & stuck_pos)
{ 
    const float point_x = stuck_pos->pose.position.x;
    const float point_y = stuck_pos->pose.position.y ;
    const geometry_msgs::Quaternion quaternion_s = stuck_pos->pose.orientation;
    tf::Quaternion rot1;
    tf::quaternionMsgToTF(quaternion_s, rot1);  
    
    double stuck_yaw = getYaw(rot1);     
    bool IsStuck = true;   
}

void LocalPlannerNode::nextgoalCallback(const geometry_msgs::PoseStamped::ConstPtr  & nextgoalmsg)
{ 
    nextgoal.pose.position.x= nextgoalmsg->pose.position.x;
    nextgoal.pose.position.y= nextgoalmsg->pose.position.y;    
    nextgoal.pose.position.z= nextgoalmsg->pose.position.z;  
     nextgoal.pose.orientation.x= nextgoalmsg->pose.position.x;  
     nextgoal.pose.orientation.y= nextgoalmsg->pose.position.y;  
     nextgoal.pose.orientation.z= nextgoalmsg->pose.position.z;  
}

void LocalPlannerNode::ExcuteCB(const roborts_msgs::LocalPlannerGoal::ConstPtr &command) {
  roborts_common::ErrorInfo error_info = GetErrorInfo();
  NodeState node_state = GetNodeState();
  
  if (node_state == NodeState::FAILURE) {
    roborts_msgs::LocalPlannerFeedback feedback;
    roborts_msgs::LocalPlannerResult result;
    feedback.error_code = error_info.error_code();
    feedback.error_msg  = error_info.error_msg();
    result.error_code   = feedback.error_code;
    as_.publishFeedback(feedback);
    as_.setAborted(result, feedback.error_msg);
    ROS_ERROR("Initialization Failed, Failed to execute action!");
    return;
  }
  if (plan_mtx_.try_lock()) {
    local_planner_->SetPlan(command->route, local_goal_);//local_goal_ =0
    plan_mtx_.unlock();
    plan_condition_.notify_one();
  }

  ROS_INFO("Send Plan!");
  if (node_state == NodeState::IDLE) {
    IsStuck = false;
    StartPlanning();
  }

  while (ros::ok()) {
    std::this_thread::sleep_for(std::chrono::microseconds(1));

    if (as_.isPreemptRequested()) {
      ROS_INFO("Action Preempted");
      if (as_.isNewGoalAvailable()) {
        as_.setPreempted();
        break;
      } else {
        as_.setPreempted();
        StopPlanning();
        break;
      }
    }
    node_state = GetNodeState();
    error_info = GetErrorInfo();

    if (node_state == NodeState::RUNNING|| node_state == NodeState::SUCCESS
        || node_state == NodeState::FAILURE|| node_state == NodeState::STUCK) {
      roborts_msgs::LocalPlannerFeedback feedback;
      roborts_msgs::LocalPlannerResult result;
      if (!error_info.IsOK()) {
        feedback.error_code = error_info.error_code();        
        feedback.error_msg = error_info.error_msg();
        SetErrorInfo(roborts_common::ErrorInfo::OK());
        as_.publishFeedback(feedback); 
      }
      if(node_state == NodeState::STUCK){
	 result.error_code = error_info.error_code();
        as_.setSucceeded(result,error_info.error_msg());
         StartPlanning();           
        break;
      }
      if(node_state == NodeState::SUCCESS) {
        result.error_code = error_info.error_code();
        as_.setSucceeded(result,error_info.error_msg());
        StopPlanning();
        break;
      } else if(node_state == NodeState::FAILURE) {
        result.error_code = error_info.error_code();
        as_.setAborted(result,error_info.error_msg());
        StopPlanning();
        break;
      }
    }
  }

}

void LocalPlannerNode::Loop() {

 
  roborts_common::ErrorInfo error_info = local_planner_->Initialize(local_cost_, tf_, visual_);
  if (error_info.IsOK()) {
    ROS_INFO("local planner algorithm initialize completed.");
  } else {
     Loop_info.pose.position.x = 1;
    ROS_WARN("local planner algorithm initialize failed.");
    SetNodeState(NodeState::FAILURE);
    SetErrorInfo(error_info);
  }
  std::chrono::microseconds sleep_time = std::chrono::microseconds(0);
  int error_count = 0;
  int change_count;
  
 while (GetNodeState() == NodeState::RUNNING) {
   
    std::unique_lock<std::mutex> plan_lock(plan_mutex_);
    plan_condition_.wait_for(plan_lock, sleep_time);
    auto begin = std::chrono::steady_clock::now();
    roborts_common::ErrorInfo error_info = local_planner_->ComputeVelocityCommands(cmd_vel_);
    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin);///////////////////////
    int need_time = 1000 /frequency_;
    sleep_time = std::chrono::milliseconds(need_time) - cost_time;
    if (sleep_time <= std::chrono::milliseconds(0)) {
      sleep_time = std::chrono::milliseconds(0);
    }

    if (error_info.IsOK()) {
      error_count = 0;
     change_count = 0;
      vel_pub_.publish(cmd_vel_);   
      if (local_planner_->IsGoalReached()) {         
        SetNodeState(NodeState::SUCCESS);
      }
    }

    else if (error_count > max_error_ && max_error_ >0) {
      Loop_info.pose.position.z = 1;
      ROS_WARN("Can not finish plan with max retries( %d )", max_error_ );
      error_info =  roborts_common::ErrorInfo(roborts_common::ErrorCode::LP_MAX_ERROR_FAILURE, "over max error.");
      SetNodeState(NodeState::STUCK);
    } else {
      error_count++;
      ROS_ERROR("Can not get cmd_vel for once. %s error count:  %d", error_info.error_msg().c_str(), error_count);////被压住速度是0

    }
    SetErrorInfo(error_info);
      }
   while(GetNodeState() == NodeState::STUCK) {
     
     Loop_info.pose.position.y = 1;
            cmd_vel_.twist.linear.x = 0;
	    cmd_vel_.twist.linear.y = 0;
	    cmd_vel_.twist.angular.z = 0;
	    
	    cmd_vel_.accel.linear.x = -std::cos(stuck_yaw);
	    cmd_vel_.accel.linear.y =- std::sin(stuck_yaw);
	    cmd_vel_.accel.angular.z = 0;
    
	    vel_pub_.publish(cmd_vel_); 
       SetErrorInfo(error_info);
    if(error_info.IsOK()){
    SetNodeState(NodeState::IDLE);
    }
  } //while stuck
  cmd_vel_.twist.linear.x = 0;
  cmd_vel_.twist.linear.y = 0;
  cmd_vel_.twist.angular.z = 0;
	    
  cmd_vel_.accel.linear.x = 0;
  cmd_vel_.accel.linear.y = 0;
  cmd_vel_.accel.angular.z = 0;
  vel_pub_.publish(cmd_vel_);
  
}
 
 

void LocalPlannerNode::SetErrorInfo(const roborts_common::ErrorInfo error_info) {
  std::lock_guard<std::mutex> guard(node_error_info_mtx_);
  node_error_info_ = error_info;
}

void LocalPlannerNode::SetNodeState(const roborts_common::NodeState& node_state) {
  std::lock_guard<std::mutex> guard(node_state_mtx_);
  node_state_ = node_state;
}

roborts_common::NodeState LocalPlannerNode::GetNodeState() {
  std::lock_guard<std::mutex> guard(node_state_mtx_);
  return node_state_;
}

roborts_common::ErrorInfo LocalPlannerNode::GetErrorInfo() {
  std::lock_guard<std::mutex> guard(node_error_info_mtx_);
  return node_error_info_;
}

void LocalPlannerNode::StartPlanning() {
  if (local_planner_thread_.joinable()) {
    local_planner_thread_.join();
  }

  SetNodeState(roborts_common::NodeState::RUNNING);
  local_planner_thread_= std::thread(std::bind(&LocalPlannerNode::Loop,this));
}

void LocalPlannerNode::StopPlanning() {
  SetNodeState(roborts_common::IDLE);
  if (local_planner_thread_.joinable()) {
    local_planner_thread_.join();
  }
}

void LocalPlannerNode::AlgorithmCB(const roborts_common::ErrorInfo &algorithm_error_info) {
  SetErrorInfo(algorithm_error_info);
}

} // namespace roborts_local_planner

void SignalHandler(int signal){
  if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
    ros::shutdown();
  }
}

int main(int argc, char **argv) {

  signal(SIGINT, SignalHandler);
  signal(SIGTERM,SignalHandler);
  ros::init(argc, argv, "local_planner_node", ros::init_options::NoSigintHandler);

  roborts_local_planner::LocalPlannerNode local_planner;

  ros::AsyncSpinner async_spinner(2);
  async_spinner.start();
  ros::waitForShutdown();
  local_planner.StopPlanning();

  return 0;
}
