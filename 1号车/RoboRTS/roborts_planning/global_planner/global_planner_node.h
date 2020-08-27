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
#ifndef ROBORTS_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
#define ROBORTS_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H


#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <tf/transform_listener.h>
#include "actionlib/server/simple_action_server.h"
#include "nav_msgs/Path.h"
#include "roborts_msgs/GlobalPlannerAction.h"

#include "alg_factory/algorithm_factory.h"
#include "state/error_code.h"
#include "io/io.h"
#include "state/node_state.h"

#include "costmap/costmap_interface.h"
#include "global_planner_base.h"
#include "proto/global_planner_config.pb.h"
#include "global_planner_algorithms.h"

#include "roborts_msgs/LocalPlannerAction.h"
#include "roborts_msgs/TwistAccel.h"

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
namespace roborts_global_planner{

/**
 * @brief Node class for global planner module.
 */
class GlobalPlannerNode {
 public:
  typedef std::shared_ptr<roborts_costmap::CostmapInterface> CostmapPtr;
  typedef std::shared_ptr<tf::TransformListener> TfPtr;
  typedef std::unique_ptr<GlobalPlannerBase> GlobalPlannerPtr;
  typedef actionlib::SimpleActionServer<roborts_msgs::GlobalPlannerAction> GlobalPlannerServer;
 /**
  * @brief Constructor including all initialization and configuration
  */
  GlobalPlannerNode();

 /**
  * @brief Destructor which stops all running threads .
  */
  ~GlobalPlannerNode();

 private:
//    /////////////////////////////
//      ros::Publisher stuck_vel_pub_; 
//      roborts_msgs::TwistAccel stuck_cmd_vel_;
//      ros::NodeHandle n;
   ///////////////////////
  /**
   * @brief Initiate the ROS configuration, planning algorithm instance,
   * planning related parameter configuration and etc.
   * @return ErrorInfo which is OK if succeed
   */
  roborts_common::ErrorInfo Init();
  /**
   * @brief Callback function for the actionlib task
   * @param msg The task goal message from actionlib client
   */
  /////////////////////

  ///////////////////////
  
  void GoalCallback(const roborts_msgs::GlobalPlannerGoal::ConstPtr &msg);
  /**
   * @brief Set the state for planning module
   * @param node_state Enumerate State for global planning
   */
  void SetNodeState(roborts_common::NodeState node_state);
  /**
   * @brief Get the state for planning module
   * @return Enumerate State for global planning
   */
  roborts_common::NodeState GetNodeState();
  /**
   * @brief Set the state for planning module
   * @param error_info Error Info for global planning
   */
  void SetErrorInfo(roborts_common::ErrorInfo error_info);
  /**
   * @brief Get the error info for planning module
   * @return Error Info for global planning
   */
  roborts_common::ErrorInfo GetErrorInfo();
  /**
   * @brief Check if the plan thread is still under execution, if not, start the plan thread.
   * @return True if start the plan thread successful, else false.
   */
  geometry_msgs::PoseStamped GetGoal();
  /**
   * @brief Set the planner goal
   * @param goal planner goal
   */
  
  //bool goal_rec_;
  void SetGoal(geometry_msgs::PoseStamped goal);
  /**
   * @brief /Start the plan thread and set the state to RUNNING
   */
  void StartPlanning();
  /**
   * @brief Stop the plan thread and set the state to IDLE
   */
  void StopPlanning();
  /**
   * @brief Plan thread which mainly include a planning cycle process:
   * 1. Get the current start pose and goal pose, validate and transform them according to frame of costmap \n
   * 2. Make a plan using the selected algorithm \n
   * 3. If success, check the completion if the current pose is close to the goal. If completed, no need of planning and jump out of the cycle!\n
   * 4. If not succeed, check if it reaches the max retries. if so, no need of planning and jump out of the cycle!\n
   * 5. If planning still needed, go to next cycle and wait for a duration which the global planning frequency decides.
   */
  void PlanThread();
  /**
   * @brief Get the Euclidean distance of two poses.
   * @param pose1 The first pose in the form of geometry_msgs::PoseStamped
   * @param pose2 The second pose in the form of geometry_msgs::PoseStamped
   * @return The Euclidean distance
   */
  double GetDistance(const geometry_msgs::PoseStamped& pose1,
                     const geometry_msgs::PoseStamped& pose2);
  /**
   * @brief Get the angle difference of two pose .
   * @param pose1 The first pose in the form of geometry_msgs::PoseStamped
   * @param pose2 The second pose in the form of geometry_msgs::PoseStamped
   * @return The angle difference
   */
  double GetAngle(const geometry_msgs::PoseStamped &pose1,    
                  const geometry_msgs::PoseStamped &pose2)  ;
  /**
   * @brief Input the path in geometry_msgs::PoseStamped vector and publish the path in nav_msgs::Path
   * @param path the path generated from global planner, indeed the discrete poses in the form of geometry_msgs::PoseStamped vector
   */
  void PathVisualization(const std::vector<geometry_msgs::PoseStamped> &path);

  //! ROS Node Handler
  ros::NodeHandle nh_;
  //! ROS Publisher for path visualization in rviz
  ros::Publisher path_pub_;
  
   //! ROS Node Handler
  ros::NodeHandle h_;
  //! ROS Publisher for path visualization in rviz
  ros::Subscriber vel_sub_;
   void VelCallback(const roborts_msgs::TwistAccel::ConstPtr& vel_msg);
   roborts_msgs::TwistAccel vel_acc_;
    roborts_msgs::TwistAccel last_vel_acc_;
  
  /////////////////////////////////////////////////
    //! ROS Node Handler for current goal
    ros::NodeHandle goal_h_;
    //! ROS Publisher for current goal
    ros::Publisher goal_pub_;
    
  //! ROS Actionlib Server for command global planning module
  GlobalPlannerServer as_;
  //! Global planner pointer
  GlobalPlannerPtr global_planner_ptr_;
  //！ Transform pointer
  TfPtr tf_ptr_;
  //! Costmap pointer
  CostmapPtr costmap_ptr_;
  //! String Type for selected algorithm
  std::string selected_algorithm_;

  //! Received goal for global planner (Input)
  geometry_msgs::PoseStamped goal_;
  //! Goal mutex
  std::mutex goal_mtx_;
  //! Path generated by global planner (Output)
  bool pause_;

  nav_msgs::Path path_;
  //! Bool flag that indicates whether or not new plan path comes
  bool new_path_;

  //! Thread for global planning progress
  std::thread plan_thread_;
  //! Planning condition variable
  std::condition_variable plan_condition_;
  //! Planning mutex
  std::mutex plan_mutex_;

  
  ///////////////////////////////////////////////////////////
  
  std::mutex stuck_plan_mutex_;

   geometry_msgs::PoseStamped current_osc_pose,last_osc_pose;
   bool is_osci;
  std_msgs::Float64MultiArray status_info;
  //////////////////////////////////////////////////////////////////////
  //! Global planner node state
  roborts_common::NodeState node_state_;
  //! Global planner node state mutex
  std::mutex node_state_mtx_;
  //! Global planner error infomation
  roborts_common::ErrorInfo error_info_;
  //! Global planner error infomation mutex
  std::mutex error_info_mtx_;

  //! Cycle Duration in microseconds
  std::chrono::microseconds cycle_duration_;
  //! Max retry count
  int max_retries_;
  //! Distance tolerance towards goal
  double goal_distance_tolerance_;
  //! Angle tolerance towards goal
  double goal_angle_tolerance_;
  
//      ros::NodeHandle debug_ini_h;
//     //! ROS Publisher for current goal
//     ros::Publisher debug_ini_pub_ = debug_ini_h.advertise< std_msgs::Float64>("/init_", 10);
//       std_msgs::Float64 init_;
//   
//       
//          ros::NodeHandle inited_h;
//     //! ROS Publisher for current goal
//     ros::Publisher inited_pub_;//inited_pub_ = inited_h.advertise<.data[1]Float64MultiArray >("/stuck_pose", 10);
//       std_msgs::Float64MultiArray inited_;
//   
//              ros::NodeHandle goal_callback_h;
//     //! ROS Publisher for current goal
//     ros::Publisher goal_callback_pub_;//goal_callback_pub_ = goal_callback_h.advertise<geometry_msgs::PoseStamped >("/stuck_pose", 10);
//       std_msgs::Float64MultiArray goal_callback_info;
//   
//           ros::NodeHandle plan_thread_h;
//     //! ROS Publisher for current goal
//     ros::Publisher plan_thread_pub_;//plan_thread_pub_ = plan_thread_h.advertise<Float64MultiArray >("/plan_thread_info", 10);
//       std_msgs::Float64MultiArray plan_thread_info;
//   
//       
   
    ros::NodeHandle planthread_h;
  ros::Publisher planthread_pub= planthread_h.advertise<geometry_msgs::PoseStamped>("planthread", 10);
  geometry_msgs::PoseStamped planthread;
  ///
      
};

} //namespace roborts_global_planner
#endif //ROBORTS_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
