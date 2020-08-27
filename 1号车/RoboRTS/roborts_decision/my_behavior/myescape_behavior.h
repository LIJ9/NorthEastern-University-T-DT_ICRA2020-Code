#ifndef ROBORTS_DECISION_MYESCAPE_BEHAVIOR_H
#define ROBORTS_DECISION_MYESCAPE_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "../example_behavior/line_iterator.h"
#include "../robot_information.h"
#include "../kill_pending.h"

namespace roborts_decision {
class MyEscapeBehavior {
 public:
  MyEscapeBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {


    escape_goals1_.header.frame_id = "map";
    escape_goals1_.pose.orientation.x = 0;
    escape_goals1_.pose.orientation.y = 0;
    escape_goals1_.pose.orientation.z = 0;
    escape_goals1_.pose.orientation.w = 1;

    escape_goals1_.pose.position.x = 0;
    escape_goals1_.pose.position.y = 0;
    escape_goals1_.pose.position.z = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

  }
  
  void Run() 
  {
    BehaviorStatusUpdate(MYESCAPE_STATUS);
    
    ROS_INFO("[2]. MyEscape"); 

    IsStartSwing = false;
    
    if (MyChassisState != BehaviorState::RUNNING) 
    {
      if(ChangedGoals)
      {
        auto robot_map_pose = blackboard_->GetRobotMapPose();
        auto dx = escape_goals1_.pose.position.x - robot_map_pose.pose.position.x;
        auto dy = escape_goals1_.pose.position.y - robot_map_pose.pose.position.y;

        tf::Quaternion rot1, rot2;
        tf::quaternionMsgToTF(escape_goals1_.pose.orientation, rot1);
        tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
        auto d_yaw =  rot1.angleShortestPath(rot2);

        chassis_executor_->Execute(escape_goals1_);
        if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 1 && d_yaw <= 1 && MyChassisState == BehaviorState::SUCCESS) 
        {
          IsUnderAttack = false;
          IsChangeGoals = false;
        }
      }
      else
      {
        auto robot_map_pose = blackboard_->GetRobotMapPose();
        auto dx = escape_goals2_.pose.position.x - robot_map_pose.pose.position.x;
        auto dy = escape_goals2_.pose.position.y - robot_map_pose.pose.position.y;

        tf::Quaternion rot1, rot2;
        tf::quaternionMsgToTF(escape_goals2_.pose.orientation, rot1);
        tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
        auto d_yaw =  rot1.angleShortestPath(rot2);

        
        chassis_executor_->Execute(escape_goals2_);
        if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 1 && d_yaw <= 1 && MyChassisState == BehaviorState::SUCCESS) 
        {
          IsUnderAttack = false;
          IsChangeGoals = false;
        }
      }	
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    escape_goals1_.header.frame_id = "map";
    escape_goals1_.pose.position.x = MyConfig.MYESCAPE_POINT1.x;
    escape_goals1_.pose.position.y = MyConfig.MYESCAPE_POINT1.y;
    escape_goals1_.pose.position.z = MyConfig.MYESCAPE_POINT1.z;
    auto master_quaternion1 = tf::createQuaternionMsgFromRollPitchYaw(0, 0, MyConfig.MYESCAPE_POINT1.yaw);
    escape_goals1_.pose.orientation = master_quaternion1;

    escape_goals2_.header.frame_id = "map";
    escape_goals2_.pose.position.x = MyConfig.MYESCAPE_POINT2.x;
    escape_goals2_.pose.position.y = MyConfig.MYESCAPE_POINT2.y;
    escape_goals2_.pose.position.z = MyConfig.MYESCAPE_POINT2.z;
    auto master_quaternion2 = tf::createQuaternionMsgFromRollPitchYaw(0, 0, MyConfig.MYESCAPE_POINT2.yaw);
    escape_goals2_.pose.orientation = master_quaternion2;
    
    return true;
  }
  //加载两个逃跑点
  void BehaviorStatusUpdate(int BehaviorStaus)
  {  
    NowStatus = BehaviorStaus;
    
    if(NowStatus != LastStatus) //BehaviorStatus is Changed!!
    {
      Cancel();
      //std::cout << "------------------------------------Cancel Chassis------------------------------------" << std::endl;   
    }
    
    LastStatus = NowStatus;
  }
  
  ~MyEscapeBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! boot position
  geometry_msgs::PoseStamped escape_goals1_;
  geometry_msgs::PoseStamped escape_goals2_;

  //! chase buffer
  std::vector<geometry_msgs::PoseStamped> chase_buffer_;
  unsigned int chase_count_;

};
}


#endif //ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H