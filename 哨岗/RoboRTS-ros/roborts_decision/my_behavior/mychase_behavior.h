#ifndef ROBORTS_DECISION_MYCHASE_BEHAVIOR_H
#define ROBORTS_DECISION_MYCHASE_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "../example_behavior/line_iterator.h"

#include "../robot_information.h"
#include "../kill_pending.h"
namespace roborts_decision {
class MyChaseBehavior {
 public:
  MyChaseBehavior(ChassisExecutor* &chassis_executor,
                 Blackboard* &blackboard,
                 const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                        blackboard_(blackboard) {
    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }
    
  }

  void Run() 
  {
    BehaviorStatusUpdate(MYCHASE_STATUS);
//     if(IsVisionDetected)
//     {
//       IsVisionFirstLost = false;
//       return;
//     }
    ROS_INFO("[2]. mychase"); 
    
    if (MyChassisState != BehaviorState::RUNNING) 
    {
      auto robot_map_pose = blackboard_->GetRobotMapPose();//机器人现在的位姿
      auto dx = LastEnemyPose.pose.position.x - robot_map_pose.pose.position.x;
      auto dy = LastEnemyPose.pose.position.y - robot_map_pose.pose.position.y;
      tf::Quaternion rot1, rot2;
      tf::quaternionMsgToTF(LastEnemyPose.pose.orientation, rot1);
      tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
      auto d_yaw =  rot1.angleShortestPath(rot2);
      chassis_executor_->Execute(LastEnemyPose);
      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 1 && d_yaw <= 1 && MyChassisState == BehaviorState::SUCCESS)
      {
        Cancel();
        IsVisionFirstLost = false;
      }
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  bool LoadParam(const std::string &proto_file_path) 
  {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) 
    {
      return false;
    }
    return true;
  }

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

  ~MyChaseBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;
  //! perception information
  Blackboard* const blackboard_;

};
}

#endif 
