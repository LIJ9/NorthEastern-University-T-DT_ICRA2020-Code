#ifndef ROBORTS_DECISION_ADDBUFF_BEHAVIOR_H
#define ROBORTS_DECISION_ADDBUFF_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "../example_behavior/line_iterator.h"

#include "../robot_information.h"
#include "../kill_pending.h"

namespace roborts_decision {
class AddHPBehavior {
 public:
  AddHPBehavior(ChassisExecutor* &chassis_executor,
                 Blackboard* &blackboard,
                 const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                        blackboard_(blackboard) 
  {
  }

  //executor_state
  void Run() 
  {
    //Addbuffover = false;
    hp_goals_.header.frame_id = "map";
    hp_goals_.pose.position.x = MyHPPoint.pose.position.x;
    hp_goals_.pose.position.y = MyHPPoint.pose.position.y;
    hp_goals_.pose.position.z = MyHPPoint.pose.position.z;
    auto hp_goals_quaternion = tf::createQuaternionMsgFromRollPitchYaw( 0,0, MyHPPoint.pose.orientation.w);
    hp_goals_.pose.orientation = hp_goals_quaternion;
    BehaviorStatusUpdate(ADDHP_STATUS);
    ROS_INFO("[2]. Add HP");
    //IsUnderAttack = IsStartCounter = IsStartSwing = false;
    IsStartSwing = false;
    if (MyChassisState != BehaviorState::RUNNING) 
    {
      auto robot_map_pose = blackboard_->GetRobotMapPose();
      auto dx = hp_goals_.pose.position.x - robot_map_pose.pose.position.x;
      auto dy = hp_goals_.pose.position.y - robot_map_pose.pose.position.y;
      tf::Quaternion rot1, rot2;
      tf::quaternionMsgToTF(hp_goals_.pose.orientation, rot1);
      tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
      auto d_yaw =  rot1.angleShortestPath(rot2);
      chassis_executor_->Execute(hp_goals_);
      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 1 && d_yaw <= 1 && MyChassisState == BehaviorState::SUCCESS)
      {
        IsAddingBuff = true;
        gametime.RecordWaitBuffSec();
        std::cout << "add hp point arrived!!!" << std::endl;
        CdUnderAttack = 20;
      }
    }
  }
  void Cancel() 
  {
    chassis_executor_->Cancel();
  }
  BehaviorState Update() 
  {
    return chassis_executor_->Update();
  }

  bool LoadParam() 
  {
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
  ~AddHPBehavior() = default;
 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //buff goals
  geometry_msgs::PoseStamped hp_goals_;
  
};
}

#endif 
