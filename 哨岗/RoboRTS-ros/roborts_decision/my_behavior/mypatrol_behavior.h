#ifndef ROBORTS_DECISION_PATROL_BEHAVIOR_H
#define ROBORTS_DECISION_PATROL_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"
#include "../example_behavior/line_iterator.h"

#include "../robot_information.h"
#include "../kill_pending.h"

namespace roborts_decision {
class MyPatrolBehavior {
 public:
  MyPatrolBehavior(ChassisExecutor* &chassis_executor,
                 Blackboard* &blackboard,
                 const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                        blackboard_(blackboard) {

    patrol_count_ = 0;
    point_size_ = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }
  }

  void Run() 
  {
    BehaviorStatusUpdate(MYPATROL_STATUS);
    ROS_INFO("[2].Patrol"); 
    if (MyChassisState != BehaviorState::RUNNING) 
    {

      if (patrol_goals_.empty()) {
        ROS_ERROR("patrol goal is empty");
        return;
      }
      chassis_executor_->Execute(patrol_goals_[patrol_count_]);
      patrol_count_ = ++patrol_count_ % point_size_;
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

    point_size_ = (unsigned int)MyConfig.PATROL_POINT.size();
    patrol_goals_.resize(point_size_);
    for (int i = 0; i != point_size_; i++) {
      patrol_goals_[i].header.frame_id = "map";
      patrol_goals_[i].pose.position.x = MyConfig.PATROL_POINT[i].x;
      patrol_goals_[i].pose.position.y = MyConfig.PATROL_POINT[i].y;
      patrol_goals_[i].pose.position.z = MyConfig.PATROL_POINT[i].z;
      auto patrol_goals_quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, MyConfig.PATROL_POINT[i].yaw);
      patrol_goals_[i].pose.orientation = patrol_goals_quaternion;
      
    }

    return true;
  }

  void BehaviorStatusUpdate(int BehaviorStaus)
  {  
    NowStatus = BehaviorStaus;
    
    if(NowStatus != LastStatus) //BehaviorStatus is Changed!!
    {    
      //Cancel();
      //std::cout << "------------------------------------Cancel Chassis------------------------------------" << std::endl;
    }
    LastStatus = NowStatus;
  }
  
  ~MyPatrolBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! patrol buffer
  std::vector<geometry_msgs::PoseStamped> patrol_goals_;
  unsigned int patrol_count_;
  unsigned int point_size_;

};
}

#endif //ROBORTS_DECISION_PATROL_BEHAVIOR_H
