#ifndef ROBORTS_DECISION_COUNTER_BEHAVIOR_H
#define ROBORTS_DECISION_COUNTER_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "../example_behavior/line_iterator.h"

#include "../robot_information.h"

namespace roborts_decision {
class CounterBehavior {
 public:
  CounterBehavior(ChassisExecutor* &chassis_executor,
                 Blackboard* &blackboard,
                 const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                        blackboard_(blackboard) 
  {
    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }   
  }
  void Run() 
  {   
    BehaviorStatusUpdate(COUNTER_STATUS);
    //更新当前状态
    ROS_INFO("[2]. Counter"); 
    IsEnableVision = true;
    IsCounting = true;
    if(CounterStartFlag_ == false)
    {
      CounterStartFlag_ = true;
      HoldUnderAttackArmorId = UnderAttackArmorId;
    }
    if(IsAngleAchieved)
    {
      CounterStartFlag_ = false;
      IsStartCounter = false;
      IsUnderAttack = false;
      IsEnableVision = false;
      IsCounting = false;
      IsAngleAchieved = false;
      std::cout <<"Counter Success!!!!!!!!!!"<< std::endl;
    }
  }
  //反击行为的实现在视觉的模块，决策为标志位的重置
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

    return true;
  }

  void BehaviorStatusUpdate(int BehaviorStaus)//BehaviorStaus = COUNTER_STATUS;
  {
    NowStatus = BehaviorStaus;
    
    if(NowStatus != LastStatus) //BehaviorStatus is Changed!!切换行为
    {
      CounterStartFlag_ = false;  
      Cancel();
      //std::cout << "------------------------------------Cancel Chassis------------------------------------" << std::endl;   
    }
    
    LastStatus = NowStatus;
  }//切换行为之后
  
  ~CounterBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;
  
  bool CounterStartFlag_;
};
}

#endif 
