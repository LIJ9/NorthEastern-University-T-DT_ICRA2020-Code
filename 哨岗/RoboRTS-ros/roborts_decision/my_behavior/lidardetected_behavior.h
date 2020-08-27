#ifndef ROBORTS_DECISION_LIDARDETECTED_BEHAVIOR_H
#define ROBORTS_DECISION_LIDARDETECTED_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "../example_behavior/line_iterator.h"

#include "../robot_information.h"

namespace roborts_decision {
class LidarDetectedBehavior {
 public:
  LidarDetectedBehavior(ChassisExecutor* &chassis_executor,
                 Blackboard* &blackboard,
                 const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                        blackboard_(blackboard) 
  //构造函数，ChassisExecutor的指针是chassis_executor，：后面是对类的对象的初始化
  {
    if (!LoadParam(proto_file_path)) 
    {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }
  }
  void Run() 
  {
    BehaviorStatusUpdate(LIDARDETECTED_STATUS);
    ROS_INFO("[2]. LidarDetectedBehavior"); 
    IsEnableVision = true;
    IsLidaring = true;
    if(LidarStartFlag_ == false)
    {
      LidarStartFlag_ = true;
      LidarEnemyPose = blackboard_->LidarGetEnemy();
      LidarYaw = blackboard_->LidarGetYaw();
    }
    if(IsAngleAchieved)
    {
      LidarStartFlag_ = false;
      IsLidarDetected = blackboard_->LidarFlagReset();
      IsEnableVision = false;
      IsLidaring = false;
      IsAngleAchieved = false;
      std::cout <<"Lidar Success!!!!!!!!!!"<< std::endl;
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
    
//       HoldLidarEnemyPose.header.frame_id = LidarEnemyPose.header.frame_id = "map";    
//       HoldLidarEnemyPose.pose.position.x = LidarEnemyPose.pose.position.x = 0;
//       HoldLidarEnemyPose.pose.position.y = LidarEnemyPose.pose.position.y = 0;
//       HoldLidarEnemyPose.pose.position.z = LidarEnemyPose.pose.position.z = 0;
//       auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 3.14);
//       HoldLidarEnemyPose.pose.orientation = LidarEnemyPose.pose.orientation = quaternion;

    HoldLidarEnemyPose = LidarEnemyPose = blackboard_->GetRobotMapPose(); //init
    return true;
  }

  void BehaviorStatusUpdate(int BehaviorStaus)
  {  
    NowStatus = BehaviorStaus;
    
    if(NowStatus != LastStatus) //BehaviorStatus is Changed!!
    {
      LidarStartFlag_ = false; 
      Cancel();
      //std::cout << "------------------------------------Cancel Chassis------------------------------------" << std::endl;   
    }
    
    LastStatus = NowStatus;
  }
  
  
  ~LidarDetectedBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  bool LidarStartFlag_;
};
}

#endif 
