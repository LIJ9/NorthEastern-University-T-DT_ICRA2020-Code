#ifndef ROBORTS_DECISION_GIMBAL_BEHAVIOR_H
#define ROBORTS_DECISION_GIMBAL_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "../example_behavior/line_iterator.h"

#include "../robot_information.h"

namespace roborts_decision {
class ShootBehavior {
 public:
  ShootBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) 
  {
    if (!LoadParam(proto_file_path)) 
    {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }
    firecmd_sub_ = shoot_nh_.subscribe("FireCmd", 1, &roborts_decision::ShootBehavior::FireCmdCB, this); 
  }

  void Run() 
  {
    BehaviorStatusUpdate(SHOOT_STATUS);
    ROS_INFO("[2]. Shoot"); 
    //auto robot_map_pose = blackboard_->GetRobotMapPose();
    //LastEnemyPose = blackboard_->GetEnemy();//获取敌方的位姿，丢失的时候用于追击
    //blackboard_->CalTargetQuaternion(robot_map_pose,LastEnemyPose);
    //LastEnemyPose.pose.position.z = 0;
    IsEnableVision = true;
    //reset some falg
    IsStartCounter = false;//锁定之后不会再切换目标
    IsVisionFirstLost = false;
    IsReceiveEnemyPose = false;//不接受队友提供的地方机器人位姿
    IsLidarDetected = blackboard_->LidarFlagReset();//雷达识别的标志位置为false
    //射击走出补弹区
  }

  void FireCmdCB(const std_msgs::Int16::ConstPtr& msg)
  {
    FireCmd = (bool)msg->data;//订阅开火指令，订阅到之后开始将开火命令置为true
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
    /* LastEnemyPose = blackboard_->GetRobotMapPose(); //init 
    LastEnemyPose.header.frame_id = "map";
    addbullet_move_goal_.header.frame_id = "map"; //init
    addbullet_move_goal_.pose.position.y = MyConfig.ADDBULLET_POINT.y;
    if(MyConfig.ROBOT_INFO.color == "RED")
      addbullet_move_goal_.pose.position.x = MyConfig.ADDBULLET_POINT.x - 1;
    else if(MyConfig.ROBOT_INFO.color == "BLUE")
      addbullet_move_goal_.pose.position.x = MyConfig.ADDBULLET_POINT.x + 1;
      //红蓝方的addbullet_point的ｘ不同，y相同 */
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
  ~ShootBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;
  
  //! perception information
  Blackboard* const blackboard_;
  
  geometry_msgs::PoseStamped addbullet_move_goal_;
  geometry_msgs::PoseStamped addbuff_move_goal_;
  
  ros::NodeHandle shoot_nh_;
  ros::Subscriber firecmd_sub_;
};
}


#endif //ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H
