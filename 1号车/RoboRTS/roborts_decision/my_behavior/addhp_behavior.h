#ifndef ROBORTS_DECISION_ADDBUFF_BEHAVIOR_H
#define ROBORTS_DECISION_ADDBUFF_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "../example_behavior/line_iterator.h"
#include <std_msgs/Int16.h>
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
    if (!LoadParam()) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

    adjust_pub_ = addhp_nh_.advertise<std_msgs::Int16>("adjust", 1);  
    //精准定位
    adjustfinish_sub_ = addhp_nh_.subscribe("adjust_over", 1, &roborts_decision::AddHPBehavior::AdjustFinishCB, this); 
  }

  //executor_state
  void Run() 
  {
    hp_goals_.header.frame_id = "map";
    hp_goals_.pose.position.x = MyHPPoint.pose.position.x;
    hp_goals_.pose.position.y = MyHPPoint.pose.position.y;
    hp_goals_.pose.position.z = MyHPPoint.pose.position.z;
    auto hp_goals_quaternion = tf::createQuaternionMsgFromRollPitchYaw( 0,0, MyHPPoint.pose.orientation.w);
    hp_goals_.pose.orientation = hp_goals_quaternion;
    BehaviorStatusUpdate(ADDHP_STATUS);
    ROS_INFO("[2]. Add HP");
    IsStartSwing = false;
    static bool ArrivedFalg = false;
    if(ArrivedFalg)
    {     
      ROS_INFO("[2]. ......Adjusting......");
      if(adjustfinishflag)
      {
        ArrivedFalg = false;
        adjustfinishflag = false;
        IsAddingHP = true;
        gametime.RecordWaitHPSec();
        CdUnderAttack = 20;
        Cancel();
        //geometry_msgs::Twist Vel0;   
        //chassis_executor_->Execute(Vel0);	//send 0 speed
      }
      return;  //warning!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }
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
      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 1 && d_yaw <= 0.1 && MyChassisState == BehaviorState::SUCCESS)
      {
        Cancel();
        adjustmsg.data = 1;
        adjust_pub_.publish(adjustmsg);
        ArrivedFalg = true;
        std::cout << "add hp point arrived!!!" << std::endl;
      }
    }
  }
  //simulate 
  /*void Run() 
  {
    hp_goals_.header.frame_id = "map";
    hp_goals_.pose.position.x = MyHPPoint.pose.position.x;
    hp_goals_.pose.position.y = MyHPPoint.pose.position.y;
    hp_goals_.pose.position.z = MyHPPoint.pose.position.z;
    auto hp_goals_quaternion = tf::createQuaternionMsgFromRollPitchYaw( 0,0, MyHPPoint.pose.orientation.w);
    hp_goals_.pose.orientation = hp_goals_quaternion;
    BehaviorStatusUpdate(ADDBULLET_STATUS);  
    ROS_INFO("[2]. Add HP");
    static bool ArrivedFalg = false;
    IsUnderAttack = IsStartCounter = IsStartSwing = false;
    if(ArrivedFalg)
    { 
      ROS_INFO("[2]. ......Adjusting......");
      if(adjustfinishflag)
      {
        ArrivedFalg = false;
        adjustfinishflag = false;
        IsAddingHP = true;
        gametime.RecordWaitHPSec();
        CdUnderAttack = 20;
      }
      return;  //warning!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }
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
      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 1 && d_yaw <= 0.1 &&MyChassisState == BehaviorState::SUCCESS) 
      {
        Cancel(); //最好加上
        adjustmsg.data = 1;
        ArrivedFalg = true;
        std::cout << "add bullet point arrived!!!" << std::endl;
        adjustfinishflag = true;
      }
    }
  }*/

  void AdjustFinishCB(const std_msgs::Int16::ConstPtr& msg)
  {
    adjustfinishflag = (bool)msg->data;
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
    adjustfinishflag = false;
    adjustmsg.data = 1;
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
  ros::NodeHandle addhp_nh_; 
  ros::Publisher  adjust_pub_;
  ros::Subscriber adjustfinish_sub_;
  std_msgs::Int16 adjustmsg;
  //buff goals
  geometry_msgs::PoseStamped hp_goals_;
  
};
}

#endif 
