#ifndef ROBORTS_DECISION_ADDBULLET_BEHAVIOR_H
#define ROBORTS_DECISION_ADDBULLET_BEHAVIOR_H

#include "io/io.h"
#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"
#include "../example_behavior/line_iterator.h"
#include "../robot_information.h"
#include <std_msgs/Int16.h>
#include "../kill_pending.h"

namespace roborts_decision {
class AddBulletBehavior {
 public:
  AddBulletBehavior(ChassisExecutor* &chassis_executor,
                 Blackboard* &blackboard,
                 const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                        blackboard_(blackboard) {

    if (!LoadParam()) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

    adjust_pub_ = addbullet_nh_.advertise<std_msgs::Int16>("adjust", 1);  
    //精准定位
    //adjustfinish_sub_ = addbullet_nh_.subscribe("cmd_vel1", 1, &roborts_decision::AddBulletBehavior::AdjustFinishCB, this);
    adjustfinish_sub_ = addbullet_nh_.subscribe("adjust_over", 1, &roborts_decision::AddBulletBehavior::AdjustFinishCB, this); 
  }

  //real world test
  void Run() 
  {
    addbullet_goals_.header.frame_id = "map";
    addbullet_goals_.pose.position.x = MyBulletPoint.pose.position.x;
    addbullet_goals_.pose.position.y = MyBulletPoint.pose.position.y;
    addbullet_goals_.pose.position.z = MyBulletPoint.pose.position.z;
    auto addbullet_goals_quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, MyBulletPoint.pose.orientation.w);
    addbullet_goals_.pose.orientation = addbullet_goals_quaternion;
    BehaviorStatusUpdate(ADDBULLET_STATUS);  
    ROS_INFO("[2]. AddBullet"); 
    //IsUnderAttack = IsStartCounter = IsStartSwing = false;
    IsStartSwing = false;
    static bool ArrivedFalg = false;
    if(ArrivedFalg)
    {     
      ROS_INFO("[2]. ......Adjusting......");
      if(adjustfinishflag)
      {
        ArrivedFalg = false;
        adjustfinishflag = false;
        IsAddingBullet = true;
        gametime.RecordWaitBulletSec();
        CdUnderAttack = 20;
        geometry_msgs::Twist Vel0;   
        chassis_executor_->Execute(Vel0);	//send 0 speed
      }
      return;  //warning!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }
    if (MyChassisState != BehaviorState::RUNNING) 
    {
      auto robot_map_pose = blackboard_->GetRobotMapPose();
      //获取机器人的世界坐标
      auto dx = addbullet_goals_.pose.position.x - robot_map_pose.pose.position.x;
      auto dy = addbullet_goals_.pose.position.y - robot_map_pose.pose.position.y;
      tf::Quaternion rot1, rot2;//类Quaternion的对象
      tf::quaternionMsgToTF(addbullet_goals_.pose.orientation, rot1);//补弹点四元数转欧拉角
      tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);//机器人当前位姿四元数转欧拉角
      auto d_yaw =  rot1.angleShortestPath(rot2);//Return the shortest angle between two quaternions.欧拉角ｙａｗ差值(弧度)
      chassis_executor_->Execute(addbullet_goals_);
      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 1 && d_yaw <= 1 && MyChassisState == BehaviorState::SUCCESS) 
      {
        Cancel(); //最好加上
        adjustmsg.data = ADDNUM;
        adjust_pub_.publish(adjustmsg);  
        ArrivedFalg = true;
        std::cout << "add bullet point arrived!!!" << std::endl;
      }
    }
  }
  //simulate 
  /*void Run() 
  {
    addbullet_goals_.header.frame_id = "map";
    addbullet_goals_.pose.position.x = MyBulletPoint.pose.position.x;
    addbullet_goals_.pose.position.y = MyBulletPoint.pose.position.y;
    addbullet_goals_.pose.position.z = MyBulletPoint.pose.position.z;
    auto addbullet_goals_quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, MyBulletPoint.pose.orientation.w);
    addbullet_goals_.pose.orientation = addbullet_goals_quaternion;
    BehaviorStatusUpdate(ADDBULLET_STATUS);  
    ROS_INFO("[2]. AddBullet"); 
    static bool ArrivedFalg = false;
    IsUnderAttack = IsStartCounter = IsStartSwing = false;
    if(ArrivedFalg)
    { 
      ROS_INFO("[2]. ......Adjusting......");
      if(adjustfinishflag)
      {
        ArrivedFalg = false;
        adjustfinishflag = false;
        IsAddingBullet = true;
        gametime.RecordWaitBulletSec();
        CdUnderAttack = 20;
        //geometry_msgs::Twist Vel0;   
        //chassis_executor_->Execute(Vel0);	//send 0 speed
      }
      return;  //warning!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }
    if (MyChassisState != BehaviorState::RUNNING) 
    {
      auto robot_map_pose = blackboard_->GetRobotMapPose();
      auto dx = addbullet_goals_.pose.position.x - robot_map_pose.pose.position.x;
      auto dy = addbullet_goals_.pose.position.y - robot_map_pose.pose.position.y;
      tf::Quaternion rot1, rot2;
      tf::quaternionMsgToTF(addbullet_goals_.pose.orientation, rot1);
      tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
      auto d_yaw =  rot1.angleShortestPath(rot2);
      chassis_executor_->Execute(addbullet_goals_);
      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 1 && d_yaw <= 2 &&MyChassisState == BehaviorState::SUCCESS) 
      {
        Cancel(); //最好加上
        adjustmsg.data = ADDNUM;
        //adjust_pub_.publish(adjustmsg);  
        ArrivedFalg = true;
        std::cout << "add bullet point arrived!!!" << std::endl;
        adjustfinishflag = true;
      }
    }
  }*/
  
  void AdjustFinishCB(const geometry_msgs::Twist::ConstPtr& msg)
  {
    adjustfinishflag = (bool)msg->angular.x;
    //adjustfinishflag = false;
  }
  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  bool LoadParam()
  {
    adjustfinishflag = false;
    adjustmsg.data = ADDNUM;
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
  
  ~AddBulletBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //buff goals
  geometry_msgs::PoseStamped addbullet_goals_;
  
  ros::NodeHandle addbullet_nh_; 
  ros::Publisher  adjust_pub_;
  ros::Subscriber adjustfinish_sub_;
  std_msgs::Int16 adjustmsg;
  
  //bool adjustfinishflag;
};
}

#endif 
