#ifndef KILL_PENDING_H
#define KILL_PENDING_H

#include "robot_information.h"
#include "behavior_tree/behavior_state.h"
#include "executor/chassis_executor.h"

roborts_decision::BehaviorState MyChassisState;

bool KillPending(roborts_decision::ChassisExecutor* &chassis_executor)
{
//   static int PendingCnt = 0;
// 
//   //MyChassisState = chassis_executor->Update();
//   
//   if(MyChassisState == roborts_decision::BehaviorState::PENDING)
//   {
//     PendingCnt++;
//     ROS_INFO("Pending times : %d ~~~~~~~~~~~~~~~~~~~~",PendingCnt);
//     if(PendingCnt >= 50) //pending 5s
//     {
//       ROS_ERROR("~~~~~~~~~~~~~~~~~~~~~Oh No!!!!I am Pending~~~~~~~~~~~~~~~~~~~~");
//       
//       auto dx = 5.0 - MyMapPose.pose.position.x;
//       auto dy = 2.5 - MyMapPose.pose.position.y;
//       auto target_yaw = std::atan2(dy, dx); 
//       
//       geometry_msgs::Twist Vel;
//       if( target_yaw - MyYaw > 0.3)
//       {
// 	Vel.angular.z = 0.5;	
//       }
//       else if( target_yaw - MyYaw < -0.3)
//       {
// 	Vel.angular.z = -0.5;	
//       }
//   
//       if(std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.1)
//       {
// 	Vel.linear.x = 0.5;
//       }
//       chassis_executor->Execute(Vel); 
//       
//       ROS_ERROR("~~~~~~~~~~~~~~~~~~~~~Oh No!!!!I am Pending~~~~~~~~~~~~~~~~~~~~");
//       IsStartSwing = true;    
//       return true;
//     }
//   }
//   else
//   {
//     //ROS_INFO("Yeah!No~~~~~~~~~~~~~~~~~~~~");
//     PendingCnt = 0;
//   }
  
//   auto dx = MyMapPose.pose.position.x - LastMyMapPose.pose.position.x;
//   auto dy = MyMapPose.pose.position.y - LastMyMapPose.pose.position.y;
//   
//   static int NoMovingCnt = 0;
//   if(std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) < 0.01 && MyChassisState == roborts_decision::BehaviorState::RUNNING)
//   {
//     NoMovingCnt++;
//     if(NoMovingCnt >= 50)  //5s
//     {  
//       ROS_ERROR("~~~~~~~~~~~~~~~~~~~~~Oh No!!!!I Can't Move~~~~~~~~~~~~~~~~~~~~");   
//       auto dx1 = 5.0 - MyMapPose.pose.position.x;
//       auto dy1 = 2.5 - MyMapPose.pose.position.y;
//       auto target_yaw = std::atan2(dy1, dx1); 
//       
//       geometry_msgs::Twist Vel;
//       if( target_yaw - MyYaw > 0.3)
//       {
// 	Vel.angular.z = 0.5;	
//       }
//       else if( target_yaw - MyYaw < -0.3)
//       {
// 	Vel.angular.z = -0.5;	
//       }
//   
//       if(std::sqrt(std::pow(dx1, 2) + std::pow(dy1, 2)) > 0.1)
//       {
// 	Vel.linear.x = 0.5;
//       }
//       
//       chassis_executor->Execute(Vel); 
//     }
//   }
//   else
//   {
//     NoMovingCnt = 0;
//   }
  
  return false;
}

#endif