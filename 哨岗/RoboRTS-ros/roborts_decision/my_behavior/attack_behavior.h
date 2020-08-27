#ifndef ROBORTS_DECISION_ATTACK_BEHAVIOR_H
#define ROBORTS_DECISION_ATTACK_BEHAVIOR_H

#include "io/io.h"
#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"
#include "../robot_information.h"
#include "../kill_pending.h"
namespace roborts_decision{
  class AttackBehavior{
  public:
    AttackBehavior(ChassisExecutor* &chassis_executor,
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
      BehaviorStatusUpdate(ATTACK_STATUS);
      ROS_INFO("[2]. My Attack");
      auto robot_map_pose = blackboard_->GetRobotMapPose();
      auto derta_y = SentryAreaMindlePose.pose.position.y-robot_map_pose.pose.position.y;
      auto derta_x = SentryAreaMindlePose.pose.position.x-robot_map_pose.pose.position.x;
      SentryAreaYaw = atan2(derta_y,derta_x);
      auto sentry_area_quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, SentryAreaYaw);
      EnemyRobort_Pose.pose.orientation = sentry_area_quaternion;
      chassis_executor_->Execute(EnemyRobort_Pose);
      if (MyChassisState != BehaviorState::RUNNING) 
      {
        auto dx = EnemyRobort_Pose.pose.position.x - robot_map_pose.pose.position.x;
	      auto dy = EnemyRobort_Pose.pose.position.y - robot_map_pose.pose.position.y;
        tf::Quaternion rot1, rot2;
        tf::quaternionMsgToTF(EnemyRobort_Pose.pose.orientation, rot1);
        tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
        auto d_yaw =  rot1.angleShortestPath(rot2);
	    if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 1 && d_yaw <= 1 && MyChassisState == BehaviorState::SUCCESS) 
        {
	        Cancel();
        }
      }
    }
    bool LoadParam(const std::string &proto_file_path) 
    {
      roborts_decision::DecisionConfig decision_config;
      if (!roborts_common::ReadProtoFromTextFile(proto_file_path, 
      &decision_config))
      {
        return false;
      }
      return true;
    }
    void Cancel() 
    {
      chassis_executor_->Cancel();
    }

    BehaviorState Update() {
      return chassis_executor_->Update();
    }
    void BehaviorStatusUpdate(int BehaviorStaus)
    {  
      NowStatus = BehaviorStaus;
      if(NowStatus != LastStatus) //BehaviorStatus is Changed!!
      {  
        Cancel();
      }
      LastStatus = NowStatus;
    }
    ~AttackBehavior() = default;
    private:
  //! executor
  ChassisExecutor* const chassis_executor_;
  //! perception information
  Blackboard* const blackboard_;
  geometry_msgs::PoseStamped attack_goal_;
  };
}
#endif