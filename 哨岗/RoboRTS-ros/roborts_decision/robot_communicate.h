#ifndef ROBORTS_DECISION_COMMUNICATE_H
#define ROBORTS_DECISION_COMMUNICATE_H

#include "blackboard/blackboard.h"
#include "robot_information.h"
#include <geometry_msgs/Vector3.h>
namespace roborts_decision {
class RobotCommunicate {
 public:
   tf::TransformBroadcaster enemy_broadcaster;//3-20 
   geometry_msgs::TransformStamped enemy_trans;//3-20
  RobotCommunicate(Blackboard* &blackboard) : blackboard_(blackboard)
  { 
    //No.1
    communicate_pub_ = communicate_nh_.advertise<std_msgs::Float64MultiArray>("DecisionToReferee", 1);
    communicate_sub_ = communicate_nh_.subscribe("RefereeToDecision", 1, &roborts_decision::RobotCommunicate::RefereeToDecisionCB, this); 
    swing_pub_ = communicate_nh_.advertise<std_msgs::Int16>("Swing_Cmd", 1);
    chassisfollowgimabl_pub_ = communicate_nh_.advertise<std_msgs::Float64MultiArray>("ChassisFollowGimbal", 1);
    sentry_sub_ = communicate_nh_.subscribe("sentry_info", 1, &roborts_decision::RobotCommunicate::RobortPose, this);
    isangleachieved_sub_ = communicate_nh_.subscribe("IsAngleAchieved", 1, &roborts_decision::RobotCommunicate::IsAngleAchievedCB, this);
    mypose_pub_ = communicate_nh_.advertise<std_msgs::Float64MultiArray>("my_pose",1);
    plot_pub_ = communicate_nh_.advertise< geometry_msgs::Vector3 >( "accel", 1000 );
  }
    /*哨岗的信息*/
  void  RobortPose(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    if(MyConfig.ROBOT_INFO.number == 1)
    {
      EnemyRobort_Pose.pose.position.x = (msg->data[0])/1000;
      EnemyRobort_Pose.pose.position.y = (msg->data[1])/1000;//enemy one robort pose
    }
    else
    {
      EnemyRobort_Pose.pose.position.x = (msg->data[2])/1000;
      EnemyRobort_Pose.pose.position.y = (msg->data[3])/1000;//enemy two robort pose
    }
    SentryAreaMindlePose.pose.position.x = (msg->data[4])/1000;
    SentryAreaMindlePose.pose.position.y = (msg->data[5])/1000;
    EnemyRobort_id = (int)msg->data[6];
  }
  void SendMsg() 
  {
    /*my pose to shaogang*/
    std_msgs::Float64MultiArray my_pose;
    my_pose.data.resize(2);
    my_pose.data[0] = MyMapPose.pose.position.x;
    my_pose.data[1] = MyMapPose.pose.position.y;
    mypose_pub_.publish(my_pose);
    //CommunicateMsg
    std_msgs::Float64MultiArray CommunicateMsg;
    CommunicateMsg.data.resize(8);  
    CommunicateMsg.data[0] = MyMapPose.pose.position.x;
    CommunicateMsg.data[1] = MyMapPose.pose.position.y;
    if(IsVisionDetected)
    {
      CommunicateMsg.data[2] = LastEnemyPose.pose.position.x;
      CommunicateMsg.data[3] = LastEnemyPose.pose.position.y;   
    }
    else
    {
      CommunicateMsg.data[2] = 0;
      CommunicateMsg.data[3] = 0;
    }   
    CommunicateMsg.data[4] = LeftBulletNum;
    if(MyConfig.ROBOT_INFO.number == 1)		//1 send cmd to 2
    {
      CommunicateMsg.data[5] = IsNewMin;
      CommunicateMsg.data[6] = No2SendCmd; 	
    }
    CommunicateMsg.data[7] = MyHP; 
    communicate_pub_.publish(CommunicateMsg);
    
    //SwingCmdMsg
    std_msgs::Int16 SwingCmdMsg; 
    SwingCmdMsg.data = IsStartSwing;
    swing_pub_.publish(SwingCmdMsg);
    
    //ChassisFollowGimbalMsg
    std_msgs::Float64MultiArray ChassisFollowGimbalMsg;
    ChassisFollowGimbalMsg.data.resize(6);
    ChassisFollowGimbalMsg.data[0] = IsEnableVision;
    ChassisFollowGimbalMsg.data[1] = IsCounting;
    ChassisFollowGimbalMsg.data[2] = IsLidaring;
    ChassisFollowGimbalMsg.data[3] = HoldUnderAttackArmorId;
    ChassisFollowGimbalMsg.data[4] = MyYaw;
    ChassisFollowGimbalMsg.data[5] = LidarYaw;
    chassisfollowgimabl_pub_.publish(ChassisFollowGimbalMsg);
  }
  void RefereeToDecisionCB(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    PartnerMapPose.pose.position.x = msg->data[0];//队友的位姿
    PartnerMapPose.pose.position.y = msg->data[1];
    
    if(msg->data[2] != 0 && msg->data[3] != 0)
      IsReceiveEnemyPose = true;
    else 
      IsReceiveEnemyPose = false;
    
    receive_enemy_pose_.pose.position.x = msg->data[2];
    receive_enemy_pose_.pose.position.y = msg->data[3];
    receive_enemy_pose_.pose.position.z = 0;
    auto robot_map_pose = blackboard_->GetRobotMapPose();
    auto receive_enemy_pose_quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 
    atan2( (receive_enemy_pose_.pose.position.y-robot_map_pose.pose.position.y), (receive_enemy_pose_.pose.position.x-robot_map_pose.pose.position.x) ) );
    receive_enemy_pose_.pose.orientation = receive_enemy_pose_quaternion;
    PartnerLeftBulletNum = msg->data[4];
    PartnerHP = msg->data[5];
    if(MyConfig.ROBOT_INFO.number == 2)		//2 receive cmd from 1
    {
      No2ReceiveFlag = (bool)msg->data[6];
      No2ReceiveCmd = (int)msg->data[7];
    }
  }
  void IsAngleAchievedCB(const std_msgs::Int16::ConstPtr& msg)
  {
    IsAngleAchieved = msg->data;//收到了就认为IsAngleAchieved = 1
  }
  ~RobotCommunicate() = default;

 private:
  Blackboard* const   blackboard_;
  ros::NodeHandle     communicate_nh_; 
  ros::Publisher      communicate_pub_;
  ros::Subscriber    	communicate_sub_;
  ros::Subscriber    	sentry_sub_;
  ros::Publisher      mypose_pub_;
  ros::Publisher      swing_pub_;
  ros::Publisher      chassisfollowgimabl_pub_;
  ros::Publisher      plot_pub_;
  ros::Subscriber     isangleachieved_sub_;
  ros::Subscriber     simulate_enemy_pose_sub_;
};
}


#endif //ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H
