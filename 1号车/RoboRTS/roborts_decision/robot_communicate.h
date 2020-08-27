#ifndef ROBORTS_DECISION_COMMUNICATE_H
#define ROBORTS_DECISION_COMMUNICATE_H

#include "blackboard/blackboard.h"
#include "robot_information.h"
#include <geometry_msgs/Vector3.h>
#include <math.h>
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
    sentry_sub_ = communicate_nh_.subscribe("robort_pose", 1, &roborts_decision::RobotCommunicate::RobortPose, this);
    isangleachieved_sub_ = communicate_nh_.subscribe("IsAngleAchieved", 1, &roborts_decision::RobotCommunicate::IsAngleAchievedCB, this);
    mypose_pub_ = communicate_nh_.advertise<std_msgs::Float64MultiArray>("red_one_pose",1);
    goal_instert_sub_ = communicate_nh_.subscribe("insert_goal",1, &roborts_decision::RobotCommunicate::InsertGoalChangedCB, this);
  }
    /*哨岗的信息*/
  void  RobortPose(const std_msgs::Float64MultiArray::ConstPtr& msg)
  { 
    if(MyConfig.ROBOT_INFO.number == 1)
    {
      EnemyRobort_Pose.pose.position.x = (msg->data[0])/1000;
      EnemyRobort_Pose.pose.position.y = (msg->data[1])/1000;//enemy one robort pose
      //ROS_ERROR("EnemyRobort_Pose.pose.position.x=%lf",EnemyRobort_Pose.pose.position.x);
      
    }
    else
    {
      EnemyRobort_Pose.pose.position.x = (msg->data[2])/1000;
      EnemyRobort_Pose.pose.position.y = (msg->data[3])/1000;//enemy two robort pose
    }
    SentryAreaMindlePose.pose.position.x = (msg->data[4])/1000;
    SentryAreaMindlePose.pose.position.y = (msg->data[5])/1000;
    EnemyRobort_id = (int)msg->data[6];
    if(EnemyRobort_id == 1 || EnemyRobort_id == 2)
    {
      SentryDirect = true;
    }
    else
    {
      SentryDirect = false;
    }
    
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
    SwingCmdMsg.data = (int)IsStartSwing;
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
  //通过裁判系统再发一次
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
  void InsertGoalChangedCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    Insert_Goal.pose.position.x =msg->pose.position.x;//在右侧加点
    Insert_Goal.pose.position.y =msg->pose.position.y;
    if(msg->pose.position.z == 0)
    {
      InsertGoal = false;
   
    }
    else
    {
      InsertGoal = true;
    }
    Insert_Goal.pose.orientation = msg->pose.orientation;
  }
  
  
  void IsAngleAchievedCB(const std_msgs::Int16::ConstPtr& msg)
  {
    IsAngleAchieved = msg->data;//收到了就认为IsAngleAchieved = 1
  }
  
  
//   void CurrentCondition(roborts_decision::Blackboard* &blackboard)
// {/*
//   roborts_decision::CheckGameStatus 	   	checkgamestatus;
//   checkgamestatus.CheckAreaStatus();*/
//   ROS_ERROR("  testtestestest??????????????????????????????????????");
//   gametime.TimeUpdate();
//   std_msgs::Float64MultiArray current_msg;
//   current_msg.data.resize(5);
//   current_msg.data[0] = blackboard->GetRobotMapPose().pose.position.x;//当前位姿
//   current_msg.data[0] = blackboard->GetRobotMapPose().pose.position.y;//当前位姿
//   
//   current_msg.data[1] = blackboard-> GetEnemy().pose.position.x;     //敌人位姿
//   current_msg.data[1] = blackboard-> GetEnemy().pose.position.y;     //敌人位姿
//   bool ifvisiondetected = blackboard->IsEnemyDetected();
//   bool iflidardetected =blackboard->IsLidarEnemyDetected();
//   if(ifvisiondetected){
//     current_msg.data[2]=(float) blackboard->EnemyDistance();//敌人距离   越大越安全  倾向进攻
//   }
//   else if (iflidardetected)
//   {
//     current_msg.data[2]= (float) blackboard->LidarEnemyDistance();//敌人距离
//     }
//     else if(!ifvisiondetected&&!iflidardetected){
//       current_msg.data[2]= 0.1;//unknown enemy
//     }
//     if(HaveBulletArea){
//      current_msg.data[3] =(float) GetDistance(MyMapPose,MyBulletPoint);  //到补弹区距离
//     }else
//       current_msg.data[3]= 10; 
//     if(HaveHPArea){
//      current_msg.data[4] =(float) GetDistance(MyMapPose,MyHPPoint);      //到加血区距离
//     }else
//      current_msg.data[4] =10;
//   current_msg.data[5]=LeftBulletNum;  //自己剩余子弹   越大越安全  倾向进攻
//                                 // current_msg.data[2] =PartnerLeftBulletNum;//队友剩余子弹
//   current_msg.data[6] = MyHP; //我的血量
//        //                          current_msg.data[4] =PartnerHP; //队友血量
//    //    current_msg.data[2] =                                         //时间
//   current_msg.data[7] = EnemyOneHP;     //对方1血量
//   current_msg.data[8] = EnemyTwoHP;     //对方2血量
//   
//   current_msg.data[9]= EnemyOneLeftBulletNum;        //对方1子弹
//   current_msg.data[10]= EnemyTwoLeftBulletNum;       //对方2子弹
//   ROS_ERROR(" current_msg.data[6] = %lf", current_msg.data[6]);
//     CoreGain coreGain;
// // coreGain.attack_g+=(1/enemy_cond)*自己到敌人距离
// //                 *友军到敌人距离*(1+(paras_f_["attack_k1"]*enemy_cond/friend1_cond));
//  // current_msg.data[11] = GetDistance(MyBulletPoint,MyEscapePoint1);//逃跑点与补弹区距离
// 
// //   if(IsUnderAttack){
// //     current_msg.data[5] = 10;//被攻击
// //   }
//     float my_condition = (log(double(MyHP)+1)+(MyHP/1000)*(LeftBulletNum/200)^(1/2))/3;
//     
// } 
// 

  
  
  
  
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
  ros::Subscriber     isangleachieved_sub_;
  ros::Subscriber     simulate_enemy_pose_sub_;
  ros::Subscriber     goal_instert_sub_;
};
}


#endif //ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H
