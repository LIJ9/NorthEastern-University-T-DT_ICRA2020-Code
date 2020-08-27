#ifndef ROBORTS_DECISION_REFEREE_SYSTEM_H
#define ROBORTS_DECISION_REFEREE_SYSTEM_H

#include <ros/ros.h>
#include "robot_information.h"


namespace roborts_decision {
class RefereeSystem {
 public:
  RefereeSystem()
  {
    //referee_sub_ = referee_nh_.subscribe("referee_system", 1, &roborts_decision::RefereeSystem::ReceiveMsg, this);
    referee_sub_ = referee_nh_.subscribe("debug_to_robots", 1, &roborts_decision::RefereeSystem::ReceiveMsg, this);
    area_status_sub = referee_nh_.subscribe("area_status_msg", 1, &roborts_decision::RefereeSystem::AreaStatusFromReferee, this);
    robort_status_sub = referee_nh_.subscribe("robort_status_msg", 1,&roborts_decision::RefereeSystem::RobortStatusFromReferee, this);
  }
    /*six area status <real status>*/
  void AreaStatusFromReferee(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    BP_Area_Status[0] = (int)msg->data[0];
    BP_Area_Status[1] = (int)msg->data[1];
    BP_Area_Status[2] = (int)msg->data[2];
    BP_Area_Status[3] = (int)msg->data[3];
    BP_Area_Status[4] = (int)msg->data[4];
    BP_Area_Status[5] = (int)msg->data[5];
  }
  void RobortStatusFromReferee(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    struct bits_t
    {
      uint32_t bit0:1;
      uint32_t bit1:1;
      uint32_t bit2:1;
      uint32_t bit3:1;
    };
    union uint2bits_t
    {
      uint32_t iNum;
      struct bits_t bits;
    }uint2bits;
    
    uint2bits.iNum = (uint32_t)msg->data[0];
    my_one_status[0] = uint2bits.bits.bit0;//1 i have been added hp
    my_one_status[1] = uint2bits.bits.bit1;//1 i have been added bullet
    my_one_status[2] = uint2bits.bits.bit2;//1 i can't shoot
    my_one_status[3] = uint2bits.bits.bit3;//1 i can't move 
    
    uint2bits.iNum = (uint32_t)msg->data[1];
    my_two_status[0] = uint2bits.bits.bit0;
    my_two_status[1] = uint2bits.bits.bit1;
    my_two_status[2] = uint2bits.bits.bit2;
    my_two_status[3] = uint2bits.bits.bit3;
    
    uint2bits.iNum = (uint32_t)msg->data[2];
    enemy_one_status[0] = uint2bits.bits.bit0;
    enemy_one_status[1] = uint2bits.bits.bit1;
    enemy_one_status[2] = uint2bits.bits.bit2;
    enemy_one_status[3] = uint2bits.bits.bit3;
    
    uint2bits.iNum = (uint32_t)msg->data[3];
    enemy_two_status[0] = uint2bits.bits.bit0;
    enemy_two_status[1] = uint2bits.bits.bit1;
    enemy_two_status[2] = uint2bits.bits.bit2;
    enemy_two_status[3] = uint2bits.bits.bit3;
  }
  void ReceiveMsg(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    /*ICRA2020
    0剩余的子弹数目
    1已发射的子弹数目
    2装甲板id
    3我的血量
    通过双方的总血量来计算总伤害量,通过总伤害和可发射子弹数来判断优势劣势
    */
   //referee_system test
    if(MyConfig.ROBOT_INFO.color == "RED")
    {
      if(MyConfig.ROBOT_INFO.number == 1)
      {
        static float LastPartnerHP; static float LastMyHP; 
        static float LastEnemyOneHP; static float LastEnemyTwoHP;
        LastMyHP = MyHP;
        LastPartnerHP = PartnerHP;
        LastEnemyOneHP = EnemyOneHP;
        LastEnemyTwoHP = EnemyTwoHP;

        GameStatus = msg->data[16];//0－未开始比赛，１－５分钟准备阶段，２－裁判系统自检阶段，３－5s倒计时，４－对战中，５－比赛结算中
        MyHP = msg->data[3];
        PartnerHP = msg->data[7];//RED2 HP
        EnemyOneHP = msg->data[11];//BLUE1 HP
        EnemyTwoHP = msg->data[15];//BLUE2 HP

        //如果当前血量大于上一次的血量，则TotalInjuryVolume+200
        if((MyHP>LastMyHP)||(PartnerHP>LastPartnerHP))
        {
          EnemyTotalInjuryVolume = (HPBASE+ADDHP-MyHP)+(HPBASE+ADDHP-PartnerHP);
        }
        else
        {
          EnemyTotalInjuryVolume = (HPBASE-MyHP)+(HPBASE-PartnerHP);
        }
        
        if((EnemyOneHP>LastEnemyOneHP)||(EnemyTwoHP>LastEnemyTwoHP))
        {
          MyTotalInjuryVolume = (HPBASE+ADDHP-EnemyOneHP)+(HPBASE+ADDHP-EnemyTwoHP);
        }
        else
        {
          MyTotalInjuryVolume = (HPBASE-EnemyOneHP)+(HPBASE-EnemyTwoHP);
        }
        
        if((MyTotalInjuryVolume)>=(EnemyTotalInjuryVolume))
        {
          IsAdvantage = true;
        }
        else
        {
          IsAdvantage = false;
        }

        LeftBulletNum = msg->data[0];
        ShootBulletNum = msg->data[1];
        PartnerLeftBulletNum = msg->data[4];
        EnemyOneLeftBulletNum = msg->data[8];
        EnemyOneLeftBulletNum = msg->data[12];

        static int LastUnderAttackArmorId = 5;//５代表不受伤害
        LastUnderAttackArmorId = UnderAttackArmorId;
        /*认为所有的伤害都是装甲板的伤害*/
        if(msg->data[2] >=9 && msg->data[2] <= 12)
          UnderAttackArmorId = msg->data[2] - 9;
        else
          UnderAttackArmorId = msg->data[2];
          /*0-4表示受到子弹伤害的5个装甲板
          5表示补受到伤害
          6表示模块离线扣血
          7表示枪口超热量扣血
          8表示底盘超功率扣血
          9-13表示受到撞击伤害的装甲板
          如果是击打伤害*/
          CdUnderAttack++;
        if(UnderAttackArmorId != LastUnderAttackArmorId)
          CdUnderAttack = 10;
        if(CdUnderAttack >= 10)	//1s
        {
          CdUnderAttack = 10;
          if(UnderAttackArmorId <= 3) // 0 1 2 3
          {
            IsUnderAttack = true;
            CdUnderAttack = 0;
          }
          else
            IsUnderAttack = false;
        }
      }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*ICRA2019
    收裁判系统的信息
    0自己可发射子弹数
    1我的血量
    2比赛进程
    3受攻击装甲板
    4当前的buff
    5敌方1号是否存活
    6敌方2号是否存活
    7补弹区的状态*/
    /*
    LastBulletStationStatus = BulletStationStatus;
    BulletStationStatus = msg->data[7];
    if(BulletStationStatus == 0 && LastBulletStationStatus == 2)
    {
      IsAddBulletFinish = true;
      ROS_INFO("IsAddBulletFinish : 1  AddBulletFinish!!!");
    }
    else
      IsAddBulletFinish = false;*/
  }

  ~RefereeSystem() = default;

 private:
  ros::NodeHandle     referee_nh_; //句柄
  ros::Subscriber     referee_sub_;
  ros::Subscriber     area_status_sub;
  ros::Publisher      area_status_pub;
  ros::Subscriber     robort_status_sub;
};
}


#endif //ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H
