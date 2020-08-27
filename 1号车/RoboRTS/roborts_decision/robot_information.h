#ifndef ROBOT_INFORMATION_H
#define ROBOT_INFORMATION_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include "my_file.h"

#define  ADDBULLET_STATUS      1
#define  ADDHP_STATUS          2
#define  SHOOT_STATUS          3
#define  MYCHASE_STATUS        4
#define  COUNTER_STATUS        5
#define  SUPPORT_STATUS        6
#define  LIDARDETECTED_STATUS  7
#define  MYESCAPE_STATUS       8
#define  MYPATROL_STATUS       9
#define  ATTACK_STATUS		     10	

#define  ADDHP  200            
#define  ADDNUM 100
#define  HPBASE      2000
#define  BULLETBASE  57   // 340/3/2=56.667
void No1RobotDiscuss();
void No2RobotReceive();

bool adjustfinishflag = false;
//bool IsStucked = false;
//own
int LastStatus,NowStatus;
bool IsNewMin = false;  geometry_msgs::PoseStamped MyMapPose; geometry_msgs::PoseStamped LastMyMapPose; double MyYaw;
bool AddBulletChance = true; bool HaveBullet = false;     
bool IsLidarDetected = false; geometry_msgs::PoseStamped LidarEnemyPose; float LidarYaw;
geometry_msgs::PoseStamped LastEnemyPose; geometry_msgs::PoseStamped EnemyPose;
bool IsVisionDetected = false; bool  LastIsVisionDetected = false; bool FireCmd = false;                                                               
bool IsVisionFirstLost = false;  bool IsRunMyChase = false;                                               
bool AddHPChance = true;
bool IsChangeGoals = false; bool ChangedGoals = false;
bool IsAddingBullet = false; bool IsAddingHP = false; bool IsChasingEnemy = false;

//partner
geometry_msgs::PoseStamped PartnerMapPose;
float PartnerLeftBulletNum = 0; bool PartnerHaveBullet = false;
float PartnerHP = 2000; //solo : 2000 PartnerHP default is 0;
bool IsReceiveEnemyPose = true/*false*/; geometry_msgs::PoseStamped receive_enemy_pose_;////////////8.7

//referee system
bool IsShoot = false;
int TotalBulletNum = 50; float LeftBulletNum = 50; int ShootBulletNum = 0;  int LastShootBulletNum = 0; int ShootBulletNumOffset = 0;
int EnemyOneLeftBulletNum = 150; int EnemyTwoLeftBulletNum = 100;
float MyHP = 2000; 
int GameStatus;
bool IsUnderAttack = false; int UnderAttackArmorId = 5; bool IsStartCounter = false;  int CdUnderAttack = 0;
int BuffStatus;
float EnemyOneHP = 2200;  float EnemyTwoHP = 2200; 
int BulletStationStatus; int LastBulletStationStatus; 
bool IsAddBulletFinish = false; bool IsAddHPFinish = false;
bool IsAdvantage = true; int MyTotalInjuryVolume = 0; int EnemyTotalInjuryVolume = 0;

//cmd
bool IsStartSwing = false;
bool IsEnableVision = false;  bool IsCounting = false; bool IsLidaring = false; bool IsAngleAchieved = false;
int HoldUnderAttackArmorId = 0; geometry_msgs::PoseStamped HoldLidarEnemyPose; float HoldLidarYaw;
bool HaveBulletArea = false; bool HaveHPArea = false;//判断是否有加血加弹区的标志位
int add_bullet_area_point = 9; int add_hp_area_point = 9; bool IfNeedAddHP = false;
bool AddBulletCmd1 = false; bool AddBulletCmd2 = false; bool AddHPCmd1 = false; bool AddHPCmd2 = false;
int No2SendCmd = 0; int No2ReceiveCmd = 0; bool No2ReceiveFlag = false;

//enemy_area_pose form sentry
geometry_msgs::PoseStamped EnemyRobort_Pose;int EnemyRobort_id; bool SentryDirect = false;
geometry_msgs::PoseStamped SentryAreaMindlePose;double SentryAreaYaw;//哨岗发出的敌方所在区域的中心坐标及我的角度
int BP_Area_Status[6];
geometry_msgs::PoseStamped MyBulletPoint; geometry_msgs::PoseStamped MyHPPoint;
geometry_msgs::PoseStamped EnemyBulletPoint; geometry_msgs::PoseStamped EnemyHPPoint;
uint16_t my_one_status[4]; uint16_t my_two_status[4]; uint16_t enemy_one_status[4]; uint16_t enemy_two_status[4];
//enemy_pose form robort camera
double Enemy_Distance;  bool Enemy_In_Distance = false;  double Angle_Gimbal_Chassis = 0.0;
//卡住时增设中间点
// geometry_msgs::PoseStamped Insert_Goal; bool InsertRGoal = false;
geometry_msgs::PoseStamped Insert_Goal; bool InsertGoal = false; bool IsGoalChanged = false; 
double IfChanged; double LastIfChanged = 0; bool AddBulletChanged = false; bool AddHPChanged = false;
bool Go_To_AddBullet = false; bool Go_To_AddHP = false; 

////哨岗决策
 bool shoot_con = false; 

 
 ////行为函数

 bool follow_addbullet = false;
 bool MyTeamAdvantage =true;
 bool need_support = false;
 double my_condition = 0; double partner_condition = 0; double myTeam_condition = 0; double virtual_dist = 0;  
 double EnemyOne_condition = 0; double EnemyTwo_condition; double EnemyTeam_condition = 0; double safety_value= 0;
 double partner_dist; int current_target = 0; 

namespace roborts_decision 
{
  class GameTime
  {
   public:
      double  startsec;
      double  gametimesec;
      int64_t gametimesec_int;
      int64_t recordbuffsec_int;
      int64_t waithpsec_int;
      int64_t waitbulletsec_int;
      
      GameTime() : 
      startsec(0),
      gametimesec(0),
      gametimesec_int(0),
      recordbuffsec_int(0)
      {
	      ros::Time::init();
      }
      
      void TimeUpdate()
      {
        gametimesec = ros::Time::now().toSec() - startsec;
        gametimesec_int = (int)gametimesec;
        ROS_INFO("[1]. Game time is: %dm, %ds, Total %fs", (int)(gametimesec_int/60), (int)(gametimesec_int%60), gametimesec); 
        if(gametimesec_int % 60 == 0)	//new min 
          IsNewMin = true;
        else
          IsNewMin = false;

        if(IsAddingHP && IsAddHPFinish)
        {
          IsAddingHP = false;
          AddHPChance = false;
        }

        if(IsAddHPFinish)
        {
          IsStartSwing = false;
          IsAddHPFinish = false;
        }

        if(IsAddingBullet)
	  // if(IsAddingBullet && IsAddBulletFinish)
        {
          IsAddingBullet = false;
          AddBulletChance = false;
        }

        if(IsAddBulletFinish)
        {
          IsStartSwing = false;
          IsAddBulletFinish = false;
        }
      }

      void RecordGameStartTime()
      {
	      startsec = ros::Time::now().toSec();
      }  

      void RecordBuffStartTime()
      {
	      recordbuffsec_int = gametimesec_int;
      }
      
      void RecordWaitHPSec()
      {
	      waithpsec_int = gametimesec_int;
      }
      
      void RecordWaitBulletSec()
      {
	      waitbulletsec_int = gametimesec_int;
      }
      
      ~GameTime() = default;
  };
}
roborts_decision::GameTime gametime;

namespace roborts_decision
{
  class CheckGameStatus
  {
  public:
    CheckGameStatus()
    {
      gamestatus_pub_ = checkgamestatus_nh_.advertise<std_msgs::Int16>("check", 1);
    }
    //加成惩罚区的状态
    void CheckAreaStatus()
    {
#if 0
      if(BP_Area_Status[add_bullet_area_point] == 0)
      {
        IsAddBulletFinish = true;
        AddBulletChance = false;
        HaveBulletArea = false;
      }
      if(BP_Area_Status[add_hp_area_point] == 0)
      {
        IsAddHPFinish = true;
        AddHPChance = false;
        HaveHPArea = false;
      }
#endif
      for(int i=0;i<6;i++)
      {
        if(BP_Area_Status[i] == 1)
        {
          add_bullet_area_point = i;
          HaveBulletArea = true;//补给完毕之后将标志位置为false
          MyBulletPoint.pose.position.x = MyConfig.BONUS_PENALTY_POINT[i].x;
          MyBulletPoint.pose.position.y = MyConfig.BONUS_PENALTY_POINT[i].y;
          MyBulletPoint.pose.position.z = MyConfig.BONUS_PENALTY_POINT[i].z;
          MyBulletPoint.pose.orientation.w = MyConfig.BONUS_PENALTY_POINT[i].yaw;
        }
        if(BP_Area_Status[i] == 2)
        {
          add_hp_area_point = i;
          HaveHPArea = true;
          MyHPPoint.pose.position.x = MyConfig.BONUS_PENALTY_POINT[i].x;
          MyHPPoint.pose.position.y = MyConfig.BONUS_PENALTY_POINT[i].y;
          MyHPPoint.pose.position.z = MyConfig.BONUS_PENALTY_POINT[i].z;
          MyHPPoint.pose.orientation.w = MyConfig.BONUS_PENALTY_POINT[i].yaw;
        }
        if(BP_Area_Status[i] == 3)
        {
          EnemyBulletPoint.pose.position.x = MyConfig.BONUS_PENALTY_POINT[i].x;
          EnemyBulletPoint.pose.position.y = MyConfig.BONUS_PENALTY_POINT[i].y;
          EnemyBulletPoint.pose.position.z = MyConfig.BONUS_PENALTY_POINT[i].z;
          EnemyBulletPoint.pose.orientation.w = MyConfig.BONUS_PENALTY_POINT[i].yaw;
        }
        if(BP_Area_Status[i] == 4)
        {
          EnemyHPPoint.pose.position.x = MyConfig.BONUS_PENALTY_POINT[i].x;
          EnemyHPPoint.pose.position.y = MyConfig.BONUS_PENALTY_POINT[i].y;
          EnemyHPPoint.pose.position.z = MyConfig.BONUS_PENALTY_POINT[i].z;
          EnemyHPPoint.pose.orientation.w = MyConfig.BONUS_PENALTY_POINT[i].yaw;
        }
      }
    }
    //判断比赛是否开始
    bool IsGameStart()
    {
      if(GameStatus == 3 && GameStatus3IsOK.data != 1)
      {
        GameStatus3IsOK.data = 1;
        gamestatus_pub_.publish(GameStatus3IsOK);
      }
      if(GameStatus == 4)
      {
        //ros::spinOnce();
        gametime.RecordGameStartTime();
        ShootBulletNumOffset = ShootBulletNum;
        return true;
      }
      std::cout << "[0] Ready to Start, GameStatus is : " <<GameStatus<<std::endl;
      return false;
    }
      
    bool IsGameOver()
    {	
      //if(GameStatus != 4)	//if(GameStatus >= 5)
      if(GameStatus >= 5)
      {
	      return true;	
      }
      return false;
    }
      
    ~CheckGameStatus() = default; 
  private:
    ros::NodeHandle checkgamestatus_nh_; 
    ros::Publisher  gamestatus_pub_;
    std_msgs::Int16 GameStatus3IsOK;
  };
}


/*---------------------------------- Tools ------------------------------------------*/
/*获得距离和角度*/
double GetDistance(const geometry_msgs::PoseStamped &pose1,
                    const geometry_msgs::PoseStamped &pose2) {
  const geometry_msgs::Point point1 = pose1.pose.position;
  const geometry_msgs::Point point2 = pose2.pose.position;
  const double dx = point1.x - point2.x;
  const double dy = point1.y - point2.y;
  return std::sqrt(dx * dx + dy * dy);
}

double GetAngle(const geometry_msgs::PoseStamped &pose1,
  const geometry_msgs::PoseStamped &pose2) {
  const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
  const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
  tf::Quaternion rot1, rot2;
  tf::quaternionMsgToTF(quaternion1, rot1);
  tf::quaternionMsgToTF(quaternion2, rot2);
  return rot1.angleShortestPath(rot2);
  //返回rot1,rot2的最小角度
}
/*用来判断哪个机器人补弹哪个机器人补血，通过距离和来判断，血量做特殊情况的处理
新的一分钟分情况来解决，1-只有加弹区 2-只有加血区 3-两个加成区都有
判断的方式，如果血量都大于400,按照就近原则加成，如果血量小于400,血量多的补弹。
如果新的一分钟一台机器人识别到并且有弹，另外一台机器人没有识别到或者没弹，
则没有识别到或者没弹的机器人补给
补弹指令不加在双车通信，改为单独判断，双车通信内容*/
void No1RobotDiscuss() 
{
  //Bullet
  if(MyConfig.ROBOT_INFO.number == 1)
  {
    auto distance_of_bullet =  GetDistance(MyMapPose,MyBulletPoint);
    auto partner_distance_of_bullet = GetDistance(PartnerMapPose,MyBulletPoint);
    auto distance_of_hp =  GetDistance(MyMapPose,MyHPPoint);
    auto partner_distance_of_hp = GetDistance(PartnerMapPose,MyHPPoint);

    /*判断是否需要补血，都大于1800不补血，一个大于1800，一个大于1000不补血，其余的情况都补血*/
    if(MyHP > 1800 && PartnerHP > 1800)
    {
      IfNeedAddHP = false;
    }
    else if(MyHP > 1800 && PartnerHP < 1800 && PartnerHP > 1000)
    {
      IfNeedAddHP = false;
    }
    else if(MyHP < 1800 && MyHP > 1000 && PartnerHP > 1800)
    {
      IfNeedAddHP = false;
    }
    else
    {
      IfNeedAddHP = true;
    }

    if(HaveBulletArea && HaveHPArea)//加弹区和加血区都有的情况
    {
      if(distance_of_bullet <= partner_distance_of_bullet)//我相对于加弹区的距离小于队友相对于加弹区的距离
      {
        if(distance_of_hp <= distance_of_bullet)//我相对于加血区的距离小于我相对于加弹区的距离
        {
          AddBulletCmd1 = false;
          AddBulletCmd2 = true;
          AddHPCmd1 = true;
          AddHPCmd2 = false;
        }
        else
        {
          AddBulletCmd1 = true;
          AddBulletCmd2 = false;
          AddHPCmd1 = false;
          AddHPCmd2 = true;
        }
      }
      else//队友相对于加弹区的距离小于我相对于加弹区的距离
      {
        if(partner_distance_of_hp <= partner_distance_of_bullet)//队友相对于加血区的距离小于队友相对于加弹区的距离
        {
          AddBulletCmd1 = true;
          AddBulletCmd2 = false;
          AddHPCmd1 = false;
          AddHPCmd2 = true;
        }
        else
        {
          AddBulletCmd1 = false;
          AddBulletCmd2 = true;
          AddHPCmd1 = true;
          AddHPCmd2 = false;  
        }
      }
      //HP
      if(MyHP <= 400 && PartnerHP <= 400)
      {
        
      }
      else if(MyHP <= 400)
      {
        AddBulletCmd1 = AddHPCmd1 = false;
        AddBulletCmd2 = AddHPCmd2 = true;
      }
      else if(PartnerHP <= 400)
      {
        AddBulletCmd1 = AddHPCmd1 = true;
        AddBulletCmd2 = AddHPCmd2 = false;
      }

      if(MyHP <= 100 && PartnerHP <= 100)
      {

      }
      else if(MyHP <= 100)
      {
        AddBulletCmd1 = AddHPCmd1 = false;
        AddBulletCmd2 = AddHPCmd2 = true;
      }
      else if(PartnerHP <= 100)
      {
        AddBulletCmd1 = AddHPCmd1 = true;
        AddBulletCmd2 = AddHPCmd2 = false;
      }
    }

    else if(HaveBulletArea && !HaveHPArea)//有加弹区，无加血区
    {
      if(distance_of_bullet <= partner_distance_of_bullet)
      {
        AddBulletCmd1 = true;
        AddBulletCmd2 = false;
        AddHPCmd1 = false;
        AddHPCmd2 = false;
      }
      else
      {
        AddBulletCmd1 = false;
        AddBulletCmd2 = true;
        AddHPCmd1 = false;
        AddHPCmd2 = false;
      }
      //HP
      if(MyHP <= 400 && PartnerHP <= 400)
      {
        
      }
      else if(MyHP <= 400)
      {
        AddBulletCmd1 = false;
        AddBulletCmd2 = true;
      }
      else if(PartnerHP <= 400)
      {
        AddBulletCmd1 = true;
        AddBulletCmd2 = false;
      }
    }

    if(MyHP <= 100 && PartnerHP <= 100)
    {

    }
    else if(MyHP <= 100)
    {
      AddBulletCmd1 = false;
      AddBulletCmd2 = true;
    }
    else if(PartnerHP <= 100)
    {
      AddBulletCmd1 = true;
      AddBulletCmd2 = false;
    }
    
    else if(!HaveBulletArea && HaveHPArea)//有加血区，无加弹区
    {
      if(distance_of_hp <= partner_distance_of_hp)
      {
        AddBulletCmd1 = false;
        AddBulletCmd2 = false;
        AddHPCmd1 = true;
        AddHPCmd2 = false;
      }
      else
      {
        AddBulletCmd1 = false;
        AddBulletCmd2 = false;
        AddHPCmd1 = false;
        AddHPCmd2 = true;
      }
      //HP
      if(MyHP <= 400 && PartnerHP <= 400)
      {
        
      }
      else if(MyHP <= 400)
      {
        AddHPCmd1 = false;
        AddHPCmd2 = true;
      }
      else if(PartnerHP <= 400)
      {
        AddHPCmd1 = true;
        AddHPCmd2 = false;
      }

      if(MyHP <= 100 && PartnerHP <= 100)
      {

      }
      else if(MyHP <= 100)
      {
        AddHPCmd1 = false;
        AddHPCmd2 = true;
      }
      else if(PartnerHP <= 100)
      {
        AddHPCmd1 = true;
        AddHPCmd2 = false;
      }
    }

    //my cmd
    if(HaveBulletArea)
    {
      if(AddBulletCmd1)
      {
        AddBulletChance = true;
      }
      else
      {
        AddBulletChance = false;
      }
    }
    
    if(HaveHPArea)
    {
      if(AddHPCmd1)
      {
        AddHPChance = true;
      }
      else
      {
        AddHPChance = false;
      }
    }
    //No2 cmd
    No2SendCmd = 0;
    if(AddBulletCmd2)
      No2SendCmd += 10;	  
    if(AddHPCmd2)
      No2SendCmd += 1;
  }
}

void No2RobotReceive() 
{
  if(MyConfig.ROBOT_INFO.number == 2 && (No2ReceiveFlag || (PartnerHP <= 100 && IsNewMin)))
  {  
    No2ReceiveFlag = false;
    
    switch(No2ReceiveCmd)
    {
      case 0:
      AddBulletCmd2 = AddHPCmd2 = false;
      break;

      case 1:
      AddBulletCmd2 = false;
      AddHPCmd2 = true;
      break;

      case 10:
      AddBulletCmd2 = true;
      AddHPCmd2 = false;
      break;

      case 11:
      AddBulletCmd2 = true;
      AddHPCmd2 = true;
      break;	

      default:
      ROS_ERROR("No2ReceiveCmd is Wrong!!!!!!!!!!");
      break;
    }
    
    //HP 
    if(PartnerHP <= 100)
    {
      ROS_ERROR("Because : No.1 no HP!!!!!!!!!!!!");
      AddBulletCmd2 = AddHPCmd2 = true;	    
    }
  
      ROS_ERROR("AddBulletCmd2: %d, AddHPCmd2: %d", AddBulletCmd2, AddHPCmd2);
    //receive cmd
    if(AddBulletCmd2)
      AddBulletChance = true;
    else
      AddBulletChance = false;
	  
    if(AddHPCmd2)
      AddHPChance = true;
    else
      AddHPChance = false;
  }
}

#endif
