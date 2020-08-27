#define USE_REFREE_SYSTEM

#include <ros/ros.h>
#include "executor/chassis_executor.h"
#include "example_behavior/goal_behavior.h"
#include "my_behavior/addhp_behavior.h"
#include "my_behavior/shoot_behavior.h"
#include "my_behavior/addbullet_behavior.h"
#include "my_behavior/support_behavior.h"
#include "my_behavior/counter_behavior.h"
#include "my_behavior/myescape_behavior.h"
#include "my_behavior/mychase_behavior.h"
#include "my_behavior/lidardetected_behavior.h"
#include "my_behavior/mypatrol_behavior.h"
#include "my_behavior/attack_behavior.h"
#include "robot_information.h"
#include "robot_communicate.h"
#include "referee_system.h"
#include "kill_pending.h"

void RoboInfoUpdate(roborts_decision::Blackboard* &blackboard, roborts_decision::ChassisExecutor* &chassis_executor);
void ResetAllFlag(roborts_decision::Blackboard* &blackboard);
void SelectDecisionMode(); 
int decisionmode = 1;//  decisionmode = 1;
void Command(); 
void DistanceFromCamera(roborts_decision::Blackboard* &blackboard);
char command = ' ';
int main(int argc, char **argv) 
{
  ros::init(argc, argv, "behavior_test_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";
  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);
  roborts_decision::GoalBehavior           	goal_behavior(chassis_executor, blackboard); 
  roborts_decision::AddBulletBehavior      	addbullet_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::AddHPBehavior        		addhp_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::ShootBehavior         	shoot_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::SupportBehavior       	support_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::CounterBehavior       	counter_behavior(chassis_executor, blackboard, full_path);  
  roborts_decision::MyEscapeBehavior      	myescape_behavior(chassis_executor, blackboard, full_path); 
  roborts_decision::MyChaseBehavior        	mychase_behavior(chassis_executor, blackboard, full_path); 
  roborts_decision::LidarDetectedBehavior  	lidardetected_behavior(chassis_executor, blackboard, full_path); 
  roborts_decision::MyPatrolBehavior       	mypatrol_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::AttackBehavior			    attack_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::RobotCommunicate       	communicate(blackboard);
  roborts_decision::RefereeSystem           referee_system;
  roborts_decision::CheckGameStatus 	   		checkgamestatus;
  ros::Rate rate(10);
#ifdef USE_REFREE_SYSTEM
  while(!checkgamestatus.IsGameStart())
  {
    ros::spinOnce();
    rate.sleep();
  }
  while(ros::ok()) 
#else
  SelectDecisionMode();  
  //选择比赛模式
  while(decisionmode == 1 && ros::ok())
#endif
  {
    ros::spinOnce();
    RoboInfoUpdate(blackboard,chassis_executor);//初始化比赛数据信息
    
#ifdef USE_REFREE_SYSTEM
    if(checkgamestatus.IsGameOver())
    {
      chassis_executor->Cancel();
      IsStartSwing = false;
      communicate.SendMsg();
      return 0;
    }
#endif
      if(IsVisionDetected) 
      {       
	      shoot_behavior.Run();//识别到之后开始shoot
      }
      else if(IsStartCounter)	
      {
	      counter_behavior.Run();//反击行为启动
      }
      if(IsAddingBullet)
      {
        ROS_INFO("[2]. ......Adding Bullet......");
        if(IsVisionDetected) 
        {       
          shoot_behavior.Run();
        }
        else if(IsStartCounter)
        {
          counter_behavior.Run();
        }
      }
      //IsAddingBuff
      else if(IsAddingBuff)
      {
        ROS_INFO("[2]. ......Adding HP......");
        if(IsVisionDetected) 
        {
          shoot_behavior.Run();
        }
        else if(IsStartCounter)
        {
          counter_behavior.Run();
        }
      }
      //addbullet_behavior
      else if(AddBulletChance && !(HaveBullet && IsVisionDetected)) //!(HaveBullet && IsVisionDetected) && !(HaveBullet && IsStartCounter)
      {
        addbullet_behavior.Run();
      }
    
      //addhp_behavior
      else if(AddHPChance && !HaveBuff && !(HaveBullet && IsVisionDetected))       
      {
        addhp_behavior.Run();
      }
      //有加成的机会并且（没有buff,没有识别到)
      //else if(HaveBullet && IsAdvantage)
      else if(HaveBullet)//如果有弹，则可以进攻
      {
        if(IsVisionDetected) 
        {
          shoot_behavior.Run();
          if(Enemy_In_Distance)
          {
            chassis_executor->Cancel();
          }
          else
          {
            chassis_executor->Execute(blackboard->GetEnemy());
          }
        }
        else if(IsStartCounter)	
        {
          counter_behavior.Run();
        }
        
//         else if(IsVisionFirstLost)	
//         {
//           mychase_behavior.Run();
//         }
        else if(IsReceiveEnemyPose)
        {
          support_behavior.Run();
        }
//         else if(IsLidarDetected)
//         {
//           lidardetected_behavior.Run();
//         }
        else
        {
          mypatrol_behavior.Run();
        }
      }
      //NoBullet，如果受到攻击并且
      else
      {
        if(IsUnderAttack && !IsChangeGoals)
        {
          IsChangeGoals = true;
          ChangedGoals = !ChangedGoals;
        }
        myescape_behavior.Run();     
      }
      
    communicate.SendMsg();
    checkgamestatus.CheckAreaStatus();
    ResetAllFlag(blackboard);
    rate.sleep();
  }
//测试模式
  auto command_thread= std::thread(Command);//命令线程
  while(decisionmode == 2 && ros::ok())
  {
    ros::spinOnce();
    RoboInfoUpdate(blackboard,chassis_executor);
    checkgamestatus.CheckAreaStatus();
    communicate.SendMsg();
    switch (command) 
    {
      case '1':
        if(IsVisionDetected) 
        {
          shoot_behavior.Run();
        }
        else if(IsVisionFirstLost && !IsStartCounter)	
        {
          mychase_behavior.Run();
        }
        break;
	
      case '2':
        if(IsLidarDetected)
        {
          lidardetected_behavior.Run();//雷达识别
        }
        break;
      case '3':
        if(IsReceiveEnemyPose)
        {
          support_behavior.Run();//支援
        }       
        break;
        
      case '4':
        mypatrol_behavior.Run();
        break;
	
      case '5':
        if(IsVisionDetected) 
        {       
          shoot_behavior.Run();
        }
        else if(IsStartCounter)	
        {
          counter_behavior.Run();
        }
        else
          std::cout << "Nothing~~~~" << std::endl;
        break;	

      case '6':
        if(IsUnderAttack && !IsChangeGoals)
        {
          IsChangeGoals = true;
          ChangedGoals = !ChangedGoals;
        }
        
        myescape_behavior.Run();//逃跑
        break;	
	
      case '7':
        if(IsAddingBullet)
        {
          ROS_INFO("[2]. ......Adding Bullet......");
          if(IsVisionDetected) 
          {       
            shoot_behavior.Run();
          }
          else if(IsStartCounter)
          {
            counter_behavior.Run();
          }
        }
        else if(AddBulletChance)
        {
          addbullet_behavior.Run();
        }   
        break;
      case '8':
        DistanceFromCamera(blackboard);
	      if(IsVisionDetected)
        {
          shoot_behavior.Run();
          if(Enemy_In_Distance)
          {
            chassis_executor->Cancel();
          }
          else
          {
            chassis_executor->Execute(blackboard->GetEnemy());
          }
        }
        else
        {
          attack_behavior.Run();
	      }
        break;
      case '0':
        goal_behavior.Run();
        break;
      case 27:
        if (command_thread.joinable())
          command_thread.join();
        return 0;
	
      default:
        break;
    }
    
    ResetAllFlag(blackboard);
    rate.sleep();
  }
  return 0;
}

void RoboInfoUpdate(roborts_decision::Blackboard* &blackboard, roborts_decision::ChassisExecutor* &chassis_executor)
{
  std::cout << " " <<std::endl;
  gametime.TimeUpdate();
  MyChassisState = chassis_executor->Update();
  MyMapPose = blackboard->GetRobotMapPose();
  MyYaw = tf::getYaw(MyMapPose.pose.orientation);//获得当前机器人的yaw角
  IsVisionDetected = blackboard->IsEnemyDetected();
  //判断视觉第一次丢失
  if(!IsVisionDetected && LastIsVisionDetected)
    IsVisionFirstLost = true;
  else
    IsVisionFirstLost = false;
  LastIsVisionDetected = IsVisionDetected;
  //雷达识别的判据，四秒判断一次，如果四秒之后没有识别到，则四秒之后的每一次循环都判断
  static int LidarCD = 0;
  LidarCD++;
  if(LidarCD >= 40)	//4s
  {
    LidarCD = 40;
    IsLidarDetected = blackboard->IsLidarEnemyDetected();
    if(IsLidarDetected)
      LidarCD = 0;
  }
  //判断子弹数量是否变化，如果这一次的子弹数量和上一次的不同，就认为子弹数量变化了\
  子弹发射的数量和剩余的数量改为下位机直接发
  if(ShootBulletNum != LastShootBulletNum)	//if BulletNum is changed
    IsShoot = true;
  else
    IsShoot = false;
  LastShootBulletNum = ShootBulletNum;
  
  /* if(IsShoot)
  {
    LeftBulletNum = TotalBulletNum - ShootBulletNum + ShootBulletNumOffset;
    if(LeftBulletNum < 0)
    {
      LeftBulletNum = 0;
      TotalBulletNum = ShootBulletNum - ShootBulletNumOffset;
    }
  } 
  static int zerocnt = 0;
  if(!IsShoot && FireCmd) //if(!IsShoot && IsVisionDetected)
  {
    zerocnt++;
    if(zerocnt >= 50) //over 5s
    {
      zerocnt = 0;
      LeftBulletNum = 0;
      TotalBulletNum = ShootBulletNum - ShootBulletNumOffset;
      ROS_ERROR("I can't shoot! Bullet Reset!!~~~~~~~~~~~~");
    }
  }

  if(IsShoot && FireCmd)
    zerocnt = 0;
  */
  //HaveBullet
  if(LeftBulletNum > 0)  //The actual test is 22
    HaveBullet = true;
  else
    HaveBullet = false;
  
  if(PartnerLeftBulletNum > 0)
    PartnerHaveBullet = true;
  else
    PartnerHaveBullet = false;
  //IsStartCounter 
  //if(IsUnderAttack && UnderAttackArmorId != 0 &&    !IsVisionDetected && !IsStartSwing)  //if(IsUnderAttack && UnderAttackArmorId != 0 &&    !IsVisionDetected && !IsStartSwing)
  if(IsUnderAttack && UnderAttackArmorId != 0 && !IsVisionDetected)  //if(HaveBullet && IsUnderAttack && UnderAttackArmorId != 0)  
    IsStartCounter = true;//如果受到伤害，并且不是最前方的装甲受到伤害，并且没有识别到，开始反击行为
  else
    IsStartCounter = false;
    
  //Swing delayt
  static int swingcnt = 0;  
//   if( (HaveBullet || IsStucked) && IsUnderAttack && !IsStartCounter)
//   if( (HaveBullet || IsStucked || IsAddingBullet || IsAddingBuff) && IsUnderAttack && !IsStartCounter)
  if(IsUnderAttack && !IsStartCounter )
  {
    IsStartSwing = true;//受到伤害并且不符合反击条件的开始摇摆
    swingcnt = 0;
  }
  if(IsStartSwing)
  {
    swingcnt++;
    if(swingcnt >= 20)
    {
      IsStartSwing = false;//进入摇摆之后摇摆2s，如果还符合条件，就继续摇摆
    }
  }
  else
  {
    swingcnt = 0;
  }

  No1RobotDiscuss();//判断哪个机器人进行补弹，哪个机器人进行补血
  No2RobotReceive();//一号机器人判断，二号机器人接受指令
  
  ROS_INFO("[3]. LeftBulletNum is : %d, and TotalBulletNum is %d, and ShootBulletNum is %d",(int)LeftBulletNum, TotalBulletNum, ShootBulletNum-ShootBulletNumOffset);
  ROS_INFO("[4]. My HP is : %d, PartnerHP is : %d",(int)MyHP,(int)PartnerHP);
  ROS_INFO("[5]. IsUnderAttack is  : %d, UnderAttackArmorId is : %d, IsStartSwing is : %d",(int)IsUnderAttack,(int)UnderAttackArmorId,(int)IsStartSwing);
  ROS_INFO("[6]. X is : %f, Y is : %f", MyMapPose.pose.position.x, MyMapPose.pose.position.y);
  ROS_INFO("[7]. adjustfinishflag is : %d ,IsAdvantage is %d ",(int)adjustfinishflag,(int)IsAdvantage);
  ROS_INFO("[8]. HaveBuff is : %d, BuffStatus is %d, BulletStationStatus is %d", (int)HaveBuff, BuffStatus,BulletStationStatus);
  ROS_INFO("[9]. AddBulletCmd1: %d, AddHPCmd1: %d, \tAddBulletCmd2: %d, AddHPCmd2: %d",AddBulletCmd1,AddHPCmd1, AddBulletCmd2,AddHPCmd2);
  /*static int stuckedcnt = 0;
  if( (int)MyMapPose.pose.position.x == (int)LastMyMapPose.pose.position.x && (int)MyMapPose.pose.position.y == (int)LastMyMapPose.pose.position.y && !IsVisionDetected)
  {
    stuckedcnt++;
    if(stuckedcnt >= 100) //10s
    {
      IsStucked = true;
      ROS_INFO("~~~~~~~~~~~~~~~I'm Stucked~~~~~~~~~~~~~~~");
    }
  }
  else
  {
    stuckedcnt = 0;
    IsStucked = false;
  }*/
}
void ResetAllFlag(roborts_decision::Blackboard* &blackboard)
{
  IsEnableVision = false;
  IsCounting = false; 
  IsLidaring = false;
  IsAngleAchieved =false;
}
//功能函数，判断敌人到自己的直线距离
void DistanceFromCamera(roborts_decision::Blackboard* &blackboard)
{
  Enemy_Distance = blackboard->EnemyDistance();
  if(Enemy_Distance < 2)
    Enemy_In_Distance = true;
  else
    Enemy_In_Distance = false;
}

void CommandInterface()
{
  std::cout << "**************************************************************************************" << std::endl;
  std::cout << "*********************************请选择要测试的行为********************************" << std::endl;
  std::cout << "1: 视觉追击" << std::endl
	    << "2: 雷达识别" << std::endl
	    << "3: 支援" << std::endl
	    << "4: 巡逻" << std::endl
	    << "5: 识别+反击" << std::endl
	    << "6: 逃跑" << std::endl
	    << "7: 补弹" << std::endl
	    << "8: 哨岗指挥追击" << std::endl
	    << "0: 手动输入" << std::endl
	    << "esc: 退出程序" << std::endl;
  std::cout << "**************************************************************************************" << std::endl;
  std::cout << "> ";
  std::cin >> command;
  
  switch(command)
  {
    case '1':
      gametime.RecordGameStartTime();
      break;
      
    case '2':
      gametime.RecordGameStartTime();
      break;

    case '3':
      gametime.RecordGameStartTime();
      break;

    case '4':
      gametime.RecordGameStartTime();
      break;
      
    case '5':
      gametime.RecordGameStartTime();
      break;	

    case '6':
      gametime.RecordGameStartTime();
      break;	
 
    case '7':
      AddHPChance = 2;
      gametime.RecordGameStartTime();
      break;	
      
    case '0':
      break;
      
    case 27:
      break;
      
    default:
      std::cout << "错误!请重新输入！" << std::endl;
      break;	
  }
}

void Command() 
{
  while (command != 27) 
  {
    CommandInterface();
  }
}

void SelectDecisionMode()
{
  while(true)
  {
    std::cout << "请选择决策模式：" << std::endl;
    std::cout << "1. 比赛模式" << std::endl;
    std::cout << "2. 测试模式" << std::endl;
    std::cout << "> ";
    std::cin >> decisionmode;
    
    ros::spinOnce();
    
    switch(decisionmode)
    {
      case 1:
        gametime.RecordGameStartTime();//开始计时
        //ShootBulletNum = 100;
        ShootBulletNumOffset = ShootBulletNum;
        std::cout <<"ShootBulletNumOffset is : "<<ShootBulletNumOffset<< std::endl;
        return;
	
      case 2:
        CommandInterface();//进入命令行接口
        gametime.RecordGameStartTime();
        return;
	
      default:
        std::cout << "错误！请重新输入！" << std::endl;
        break;
    }
  }
}