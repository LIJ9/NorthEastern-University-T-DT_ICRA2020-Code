//#define USE_REFREE_SYSTEM

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
#include "math.h"

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
  roborts_decision::AddHPBehavior        	addhp_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::ShootBehavior         	shoot_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::SupportBehavior       	support_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::CounterBehavior       	counter_behavior(chassis_executor, blackboard, full_path);  
  roborts_decision::MyEscapeBehavior      	myescape_behavior(chassis_executor, blackboard, full_path); 
  roborts_decision::MyChaseBehavior        	mychase_behavior(chassis_executor, blackboard, full_path); 
  roborts_decision::LidarDetectedBehavior  	lidardetected_behavior(chassis_executor, blackboard, full_path); 
  roborts_decision::MyPatrolBehavior       	mypatrol_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::AttackBehavior	        attack_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::RobotCommunicate       	communicate(blackboard);
  roborts_decision::RefereeSystem               referee_system;
  roborts_decision::CheckGameStatus 	   	checkgamestatus;
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
        if(Enemy_In_Distance)
        {
          chassis_executor->Cancel();
        }
        else
        {
          chassis_executor->Execute(LastEnemyPose);
        }
      }
      else if(IsStartCounter)	
      {
         counter_behavior.Run();//反击行为启动
      }
      //IsAddingHP
      if(IsAddingHP)//需要添加限制条件
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
      else if(IsAddingBullet)
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

      //addbullet_behavior
      else if(AddBulletChance && !(HaveBullet && IsVisionDetected) && !IsAddingBullet)//补弹条件:有补弹区 &没有子弹 没有敌人 &没有在补弹
      {
        addbullet_behavior.Run();
      }

      //addhp_behavior
      else if(AddHPChance && IfNeedAddHP && !IsVisionDetected && !IsAddingHP)
      {
        addhp_behavior.Run();
      }
      
      
      
      else if(MyTeamAdvantage)//如果有弹，则可以进攻
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
            chassis_executor->Execute(LastEnemyPose);
          }
        }
        else if(IsStartCounter)
        {
          counter_behavior.Run();
        }
        else if(IsVisionFirstLost)	
        {
          mychase_behavior.Run();
        }
        else if(SentryDirect)
        {
          attack_behavior.Run();
        }
        else if(IsReceiveEnemyPose)
        {
          support_behavior.Run();
        }
         else if(IsLidarDetected)
        {
          lidardetected_behavior.Run();
        } 
        else
        {
          mypatrol_behavior.Run();
        }
      }
      //NoBullet，如果受到攻击
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
    DistanceFromCamera(blackboard);//判断敌人是否在攻击范围内
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
            chassis_executor->Execute(LastEnemyPose);
          }
        }
        else
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
// 	   DistanceFromCamera(blackboard);
// 	 if(IsVisionDetected)
//         {
//           shoot_behavior.Run();
//           if(Enemy_In_Distance)
//           {
//             chassis_executor->Cancel();
//           }
//           else
//           {
//             chassis_executor->Execute(LastEnemyPose);
//           }
//         }
//         else
//         {
//           mychase_behavior.Run();
//         }
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
	  //ROS_ERROR("test289");
          addbullet_behavior.Run();
        }
        else if(follow_addbullet)
	{
	  attack_behavior.Run();
	}
	else if(need_support)
	{
 	    support_behavior.Run();
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
            IsStartSwing = true;//此处的标志位置位没有更新
          }
          else
          {
            chassis_executor->Execute(LastEnemyPose);
          }
        }
        else
        {
          attack_behavior.Run();
	      }
        break;
	   case '9':
        if(IsAddingHP)
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
        else if(1/*AddBulletChance*/)
        {
          addhp_behavior.Run();
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
   ////8.10  ////我的状态
  double t1 = log(MyHP/1000+1);
  double t2 = (MyHP/1000)*std::sqrt(LeftBulletNum/200);
  my_condition = (t1+t2)/3;
 if(my_condition>0.5){
   shoot_con = true;
}
else{
  shoot_con = false;
}

  ////队友状态
  double p1 = log(PartnerHP/1000+1);
  double p2 = (PartnerHP/1000)*std::sqrt(PartnerLeftBulletNum/200);
  partner_condition = (p1+p2)/3;

//   //我队状态
   double q0 = PartnerHP + MyHP;
   double q1= log(q0/1000+1);
   double q2 = (q0/1000)*std::sqrt(TotalBulletNum/200);
   //myTeam_condition = (q1+q2)/3;
  myTeam_condition = 0.85;
  ////距离
   //virtual_dist = 16/((blackboard->EnemyDistance()+4)*(blackboard->EnemyDistance()+4));  //需要vision detected 

  
  ////敌1 状态
  double e1 = log(EnemyOneHP/1000+1);
  double e2 = (EnemyOneHP/1000)*std::sqrt(EnemyOneLeftBulletNum/200);
  EnemyOne_condition = (e1+e2)/3;
  
  ////敌2状态
  double e_1 = log(EnemyTwoHP/1000+1);
  double e_2 = (EnemyTwoHP/1000)*std::sqrt(EnemyTwoLeftBulletNum/200);
  EnemyTwo_condition = (e_1+e_2)/3;
//   //enemy_team 状态
   double a0 = EnemyOneHP + EnemyTwoHP;
   double a1 = EnemyOneLeftBulletNum+EnemyOneLeftBulletNum;
   double a2= (a0/1000+1);
   double a3 = (a0/1000)*std::sqrt(a1/200);
   //double EnemyTeam_condition = (a2+a3)/3;
  EnemyTeam_condition = 0.88; 
  ////安全增益///哨岗决策
  double para_safety = 8;
  double dx = MyMapPose.pose.position.x - PartnerMapPose.pose.position.x;
  double dy = MyMapPose.pose.position.y - PartnerMapPose.pose.position.y;
  partner_dist =  std::sqrt(dx * dx + dy * dy);
  safety_value = para_safety*partner_condition*virtual_dist;
  


  if(myTeam_condition>=EnemyTeam_condition)
  {
    MyTeamAdvantage = true;
  }

////8.11 MYHP=1100,PARTNERHP= 1100,mybullet = 50,partnerbullet = 30,H3=1200 H4=1100,B3=30,B4=20  我队劣势 但是没在怕的 行为还是追击
  if(EnemyTeam_condition>myTeam_condition)
  {
    if(30*TotalBulletNum>(EnemyOneHP + EnemyTwoHP) && MyHP>40*(EnemyOneLeftBulletNum+EnemyTwoLeftBulletNum))
    {
      if (EnemyOne_condition < EnemyTwo_condition)
      {
	current_target = 1;
      }
      else 
	current_target = 2;
    }
    else 
     MyTeamAdvantage = false;
  }

  std::cout << " " <<std::endl;
  gametime.TimeUpdate();
  MyChassisState = chassis_executor->Update();
  MyMapPose = blackboard->GetRobotMapPose();
  if(EnemyPose.pose.position.x > 0 || EnemyPose.pose.position.y > 0)
  {
    LastEnemyPose.header.frame_id = "map";
    LastEnemyPose.pose.position.x = EnemyPose.pose.position.x;
    LastEnemyPose.pose.position.y = EnemyPose.pose.position.y;
    LastEnemyPose.pose.position.z = 0;
    auto derta_y = LastEnemyPose.pose.position.y-MyMapPose.pose.position.y;
    auto derta_x = LastEnemyPose.pose.position.x-MyMapPose.pose.position.x;
    auto enemy_yaw_to_me = atan2(derta_y,derta_x);
    auto enemy_area_quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, enemy_yaw_to_me);
    LastEnemyPose.pose.orientation = enemy_area_quaternion;
  }
  
  EnemyPose = blackboard->GetEnemy();//获取敌方的位姿，丢失的时候用于追击，视觉没有识别到时LastEnemyPose没有数据
  MyYaw = tf::getYaw(MyMapPose.pose.orientation);//获得当前机器人的yaw角
  IsVisionDetected =true; /*blackboard->IsEnemyDetected();*/
  //判断视觉第一次丢失当前帧没有识别到，上一帧识别到
  if((!IsVisionDetected && LastIsVisionDetected) || IsChasingEnemy)
    IsVisionFirstLost = true;
  else
    IsVisionFirstLost = false;
  //if(!IsChasingEnemy)
  //{
  LastIsVisionDetected = IsVisionDetected;//一帧之后被覆盖掉
  //}
  //雷达识别的判据，四秒判断一次，如果四秒之后没有识别到，则四秒之后的每一次循环都判断
  IsLidarDetected = blackboard->IsLidarEnemyDetected();
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
  
  //HaveBullet
  if(LeftBulletNum > 10)  //The actual test is 22
    HaveBullet = true;
  else
    HaveBullet = false;
  
  if(PartnerLeftBulletNum > 0)
    PartnerHaveBullet = true;
  else
    PartnerHaveBullet = false;
  //IsStartCounter 552
  
  //if(IsUnderAttack && UnderAttackArmorId != 0 &&    !IsVisionDetected && !IsStartSwing)  //if(IsUnderAttack && UnderAttackArmorId != 0 &&    !IsVisionDetected && !IsStartSwing)
  if(IsUnderAttack && UnderAttackArmorId != 0 && !IsVisionDetected)  //if(HaveBullet && IsUnderAttack && UnderAttackArmorId != 0)  
    IsStartCounter = true;//如果受到伤害，并且不是最前方的装甲受到伤害，并且没有识别到，开始反击行为
  else
    IsStartCounter = false;

  //Swing delayt
  static int swingcnt = 0;  
  if(IsUnderAttack && !IsStartCounter)
  {
    IsStartSwing = true;//前装甲受到伤害就开始摇摆
    swingcnt = 0;
  }

  if(IsAddingBullet)
    IsStartSwing = true;
  if(IsAddingHP)
    IsStartSwing = true;

  if(IsStartSwing && !IsAddingBullet && !IsAddingHP)
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
  
  //ROS_INFO("[3]. LeftBulletNum is : %d, and TotalBulletNum is %d, and ShootBulletNum is %d",(int)LeftBulletNum, TotalBulletNum, ShootBulletNum-ShootBulletNumOffset);
  //ROS_INFO("[4]. My HP is : %d, PartnerHP is : %d",(int)MyHP,(int)PartnerHP);
  ROS_INFO("[3]. IsUnderAttack is  : %d, UnderAttackArmorId is : %d, IsStartSwing is : %d",(int)IsUnderAttack,(int)UnderAttackArmorId,(int)IsStartSwing);
  //ROS_INFO("[6]. X is : %f, Y is : %f", MyMapPose.pose.position.x, MyMapPose.pose.position.y);
  //ROS_INFO("[7]. adjustfinishflag is : %d ,IsAdvantage is %d  ,IsAddingHP is %d ,IsAddingBullet is %d ",(int)adjustfinishflag,(int)IsAdvantage,(int)IsAddingHP,(int)IsAddingBullet);
  //ROS_INFO("[8]. AddBulletCmd1: %d, AddHPCmd1: %d, \tAddBulletCmd2: %d, AddHPCmd2: %d",(int)AddBulletCmd1,(int)AddHPCmd1,(int)AddBulletCmd2,(int)AddHPCmd2);
  //ROS_INFO("[9]. IsAddBulletFinish is : %d ,IsAddHPFinish is %d ,SentryDirect is %d ",(int)IsAddBulletFinish,(int)IsAddHPFinish,(int)SentryDirect);
 // ROS_INFO("[10].IfNeedAddHP is %d ,AddHPChance is %d ,AddBulletChance is %d",(int)IfNeedAddHP,(int)AddHPChance,(int)AddBulletChance);
  //ROS_INFO("[11].IsVisionDetected is %d ,Enemy_Distance is %f IsVisionFirstLost is %d ",(int)IsVisionDetected,Enemy_Distance,(int)IsVisionFirstLost);
//    int behitcnt = 0;
//   if(MyMapPose.pose.position.x>3.9&&MyMapPose.pose.position.y<2.0){
//     behitcnt++;
//     MyHP -=5*behitcnt;
//     LeftBulletNum-=2*behitcnt;;
  


   ROS_INFO("[4].my_condition is %.1lf ",my_condition);
   //ROS_INFO("[13].virtual_dist is %.1lf ",virtual_dist);
   ROS_INFO("[5].safety_value is %.1lf ",safety_value);
  // ROS_INFO("[15]. partner X is : %f, partner Y is : %f", PartnerMapPose.pose.position.x, PartnerMapPose.pose.position.y);
}

void ResetAllFlag(roborts_decision::Blackboard* &blackboard)
{
  IsEnableVision = false;
  IsCounting = false; 
  IsLidaring = false;
  IsAngleAchieved =false;
}

//功能函数，判断敌人到自己的直线距离是否小于2米
void DistanceFromCamera(roborts_decision::Blackboard* &blackboard)
{
  Enemy_Distance = blackboard->EnemyDistance();//识别不到的时候Enemy_Distance = 0
  if(Enemy_Distance > 0)
  {
    if(Enemy_Distance < 1.0)
    {
      Enemy_In_Distance = true;
    }
    else
    {
      Enemy_In_Distance = false;
    }
  }
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
	    << "9: 加血" << std::endl
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
      AddBulletChance =true;
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