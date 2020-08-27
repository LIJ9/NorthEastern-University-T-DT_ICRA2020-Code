#include<ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <iostream>
#include <std_msgs/Float64MultiArray.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <termio.h>
#include <stdio.h> 
#include <thread>
#include <unistd.h>
using namespace std;
#define ADDBULLET_TYPE  1
#define ADDHP_TYPE      2

int Bullet_Num_left[4] = {50 ,0,50,0};
int Bullet_Num_Shot[4] = {0};
int HP[4] = {2000,300,2000,2000}; 
int ARMOR_ID[4] = {5,5,5,5}; 
int GAMESTATUS = 4;//默认开始
int second = 0;
bool IsStartGame = false;

int scanKeyboard()
{
  int in;
  struct termios new_settings;
  struct termios stored_settings;
  tcgetattr(0,&stored_settings);
  new_settings = stored_settings;
  new_settings.c_lflag &= (~ICANON);
  new_settings.c_cc[VTIME] = 0;
  tcgetattr(0,&stored_settings);
  new_settings.c_cc[VMIN] = 1;
  tcsetattr(0,TCSANOW,&new_settings);
  in = getchar();
  tcsetattr(0,TCSANOW,&stored_settings);
  return in;
}

void SetValueFromKeyboard()
{
  int GameMode;
  int RobotID;
  int ValueType;
  while(1)
  {
    if(GAMESTATUS == 0)//如果比赛没有开始
    {
      GameMode = scanKeyboard();
      if(GameMode == 98)//'b'
      {
        GAMESTATUS = 4;
        if(second != 0)
        {
          second = 0;
        }
        IsStartGame = true;
      }
    }
    else//如果比赛开始了
    {
      RobotID = scanKeyboard();//选择修改几号机器人
      while(RobotID == 49)//如果是1号机器人
      {
        ValueType = scanKeyboard();//选择修改的是哪一个参数
        switch (ValueType)
        {
          case 111://'o'
            GAMESTATUS = 0;
            break;
          case 48://'0'
            Bullet_Num_left[0] += 100;
            break;
          case 49://'1'
            Bullet_Num_left[0] -= 2;
            break;
          case 50://'2'
            HP[0] += 200;
            break;
          case 51://'3'
            HP[0] -= 40;
            break;
          case 113://'q'
            ARMOR_ID[0] = 0;
            break;
          case 119://'w'
            ARMOR_ID[0] = 1;
            break;
          case 101://'e'
            ARMOR_ID[0] = 2;
            break;
          case 114://'r'
            ARMOR_ID[0] = 3;
            break;
          case 116://'t'
            ARMOR_ID[0] = 5;
            break;
          default:
          break;
        }
        if(ValueType == 57)//'9'返回上一级选择对象
        {
          break;
        }
        sleep(0.1);
      }
      while(RobotID == 50)//如果是2号机器人
      {
        ValueType = scanKeyboard();//选择修改的是哪一个参数
        switch (ValueType)
        {
          case 111://'o'
            GAMESTATUS = 0;
            break;
          case 48://'0'
            Bullet_Num_left[1] += 100;
            break;
          case 49://'1'
            Bullet_Num_left[1] -= 2;
            break;
          case 50://'2'
            HP[1] += 200;
            break;
          case 51://'3'
            HP[1] -= 40;
            break;
          case 113://'q'
            ARMOR_ID[1] = 0;
            break;
          case 119://'w'
            ARMOR_ID[1] = 1;
            break;
          case 101://'e'
            ARMOR_ID[1] = 2;
            break;
          case 114://'r'
            ARMOR_ID[1] = 3;
            break;
          case 116://'t'
            ARMOR_ID[1] = 5;
            break;
          default:
          break; 
        }
        if(ValueType == 57)//'9'返回上一级选择对象
        {
          break;
        }
        sleep(0.1);
      }
      while(RobotID == 51)//如果是3号机器人
      {
        ValueType = scanKeyboard();//选择修改的是哪一个参数
        switch (ValueType)
        {
          case 111://'o'
            GAMESTATUS = 0;
            break;
          case 48://'0'
            Bullet_Num_left[2] += 100;
            break;
          case 49://'1'
            Bullet_Num_left[2] -= 2;
            break;
          case 50://'2'
            HP[2] += 200;
            break;
          case 51://'3'
            HP[2] -= 40;
            break;
          case 113://'q'
            ARMOR_ID[2] = 0;
            break;
          case 119://'w'
            ARMOR_ID[2] = 1;
            break;
          case 101://'e'
            ARMOR_ID[2] = 2;
            break;
          case 114://'r'
            ARMOR_ID[2] = 3;
            break;
          case 116://'t'
            ARMOR_ID[2] = 5;
            break;
          default:
          break; 
        }
        if(ValueType == 57)//'9'返回上一级选择对象
        {
          break;
        }
        sleep(0.1);
      }
      while(RobotID == 52)//如果是4号机器人
      {
        ValueType = scanKeyboard();//选择修改的是哪一个参数
        switch (ValueType)
        {
          case 111://'o'
            GAMESTATUS = 0;
            break;
          case 48://'0'
            Bullet_Num_left[3] += 100;
            break;
          case 49://'1'
            Bullet_Num_left[3] -= 2;
            break;
          case 50://'2'
            HP[3] += 200;
            break;
          case 51://'3'
            HP[3] -= 40;
            break;
          case 113://'q'
            ARMOR_ID[3] = 0;
            break;
          case 119://'w'
            ARMOR_ID[3] = 1;
            break;
          case 101://'e'
            ARMOR_ID[3] = 2;
            break;
          case 114://'r'
            ARMOR_ID[3] = 3;
            break;
          case 116://'t'
            ARMOR_ID[3] = 5;
            break;
          default:
          break; 
        }
        if(ValueType == 57)//'9'返回上一级选择对象
        {
          break;
        }
        sleep(0.1);
      }
      if(RobotID == 111)//'o'
        GAMESTATUS = 0;
    }
    sleep(0.1);
  }
}
class DebugSys{
  public:
    DebugSys()
    {
      randperm(6);
      ros::Rate info_loop_rate(10);
      roborts_info_pub = debug_nh.advertise<std_msgs::Float64MultiArray>("debug_to_robots", 1);
      chatter_pub = debug_nh.advertise<std_msgs::Float64MultiArray>("area_status_msg", 1);
      robot_pose_pub = debug_nh.advertise<std_msgs::Float64MultiArray>("sentry_info", 1);
      red_one_pose_sub = debug_nh.subscribe("red_one_pose", 1, &DebugSys::RedOnePose,this);
      //blue_one_pose_sub = debug_nh.subscribe("blue_one_pose", 1, &DebugSys::BlueOnePose,this);
      while(ros::ok())
      {
        is_supply_over = IsSupplyOver();
        SendAreaStatus();//发送加成惩罚区的状态及ID，当前版本为123456同时随机有
        SentrySimulate();//发送哨岗所发出的信息，当前为blue_robort对应的区域
        SendRobotInfo();//发送机器人的信息，血量、子弹等
        PrintRobotInfo();//打印四台机器人的信息到窗口
        ros::spinOnce();
        info_loop_rate.sleep();
      }
    }
    //the information of roborts
    void PrintRobotInfo()
    {
      if(GAMESTATUS == 0)
      {
        //cout<<"wait for game begin!!!   GAMESTATUS is "<<GAMESTATUS<<endl;
      }
      else
      {
        
        cout<<"the game is in progress!!!   GAMESTATUS is "<<GAMESTATUS<<endl;
        for(int i=0;i<2;i++)
        {
          cout<<"RED "<<i<<", HP is : "<<HP[i]<<", ARMOR_ID is : "<<ARMOR_ID[i]<<endl;
          cout<<"shot num is :"<<Bullet_Num_Shot[i]<<", left num is :"<<Bullet_Num_left[i]<<endl;
          cout<<""<<endl;
        }
        for(int i=2;i<4;i++)
        {
          cout<<"BLUE "<<i-2<<", HP is : "<<HP[i]<<", ARMOR_ID is : "<<ARMOR_ID[i]<<endl;
          cout<<"shot num is :"<<Bullet_Num_Shot[i]<<", left num is :"<<Bullet_Num_left[i]<<endl;
          cout<<""<<endl;
        }
      }
    }

    void SentrySimulate()
    {
      Robort_Pose.data.resize(7);
      if (EnemyRobort_Pose1.pose.position.x < 1.5)
      {
        if (EnemyRobort_Pose1.pose.position.y < 3.48)
        {
          Robort_Pose.data[0] = 1250;
          Robort_Pose.data[1] = 2810;//该区域第一个点的坐标
          Robort_Pose.data[2] = 2440;
          Robort_Pose.data[3] = 1650;//该区域第二个点的坐标
          Robort_Pose.data[4] = (0 + 1500)/2;
          Robort_Pose.data[5] = (0 + 3480)/2;
        }
        else
        {
          Robort_Pose.data[0] = 2270;
          Robort_Pose.data[1] = 3480;
          Robort_Pose.data[2] = 5460;
          Robort_Pose.data[3] = 3480;
          Robort_Pose.data[4] = (0 + 6380)/2;
          Robort_Pose.data[5] = (3480 + 4480)/2;
        }
      }/*x<1.5时，如果y<4.1时，则是1区域，否则是2区域*/
      
      if (EnemyRobort_Pose1.pose.position.x >= 1.5 && EnemyRobort_Pose1.pose.position.x <= 4.04)
      {
        if (EnemyRobort_Pose1.pose.position.y < 1.0)
        {
          Robort_Pose.data[0] = 2620;
          Robort_Pose.data[1] = 1000;
          Robort_Pose.data[2] = 5810;
          Robort_Pose.data[3] = 1000;
          Robort_Pose.data[4] = (1700 + 8080)/2;
          Robort_Pose.data[5] = (0 + 1000)/2;
        }
        else if(EnemyRobort_Pose1.pose.position.y > 3.48)
        {
          Robort_Pose.data[0] = 2270;
          Robort_Pose.data[1] = 3480;
          Robort_Pose.data[2] = 5460;
          Robort_Pose.data[3] = 3480;
          Robort_Pose.data[4] = (0 + 6380)/2;
          Robort_Pose.data[5] = (3480 + 4480)/2;
        }
        else
        {
          Robort_Pose.data[0] = 2570;
          Robort_Pose.data[1] = 3240;
          Robort_Pose.data[2] = 2970;
          Robort_Pose.data[3] = 1240;
          Robort_Pose.data[4] = (1500 + 4040)/2;
          Robort_Pose.data[5] = (1000 + 3480)/2;
        }
      }
      if (EnemyRobort_Pose1.pose.position.x > 4.04 && EnemyRobort_Pose1.pose.position.x <= 6.58)
      {
        if (EnemyRobort_Pose1.pose.position.y < 1.0)
        {
          Robort_Pose.data[0] = 2620;
          Robort_Pose.data[1] = 1000;
          Robort_Pose.data[2] = 5810;
          Robort_Pose.data[3] = 1000;
          Robort_Pose.data[4] = (1700 + 8080)/2;
          Robort_Pose.data[5] = (0 + 1000)/2;
        }
        else if(EnemyRobort_Pose1.pose.position.y > 3.48)
        {
          Robort_Pose.data[0] = 2270;
          Robort_Pose.data[1] = 3480;
          Robort_Pose.data[2] = 5460;
          Robort_Pose.data[3] = 3480;
          Robort_Pose.data[4] = (0 + 6380)/2;
          Robort_Pose.data[5] = (3480 + 4480)/2;
        }
        else
        {
          Robort_Pose.data[0] = 5110;
          Robort_Pose.data[1] = 3240;
          Robort_Pose.data[2] = 5510;
          Robort_Pose.data[3] = 1240;
          Robort_Pose.data[4] = (4040 + 6580)/2;
          Robort_Pose.data[5] = (1000 + 3480)/2;
        }
      }
      if (EnemyRobort_Pose1.pose.position.x > 6.58)
      {
        if (EnemyRobort_Pose1.pose.position.y < 1.0)
        {
          Robort_Pose.data[0] = 3250;
          Robort_Pose.data[1] = 1000;
          Robort_Pose.data[2] = 5450;
          Robort_Pose.data[3] = 1000;
          Robort_Pose.data[4] = (1500 + 8100)/2;
          Robort_Pose.data[5] = (0 + 1000)/2;
        }
        else
        {
          Robort_Pose.data[0] = 5640;
          Robort_Pose.data[1] = 2830;
          Robort_Pose.data[2] = 6830;
          Robort_Pose.data[3] = 1670;
          Robort_Pose.data[4] = (6580 + 8080)/2;
          Robort_Pose.data[5] = (1000 + 4480)/2;
        }
      }/*x<1.5时，如果y<4.1时，则是1区域，否则是2区域*/
      //是否识别到的标志位识别到返回识别到的敌方机器人ID      
      Robort_Pose.data[6] = 0;

      robot_pose_pub.publish(Robort_Pose);
    }

    void RedOnePose(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
      RedOne_Pose.pose.position.x = msg->data[0];
      RedOne_Pose.pose.position.y = msg->data[1];
    }

    void BlueOnePose(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
      EnemyRobort_Pose1.pose.position.x = msg->data[0];
      EnemyRobort_Pose1.pose.position.y = msg->data[1];//enemy one robort pose
    }

    bool IsSupplyOver()
    {
      int add_bullet_area;
      int add_hp_area;
      int is_supply = false;
      for(int i = 0; i < 6; i++)
      {
        if(env_status.data[i] == 1)
        {
          add_bullet_area = i;
        }
        if(env_status.data[i] == 2)
        {
          add_hp_area = i;
        }
      }
      cout<<"RedOne_Pose.pose.position.x is : "<<RedOne_Pose.pose.position.x<<endl;
      cout<<"RedOne_Pose.pose.position.y is : "<<RedOne_Pose.pose.position.y<<endl;
      //cout<<"add_bullet_area is : "<<add_bullet_area<<endl;
      //cout<<"add_hp_area is : "<<add_hp_area<<endl;
      //supply_area可能是0-5对应场上的第1-6个加成惩罚区
      if(add_bullet_area == 0)
      {
        if((RedOne_Pose.pose.position.x > 0.23) && (RedOne_Pose.pose.position.x < 0.77) && (RedOne_Pose.pose.position.y > 2.55) && (RedOne_Pose.pose.position.y < 3.03))
        {
          IsAddBullet = true;
          wait_sec++;
          if(wait_sec == 50)
          {
            is_supply = true;
            wait_sec = 0;
          }
          cout<<"wait_sec is : "<<wait_sec<<endl;
        }
        else
        {
          IsAddBullet = false;
        }
      }
      else if(add_bullet_area == 1)
      {
        if(RedOne_Pose.pose.position.x > 1.63 && RedOne_Pose.pose.position.x < 2.17 && RedOne_Pose.pose.position.y > 1.41 && RedOne_Pose.pose.position.y < 1.89)
        {
          IsAddBullet = true;
          wait_sec++;
          if(wait_sec == 50)
          {
            is_supply = true;
            wait_sec = 0;
          }
          cout<<"wait_sec is : "<<wait_sec<<endl;
        }
        else
        {
          IsAddBullet = false;
        }
      }

      else if(add_bullet_area == 2)
      {
        if(RedOne_Pose.pose.position.x > 3.77 && RedOne_Pose.pose.position.x < 4.31 && RedOne_Pose.pose.position.y > 3.795 && RedOne_Pose.pose.position.y < 4.275)
        {
          IsAddBullet = true;
          wait_sec++;
          if(wait_sec == 50)
          {
            is_supply = true;
            wait_sec = 0;
          }
          cout<<"wait_sec is : "<<wait_sec<<endl;
        }
        else
        {
          IsAddBullet = false;
        }
      }

      else if(add_bullet_area == 3)
      {
        if(RedOne_Pose.pose.position.x > 7.31 && RedOne_Pose.pose.position.x < 7.85 && RedOne_Pose.pose.position.y > 1.45 && RedOne_Pose.pose.position.y < 1.93 )
        {
          IsAddBullet = true;
          wait_sec++;
          if(wait_sec == 50)
          {
            is_supply = true;
            wait_sec = 0;
          }
          cout<<"wait_sec is : "<<wait_sec<<endl;
        }
        else
        {
          IsAddBullet = false;
        }
      }

      else if(add_bullet_area == 4)
      {
        if(RedOne_Pose.pose.position.x > 5.91 && RedOne_Pose.pose.position.x < 6.45 && RedOne_Pose.pose.position.y > 2.59 && RedOne_Pose.pose.position.y < 3.07)
        {
          IsAddBullet = true;
          wait_sec++;
          if(wait_sec == 50)
          {
            is_supply = true;
            wait_sec = 0;
          }
          cout<<"wait_sec is : "<<wait_sec<<endl;
        }
        else
        {
          IsAddBullet = false;
        }
      }

      else
      {
        if(RedOne_Pose.pose.position.x > 3.77 && RedOne_Pose.pose.position.x < 4.31 && RedOne_Pose.pose.position.y > 0.205 && RedOne_Pose.pose.position.y < 0.685)
        {
          IsAddBullet = true;
          wait_sec++;
          if(wait_sec == 50)
          {
            is_supply = true;
            wait_sec = 0;
          }
          cout<<"wait_sec is : "<<wait_sec<<endl;
        }
        else
        {
          IsAddBullet = false;
        }
        
      }

      if(add_hp_area == 0)
      {
        if(RedOne_Pose.pose.position.x > 0.23 && RedOne_Pose.pose.position.x < 0.77 && RedOne_Pose.pose.position.y > 2.55 && RedOne_Pose.pose.position.y < 3.03)
        {
          IsAddHP = true;
          wait_sec++;
          if(wait_sec == 50)
          {
            is_supply = true;
            wait_sec = 0;
          }
          cout<<"wait_sec is : "<<wait_sec<<endl;
        }
        else
        {
          IsAddHP = false;
        }
      }
      else if(add_hp_area == 1)
      {
        if(RedOne_Pose.pose.position.x > 1.63 && RedOne_Pose.pose.position.x < 2.17 && RedOne_Pose.pose.position.y > 1.41 && RedOne_Pose.pose.position.y < 1.89)
        {
          IsAddHP = true;
          wait_sec++;
          if(wait_sec == 50)
          {
            is_supply = true;
            wait_sec = 0;
          }
          cout<<"wait_sec is : "<<wait_sec<<endl;
        }
        else
        {
          IsAddHP = false;
        }
      }

      else if(add_hp_area == 2)
      {
        if(RedOne_Pose.pose.position.x > 3.77 && RedOne_Pose.pose.position.x < 4.31 && RedOne_Pose.pose.position.y > 3.795 && RedOne_Pose.pose.position.y < 4.275)
        {
          IsAddHP = true;
          wait_sec++;
          if(wait_sec == 50)
          {
            is_supply = true;
            wait_sec = 0;
          }
          cout<<"wait_sec is : "<<wait_sec<<endl;
        }
        else
        {
          IsAddHP = false;
        }
      }

      else if(add_hp_area == 3)
      {
        if(RedOne_Pose.pose.position.x > 7.31 && RedOne_Pose.pose.position.x < 7.85 && RedOne_Pose.pose.position.y > 1.45 && RedOne_Pose.pose.position.y < 1.93 )
        {
          IsAddHP = true;
          wait_sec++;
          if(wait_sec == 50)
          {
            is_supply = true;
            wait_sec = 0;
          }
          cout<<"wait_sec is : "<<wait_sec<<endl;
        }
        else
        {
          IsAddHP = false;
        }
      }

      else if(add_hp_area == 4)
      {
        if(RedOne_Pose.pose.position.x > 5.91 && RedOne_Pose.pose.position.x < 6.45 && RedOne_Pose.pose.position.y > 2.59 && RedOne_Pose.pose.position.y < 3.07)
        {
          IsAddHP = true;
          wait_sec++;
          if(wait_sec == 50)
          {
            is_supply = true;
            wait_sec = 0;
          }
          cout<<"wait_sec is : "<<wait_sec<<endl;
        }
        else
        {
          IsAddHP = false;
        }
      }
      else
      {
        if(RedOne_Pose.pose.position.x > 3.77 && RedOne_Pose.pose.position.x < 4.31 && RedOne_Pose.pose.position.y > 0.205 && RedOne_Pose.pose.position.y < 0.685)
        {
          IsAddHP = true;
          wait_sec++;
          if(wait_sec == 50)
          {
            is_supply = true;
            wait_sec = 0;
          }
          cout<<"wait_sec is : "<<wait_sec<<endl;
        }
        else
        {
          IsAddHP = false;
        }
      }
      return is_supply;
    }

    void randperm(int Num)
    {
      vector<int> temp;//temp 定义为整型向量
      for (int i = 0; i < Num; ++i)
      {
        temp.push_back(i + 1);
        //在该向量的尾部加上一个元素i+1，处理完的结果是生成了一个1-6的数组
      }
      random_shuffle(temp.begin(), temp.end());
      //对temp.begin(), temp.end()中间的数随机排序
      for (int i = 0; i < temp.size(); i++)
      {
        env_status.data.resize(Num);
        env_status.data[i] = temp[i];
      }
    }

    void SendAreaStatus()
    {
      int Num = 6;
      if(second == 600 && IsStartGame)//60*10,一分钟更换一次
      {
        vector<int> temp;//temp 定义为整型向量
        for (int i = 0; i < Num; ++i)
        {
          temp.push_back(i + 1);
          //在该向量的尾部加上一个元素i+1，处理完的结果是生成了一个1-6的数组
        }
        random_shuffle(temp.begin(), temp.end());
        //对temp.begin(), temp.end()中间的数随机排序
        for (int i = 0; i < temp.size(); i++)
        {
          env_status.data.resize(Num);
          env_status.data[i] = temp[i];
        }
        second = 0;
      }
      cout<<"Area1 : "<<env_status.data[0]
        <<"  Area2 : "<<env_status.data[1]
        <<"  Area3 : "<<env_status.data[2]
        <<"  Area4 : "<<env_status.data[3]
        <<"  Area5 : "<<env_status.data[4]
        <<"  Area6 : "<<env_status.data[5]<<endl;
      for(int j = 0; j < Num; j++)
      {
        //当这个区域是加成区并且已经过了5s则发0,加血区对应的血量加200，加弹区对应的子弹加100
        if(env_status.data[j] == 1 && is_supply_over && IsAddBullet)
        {
          env_status.data[j] = 0;
          HP[0] += 200;
          HP[1] += 200;
        }
        if(env_status.data[j] == 2 && is_supply_over && IsAddHP)
        {
          env_status.data[j] = 0;
          Bullet_Num_left[0] += 100;
          Bullet_Num_left[1] += 100;
        }
      }
      //cout<<"is_supply_over is : "<<(int)is_supply_over<<endl;
      second++;
      if(second >= 600)
      {
        second = 600;
      }
      cout<<"second is : "<<second/10<<endl;
      chatter_pub.publish(env_status);
    }

    void SendRobotInfo()
    {
      std_msgs::Float64MultiArray robot_info;
      robot_info.data.resize(17);
      if(GAMESTATUS == 4)
      {
        robot_info.data[0] = Bullet_Num_left[0];
        robot_info.data[1] = Bullet_Num_Shot[0];
        robot_info.data[2] = ARMOR_ID[0];
        robot_info.data[3] = HP[0];
        robot_info.data[4] = Bullet_Num_left[1];
        robot_info.data[5] = Bullet_Num_Shot[1];
        robot_info.data[6] = ARMOR_ID[1];
        robot_info.data[7] = HP[1];
        robot_info.data[8] = Bullet_Num_left[2];
        robot_info.data[9] = Bullet_Num_Shot[2];
        robot_info.data[10] = ARMOR_ID[2];
        robot_info.data[11] = HP[2];
        robot_info.data[12] = Bullet_Num_left[3];
        robot_info.data[13] = Bullet_Num_Shot[3];
        robot_info.data[14] = ARMOR_ID[3];
        robot_info.data[15] = HP[3];
        robot_info.data[16] = GAMESTATUS;
      }
      else
      {
        for(int i = 0; i < 17; i++)
        {
          robot_info.data[i] = 0;
        }
      }
      roborts_info_pub.publish(robot_info);
    }
    ~DebugSys() = default;
  private:
    ros::Publisher chatter_pub;
    ros::Publisher robot_pose_pub;
    ros::Subscriber red_one_pose_sub;
    ros::Subscriber blue_one_pose_sub;
    std_msgs::Float64MultiArray env_status;
    std_msgs::Float64MultiArray Robort_Pose;
    geometry_msgs::PoseStamped RedOne_Pose;
    geometry_msgs::PoseStamped EnemyRobort_Pose1;
    int wait_sec = 0;
    bool is_supply_over; bool IsAddBullet = false; bool IsAddHP = false;
    /*一共四位数，依次是R1,R2,B1,B2
    GAMESTATUS等于1开始比赛，等于0是等待比赛开始
    ARMOR_ID 0 1 2 3 分别表示前左后右
    */
    ros::NodeHandle   debug_nh;
    ros::Publisher    roborts_info_pub;
    ros::Subscriber   roborts_info_sub;
};

void thread_debug()
{
  DebugSys debugsys;
  ros::spinOnce();
}
int main(int argc, char**argv)
{
  ros::init(argc, argv, "decision_simulate");
  thread thread_one(thread_debug);
  thread thread_two(SetValueFromKeyboard);//通过键盘改变机器人的信息
  //thread_one.join();
  //thread_two.join();
  ros::waitForShutdown();
  ros::spinOnce();
}