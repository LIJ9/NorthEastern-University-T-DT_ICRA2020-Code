#ifndef MY_FILE_H
#define MY_FILE_H

#include "io/io.h"
#include <fstream>
#include <ros/package.h>
#include<tf/tf.h>
int Bullet_NUM_First;
namespace roborts_decision 
{
  class RobotPoint
  {
  	public :
    double x;
    double y;
    double z;
    double yaw;
    
    RobotPoint() : x(0),y(0),z(0),yaw(0)
    {}
    
    ~RobotPoint() = default;
  };
  
  class RobotInfo
  {
  public :
    std::string color;
    int number;
    
    RobotInfo() : color("RED"),number(1)
    {}
    
    ~RobotInfo() = default;
  };
  
  class RobotConfig
  {
  public:
    RobotInfo           ROBOT_INFO;
    RobotPoint          ADDBUFF_POINT;
    RobotPoint          ADDBULLET_POINT;
    RobotPoint          MYESCAPE_POINT1;
    RobotPoint          MYESCAPE_POINT2;
    std::vector <RobotPoint> PATROL_POINT;//std::vector定义的变量可存放多个值,一个vector为一维，两个则为２维
    std::vector <RobotPoint> BONUS_PENALTY_POINT;
    RobotConfig()
    {
      std::string RobotConfigPath = ros::package::getPath("roborts_decision") + "/my_config/my_config.txt";
      ReadConfigFromTxt(RobotConfigPath);
      
      
    }
    void ReadPoint(RobotPoint& PointType, std::ifstream& infile)
    //ifstream是输入流，将输入流给地址infile
    {
      std::string s,ts;
      std::getline(infile, s);//从传给地址infile输入，赋值给变量s
      
      std::istringstream istring(s);//istringstream类用于执行C++风格的串流的输入操作
      istring >> ts;
      PointType.x = atof(ts.c_str());
      istring >> ts;
      PointType.y = atof(ts.c_str());
      istring >> ts;
      PointType.z = atof(ts.c_str());
      istring >> ts;
      PointType.yaw = atof(ts.c_str());
      //一个读取四个数据,用于读取单个数据
    }
    
    void ReadPoint(std::vector<RobotPoint>& PointType, std::ifstream& infile)
    {       
      while(1)
      {
		std::string s,ts;
		std::getline(infile, s);
		if(s.empty())
		return;
		std::istringstream istring(s);
		RobotPoint TempPoint;
		istring >> ts;
		TempPoint.x = atof(ts.c_str());
		//atof把字符串转换成浮点数，直至遇到第一个空格
		//c_str()函数返回一个指向正规C字符串的指针常量, 内容与本string串相同
		istring >> ts;
		TempPoint.y = atof(ts.c_str());
		istring >> ts;
		TempPoint.z = atof(ts.c_str());
		istring >> ts;
		TempPoint.yaw = atof(ts.c_str()); 
		PointType.push_back(TempPoint);
      }  
    }
    //用于读取巡逻点
    //读取配置文件中的参数
    void ReadConfigFromTxt(std::string& file)
    {
      std::ifstream infile;//ifstream输入流的对象
      infile.open(file.data()); //打开文件my_config

      if(!infile.is_open())
      {
		ROS_ERROR("Can't open file : my_config.txt");
		return;
      }
      //返回一个C风格的字符串,只要不是空指针，就说明加载了文件
      std::string s,ts; 
      std::getline(infile, s);//getline从输入流中读取字符, 并把它们转换成字符串
      //s为从输入流（txt文件）中读取字符，并将他们转化为字符串
      std::istringstream istring(s);     
	  //它可以创建一个对象，然后这个对象就可以绑定一行字符串，然后以空格为分隔符把该行分隔开来
      istring >> ts;//输入一个
      ROBOT_INFO.color = ts;
      istring >> ts;
      ROBOT_INFO.number = atoi(ts.c_str());	//将读取到的字符串转化为整型数据
	  if(ROBOT_INFO.number == 1)
	  {
		Bullet_NUM_First = 50;
	  }
      if(ROBOT_INFO.color == "RED")
      {  	
		while (std::getline(infile, s))
		{	    
			if (s == "BONUS_PENALTY_POINT")
			{
				ReadPoint(BONUS_PENALTY_POINT,infile); 
				std::cout <<"BONUS_PENALTY_POINT is OK"<<std::endl;
			}
			
						
			if(s == "NO1_MYESCAPE_POINT1" && ROBOT_INFO.number == 1)
			{
				ReadPoint(MYESCAPE_POINT1,infile); 
				std::cout <<"MYESCAPE_POINT1 is OK"<<std::endl;	    
			}
			if(s == "NO1_MYESCAPE_POINT2" && ROBOT_INFO.number == 1)
			{
				ReadPoint(MYESCAPE_POINT2,infile); 
				std::cout <<"MYESCAPE_POINT2 is OK"<<std::endl;	    
			}
			
			if(s == "NO2_MYESCAPE_POINT1" && ROBOT_INFO.number == 2)
			{
				ReadPoint(MYESCAPE_POINT1,infile); 
				std::cout <<"MYESCAPE_POINT1 is OK"<<std::endl;	    
			}
			
			if(s == "NO2_MYESCAPE_POINT2" && ROBOT_INFO.number == 2)
			{
				ReadPoint(MYESCAPE_POINT2,infile); 
				std::cout <<"MYESCAPE_POINT2 is OK"<<std::endl;	    
			}
			
			if(s == "RED1_PATROL_POINT" && ROBOT_INFO.number == 1)
			{
				ReadPoint(PATROL_POINT,infile);
				std::cout <<"RED1 PATROL_POINT is OK, and size is : "<<PATROL_POINT.size()<<std::endl;
			}
			
			if(s == "RED2_PATROL_POINT" && ROBOT_INFO.number == 2)
			{
				ReadPoint(PATROL_POINT,infile);
				std::cout <<"RED2 PATROL_POINT is OK, and size is : "<<PATROL_POINT.size()<<std::endl;
			}
		}
      }
      else if(ROBOT_INFO.color == "BLUE")
      {  	
		while (std::getline(infile, s))
		{	  
			
			if (s == "BONUS_PENALTY_POINT")
			{
				ReadPoint(BONUS_PENALTY_POINT,infile); 
				std::cout <<"BONUS_PENALTY_POINT is OK"<<std::endl;
			}
									
			if(s == "NO1_MYESCAPE_POINT1" && ROBOT_INFO.number == 1)
			{
				ReadPoint(MYESCAPE_POINT1,infile); 
				std::cout <<"MYESCAPE_POINT1 is OK"<<std::endl;	    
			}
			
			if(s == "NO1_MYESCAPE_POINT2" && ROBOT_INFO.number == 1)
			{
				ReadPoint(MYESCAPE_POINT2,infile); 
				std::cout <<"MYESCAPE_POINT2 is OK"<<std::endl;	    
			}
			
			if(s == "NO2_MYESCAPE_POINT1" && ROBOT_INFO.number == 2)
			{
				ReadPoint(MYESCAPE_POINT1,infile); 
				std::cout <<"MYESCAPE_POINT1 is OK"<<std::endl;	    
			}
			
			if(s == "NO2_MYESCAPE_POINT2" && ROBOT_INFO.number == 2)
			{
				ReadPoint(MYESCAPE_POINT2,infile); 
				std::cout <<"MYESCAPE_POINT2 is OK"<<std::endl;	    
			}
			
			if(s == "BLUE1_PATROL_POINT" && ROBOT_INFO.number == 1)
			{
				ReadPoint(PATROL_POINT,infile);
				std::cout <<"BLUE1 PATROL_POINT is OK, and size is : "<<PATROL_POINT.size()<<std::endl;
				//size其实就是计算不同类型容器中的元素个数的
			}
			
			if(s == "BLUE2_PATROL_POINT" && ROBOT_INFO.number == 2)
			{
				ReadPoint(PATROL_POINT,infile);
				std::cout <<"BLUE2 PATROL_POINT is OK, and size is : "<<PATROL_POINT.size()<<std::endl;
			}
		}
      }
      else
      {
	ROS_ERROR("Color is wrong!!!");
      }
      
      std::cout <<"***************************"<<std::endl;
      std::cout <<"ROBOT_INFO.color is : "<<ROBOT_INFO.color<<"   ROBOT_INFO.number is : "<<ROBOT_INFO.number<<std::endl;
      std::cout << "ADDBUFF_POINT is :" << ADDBUFF_POINT.x << " "<< ADDBUFF_POINT.y <<" "<<ADDBUFF_POINT.z <<" "<<ADDBUFF_POINT.yaw <<std::endl;
      std::cout << "ADDBULLET_POINT is :" << ADDBULLET_POINT.x << " "<< ADDBULLET_POINT.y << " "<<ADDBULLET_POINT.z << " "<< ADDBULLET_POINT.yaw <<std::endl;
      std::cout << "MYESCAPE_POINT1 is :" << MYESCAPE_POINT1.x << " "<< MYESCAPE_POINT1.y <<" "<<MYESCAPE_POINT1.z <<" "<<MYESCAPE_POINT1.yaw <<std::endl;
      std::cout << "MYESCAPE_POINT2 is :" << MYESCAPE_POINT2.x << " "<< MYESCAPE_POINT2.y << " "<<MYESCAPE_POINT2.z << " "<< MYESCAPE_POINT2.yaw <<std::endl;
      std::cout << "PATROL_POINT is : " << PATROL_POINT[0].x << std::endl;
      std::cout << "PATROL_POINT is : " << BONUS_PENALTY_POINT[0].x << std::endl;
      std::cout <<"***************************"<<std::endl;
      infile.close();                
    }
    ~RobotConfig() = default;    
  };
}
roborts_decision::RobotConfig MyConfig;

#endif