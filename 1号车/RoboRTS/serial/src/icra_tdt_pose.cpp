#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
//#include <sensor_msgs/Imu.h>
#include <math.h>
//#include "my_file.h"//还原时去掉注释
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>

#define RAD2DEG(x) ((x)*180./M_PI)
double slide=0;
double cos_angle=0;
double cal_angle=0;

double ans=0;
int init_time=-1;
double add_angle=0.0;//比赛开始后针对不同起始区增添的角度
double x_wheel_pos = -0.10;
double y_wheel_pos = 0.0;
double globalpose_x=0.0;
double globalpose_y=0.0;
double globalpose_angle=0.0;
double init_globalpose_x=0.0;
double init_globalpose_y=0.0;
double init_globalpose_angle=0.0;
double x_wheel_vel = 0.0;
double y_wheel_vel = 0.0;
double x_origin_wheel_vel = 0.0;
double y_origin_wheel_vel = 0.0;
double x_base_vel= 0.0;
double y_base_vel = 0.0;
double robot_x_base_vel=0;
double robot_y_base_vel=0;
double theta = 0.0;
double theta_vel = 0.0;
double theta_init=0.0;
double theta_final=0.0;
double delta_theta = 0.0;
double x_base_pos = 0.0;
double y_base_pos = 0.0;
double x_check_pos= 0.0;
double y_check_pos= 0.0;
double ans_check;
double uwb_x_pos=0.0;
double uwb_y_pos=0.0;
double uwb_angular=0.0;
double last_vel_time=0.0;
double delta_time=0.0;
double back_x_offset=-0.05;
double back_y_offset=0;
double right_x_offset=0.18;
double right_y_offset=0;
double front_x_offset=0.18;
double front_y_offset=0;
double left_x_offset=0.18;
double left_y_offset=0;
double change_angle=0.0;
double change_angle_init=0.0;
double change_time=0.0;
float degree=0;
//double change_angle_delta=0.0;
double max_change_angle_delta=0.0;
double change_angle_uwb=0.0;
double change_angle_init_uwb=0.0;
double change_angle_delta_uwb=0.0;
double max_change_angle_delta_uwb=0.0;
struct laser
{
	double front;
	double left;
	double back;
	double right;
	double front_used;
	double left_used;
	double back_used;
	double right_used;
	double ans;
};
laser laser_;
     //geometry_msgs::TransformStamped map_trans;
std::vector<double> angle;
 geometry_msgs::PoseWithCovarianceStamped revify_pose;
int revify_sign=0;//0表示不校准，1表示校准，2表示发布
//x方向对应y摩擦轮
//y方向对应x摩擦轮
double myabs(double msg)
{
	msg=msg>0?msg:-msg;
	return msg;
}
void uwbCallback(const geometry_msgs::Twist& pos){
uwb_x_pos=pos.linear.x;
uwb_y_pos=pos.linear.y;
change_angle_uwb=pos.angular.z;

laser_.back=pos.angular.x;
	}
void globalposeCallback(const geometry_msgs::Twist& pos){
globalpose_x=pos.linear.x;
globalpose_y=pos.linear.y;
globalpose_angle=pos.angular.z;
	}
void init_globalposeCallback(const geometry_msgs::Twist& pos){
init_globalpose_x=pos.linear.x;
init_globalpose_y=pos.linear.y;
init_globalpose_angle=pos.angular.z;
	}
void velCallback(const geometry_msgs::Twist& vel) {
	//callback every time the robot's linear velocity is received
	ros::Time current_time = ros::Time::now();
	
	x_origin_wheel_vel = vel.linear.x;
	y_origin_wheel_vel = vel.linear.y;
	theta_init=vel.angular.x;//直接读取陀螺仪角度
	theta_vel = vel.angular.z;//角速度
	//laser_.back = vel.angular.x;
//ROS_INFO("vel=%f,%f,%f",x_origin_wheel_vel,y_origin_wheel_vel,theta_vel);//9-1
/*	if (theta_vel < 0.02&&theta_vel >- 0.02)
	{
		theta_vel = 0;
	}*/
	//第一帧赋初值theta_init
	if (last_vel_time == 0)//用于结算轮子位移，不能删除
	{
		last_vel_time = current_time.toSec();
	//	delta_time = 0;
	}
	//else
	//{
	  if(!revify_sign)//不校准时在更新，以防校准时有干扰
	  {
		//时间差
		delta_time = current_time.toSec() - last_vel_time;
		//累加出角度
		//theta += theta_vel*delta_time;
//ROS_INFO_STREAM("theta="<<theta);
		//角度变换，变到0-2pi度数
		if (theta_init > 6.2832)
		{
			theta_init -= 6.2832; 
		}
		else if (theta_init < 0)
		{
			theta_init += 6.2832;
		}
		if(init_time==-1)
		{
		  change_angle_init=theta_init ;//直接读取角度作为初始角度
		}
		theta=theta_init-change_angle_init;//用与第一时刻的角度差角度差表示所有角度差
		//换算速度坐标系至odom坐标系(由于小车坐标系与陀螺仪坐标系旋转相反，采用原始相反的theta角度,x=x*cos(theta)+y*sin(-theta);
	    x_wheel_vel = x_origin_wheel_vel*sin(-(theta+add_angle)) + y_origin_wheel_vel*cos((theta+add_angle));
		y_wheel_vel = x_origin_wheel_vel*cos(-(theta+add_angle))+ y_origin_wheel_vel*sin((theta+add_angle));
//ROS_INFO("wheel_vel:%f,%f,%f",x_wheel_vel,y_thetawheel_vel,theta_vel);//9-1
		//计算小车速度
		x_base_vel = x_wheel_vel+ 0.10*theta_vel*sin((theta+add_angle));
		y_base_vel = y_wheel_vel+ 0.10*theta_vel*cos((theta+add_angle));
//ROS_INFO("base_vel:%f,%f,%f",x_base_vel,y_base_vel,theta_vel);//9-1
		robot_x_base_vel=x_base_vel*cos((theta+add_angle))+y_base_vel*sin((theta+add_angle));
		robot_y_base_vel=x_base_vel*sin(-(theta+add_angle))+y_base_vel*cos((theta+add_angle));
		//累加成位移
		x_wheel_pos += x_wheel_vel*delta_time;
		y_wheel_pos += y_wheel_vel*delta_time;
	  //ROS_INFO("wheel_pose:%f,%f,%f",x_wheel_pos,y_wheel_pos ,delta_time);//9-1
		//建立正交轮坐标系
		//x_wheel_pos = x_wheel_pos - 0.275;
		//建立小车坐标系
		x_base_pos = x_wheel_pos + 0.10*cos((theta+add_angle));
		y_base_pos = y_wheel_pos + 0.10*sin((theta+add_angle));
	  //ROS_INFO("base_pose: %f,  %f",x_base_pos,y_base_pos);//9-1
		//时间重新赋值
		last_vel_time = current_time.toSec();
		init_time++;
	  }
	//}
}
short int reinit_sign;
void reinitCallback(const std_msgs::Int16::ConstPtr& sign)
{
	reinit_sign = sign->data;
	ROS_ERROR_STREAM("reinit_sign: "<<reinit_sign);
}

/*
void checkCallback(const std_msgs::Int16& pos)
{
      revify_sign=1;//校准标志位为1
		ans=(max_change_angle_delta);//实际角度
		ROS_ERROR_STREAM("ans"<<ans);
		x_check_pos = x_wheel_pos + 0.10*cos(ans);
		y_check_pos = y_wheel_pos + 0.10*sin(ans);
		ROS_ERROR_STREAM("ans"<<ans<<"x_check_pos"<<x_check_pos<<"y_check_pos"<<y_check_pos);
		if(MyConfig.ROBOT_INFO.color=="RED"&&MyConfig.ROBOT_INFO.number==1)
		{
		 // x_base_pos=x_check_pos;
		 // y_base_pos=y_check_pos;
		  x_base_pos=0.5;
		  y_base_pos=0.5;
 		  x_wheel_pos =x_base_pos- 0.10*cos(ans);
		  y_wheel_pos=y_base_pos- 0.10*sin(ans);
		  add_angle=0;
		}
		else if(MyConfig.ROBOT_INFO.color=="RED"&&MyConfig.ROBOT_INFO.number==2)
		{
		  //ROS_INFO_STREAM("1111111111");
		  x_base_pos=7.5;
		  y_base_pos=0.5;
 		  x_wheel_pos=x_base_pos- 0.10*cos(ans);
		  y_wheel_pos=y_base_pos- 0.10*sin(ans);
		  ans=ans+1.57;
		  ans>6.2832?ans-=6.2832:ans;
		  add_angle=1.57;
		  
		}
		else if(MyConfig.ROBOT_INFO.color=="BLUE"&&MyConfig.ROBOT_INFO.number==1)
		{
		  x_base_pos=7.5;
		  y_base_pos=4.5;
		  x_wheel_pos=x_base_pos- 0.10*cos(ans);
		  y_wheel_pos=y_base_pos- 0.10*sin(ans);
		  ans=ans+3.14;
		ans>6.28?ans-=6.28:ans;
		add_angle=3.14;
		//ROS_WARN_STREAM("x_check_pos"<<x_check_pos<<"y_check_pos"<<y_check_pos);
		}
		else if(MyConfig.ROBOT_INFO.color=="BLUE"&&MyConfig.ROBOT_INFO.number==2)
		{
		  x_base_pos=0.5;
		  y_base_pos=4.5;
		  x_wheel_pos=x_base_pos- 0.10*cos(ans);
		  y_wheel_pos=y_base_pos- 0.10*sin(ans);
		  ans=ans+4.71;
		  ans>6.28?ans-=6.28:ans;
		  add_angle=4.71;

		}
	
ROS_WARN_STREAM("change x="<<x_base_pos<<"change y="<<y_base_pos <<"change angle:"<<ans);
}*/
int main(int argc, char** argv){
  geometry_msgs::Twist base_vel;
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle n;
 tf::TransformBroadcaster map_broadcaster;
 geometry_msgs::TransformStamped map_trans;
angle.push_back(0);
angle.push_back(0);
angle.push_back(0);
 //ros::Time current_time_now = ros::Time::now();
  ros::Subscriber sub = n.subscribe("raw_msg", 50, velCallback);
  ros::Subscriber uwb_pose_sub_=n.subscribe("uwb_msg",2,uwbCallback);
  //ros::Subscriber sub_scan = n.subscribe("scan",1000,sendCallback);//位置？
  ros::Subscriber sub_global=n.subscribe("globalpose",10,globalposeCallback);
  ros::Subscriber init_sub_global=n.subscribe("init_globalpose",10,init_globalposeCallback);
  ros::Subscriber reload_init = n.subscribe("reinit", 50, reinitCallback);
//初始定位话题
  //ros::Subscriber recharify_pose_sub_=n.subscribe("check",2,checkCallback);
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("raw_vel", 1000);
  ros::Publisher pos_pub = n.advertise<geometry_msgs::Twist>("raw_pos", 1000);
  ros::Publisher pose_pub=n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);
  tf::TransformBroadcaster odom_broadcaster;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  //ROS_WARN_STREAM("color"<<MyConfig.ROBOT_INFO.color<<"num"<<MyConfig.ROBOT_INFO.number);

ros::Rate rate(20);
  while (n.ok()) {
    ros::Time current_time_now1 = ros::Time::now();
   // ROS_ERROR_STREAM("current_time_now.toSec()"<<current_time_now1);
   
    if (init_time==0)
    {
      change_time=current_time_now1.toSec();
     //ROS_ERROR_STREAM("change_angle_init"<<change_angle_init<<"change_time"<<change_time);
      //change_angle_init_uwb=change_angle_uwb;
    }
    //ROS_ERROR_STREAM("current_time_now.toSec()-change_time"<<current_time_now1.toSec()-change_time);
    if(current_time_now1.toSec()-change_time>10.0)
    {
      change_time=current_time_now1.toSec();
      //ROS_ERROR_STREAM("change_time"<<change_time);
      //change_angle_delta=theta-change_angle_init;
      //change_angle_delta_uwb=change_angle_uwb-change_angle_init_uwb;
     // ROS_ERROR_STREAM("change_angle_delta="<<change_angle_delta<<"change_angle_uwb="<<change_angle_uwb);
      if((theta-delta_theta)>0.12||(theta-delta_theta)<-0.12)//每10s的角度差
      {
	max_change_angle_delta=theta;
	delta_theta=theta;
	ROS_ERROR_STREAM("change_time"<<change_time<<"theta"<<theta);
	
      }
      else
      {
	delta_theta=theta;//零偏置也去掉
      }
      if(change_angle_delta_uwb>5)
      {
	max_change_angle_delta_uwb+=change_angle_delta_uwb;
      }   
      
    }
//if(revify_sign==1)
if(reinit_sign==1)
{
//ROS_WARN_STREAM("change_angle_delta_uwb"<<change_angle_delta_uwb<<"max_change_angle_delta"<<max_change_angle_delta);
      revify_pose.header.stamp=ros::Time::now();
revify_pose.header.frame_id="map";
revify_pose.pose.pose.position.x=0.5;//x_base_pos;
revify_pose.pose.pose.position.y=0.5;//y_base_pos;
revify_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(ans);
revify_pose.pose.covariance[0] = 0.25;
revify_pose.pose.covariance[7] = 0.25;
//revify_pose.pose.covariance[14] = FLT_MAX;
//revify_pose.pose.covariance[21] = FLT_MAX;
//revify_pose.pose.covariance[28] =FLT_MAX;
revify_pose.pose.covariance[35] = 0.06;
//if(!(init_globalpose_x>0&&init_globalpose_x<8&&init_globalpose_y>0&&init_globalpose_y<5&&myabs(init_globalpose_x-x_base_pos)<0.2&&myabs(init_globalpose_y-y_base_pos)<0.2))
//{
pose_pub.publish(revify_pose);
ROS_INFO_STREAM("global coordinate x="<<revify_pose.pose.pose.position.x<<"global coordinate y="<<revify_pose.pose.pose.position.y <<"rotation angle:"<<ans);
}
  revify_sign=0;
 
//还原里程计
//x_base_pos=0;
//y_base_pos=0;
//x_wheel_pos=-0.1;
//y_wheel_pos=0;

   // }
	  ros::spinOnce();
	  ros::Time current_time_now = ros::Time::now();

	  //odom_trans：odom-正交轮
	  geometry_msgs::TransformStamped odom_trans, base_center,uwb_pos,change_trans;
	  
	  //方向四元数
	  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
	  odom_trans.header.frame_id = "odom";
	  odom_trans.child_frame_id = "orthgonality_wheel";
	  //robot's position in x,y, and z
	  odom_trans.transform.translation.x = x_wheel_pos;
	  odom_trans.transform.translation.y = y_wheel_pos;
//ROS_INFO_STREAM("wheel:x="<<x_wheel_pos<<"y="<<y_wheel_pos<<"theta="<<theta);
	  odom_trans.transform.translation.z = 0.0;
	  //robot's heading in quaternion
	  odom_trans.transform.rotation = odom_quat;
	  odom_trans.header.stamp = current_time_now;
	  //publish robot's tf using odom_trans object
	  odom_broadcaster.sendTransform(odom_trans);
	  //base_center：odom-底盘中心
	  geometry_msgs::Quaternion base_quat = tf::createQuaternionMsgFromYaw(theta+add_angle);
	  base_center.header.frame_id = "odom";
	  base_center.child_frame_id = "base_footprint";
	  base_center.header.stamp = current_time_now;
	  base_center.transform.translation.x = x_base_pos;
	  base_center.transform.translation.y = y_base_pos;
	  base_center.transform.translation.z = 0.0;
	  base_center.transform.rotation = base_quat;
	  odom_broadcaster.sendTransform(base_center);
//ROS_INFO_STREAM("base:x="<<x_base_pos<<"y="<<y_base_pos<<"theta="<<theta);
//ROS_INFO_STREAM("slide"<<slide<<"cos_angle"<<cos_angle<<"cal_angle"<<cal_angle<<"angle"<<ans);
	  //小车位置
	  geometry_msgs::Twist base_pos;
	  base_pos.linear.x = x_base_pos;
	  base_pos.linear.y = y_base_pos;
 	  base_pos.angular.z =theta+add_angle;//theta
	  pos_pub.publish(base_pos);
	  //小车中心速度
	  geometry_msgs::Twist base_vel;
	  base_vel.linear.x = robot_x_base_vel;
	  base_vel.linear.y = robot_y_base_vel;
	  base_vel.angular.z = theta_vel;
	  pub.publish(base_vel);
	  /*change_trans.header.frame_id = "map";
	  change_trans.child_frame_id = "odom";
	  //robot's position in x,y, and z
	  change_trans.transform.translation.x = 0;
	  change_trans.transform.translation.y = 0;
	  change_trans.transform.translation.z = 0.0;
	  change_trans.transform.rotation = odom_quat;
	  change_trans.header.stamp = current_time_now;
	  odom_broadcaster.sendTransform(change_trans);*/
 //导航模块
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time_now;
    odom.header.frame_id = "odom";
    //robot's position in x,y, and z
    odom.pose.pose.position.x = x_base_pos;
    odom.pose.pose.position.y = y_base_pos;
    odom.pose.pose.position.z = 0.0;
    //robot's heading in quaternion
    odom.pose.pose.orientation = base_quat;

    odom.child_frame_id = "base_footprint";
    //linear speed from encoders
    odom.twist.twist.linear.x = robot_x_base_vel;
    odom.twist.twist.linear.y = robot_y_base_vel;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    //angular speed from IMU
    odom.twist.twist.angular.z = theta_vel;

	//TODO: include covariance matrix here
	odom.pose.covariance[0] = 20;
	odom.pose.covariance[7] = 20;
	odom.pose.covariance[14] = FLT_MAX;
	odom.pose.covariance[21] = FLT_MAX;
	odom.pose.covariance[28] =FLT_MAX;
	odom.pose.covariance[35] = 50;

	odom.twist.covariance[0] = .1; 
	odom.twist.covariance[7] = .1; 
	odom.twist.covariance[14] = 1000000000;
	odom.twist.covariance[21] = 1000000000;
	odom.twist.covariance[28] = 1000000000;
	odom.twist.covariance[35] = .1; 

    odom_pub.publish(odom);
                //uwbweizhi
	  geometry_msgs::Quaternion uwb_quat = tf::createQuaternionMsgFromYaw(uwb_angular);    
	  uwb_pos.header.frame_id = "odom";
     uwb_pos.child_frame_id = "uwb";
    uwb_pos.transform.translation.x = uwb_x_pos;
    uwb_pos.transform.translation.y = uwb_y_pos;
    uwb_pos.transform.translation.z = 0.0;
    uwb_pos.transform.rotation = uwb_quat;
    uwb_pos.header.stamp = current_time_now;
    //publish robot's tf using odom_trans object
    odom_broadcaster.sendTransform(uwb_pos);
	  rate.sleep();
  }
}
