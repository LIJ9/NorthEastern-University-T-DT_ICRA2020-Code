#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <math.h>
std::vector<double> angle;
geometry_msgs::Twist pose;
geometry_msgs::Twist cmd_pose;
#define RAD2DEG(x) ((x)*180./M_PI)
double laser;
int adjust_sign=0;
double degree=0;
int num;

double ans[5]={0,0,0,0,0};
double diff_i_angle=0;
double out_v_angle=0;
double diff_i_y=0;
double out_v_y=0;

double diff_i_x=0;
double out_v_x=0;
double offset_front=0;
double slide[4]={0,0,0,0};
double cos_angle[4]={0,0,0,0};
double cal_angle[4]={0,0,0,0};
double diff_y=0;
double diff_angle=0;
double diff_x=0;
double offset_front_x=0.1;
double x_vel;
double y_vel;
double offset_back_x=0.05;//激光模块到补弹口中心，正向为正
double limit(double x,double a,double b)
{
if (x>a)
return a;
else if(x<b)
return b;
else
return x;
}
double myabs(double x)
{
if (x<0)
return -x;
else
return x;
}
void sendCallback(const sensor_msgs::LaserScan& msg)//right
{
  //ROS_INFO("11");
int count = msg.scan_time / msg.time_increment;
for(int i = 0; i < count; i++)
 {
   float degree = RAD2DEG(msg.angle_min + msg.angle_increment * i);
      if(degree>-90.1&&degree<-89.9)
{
        //ROS_INFO("right=: [%f, %f]", degree, msg.ranges[i]);
angle[0]=msg.ranges[i];
pose.linear.y=msg.ranges[i];
}
if(degree>-85.1&&degree<-84.9)
{
       // ROS_INFO("right=: [%f, %f]", degree, msg.ranges[i]);
angle[1]=msg.ranges[i];
}
if(degree>-80.1&&degree<-79.9)
{
      // ROS_INFO("right=: [%f, %f]", degree, msg.ranges[i]);
angle[2]=msg.ranges[i];
}
if(degree>-75.1&&degree<-74.9)
{
      // ROS_INFO("right=: [%f, %f]", degree, msg.ranges[i]);
angle[3]=msg.ranges[i];
}
if(degree>-70.1&&degree<-69.9)
{
      // ROS_INFO("right=: [%f, %f]", degree, msg.ranges[i]);
angle[4]=msg.ranges[i];
}
   }
 slide[0]=sqrt(angle[0]*angle[0]+angle[2]*angle[2]-2*angle[0]*angle[2]*cos(0.174));
   cos_angle[0]=(angle[0]*angle[0]+slide[0]*slide[0]-angle[2]*angle[2])/(2*angle[0]*slide[0]);
   cal_angle[0]=(cos_angle[0]>0.99?0.99:(cos_angle[0]<-0.99)?-0,99:acos(cos_angle[0]));
//ROS_INFO_STREAM("angle0"<<angle[0]<<"angle2="<<angle[2]<<"slide"<<slide[0]<<"cos_sngle"<<cal_angle[0]);
ans[1]=1.57-cal_angle[0];//cal_angle-90+180
 slide[1]=sqrt(angle[0]*angle[0]+angle[1]*angle[1]-2*angle[0]*angle[1]*cos(0.087));
  cos_angle[1]=(angle[0]*angle[0]+slide[1]*slide[1]-angle[1]*angle[1])/(2*angle[0]*slide[1]);
  cal_angle[1]=(cos_angle[1]>0.99?0.99:(cos_angle[1]<-0.99)?-0,99:acos(cos_angle[1]));
ans[2]=1.57-cal_angle[1];//cal_angle-90+180
 slide[2]=sqrt(angle[0]*angle[0]+angle[2]*angle[2]-2*angle[0]*angle[2]*cos(0.262));
  cos_angle[2]=(angle[0]*angle[0]+slide[2]*slide[2]-angle[2]*angle[2])/(2*angle[0]*slide[2]);
  cal_angle[2]=(cos_angle[2]>0.99?0.99:(cos_angle[2]<-0.99)?-0,99:acos(cos_angle[2]));
ans[3]=1.57-cal_angle[2];//cal_angle-90+180
slide[3]=sqrt(angle[0]*angle[0]+angle[3]*angle[3]-2*angle[0]*angle[3]*cos(0.349));
  cos_angle[3]=(angle[0]*angle[0]+slide[3]*slide[3]-angle[3]*angle[3])/(2*angle[0]*slide[3]);
  cal_angle[3]=(cos_angle[3]>0.99?0.99:(cos_angle[3]<-0.99)?-0,99:acos(cos_angle[3]));
ans[4]=1.57-cal_angle[3];//cal_angle-90+180
//ROS_INFO_STREAM("cal1="<<ans[1]<<"ans2="<<ans[2]<<"ans3="<<ans[3]<<"ans4="<<ans[4]);
ans[0]=(ans[1]+ans[2]+ans[3]+ans[4])/4;
//diff_x=(angle[0]+offset_front_x*tan(3.1416-ans[0]))*cos(3.1416-ans[0])-0.5;//距侧面
diff_y=(angle[0]+offset_front_x*tan(ans[0]))*cos(ans[0])-0.6;
diff_x=(laser+offset_back_x)*cos(ans[0])-0.4;//距底面,单位m
//ROS_INFO_STREAM("diff_x"<<diff_x<<"diff_y"<<diff_y);
}
void posCallback(const geometry_msgs::Twist &pos)
{
laser=pos.angular.x;//测距信息
//pose.linear.y=pos.linear.y;
pose.angular.z=pos.angular.z;//角速度信息
}
void adjust_Callback(const std_msgs::Int16 &sign)
{
adjust_sign=1;
num=sign.data;
}
void vel_Callback(const geometry_msgs::Twist &vel)
{
  x_vel=vel.linear.x;
  y_vel=vel.linear.y;
}
int main(int argc,char** argv)
{
ros::init(argc,argv,"add_bufflet");
	ros::NodeHandle n;
angle.push_back(0);
angle.push_back(0);
angle.push_back(0);
ros::Subscriber sub_scan = n.subscribe("scan",1000,sendCallback);//位置？
ros::Publisher pub_vel=n.advertise<geometry_msgs::Twist>("cmd_vel1",1);
ros::Subscriber laser_scan = n.subscribe("uwb_msg",1000,posCallback);
ros::Subscriber adjust_sub = n.subscribe("adjust",1000,adjust_Callback);
ros::Subscriber vel_sub = n.subscribe("raw_vel",1000,vel_Callback);
ros::Rate rate(10);
while(ros::ok())
{
//ROS_INFO_STREAM("laser="<<laser<<"y"<<pose.linear.y<<"angle"<<ans[0]);
//ROS_INFO_STREAM("diff_x"<<diff_x<<"diff_y"<<diff_y<<"angle"<<ans[0]);
//ROS_INFO_STREAM("1="<<ans[0]<<"2="<<ans[1]<<"3="<<ans[2]<<"4="<<ans[3]<<"5="<<ans[4]);
if (adjust_sign)
{
//调整pid
diff_angle=ans[0];
diff_i_angle+=diff_angle*0.01;
diff_i_angle=limit(diff_i_angle,1,-1);
//out_v_angle=diff_angle*3+diff_i_angle*10;
out_v_angle=diff_angle*3;
cmd_pose.angular.z=limit(out_v_angle,1.0,-1.0);//方向相反
//double diff_y=angle[0]*cos(ans[0])-60;
diff_i_y+=diff_y*0.01;
diff_i_y=limit(diff_i_y,1,-1);
//out_v_y=diff_y*3+diff_i_y*10;
out_v_y=diff_y*3+0.4*y_vel;
cmd_pose.linear.y=-limit(out_v_y,1,-1);
//double diff_x=laser-50;
diff_i_x+=diff_x*0.01;
//diff_i_x=limit(diff_i_x,1,-1);
//out_v_x=diff_x*3+diff_i_x*10;
out_v_x=diff_x*3+0.4*x_vel;
cmd_pose.linear.x=-limit(out_v_x,1,-1);
//if(abs(diff_x)<5&&abs(diff_y)<5&&abs(cmd_pose.linear.x)<0.5&&abs(cmd_pose.linear.y)<0.5&&adjust_sign==1&&abs(diff_angle)<0.2)//保证发布补弹信息只有1次
ROS_INFO_STREAM("out_v_x="<<cmd_pose.linear.x<<"out_v_y"<<cmd_pose.linear.y<<"cmd_pose.angular.z="<<cmd_pose.angular.z<<"adjust_sign"<<adjust_sign);
ROS_INFO_STREAM("diff_x="<<diff_x<<"diff_y"<<diff_y);
//if(adjust_sign>0&&myabs(diff_angle)<0.1&&myabs(cmd_pose.angular.z)>0&&myabs(cmd_pose.angular.z)<0.5&&myabs(diff_x)<0.05&&myabs(diff_y)<0.05)
if(adjust_sign>0&&myabs(diff_angle<0.1)&&(diff_y<0.05)&&diff_y>-0.05&&(diff_x<0.0)&&(diff_x>-0.05))
{
ROS_INFO_STREAM("OK!!!!!!!!!!!!!!");
//
//adjust_sign=2;//为2后不再发布补弹真
adjust_sign++;
}
cmd_pose.angular.x=0;//补弹假
cmd_pose.angular.y=0;
pub_vel.publish(cmd_pose);

if(adjust_sign==4)
{
  ROS_INFO_STREAM("finish");
cmd_pose.angular.x=1;//第四位是补弹指令
cmd_pose.angular.z=0;//发送旋转标志为0
cmd_pose.linear.x=0;//发送旋转标志为0
cmd_pose.linear.y=0;//发送旋转标志为0
cmd_pose.angular.y=num;
adjust_sign=0;//每次发布补弹后再发布一次，还原补弹标志位
pub_vel.publish(cmd_pose);
cmd_pose.angular.x=0;//补弹假
cmd_pose.angular.y=0;
//pub_vel.publish(cmd_pose);
}
}
ros::spinOnce();
rate.sleep();
}
return 0;
}


