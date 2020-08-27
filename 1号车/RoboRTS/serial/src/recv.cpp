#include "recv.h"

namespace serial {
  Usart_serial::Usart_serial() {
  Init();
}
 struct vec2s
{
	int16_t a[2];
};
union float2short
{
	vec2s buffer;
	float a;
};
void Usart_serial::Init() 
{
  chatter_pub = nh_.advertise<geometry_msgs::Twist>("raw_msg", 1000);      
  vision_pub = nh_.advertise<geometry_msgs::Twist>("vision_tonuc", 1); 
  uwb_pub = nh_.advertise<geometry_msgs::Twist>("uwb_msg", 1000);              
  referee_pub = nh_.advertise<std_msgs::Float64MultiArray>("referee_system", 1000); 
  usart_sub = nh_.subscribe("usart_send", 1, &Usart_serial::usart_callback, this);
  communicate_pub=nh_.advertise<std_msgs::Float64MultiArray>("RefereeToDecision", 10);
  odom_position_pub = nh_.advertise<geometry_msgs::PoseStamped>("odom_position",2);
  area_refereeto_pub = nh_.advertise<std_msgs::Float64MultiArray>("area_status_msg", 1000);
  robort_status_pub = nh_.advertise<std_msgs::Float64MultiArray>("robort_status_msg", 1000);
}

void Usart_serial::usart_callback(const std_msgs::Int16MultiArray::ConstPtr &usart_msg) {
  //ROS_INFO_STREAM("usart_msg[0]"<<usart_msg->data[0]);
  switch (usart_msg->data[0])
  {
    case 252:
      area_status.data.resize(6);
      robort_status.data.resize(4);
      area_status.data[0] = usart_msg->data[1];//area_one_status
      area_status.data[1] = usart_msg->data[2];//area_two_status
      area_status.data[2] = usart_msg->data[3];//area_three_status
      area_status.data[3] = usart_msg->data[4];//area_four_status
      area_status.data[4] = usart_msg->data[5];//area_five_status
      area_status.data[5] = usart_msg->data[6];//area_six_status
      area_refereeto_pub.publish(area_status);
      
      robort_status.data[0] = usart_msg->data[7];//my_1car_status
      robort_status.data[1] = usart_msg->data[8];//my_2car_status
      robort_status.data[2] = usart_msg->data[9];//enemy_1car_status
      robort_status.data[3] = usart_msg->data[10];//enemy_2car_status
      robort_status_pub.publish(robort_status);
    break;
    case 253:
      float2short read_usart;
      read_usart.buffer.a[0] = usart_msg->data[0];//odom_position.x
      read_usart.buffer.a[1] = usart_msg->data[1];
      odom_position.pose.position.x = read_usart.a;
      read_usart.buffer.a[0] = usart_msg->data[2];//odom_position.y
      read_usart.buffer.a[1] = usart_msg->data[3];
      odom_position.pose.position.y = read_usart.a;

      //ROS_ERRORSTREAM("odom_x: "<<odom_position.pose.position.x<<"  odom_y: "<<odom_position.pose.position.y);

      odom_position_pub.publish(odom_position);
    break;
    case 254:
      mcu_msgs.linear.x=-usart_msg->data[1]/1000.0;//Class_Usart::a,正方向和最开始反了
      mcu_msgs.linear.y=-usart_msg->data[2]/1000.0;
      mcu_msgs.angular.x=-usart_msg->data[3]/100.0;

      //ROS_ERROR_STREAM("theta: "<<mcu_msgs.angular.x);
      
      //角速度反馈偏小，加大系数
      mcu_msgs.angular.z=-usart_msg->data[4]/100.0;
      chatter_pub.publish(mcu_msgs);

      uwb_msgs.linear.x=usart_msg->data[5]/1000.0;
      uwb_msgs.linear.y=usart_msg->data[6]/1000.0;
      uwb_msgs.angular.z=usart_msg->data[7]/100.0;		
    break;
    case 255:
      vision_msgs.linear.x=usart_msg->data[1]/100.0;//Class_Usart::a
      vision_msgs.linear.y=usart_msg->data[2]/100.0;
      vision_msgs.angular.z=usart_msg->data[3];
      vision_msgs.angular.x=usart_msg->data[4]/100.0;
      vision_pub.publish(vision_msgs);
    break;
    case 170: 
      referee.data.resize(8);
      uwb_msgs.angular.x=usart_msg->data[4]/1000.0;//laser,m
      uwb_pub.publish(uwb_msgs);
      //ROS_INFO("%f,%f,%f;\n",uwb_msgs.linear.x,uwb_msgs.linear.y,uwb_msgs.angular.z);
      referee.data[0]=usart_msg->data[5];//射击子弹数量
      referee.data[1]=usart_msg->data[6];//剩余血量
      referee.data[2]=usart_msg->data[7];//比赛进程位
      referee.data[3]=usart_msg->data[8];//受伤装甲板ID
      referee.data[4]=usart_msg->data[9];//当前拥有的buff
      referee.data[5]=usart_msg->data[1];//敌方1号车存活/死亡
      referee.data[6]=usart_msg->data[2];//敌方2号车存活/死亡
      referee.data[7]=usart_msg->data[3];//jiacheng
      /*std::cout<<"MY_HP 					:    "<< referee.data[1]<<std::endl;
      std::cout<<"剩余血量 				:    "<< referee.data[2]<<std::endl;
      std::cout<<"比赛进程位				:    "<< referee.data[3]<<std::endl;
      std::cout<<"受伤装甲板ID 			:    "<< referee.data[4]<<std::endl;
      std::cout<<"当前拥有的buff			:    "<< referee.data[5]<<std::endl;
      std::cout<<"敌方2号车存活/死亡 	:    "<< referee.data[6]<<std::endl;
      std::cout<<"jiacheng 				:    "<< referee.data[7]<<std::endl;*/
      referee_pub.publish(referee);
    break;
    case 187:
      conmunicate.data.resize(8);
      conmunicate.data[0]=usart_msg->data[1];
      conmunicate.data[0]=conmunicate.data[0]/100;//友方
      conmunicate.data[1]=usart_msg->data[2];
      conmunicate.data[1]=conmunicate.data[1]/100;//友方
      conmunicate.data[2]=usart_msg->data[3];
      conmunicate.data[2]=conmunicate.data[2]/100;//敌方
      conmunicate.data[3]=usart_msg->data[4];
      conmunicate.data[3]=conmunicate.data[3]/100;//敌方
      conmunicate.data[4]=usart_msg->data[6];//剩余子弹
      conmunicate.data[5]=usart_msg->data[8];//youfangshengyuxueliang
      conmunicate.data[6]=usart_msg->data[5];//juecewanchengbiaozhiwei 
      conmunicate.data[7]=usart_msg->data[7];//budan jiachengqu
      //ROS_WARN_STREAM(referee.data[0]<<"%%%%%%%%%%%"<<usart_msg->data[1]);
      communicate_pub.publish(conmunicate);	
    break;
  }
}
}
int main(int argc,char** argv)
{
	ros::init(argc,argv,"serial_recv");
	serial::Usart_serial usart_serial;//不要加括号，它是个变量，不是个函数！！！！！！！！！！！
	ROS_INFO_STREAM("usart_msg[4]");
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();
	ros::waitForShutdown();
}
