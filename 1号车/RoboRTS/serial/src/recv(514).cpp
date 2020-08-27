#include "recv.h"

namespace serial {
  Usart_serial::Usart_serial() {
  Init();
}

void Usart_serial::Init() {
  chatter_pub = nh_.advertise<geometry_msgs::Twist>("raw_msg", 1000);      
  vision_pub = nh_.advertise<geometry_msgs::Twist>("vision_tonuc", 1); 
  uwb_pub = nh_.advertise<geometry_msgs::Twist>("uwb_msg", 1000);              
  referee_pub = nh_.advertise<std_msgs::Float64MultiArray>("referee_system", 1000); 
  usart_sub = nh_.subscribe("usart_send", 1, &Usart_serial::usart_callback, this);
  communicate_pub=nh_.advertise<std_msgs::Float64MultiArray>("RefereeToDecision", 10);  
   
}
void Usart_serial::usart_callback(const std_msgs::Int16MultiArray::ConstPtr &usart_msg) {
  //ROS_INFO_STREAM("usart_msg[0]"<<usart_msg->data[0]);
  switch (usart_msg->data[0])
  {
    case 254:
      mcu_msgs.linear.x=-usart_msg->data[1]/1000.0;//Class_Usart::a,正方向和最开始反了
      mcu_msgs.linear.y=-usart_msg->data[2]/1000.0;
      mcu_msgs.angular.x=-usart_msg->data[3]/100.0*1.1;//角速度反馈偏小，加大系数
      mcu_msgs.angular.z=-usart_msg->data[4]/100.0;
			chatter_pub.publish(mcu_msgs);
      break;
     case 255:
       vision_msgs.linear.x=usart_msg->data[1]/100.0;//Class_Usart::a
 			vision_msgs.linear.y=usart_msg->data[2]/100.0;
 			vision_msgs.angular.z=usart_msg->data[3];
 			vision_msgs.angular.x=usart_msg->data[4]/100.0;
 			vision_pub.publish(vision_msgs);
       break;
    case 170:
            
	    referee.data.resize(5);
      uwb_msgs.linear.x=usart_msg->data[1]/1000.0;
			uwb_msgs.linear.y=usart_msg->data[2]/1000.0;
			uwb_msgs.angular.z=usart_msg->data[3]/100.0;
			uwb_msgs.angular.x=usart_msg->data[4]/1000.0;//laser,m
			uwb_pub.publish(uwb_msgs);
			//ROS_INFO("%f,%f,%f;\n",uwb_msgs.linear.x,uwb_msgs.linear.y,uwb_msgs.angular.z);
			referee.data[0]=usart_msg->data[5];//射击子弹数量
			referee.data[1]=usart_msg->data[6];//剩余血量
			referee.data[2]=usart_msg->data[7];//比赛进程位
			referee.data[3]=usart_msg->data[8];//受伤装甲板ID
			referee.data[4]=usart_msg->data[9];//当前拥有的buff
			referee_pub.publish(referee);
			break;
		case 187:
		
	    conmunicate.data.resize(8);
	    conmunicate.data[0]=usart_msg->data[1];
	    conmunicate.data[0]=conmunicate.data[0]/100;
	    conmunicate.data[1]=usart_msg->data[2]/100;
	    conmunicate.data[1]=conmunicate.data[1]/100;
	    conmunicate.data[2]=usart_msg->data[3]/100;
	    conmunicate.data[2]=conmunicate.data[2]/100;
	    conmunicate.data[3]=usart_msg->data[4]/100;
	    conmunicate.data[3]=conmunicate.data[3]/100;
	    conmunicate.data[4]=usart_msg->data[6]/100;
	    conmunicate.data[5]=usart_msg->data[5];
	    conmunicate.data[6]=usart_msg->data[7];
	    conmunicate.data[7]=usart_msg->data[8];
			//ROS_INFO("%f,%f,%f,%f,%f;\n",referee.data[0],referee.data[1],referee.data[2],referee.data[3],referee.data[4]);
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
