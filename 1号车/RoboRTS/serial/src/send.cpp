#include<ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
int main(int argc, char** argv)
{
ros::init(argc, argv, "dock_server");

    ros::NodeHandle n;
      std_msgs::Float64MultiArray conmunicate;
      ros::Publisher referee_pub = n.advertise<std_msgs::Float64MultiArray>("DecisionToReferee", 1);
ros::Rate r(1);
conmunicate.data.resize(8);
	    conmunicate.data[0]=1.1;
	    conmunicate.data[1]=2.2;
	    conmunicate.data[2]=3.3;
	    conmunicate.data[3]=4.4;
	    conmunicate.data[4]=0.6;
	    conmunicate.data[5]=10;
	    conmunicate.data[6]=10;
	    conmunicate.data[7]=2000;
	    
while(ros::ok())
{
	    //conmunicate.data[7]=1;
			//ROS_INFO("%f,%f,%f,%f,%f,\n",referee.data[0],referee.data[1],referee.data[2],referee.data[3],referee.data[4]);
ROS_INFO_STREAM("1");
	referee_pub.publish(conmunicate);
ros::spinOnce();
r.sleep();
}	
return 0;
}
