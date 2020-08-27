#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
double old_x;
double old_y;
double yaw;
double pitch;
double roll;
int main(int argc, char** argv)
{
ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle node;
 tf::TransformListener listener;//新建监听对象
tf::TransformBroadcaster odom_broadcaster;
 geometry_msgs::TransformStamped odom_trans;
odom_trans.header.frame_id = "map";
odom_trans.child_frame_id = "odom";
ros::Time current_time_now = ros::Time::now();

  ros::Rate rate(1.0);
tf::StampedTransform transform;//定义存放转换信息（平动，转动）的变量
 while (node.ok()){
odom_trans.header.stamp = current_time_now;
try{
      listener.lookupTransform("/map", "/odom",
                               ros::Time(0), transform);//可以获得两个坐标系之间转换的关系，包括旋转与平移。转换得出的坐标是在“/turtle2”坐标系下的 
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
transform.getBasis().getEulerYPR(yaw, pitch, roll);
ROS_INFO_STREAM("current_time_now"<<current_time_now<<"x"<<transform.getOrigin().x()<<"y"<<transform.getOrigin().y()<<"yaw"<<yaw);

geometry_msgs::Quaternion map_quat = tf::createQuaternionMsgFromYaw(yaw);
//if((transform.getOrigin().x()-old_x)>3&&(transform.getOrigin().y()-old_y)>2)
//{
//ROS_ERROR_STREAM("error!!!!!"<<transform.getOrigin().x()-old_x<<"!!!!"<<transform.getOrigin().y()-old_y);

odom_trans.transform.translation.x =8-transform.getOrigin().x();
odom_trans.transform.translation.y = 5-transform.getOrigin().y();
odom_trans.transform.rotation = map_quat;
odom_broadcaster.sendTransform(odom_trans);
//}
old_x=transform.getOrigin().x();
old_y=transform.getOrigin().y();
 rate.sleep();
  }
  return 0;
}

