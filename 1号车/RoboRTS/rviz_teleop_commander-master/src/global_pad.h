#ifndef GLOBAL_PAD_H
#define GLOBAL_PAD_H


#include <QLabel>

//所需要包含的头文件
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin基类的头文件


//////////////////////////////////////////////////////////////
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
//////////////////////////////////////////////////////////////

class QLineEdit;
namespace rviz_teleop_commander
{
// 所有的plugin都必须是rviz::Panel的子类
class GlobalPanel: public rviz::Panel
{
// 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
Q_OBJECT
public:
  // 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
  GlobalPanel( QWidget* parent = 0 );
/*Q_OBJECT
public:*/  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
  
  void Global_chatterCallback(const geometry_msgs::PoseStamped &msg);
  

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  Q_SIGNALS:
  
  //Global
  void update_Global();
  
  /////////////////////////////////////////////////////////////////////////////////
  
  
  // 内部槽.
protected Q_SLOTS:
  
  void update_Show();
  void Global_Subscribe();

  /////////////////////////////////////////////////////////////////////////////////
  
  
  // 内部变量.
protected:

  //Global输入框
  QLineEdit* Global_pose_label;
  QLineEdit* Global_stuck_label;
  QLineEdit* Global_goal_label;
  QLineEdit* Global_plan_label;
  QLineEdit* Global_time_label;

  //Global Parameter
  ros::Subscriber Global_sub;
  double Global_pose;
  double Global_stuck;
  double Global_goal;
  double Global_plan;
  double Global_time;

  // The ROS node handle.
  ros::NodeHandle nh_;
};

} // end namespace rviz_teleop_commander

#endif // TELEOP_PANEL_H
