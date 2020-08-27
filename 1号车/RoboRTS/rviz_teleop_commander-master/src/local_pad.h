#ifndef LOCAL_PAD_H
#define LOCAL_PAD_H


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
class LocalPanel: public rviz::Panel
{
// 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
Q_OBJECT
public:
  // 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
  LocalPanel( QWidget* parent = 0 );
/*Q_OBJECT
public:*/  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
  
  void Local_chatterCallback(const geometry_msgs::PoseStamped &msg);
  

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  Q_SIGNALS:
  
  //Local
  void update_Local();
  
  /////////////////////////////////////////////////////////////////////////////////
  
  
  // 内部槽.
protected Q_SLOTS:
  
  void update_Show();
  void Local_Subscribe();

  /////////////////////////////////////////////////////////////////////////////////
  
  
  // 内部变量.
protected:

  //Local输入框
  QLineEdit* Local_initialize_label;
  QLineEdit* Local_stuck_label;
  QLineEdit* Local_plan_label;
  QLineEdit* Local_cmd_vel_label;

  //Local Parameter
  ros::Subscriber Local_sub;
  double Local_initialize;
  double Local_stuck;
  double Local_plan;
  double Local_cmd_vel;

  // The ROS node handle.
  ros::NodeHandle nh_;
};

} // end namespace rviz_teleop_commander

#endif // TELEOP_PANEL_H
