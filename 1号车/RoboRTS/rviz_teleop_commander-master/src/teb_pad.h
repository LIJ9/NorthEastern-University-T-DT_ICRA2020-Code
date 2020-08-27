#ifndef TEB_PAD_H
#define TEB_PAD_H


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
class TEBPanel: public rviz::Panel
{
// 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
Q_OBJECT
public:
  // 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
  TEBPanel( QWidget* parent = 0 );
/*Q_OBJECT
public:*/  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
  
  void TEB_chatterCallback(const geometry_msgs::PoseStamped &msg);
  

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////
  Q_SIGNALS:
  
  //TEB
  void update_TEB();
  
  /////////////////////////////////////////////////////////////////////////////////
  
  
  // 内部槽.
protected Q_SLOTS:
  
  void update_Show();
  void TEB_Subscribe();

  /////////////////////////////////////////////////////////////////////////////////
  
  
  // 内部变量.
protected:

  //TEB输入框
  QLineEdit* TEB_initialize_label;
  QLineEdit* TEB_trajectory_label;
  QLineEdit* TEB_oscillating_label;
  QLineEdit* TEB_transform_label;
  QLineEdit* TEB_empty_label;
  QLineEdit* TEB_optimal_label;
  QLineEdit* TEB_velocity_label;
  //TEB Parameter
  ros::Subscriber TEB_sub;
  double TEB_initialize;
  double TEB_trajectory;
  double TEB_oscillating;
  double TEB_transform;
  double TEB_empty;
  double TEB_optimal;
  double TEB_velocity;

  // The ROS node handle.
  ros::NodeHandle nh_;
};

} // end namespace rviz_teleop_commander

#endif // TELEOP_PANEL_H
