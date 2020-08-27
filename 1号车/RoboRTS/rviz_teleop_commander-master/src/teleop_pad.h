#ifndef TELEOP_PAD_H
#define TELEOP_PAD_H


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
class TeleopPanel: public rviz::Panel
{
// 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
Q_OBJECT
public:
  // 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
  TeleopPanel( QWidget* parent = 0 );

  // 重载rviz::Panel积累中的函数，用于保存、加载配置文件中的数据，在我们这个plugin
  // 中，数据就是topic的名称
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;


/*Q_OBJECT
public:*/  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void chatterCallback(const std_msgs::String::ConstPtr& msg);
  
  
  
  void Global_chatterCallback(const geometry_msgs::PoseStamped &msg);
  
  void AStar_chatterCallback(const geometry_msgs::PoseStamped &msg);
  
  void Local_chatterCallback(const geometry_msgs::PoseStamped &msg);
  
  void TEB_chatterCallback(const geometry_msgs::PoseStamped &msg);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  
  
  // 公共槽.
public Q_SLOTS:
  // 当用户输入topic的命名并按下回车后，回调用此槽来创建一个相应名称的topic publisher
  void setTopic( const QString& topic );

  
  /////////////////////////////////////////////////////////////////////////////////
  Q_SIGNALS:
  void update_identity_show();
  
  void update_topic_test();
  
  
  //Global
  void update_Global();
  //AStar
  void update_AStar();
  //Local
  void update_Local();
  //TEB
  void update_TEB();  
  /////////////////////////////////////////////////////////////////////////////////
  
  
  // 内部槽.
protected Q_SLOTS:
  void sendVel();                 // 发布当前的速度值
  void update_Linear_Velocity();  // 根据用户的输入更新线速度值
  void update_Angular_Velocity(); // 根据用户的输入更新角速度值
  void updateTopic();             // 根据用户的输入更新topic name
  
  
  /////////////////////////////////////////////////////////////////////////////////
  void update_Show();
  
  void topic_update_Show();
  
  void update_identity_show_zhao();
  
  void chatterCallback_carry();
  
  
  
  
  void Global_Subscribe();
  
  void AStar_Subscribe();
  
  void Local_Subscribe();
  
  void TEB_Subscribe();
  
  
  
  


  
  /////////////////////////////////////////////////////////////////////////////////
  
  
  // 内部变量.
protected:
  // topic name输入框
  QLineEdit* output_topic_editor_;
  QString output_topic_;
  
  // 线速度值输入框
  QLineEdit* output_topic_editor_1;
  QString output_topic_1;
  
  // 角速度值输入框
  QLineEdit* output_topic_editor_2;
  QString output_topic_2;
  
  // 时间输入框
  //QLabel* qlabel;
  QLineEdit* qlabel;
  
  QLineEdit* qlabel_1;
    QString output_topic_3 = "AHH";
  int tmp_counter = 0;
  std::string topic_msg;
  
  //Global输入框
  QLineEdit* Global_pose_label;
  QLineEdit* Global_stuck_label;
  QLineEdit* Global_goal_label;
  QLineEdit* Global_plan_label;
  QLineEdit* Global_time_label;
  
  
  //AStar输入框
  QLineEdit* AStar_GPtransform_label;
  QLineEdit* AStar_Maptransform_label;
  QLineEdit* AStar_goal_label;
  
  
  
  //Local输入框
  QLineEdit* Local_initialize_label;
  QLineEdit* Local_stuck_label;
  QLineEdit* Local_plan_label;
  QLineEdit* Local_cmd_vel_label;
  

  
  //TEB输入框
  QLineEdit* TEB_initialize_label;
  QLineEdit* TEB_trajectory_label;
  QLineEdit* TEB_oscillating_label;
  QLineEdit* TEB_transform_label;
  QLineEdit* TEB_empty_label;
  QLineEdit* TEB_optimal_label;
  QLineEdit* TEB_velocity_label;

  
  //Global Parameter
  ros::Subscriber Global_sub;
  double Global_pose;
  double Global_stuck;
  double Global_goal;
  double Global_plan;
  double Global_time;
  
  
  //AStar Parameter
  ros::Subscriber AStar_sub;
  double AStar_GPtransform;
  double AStar_Maptransform;
  double AStar_goal;
  
  
  //Local Parameter
  ros::Subscriber Local_sub;
  double Local_initialize;
  double Local_stuck;
  double Local_plan;
  double Local_cmd_vel;
  
  
  //TEB Parameter
  ros::Subscriber TEB_sub;
  double TEB_initialize;
  double TEB_trajectory;
  double TEB_oscillating;
  double TEB_transform;
  double TEB_empty;
  double TEB_optimal;
  double TEB_velocity;
  
  
  
  
  // ROS的publisher，用来发布速度topic
  ros::Publisher velocity_publisher_;

  // The ROS node handle.
  ros::NodeHandle nh_;
  
  ros::Subscriber sub;
  
  
  
  

  // 当前保存的线速度和角速度值
  float linear_velocity_;
  float angular_velocity_;
};

} // end namespace rviz_teleop_commander

#endif // TELEOP_PANEL_H
