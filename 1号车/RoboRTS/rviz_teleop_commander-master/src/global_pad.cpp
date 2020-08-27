#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <geometry_msgs/Twist.h>
#include <QDebug>

#include "global_pad.h"

namespace rviz_teleop_commander
{

// 构造函数，初始化变量
GlobalPanel::GlobalPanel( QWidget* parent )
  : rviz::Panel( parent )
  , Global_pose( 0 )
  , Global_stuck( 0 )
  , Global_goal( 0 )
  , Global_plan( 0 )
  , Global_time( 0 )
{
  // 创建一个输入topic命名的窗口
//   QVBoxLayout* topic_layout = new QVBoxLayout;
//   topic_layout->addWidget( new QLabel( "Teleop Topic:" ));
//   output_topic_editor_ = new QLineEdit;
//   topic_layout->addWidget( output_topic_editor_ );
// 
//   // 创建一个输入线速度的窗口
//   topic_layout->addWidget( new QLabel( "Linear Velocity:" ));
//   output_topic_editor_1 = new QLineEdit;
//   topic_layout->addWidget( output_topic_editor_1 );
// 
//   // 创建一个输入角速度的窗口
//   topic_layout->addWidget( new QLabel( "Angular Velocity:" ));
//   output_topic_editor_2 = new QLineEdit;
//   topic_layout->addWidget( output_topic_editor_2 ,tmp_counter);
//   
// 
//    // 创建一个输入自定义的窗口
//   topic_layout->addWidget( new QLabel( "Show:" ));
//   //qlabel = new QLabel();
//   qlabel = new QLineEdit;
//   qlabel->setText("Show");
//   topic_layout->addWidget(qlabel);
//   
//   
//   topic_layout->addWidget( new QLabel( "Test:" ));
//   //qlabel = new QLabel();
//   qlabel_1 = new QLineEdit;
//   qlabel_1->setText("Show");
//   topic_layout->addWidget(qlabel_1);
//   
//   QHBoxLayout* layout = new QHBoxLayout;
//   layout->addLayout( topic_layout );
//   setLayout( layout );
  /**************************************************Global_Topic***************************************************/
  QVBoxLayout* Global_Topic = new QVBoxLayout;
  Global_Topic->addWidget( new QLabel( "Global_Topic" ));
  
  
  
  
  Global_Topic->addWidget( new QLabel( "Get Robot Pose:" ));
  Global_pose_label = new QLineEdit;
  Global_pose_label->setText("NORMAL");//ERROR
  Global_Topic->addWidget(Global_pose_label);
  
  
  Global_Topic->addWidget( new QLabel( "Get Global State:" ));
  Global_stuck_label = new QLineEdit;
  Global_stuck_label->setText("NORMAL");//STUCK
  Global_Topic->addWidget(Global_stuck_label);
  
  
  Global_Topic->addWidget( new QLabel( "Get Global Goal:" ));
  Global_goal_label = new QLineEdit;
  Global_goal_label->setText("NORMAL");
  Global_Topic->addWidget(Global_goal_label);
  
  
  Global_Topic->addWidget( new QLabel( "Global Plan:" ));
  Global_plan_label = new QLineEdit;
  Global_plan_label->setText("NORMAL");//ERROR
  Global_Topic->addWidget(Global_plan_label);
  
  
  Global_Topic->addWidget( new QLabel( "Get Plan Time:" ));
  Global_time_label = new QLineEdit;
  Global_time_label->setText("NORMAL");//Beyond Expected
  Global_Topic->addWidget(Global_time_label);
  
  
  
  QHBoxLayout* global_layout = new QHBoxLayout;
  global_layout->addLayout( Global_Topic );
  setLayout( global_layout );
  /*****************************************************************************************************************/
  // 创建一个定时器，用来定时发布消息
  QTimer* output_timer = new QTimer( this );
  
  //Global
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( Global_Subscribe() ));
  
  
  connect( Global_pose_label, SIGNAL(update_Global()), this, SLOT(update_Show()));
  connect( Global_stuck_label, SIGNAL(update_Global()), this, SLOT(update_Show()));
  connect( Global_goal_label, SIGNAL(update_Global()), this, SLOT(update_Show()));
  connect( Global_plan_label, SIGNAL(update_Global()), this, SLOT(update_Show()));
  connect( Global_time_label, SIGNAL(update_Global()), this, SLOT(update_Show()));
  
  // 设置定时器的周期，100ms
  output_timer->start( 100 );
}

void GlobalPanel::update_Show()
{}
/**************************************************Global_Topic***************************************************/

void GlobalPanel::Global_Subscribe()
{
  GlobalPanel::Global_sub = nh_.subscribe("/planthread", 1000, &GlobalPanel::Global_chatterCallback ,this);
  Q_EMIT update_Global();
}

void GlobalPanel::Global_chatterCallback(const geometry_msgs::PoseStamped &msg)
{
  GlobalPanel::Global_pose = msg.pose.position.x;
  GlobalPanel::Global_stuck = msg.pose.position.y;
  GlobalPanel::Global_goal = msg.pose.position.z;
  GlobalPanel::Global_plan = msg.pose.orientation.x;
  GlobalPanel::Global_time = msg.pose.orientation.y;
  
  if(GlobalPanel::Global_pose == 1)
    Global_pose_label->setText("NORMAL");
  else
    Global_pose_label->setText("ERROR");
  
  if(GlobalPanel::Global_stuck == 1)
    Global_stuck_label->setText("NORMAL");
  else
    Global_stuck_label->setText("STUCK");
  
  if(GlobalPanel::Global_goal == 1)
    Global_goal_label->setText("NORMAL");
  else
    Global_goal_label->setText("Current Goal Not Valid");
  
  if(GlobalPanel::Global_plan == 1)
    Global_plan_label->setText("NORMAL");
  else
    Global_plan_label->setText("ERROR");
  
  if(GlobalPanel::Global_time == 1)
    Global_time_label->setText("NORMAL");
  else
    Global_time_label->setText("Beyond Expected");
}
/*****************************************************************************************************************/
} // end namespace rviz_teleop_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_teleop_commander::GlobalPanel,rviz::Panel )
// END_TUTORIAL
