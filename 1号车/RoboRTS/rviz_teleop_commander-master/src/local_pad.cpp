#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <geometry_msgs/Twist.h>
#include <QDebug>

#include "local_pad.h"

namespace rviz_teleop_commander
{

// 构造函数，初始化变量
LocalPanel::LocalPanel( QWidget* parent )
  : rviz::Panel( parent )
  , Local_initialize( 0 )
  , Local_stuck( 0 )
  , Local_plan( 0 )
  , Local_cmd_vel( 0 )
{
  /**************************************************Local_Topic****************************************************/
  QVBoxLayout* Local_Topic = new QVBoxLayout;
  Local_Topic->addWidget( new QLabel( "Local_Topic" ));
  
  
  
  
  Local_Topic->addWidget( new QLabel( "Local Initial:" ));
  Local_initialize_label = new QLineEdit;
  Local_initialize_label->setText("NORMAL");//ERROR
  Local_Topic->addWidget(Local_initialize_label);
  
  
  Local_Topic->addWidget( new QLabel( "Stuck Vel:" ));
  Local_stuck_label = new QLineEdit;
  Local_stuck_label->setText("NORMAL");//STUCK
  Local_Topic->addWidget(Local_stuck_label);
  
  
  Local_Topic->addWidget( new QLabel( "Local Plan:" ));
  Local_plan_label = new QLineEdit;
  Local_plan_label->setText("NORMAL");//Can not Finish Plan
  Local_Topic->addWidget(Local_plan_label);
  
  
  Local_Topic->addWidget( new QLabel( "Get cmd_vel:" ));
  Local_cmd_vel_label = new QLineEdit;
  Local_cmd_vel_label->setText("NORMAL");//Can't Get cmd_vel
  Local_Topic->addWidget(Local_cmd_vel_label);
  
  
  
  QHBoxLayout* Local_layout = new QHBoxLayout;
  Local_layout->addLayout( Local_Topic );
  setLayout( Local_layout );
  /*****************************************************************************************************************/
  // 创建一个定时器，用来定时发布消息
  QTimer* output_timer = new QTimer( this );
  
  //Local
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( Local_Subscribe() ));
  
  //Local
  connect( Local_initialize_label, SIGNAL(update_Local()), this, SLOT(update_Show()));
  connect( Local_stuck_label, SIGNAL(update_Local()), this, SLOT(update_Show()));
  connect( Local_plan_label, SIGNAL(update_Local()), this, SLOT(update_Show()));
  connect( Local_cmd_vel_label, SIGNAL(update_Local()), this, SLOT(update_Show()));
  
  // 设置定时器的周期，100ms
  output_timer->start( 100 );
}

void LocalPanel::update_Show()
{}
/**************************************************Local_Topic****************************************************/

void LocalPanel::Local_Subscribe()
{
  LocalPanel::Local_sub = nh_.subscribe("/LOOP_info", 1000, &LocalPanel::Local_chatterCallback ,this);
  Q_EMIT update_Local();
}

void LocalPanel::Local_chatterCallback(const geometry_msgs::PoseStamped &msg)
{
  LocalPanel::Local_initialize = msg.pose.position.x;
  LocalPanel::Local_stuck = msg.pose.position.y;
  LocalPanel::Local_plan = msg.pose.position.z;
  LocalPanel::Local_cmd_vel = msg.pose.orientation.x;
  
  if(LocalPanel::Local_initialize == 1)
    Local_initialize_label->setText("NORMAL");
  else
    Local_initialize_label->setText("ERROR");
  
  if(LocalPanel::Local_stuck == 1)
    Local_stuck_label->setText("NORMAL");
  else
    Local_stuck_label->setText("STUCK");
  
  if(LocalPanel::Local_plan == 1)
    Local_plan_label->setText("NORMAL");
  else
    Local_plan_label->setText("Can't Finish Plan");
  
  if(LocalPanel::Local_cmd_vel == 1)
    Local_cmd_vel_label->setText("NORMAL");
  else
    Local_cmd_vel_label->setText("Can't Get cmd_vel");
}
/*****************************************************************************************************************/
} // end namespace rviz_teleop_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_teleop_commander::LocalPanel,rviz::Panel )
// END_TUTORIAL
