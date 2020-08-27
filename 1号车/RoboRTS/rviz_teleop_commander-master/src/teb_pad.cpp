#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <geometry_msgs/Twist.h>
#include <QDebug>

#include "teb_pad.h"

namespace rviz_teleop_commander
{

// 构造函数，初始化变量
TEBPanel::TEBPanel( QWidget* parent )
  : rviz::Panel( parent )
  , TEB_initialize( 0 )
  , TEB_trajectory( 0 )
  , TEB_oscillating( 0 )
  , TEB_transform( 0 )
  , TEB_empty( 0 )
  , TEB_optimal( 0 )
  , TEB_velocity( 0 )
{
  /**************************************************TEB_Topic******************************************************/
  QVBoxLayout* TEB_Topic = new QVBoxLayout;
  TEB_Topic->addWidget( new QLabel( "TEB_Topic" ));
  
  
  
  
  TEB_Topic->addWidget( new QLabel( "TEB Initial:" ));
  TEB_initialize_label = new QLineEdit;
  TEB_initialize_label->setText("NORMAL");//ERROR
  TEB_Topic->addWidget(TEB_initialize_label);
  
  
  TEB_Topic->addWidget( new QLabel( "TEB  Trajectory:" ));
  TEB_trajectory_label = new QLineEdit;
  TEB_trajectory_label->setText("NORMAL");//Not Feasible
  TEB_Topic->addWidget(TEB_trajectory_label);
  
  
  TEB_Topic->addWidget( new QLabel( "TEB State:" ));
  TEB_oscillating_label = new QLineEdit;
  TEB_oscillating_label->setText("NORMAL");//Oscillating
  TEB_Topic->addWidget(TEB_oscillating_label);
  
  
  TEB_Topic->addWidget( new QLabel( "TEB Plan Transform:" ));
  TEB_transform_label = new QLineEdit;
  TEB_transform_label->setText("NORMAL");//ERROR
  TEB_Topic->addWidget(TEB_transform_label);
  
  
  TEB_Topic->addWidget( new QLabel( "TEB Transformed Plan:" ));
  TEB_empty_label = new QLineEdit;
  TEB_empty_label->setText("NORMAL");//EMPTY
  TEB_Topic->addWidget(TEB_empty_label);
  
  
  
  TEB_Topic->addWidget( new QLabel( "TEB Optimal Error:" ));
  TEB_optimal_label = new QLineEdit;
  TEB_optimal_label->setText("NORMAL");//EMPTY
  TEB_Topic->addWidget(TEB_optimal_label);
  
  
  TEB_Topic->addWidget( new QLabel( "TEB Get Velocity:" ));
  TEB_velocity_label = new QLineEdit;
  TEB_velocity_label->setText("NORMAL");//Can't Get Velocity
  TEB_Topic->addWidget(TEB_velocity_label);
  
  
  
  QHBoxLayout* TEB_layout = new QHBoxLayout;
  TEB_layout->addLayout( TEB_Topic );
  setLayout( TEB_layout );
  /*****************************************************************************************************************/
  // 创建一个定时器，用来定时发布消息
  QTimer* output_timer = new QTimer( this );
  
  //TEB
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( TEB_Subscribe() ));
  
  
  //TEB
  connect( TEB_initialize_label, SIGNAL(update_TEB()), this, SLOT(update_Show()));
  connect( TEB_trajectory_label, SIGNAL(update_TEB()), this, SLOT(update_Show()));
  connect( TEB_oscillating_label, SIGNAL(update_TEB()), this, SLOT(update_Show()));
  connect( TEB_transform_label, SIGNAL(update_TEB()), this, SLOT(update_Show()));
  connect( TEB_empty_label, SIGNAL(update_TEB()), this, SLOT(update_Show()));
  connect( TEB_optimal_label, SIGNAL(update_TEB()), this, SLOT(update_Show()));
  connect( TEB_velocity_label, SIGNAL(update_TEB()), this, SLOT(update_Show()));
  
  // 设置定时器的周期，100ms
  output_timer->start( 100 );
}

void TEBPanel::update_Show()
{}
/**************************************************TEB_Topic******************************************************/

void TEBPanel::TEB_Subscribe()
{
  TEBPanel::TEB_sub = nh_.subscribe("/cp_vel_info", 1000, &TEBPanel::TEB_chatterCallback ,this);
  Q_EMIT update_TEB();
}

void TEBPanel::TEB_chatterCallback(const geometry_msgs::PoseStamped &msg)
{
  TEBPanel::TEB_initialize = msg.pose.position.x;
  TEBPanel::TEB_trajectory = msg.pose.position.y;
  TEBPanel::TEB_oscillating = msg.pose.position.z;
  TEBPanel::TEB_transform = msg.pose.orientation.x;
  TEBPanel::TEB_empty = msg.pose.orientation.y;
  TEBPanel::TEB_optimal = msg.pose.orientation.z;
  TEBPanel::TEB_velocity = msg.pose.orientation.w;  
  
  if(TEBPanel::TEB_initialize == 1)
    TEB_initialize_label->setText("NORMAL");
  else
    TEB_initialize_label->setText("ERROR");
  
  if(TEBPanel::TEB_trajectory == 1)
    TEB_trajectory_label->setText("NORMAL");
  else
    TEB_trajectory_label->setText("Not Feasible");
  
  if(TEBPanel::TEB_oscillating == 1)
    TEB_oscillating_label->setText("NORMAL");
  else
    TEB_oscillating_label->setText("Oscillating");
  
  if(TEBPanel::TEB_transform == 1)
    TEB_transform_label->setText("NORMAL");
  else
    TEB_transform_label->setText("ERROR");
  
  if(TEBPanel::TEB_empty == 1)
    TEB_empty_label->setText("NORMAL");
  else
    TEB_empty_label->setText("EMPTY");
  
  if(TEBPanel::TEB_optimal == 1)
    TEB_optimal_label->setText("NORMAL");
  else
    TEB_optimal_label->setText("EMPTY");
  
  if(TEBPanel::TEB_velocity == 1)
    TEB_velocity_label->setText("NORMAL");
  else
    TEB_velocity_label->setText("Can't Get Velocity");
}
/*****************************************************************************************************************/
} // end namespace rviz_teleop_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_teleop_commander::TEBPanel,rviz::Panel )
// END_TUTORIAL
