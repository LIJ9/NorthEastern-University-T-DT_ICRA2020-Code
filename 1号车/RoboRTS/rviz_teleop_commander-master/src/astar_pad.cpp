#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <geometry_msgs/Twist.h>
#include <QDebug>

#include "astar_pad.h"

namespace rviz_teleop_commander
{

// 构造函数，初始化变量
AStarPanel::AStarPanel( QWidget* parent )
  : rviz::Panel( parent )
  , AStar_GPtransform( 0 )
  , AStar_Maptransform( 0 )
  , AStar_goal( 0 )
{
  /**************************************************AStar**********************************************************/
  QVBoxLayout* AStar_Topic = new QVBoxLayout;
  AStar_Topic->addWidget( new QLabel( "AStar_Topic" ));
  

  
  
  AStar_Topic->addWidget( new QLabel( "GP Transform:" ));
  AStar_GPtransform_label = new QLineEdit;
  AStar_GPtransform_label->setText("NORMAL");//ERROR
  AStar_Topic->addWidget(AStar_GPtransform_label);
  
  
  AStar_Topic->addWidget( new QLabel( "Map Frame Transform:" ));
  AStar_Maptransform_label = new QLineEdit;
  AStar_Maptransform_label->setText("NORMAL");//STUCK
  AStar_Topic->addWidget(AStar_Maptransform_label);
  
  
  AStar_Topic->addWidget( new QLabel( "CHANGED A GOAL:" ));
  AStar_goal_label = new QLineEdit;
  AStar_goal_label->setText("NORMAL");//WARNING:Goal Changed
  AStar_Topic->addWidget(AStar_goal_label);
  
  
  
  QHBoxLayout* AStar_layout = new QHBoxLayout;
  AStar_layout->addLayout( AStar_Topic );
  setLayout( AStar_layout );
  /*****************************************************************************************************************/
  // 创建一个定时器，用来定时发布消息
  QTimer* output_timer = new QTimer( this );
  
  //AStar
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( AStar_Subscribe() ));
  
  //AStar
  connect( AStar_GPtransform_label, SIGNAL(update_AStar()), this, SLOT(update_Show()));
  connect( AStar_Maptransform_label, SIGNAL(update_AStar()), this, SLOT(update_Show()));
  connect( AStar_goal_label, SIGNAL(update_AStar()), this, SLOT(update_Show()));
  
  
  // 设置定时器的周期，100ms
  output_timer->start( 100 );
}

void AStarPanel::update_Show()
{}
/**************************************************AStar_Topic****************************************************/

void AStarPanel::AStar_Subscribe()
{
  AStarPanel::AStar_sub = nh_.subscribe("/Aplan_info", 1000, &AStarPanel::AStar_chatterCallback ,this);
  Q_EMIT update_AStar();
}

void AStarPanel::AStar_chatterCallback(const geometry_msgs::PoseStamped &msg)
{
  AStarPanel::AStar_GPtransform = msg.pose.position.x;
  AStarPanel::AStar_Maptransform = msg.pose.position.y;
  AStarPanel::AStar_goal = msg.pose.position.z;
  
  if(AStarPanel::AStar_GPtransform == 1)
    AStar_GPtransform_label->setText("NORMAL");
  else
    AStar_GPtransform_label->setText("ERROR");
  
  if(AStarPanel::AStar_Maptransform == 1)
    AStar_Maptransform_label->setText("NORMAL");
  else
    AStar_Maptransform_label->setText("STUCK");
  
  if(AStarPanel::AStar_goal == 1)
    AStar_goal_label->setText("NORMAL");
  else
    AStar_goal_label->setText("WARNING:Goal Changed");
}
/*****************************************************************************************************************/
} // end namespace rviz_teleop_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_teleop_commander::AStarPanel,rviz::Panel )
// END_TUTORIAL
