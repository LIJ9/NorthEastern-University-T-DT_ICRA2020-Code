#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <geometry_msgs/Twist.h>
#include <QDebug>

#include "teleop_pad.h"

namespace rviz_teleop_commander
{

// 构造函数，初始化变量
TeleopPanel::TeleopPanel( QWidget* parent )
  : rviz::Panel( parent )
  , linear_velocity_( 0 )
  , angular_velocity_( 0 )
{
  // 创建一个输入topic命名的窗口
  QVBoxLayout* topic_layout = new QVBoxLayout;
  topic_layout->addWidget( new QLabel( "Teleop Topic:" ));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_ );

  // 创建一个输入线速度的窗口
  topic_layout->addWidget( new QLabel( "Linear Velocity:" ));
  output_topic_editor_1 = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_1 );

  // 创建一个输入角速度的窗口
  topic_layout->addWidget( new QLabel( "Angular Velocity:" ));
  output_topic_editor_2 = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_2 ,tmp_counter);
  

   // 创建一个输入自定义的窗口
  topic_layout->addWidget( new QLabel( "Show:" ));
  //qlabel = new QLabel();
  qlabel = new QLineEdit;
  qlabel->setText("Show");
  topic_layout->addWidget(qlabel);
  
  
  topic_layout->addWidget( new QLabel( "Test:" ));
  //qlabel = new QLabel();
  qlabel_1 = new QLineEdit;
  qlabel_1->setText("Show");
  topic_layout->addWidget(qlabel_1);
  
  QHBoxLayout* layout = new QHBoxLayout;
  layout->addLayout( topic_layout );
  setLayout( layout );
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
  
  
  /**************************************************AStar**********************************************************/
  QVBoxLayout* AStar_Topic = new QVBoxLayout;
  Global_Topic->addWidget( new QLabel( "AStar_Topic" ));
  

  
  
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
  
  
  
  ////////////////////////////////////////////////////////////////////////////////////////
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( update_identity_show_zhao() ));
  
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( chatterCallback_carry() ));
  
  
  
  
  //Global
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( Global_Subscribe() ));
  //AStar
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( AStar_Subscribe() ));
  //Local
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( Local_Subscribe() ));
  //TEB
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( TEB_Subscribe() ));
  
  ////////////////////////////////////////////////////////////////////////////////////////

  // 设置信号与槽的连接
  connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));             // 输入topic命名，回车后，调用updateTopic()
  connect( output_topic_editor_1, SIGNAL( editingFinished() ), this, SLOT( update_Linear_Velocity() )); // 输入线速度值，回车后，调用update_Linear_Velocity()
  connect( output_topic_editor_2, SIGNAL( editingFinished() ), this, SLOT( update_Angular_Velocity() ));// 输入角速度值，回车后，调用update_Angular_Velocity()
  
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  connect( qlabel, SIGNAL(update_identity_show()), this, SLOT(update_Show()));
  connect( qlabel_1, SIGNAL(update_topic_test()), this, SLOT(update_Show()));
  
  //global
  connect( Global_pose_label, SIGNAL(update_Global()), this, SLOT(update_Show()));
  connect( Global_stuck_label, SIGNAL(update_Global()), this, SLOT(update_Show()));
  connect( Global_goal_label, SIGNAL(update_Global()), this, SLOT(update_Show()));
  connect( Global_plan_label, SIGNAL(update_Global()), this, SLOT(update_Show()));
  connect( Global_time_label, SIGNAL(update_Global()), this, SLOT(update_Show()));
  
  //AStar
  connect( AStar_GPtransform_label, SIGNAL(update_AStar()), this, SLOT(update_Show()));
  connect( AStar_Maptransform_label, SIGNAL(update_AStar()), this, SLOT(update_Show()));
  connect( AStar_goal_label, SIGNAL(update_AStar()), this, SLOT(update_Show()));
  
  //Local
  connect( Local_initialize_label, SIGNAL(update_Local()), this, SLOT(update_Show()));
  connect( Local_stuck_label, SIGNAL(update_Local()), this, SLOT(update_Show()));
  connect( Local_plan_label, SIGNAL(update_Local()), this, SLOT(update_Show()));
  connect( Local_cmd_vel_label, SIGNAL(update_Local()), this, SLOT(update_Show()));
  
  //TEB
  connect( TEB_initialize_label, SIGNAL(update_TEB()), this, SLOT(update_Show()));
  connect( TEB_trajectory_label, SIGNAL(update_TEB()), this, SLOT(update_Show()));
  connect( TEB_oscillating_label, SIGNAL(update_TEB()), this, SLOT(update_Show()));
  connect( TEB_transform_label, SIGNAL(update_TEB()), this, SLOT(update_Show()));
  connect( TEB_empty_label, SIGNAL(update_TEB()), this, SLOT(update_Show()));
  connect( TEB_optimal_label, SIGNAL(update_TEB()), this, SLOT(update_Show()));
  connect( TEB_velocity_label, SIGNAL(update_TEB()), this, SLOT(update_Show()));
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // 设置定时器的回调函数，按周期调用sendVel()
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));
  

  // 设置定时器的周期，100ms
  output_timer->start( 100 );
}

// 更新线速度值
void TeleopPanel::update_Linear_Velocity()
{
    // 获取输入框内的数据
    QString temp_string = output_topic_editor_1->text();
	
	// 将字符串转换成浮点数
    float lin = temp_string.toFloat();  
	
	// 保存当前的输入值
    linear_velocity_ = lin;
    
    
    
    //label_content_->setText(temp_string);
    
}

// 更新角速度值
void TeleopPanel::update_Angular_Velocity()
{
    QString temp_string = output_topic_editor_2->text();
    float ang = temp_string.toFloat() ;  
    angular_velocity_ = ang;
}

// 更新topic命名
void TeleopPanel::updateTopic()
{
  setTopic( output_topic_editor_->text() );
}

// 设置topic命名
void TeleopPanel::setTopic( const QString& new_topic )
{
  // 检查topic是否发生改变.
  if( new_topic != output_topic_ )
  {
    output_topic_ = new_topic;
	
    // 如果命名为空，不发布任何信息
    if( output_topic_ == "" )
    {
      velocity_publisher_.shutdown();
    }
	// 否则，初始化publisher
    else
    {
      velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>( output_topic_.toStdString(), 1 );
    }

    Q_EMIT configChanged();
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TeleopPanel::update_identity_show_zhao()
 {
     switch(tmp_counter)
     {
       case 0:
       {
 	output_topic_3 = "A";
 	tmp_counter++;
 	break;
       }
       case 1:
       {
 	output_topic_3 = "B";
 	tmp_counter++;
 	break;
       }
       case 2:
       {
 	output_topic_3 = "C";
 	tmp_counter++;
 	break;
       }
       case 3:
       {
 	output_topic_3 = "D";
 	tmp_counter = 0;
 	break;
       }
       default:
       {
 	tmp_counter = 0;
 	break;
       }
     }
     
     //qlabel = new Qlabel();
     
     qlabel->setText(output_topic_3);
     Q_EMIT update_identity_show();
 }

void TeleopPanel::update_Show()
{}


void TeleopPanel::topic_update_Show()
{
  qlabel_1->setText(output_topic_3);
  std::cout<<" topic_msg is "<<topic_msg<<std::endl;
}

void TeleopPanel::chatterCallback_carry()
{
  sub = nh_.subscribe("/chatter", 1000 ,&TeleopPanel::chatterCallback ,this);
  
  std::cout<<"1111111111111111111111"<<std::endl;
  Q_EMIT update_topic_test();
  
}

/**************************************************Global_Topic***************************************************/

void TeleopPanel::Global_Subscribe()
{
  TeleopPanel::Global_sub = nh_.subscribe("/planthread", 1000, &TeleopPanel::Global_chatterCallback ,this);
  Q_EMIT update_Global();
}

void TeleopPanel::Global_chatterCallback(const geometry_msgs::PoseStamped &msg)
{
  TeleopPanel::Global_pose = msg.pose.position.x;
  TeleopPanel::Global_stuck = msg.pose.position.y;
  TeleopPanel::Global_goal = msg.pose.position.z;
  TeleopPanel::Global_plan = msg.pose.orientation.x;
  TeleopPanel::Global_time = msg.pose.orientation.y;
  
  if(TeleopPanel::Global_pose == 1)
    Global_pose_label->setText("NORMAL");
  else
    Global_pose_label->setText("ERROR");
  
  if(TeleopPanel::Global_stuck == 1)
    Global_pose_label->setText("NORMAL");
  else
    Global_pose_label->setText("STUCK");
  
  if(TeleopPanel::Global_goal == 1)
    Global_pose_label->setText("NORMAL");
  else
    Global_pose_label->setText("Current Goal Not Valid");
  
  if(TeleopPanel::Global_plan == 1)
    Global_pose_label->setText("NORMAL");
  else
    Global_pose_label->setText("ERROR");
  
  if(TeleopPanel::Global_time == 1)
    Global_pose_label->setText("NORMAL");
  else
    Global_pose_label->setText("Beyond Expected");
}
/*****************************************************************************************************************/



/**************************************************AStar_Topic****************************************************/

void TeleopPanel::AStar_Subscribe()
{
  TeleopPanel::Global_sub = nh_.subscribe("/Aplan_info", 1000, &TeleopPanel::Global_chatterCallback ,this);
  Q_EMIT update_AStar();
}

void TeleopPanel::AStar_chatterCallback(const geometry_msgs::PoseStamped &msg)
{
  TeleopPanel::AStar_GPtransform = msg.pose.position.x;
  TeleopPanel::AStar_Maptransform = msg.pose.position.y;
  TeleopPanel::AStar_goal = msg.pose.position.z;
  
  if(TeleopPanel::AStar_GPtransform == 1)
    AStar_GPtransform_label->setText("NORMAL");
  else
    AStar_GPtransform_label->setText("ERROR");
  
  if(TeleopPanel::AStar_Maptransform == 1)
    AStar_Maptransform_label->setText("NORMAL");
  else
    AStar_Maptransform_label->setText("STUCK");
  
  if(TeleopPanel::AStar_goal == 1)
    AStar_goal_label->setText("NORMAL");
  else
    AStar_goal_label->setText("WARNING:Goal Changed");
}
/*****************************************************************************************************************/



/**************************************************Local_Topic****************************************************/

void TeleopPanel::Local_Subscribe()
{
  TeleopPanel::Global_sub = nh_.subscribe("/LOOP_info", 1000, &TeleopPanel::Global_chatterCallback ,this);
  Q_EMIT update_Local();
}

void TeleopPanel::Local_chatterCallback(const geometry_msgs::PoseStamped &msg)
{
  TeleopPanel::Local_initialize = msg.pose.position.x;
  TeleopPanel::Local_stuck = msg.pose.position.y;
  TeleopPanel::Local_plan = msg.pose.position.z;
  TeleopPanel::Local_cmd_vel = msg.pose.orientation.x;
  
  if(TeleopPanel::Local_initialize == 1)
    Local_initialize_label->setText("NORMAL");
  else
    Local_initialize_label->setText("ERROR");
  
  if(TeleopPanel::Local_stuck == 1)
    Local_stuck_label->setText("NORMAL");
  else
    Local_stuck_label->setText("STUCK");
  
  if(TeleopPanel::Local_plan == 1)
    Local_plan_label->setText("NORMAL");
  else
    Local_plan_label->setText("Can't Finish Plan");
  
  if(TeleopPanel::Local_cmd_vel == 1)
    Local_cmd_vel_label->setText("NORMAL");
  else
    Local_cmd_vel_label->setText("Can't Get cmd_vel");
}
/*****************************************************************************************************************/



/**************************************************TEB_Topic******************************************************/

void TeleopPanel::TEB_Subscribe()
{
  TeleopPanel::Global_sub = nh_.subscribe("/cp_vel_info", 1000, &TeleopPanel::Global_chatterCallback ,this);
  Q_EMIT update_TEB();
}

void TeleopPanel::TEB_chatterCallback(const geometry_msgs::PoseStamped &msg)
{
  TeleopPanel::TEB_initialize = msg.pose.position.x;
  TeleopPanel::TEB_trajectory = msg.pose.position.y;
  TeleopPanel::TEB_oscillating = msg.pose.position.z;
  TeleopPanel::TEB_transform = msg.pose.orientation.x;
  TeleopPanel::TEB_empty = msg.pose.orientation.y;
  TeleopPanel::TEB_optimal = msg.pose.orientation.z;
  TeleopPanel::TEB_velocity = msg.pose.orientation.w;  
  
  if(TeleopPanel::TEB_initialize == 1)
    TEB_initialize_label->setText("NORMAL");
  else
    TEB_initialize_label->setText("ERROR");
  
  if(TeleopPanel::TEB_trajectory == 1)
    TEB_trajectory_label->setText("NORMAL");
  else
    TEB_trajectory_label->setText("Not Feasible");
  
  if(TeleopPanel::TEB_oscillating == 1)
    TEB_oscillating_label->setText("NORMAL");
  else
    TEB_oscillating_label->setText("Oscillating");
  
  if(TeleopPanel::TEB_transform == 1)
    TEB_transform_label->setText("NORMAL");
  else
    TEB_transform_label->setText("ERROR");
  
  if(TeleopPanel::TEB_empty == 1)
    TEB_empty_label->setText("NORMAL");
  else
    TEB_empty_label->setText("EMPTY");
  
  if(TeleopPanel::TEB_optimal == 1)
    TEB_optimal_label->setText("NORMAL");
  else
    TEB_optimal_label->setText("EMPTY");
  
  if(TeleopPanel::TEB_velocity == 1)
    TEB_velocity_label->setText("NORMAL");
  else
    TEB_velocity_label->setText("Can't Get Velocity");
}
/*****************************************************************************************************************/





void TeleopPanel::chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_ERROR("I heard: [%s]", msg->data.c_str());
  topic_msg = msg->data.c_str();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// 发布消息
void TeleopPanel::sendVel()
{
  if( ros::ok() && velocity_publisher_ )
  {
    geometry_msgs::Twist msg;
    msg.linear.x = linear_velocity_;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = angular_velocity_;
    velocity_publisher_.publish( msg );
  }
}

// 重载父类的功能
void TeleopPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}


// 重载父类的功能，加载配置数据
void TeleopPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    output_topic_editor_->setText( topic );
    updateTopic();
  }
}

} // end namespace rviz_teleop_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_teleop_commander::TeleopPanel,rviz::Panel )
// END_TUTORIAL
