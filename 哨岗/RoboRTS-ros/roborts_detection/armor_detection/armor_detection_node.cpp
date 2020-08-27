/****************************************************************************
 *@brief		1号哨岗
 *@version	1.0.0.0
 *@author		Way
 * 
 *@Change History:
 * -------------------------------------------------------------------------
 * <Date>	      |<version>	   |<Author>	|<Description>
 * -------------------------------------------------------------------------
 * 2019/10/22   |1.0.0.1       |Way		    |2019ICRA
 *-------------------------------------------------------------------------- 
 * 2020/08/24   |1.0.0.2       |Way		    |最终格式规范
 *--------------------------------------------------------------------------
 ***************************************************************************/

#include <unistd.h>
#include "armor_detection_node.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "roborts_msgs/ArmorDetectionAction.h"
#include <actionlib/client/terminal_state.h>

namespace roborts_detection {

ArmorDetectionNode::ArmorDetectionNode():
    node_state_(roborts_common::IDLE),
    demensions_(3),
    initialized_(false),
    detected_enemy_(false),
    undetected_count_(0),
    as_(nh_, "armor_detection_node_action", boost::bind(&ArmorDetectionNode::ActionCB, this, _1), false)
{
  initialized_ = false;
  enemy_nh_ = ros::NodeHandle();
  if (Init().IsOK()) 
  {
		ROS_ERROR("armor_detection_node initalized successed!");
    initialized_ = true;
    node_state_ = roborts_common::IDLE;
  } else {
    ROS_ERROR("armor_detection_node initalized failed!");
    node_state_ = roborts_common::FAILURE;
  } 
  as_.start();
}

float yaw_receive;
float pitch_receive;
float yaw_new;

/*ChassisFollowGimbal*/
float vision_permit;//0
float being_hit;//1
float laser_identify;//2
float rev_stricken_armor_id;//3
float chassis_angle;//4
float laser_target_angle;//5
int stricken_armor_id;
float game_state;
std_msgs::Int16 firecmd;

void ArmorDetectionNode::set_last_being_hit(float being_hit)
{
  last_being_hit = being_hit;
}

bool ArmorDetectionNode::get_last_being_hit()const{
  
  return last_being_hit;
}

void ArmorDetectionNode::set_rotated_angle(float cal_rotated_angle)
{
  rotated_angle = cal_rotated_angle;
}

bool ArmorDetectionNode::get_rotated_angle()const
{ 
  return rotated_angle;
}

void ArmorDetectionNode::set_final_chassis_angle(float cal_final_chassis_angle)
{
  final_chassis_angle = cal_final_chassis_angle;
}

bool ArmorDetectionNode::get_final_chassis_angle()const
{ 
  return final_chassis_angle;
}

void ArmorDetectionNode::sendCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
 yaw_receive=msg->linear.x;
 pitch_receive=msg->linear.y;
 yaw_new = msg->angular.z;
}

void ArmorDetectionNode::decision_sendCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if(!msg->data.empty())
  {
    vision_permit=msg->data[0];  
    being_hit=msg->data[1];
    laser_identify=msg->data[2];
    rev_stricken_armor_id=msg->data[3];  
    stricken_armor_id = (int) rev_stricken_armor_id; 
    if(msg->data[4]>=0)
      chassis_angle=msg->data[4]*57.32;
    else
      chassis_angle=360+msg->data[4]*57.32;
    
    if(msg->data[5]>=0)
      laser_target_angle=msg->data[5]*57.32;
    else
      laser_target_angle=360+msg->data[5]*57.32; 
  }
}

void ArmorDetectionNode::gamestate_sendCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if(!msg->data.empty())
  {
  game_state=msg->data[2];
  }
}

 void ArmorDetectionNode::car1Info_sendCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
 {
   if(!msg->data.empty())
   {
    car1_x=msg->data[0];
    car1_y=msg->data[1];
   }
   
 }
  void ArmorDetectionNode::car2Info_sendCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
 {
   if(!msg->data.empty())
   {
    car2_x=msg->data[0];
    car2_y=msg->data[1];
   }
   
 }
 void ArmorDetectionNode::node2Info_sendCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
 {
   if(!msg->data.empty())
   {
    Point1_x=msg->data[0];
    Point1_y=msg->data[1];
    
    Point2_x=msg->data[2];
    Point2_y=msg->data[3];
    
    Point3_x=msg->data[4];
    Point3_y=msg->data[5];
    
    num2=msg->data[6];
    id2=msg->data[7];
   }
   
 }

ErrorInfo ArmorDetectionNode::Init() 
{
  //块消息发送
  block_info_pub=enemy_nh_.advertise<std_msgs::Float64MultiArray>("block_info",100);
  enemy_info_pub_ = enemy_nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 100);
  chatter_pub = enemy_nh_.advertise<std_msgs::Float64MultiArray>("chatter", 1000); 
  //vision_pub = enemy_nh_.advertise<geometry_msgs::Twist>("vision_tomcu", 1000); 
  vision_pub = enemy_nh_.advertise<std_msgs::Float64MultiArray>("vision_tomcu", 1000);
  decision_pub = enemy_nh_.advertise<std_msgs::Int16>("IsAngleAchieved", 1000); 
  firecommand_pub = enemy_nh_.advertise<std_msgs::Int16>("FireCmd", 1000); 
  sub_vision=enemy_nh_.subscribe("vision_tonuc", 1000,&ArmorDetectionNode::sendCallback, this);  
  sub_decision=enemy_nh_.subscribe("ChassisFollowGimbal", 1,&ArmorDetectionNode::decision_sendCallback, this); 
  sub_gamestate=enemy_nh_.subscribe("referee_system", 1,&ArmorDetectionNode::gamestate_sendCallback, this);
  car1_info=enemy_nh_.subscribe("my_robort_one_pose", 1,&ArmorDetectionNode::car1Info_sendCallback, this);
  car2_info=enemy_nh_.subscribe("my_robort_two_pose", 1,&ArmorDetectionNode::car2Info_sendCallback, this);
  node2_info=enemy_nh_.subscribe("block_info2", 1,&ArmorDetectionNode::node2Info_sendCallback, this);
  image_transport::ImageTransport video_it(enemy_nh_);
  video_pubs_ = video_it.advertise("video0_img", 1, true);
   
  Amors armor_detection_param;
  std::string file_name = ros::package::getPath("roborts_detection") + "/armor_detection/config/armor_detection.prototxt"; 
  bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &armor_detection_param);
  camera_num_=armor_detection_param.armor().size();
  std::cout<<"camera_num_"<<camera_num_<<std::endl;
  if (!read_state) 
  {
    ROS_ERROR("Cannot open %s", file_name.c_str());
    return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
  }
  gimbal_control_.resize(camera_num_);
  selected_algorithm.resize(camera_num_);
  cv_toolbox_.resize(camera_num_);
  armor_detector_.resize(camera_num_);
  undetected_armor_delay_.resize(camera_num_);
  
  for(int i=0;i<camera_num_;i++)
  {
    std::cout<<"successfully"<< i<<std::endl;
    gimbal_control_[i].Init(armor_detection_param.armor(i).camera_gimbal_transform().offset_x(),
                       armor_detection_param.armor(i).camera_gimbal_transform().offset_y(),
                       armor_detection_param.armor(i).camera_gimbal_transform().offset_z(),
                       armor_detection_param.armor(i).camera_gimbal_transform().offset_pitch(),
                       armor_detection_param.armor(i).camera_gimbal_transform().offset_yaw(), 
                       armor_detection_param.armor(i).projectile_model_info().init_v(),
                       armor_detection_param.armor(i).projectile_model_info().init_k());
     std::cout<<"successfully"<<i<<std::endl;
    //create the selected algorithms
    selected_algorithm[i] = armor_detection_param.armor(i).selected_algorithm();
     std::cout<<"successfully"<<i<<std::endl;
    // create image receiver
    cv_toolbox_[i] =std::make_shared<CVToolbox>(armor_detection_param.armor(i).camera_name());
    std::cout<<"successfully"<<i<<std::endl;
    // create armor detection algorithm
    armor_detector_[i]= roborts_common::AlgorithmFactory<ArmorDetectionBase,std::shared_ptr<CVToolbox>>::CreateAlgorithm
	 (selected_algorithm[i], cv_toolbox_[i]);
    undetected_armor_delay_[i] = armor_detection_param.armor(i).undetected_armor_delay();     
  }
   return ErrorInfo(ErrorCode::OK);
  last_laser_identify = 0;
  final_chassis_angle = 0;
  rotated_angle = 0;
  stricken_armor_id = 0;
  vice_being_hit = 0;
  vice_laser_identify = 0; 
    
}

//-------------------------------------------[ActionCB]----------------------------------------------//
//@brief:
//--------------------------------------------------------------------------------------------------//
void ArmorDetectionNode::ActionCB(const roborts_msgs::ArmorDetectionGoal::ConstPtr &data) 
{
  roborts_msgs::ArmorDetectionFeedback feedback;
  roborts_msgs::ArmorDetectionResult result;
  bool undetected_msg_published = false;
  if(!initialized_)
  {
    feedback.error_code = error_info_.error_code();
    feedback.error_msg  = error_info_.error_msg();
    as_.publishFeedback(feedback);
    as_.setAborted(result, feedback.error_msg);
    ROS_INFO("Initialization Failed, Failed to execute action!");
    return;
  }
  switch (data->command) {
    case 1:
      StartThread();
      break;
    case 2:
      PauseThread();
      break;
    case 3:
      StopThread();
      break;
    default:
      break;
  } 
  ros::Rate rate(25);
}

//-------------------------------------------[ExeuteLoop函数]---------------------------------------//
//@brief:
//--------------------------------------------------------------------------------------------------//
void ArmorDetectionNode::ExecuteLoop(const unsigned int index) 
{
  std::cout<<"circle"<<std::endl;
  undetected_count_ = undetected_armor_delay_[index];
  while(running_) 
  {
    usleep(1);
    if (node_state_ == NodeState::RUNNING) 
    {
      cv::Point3f target_3d1;//块目标点1信息
      cv::Point3f target_3d2;//块目标点2信息
      cv::Point3f target_3d3;//块目中点坐标
      int num=0;
      ErrorInfo error_info = armor_detector_[index]->DetectArmor_00(video_img, detected_enemy_, target_3d1,target_3d2,target_3d3,num,msg1);  
      sensor_msgs::ImagePtr video_msg;
      video_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", video_img).toImageMsg();
      video_pubs_.publish(video_msg); 
      std_msgs::Float64MultiArray block_msg;//发布块消息0
      if(num2==0)//camera2 未检测到车辆
      {
	std::cout<<"camera2 未检测到车辆!!!"<<std::endl;
// 	float dis1=pow((car1_x-target_3d1.x),2)+pow((car1_y-target_3d1.y),2);
// 	float dis2=pow((car1_x-target_3d2.x),2)+pow((car1_x-target_3d2.x),2);
	float dis1=1;
	float dis2=2;
	if (dis1<dis2)
	{
	  //点1坐标
	  block_msg.data.push_back(target_3d1.x);
	  block_msg.data.push_back(target_3d1.y);
	  //点2坐标
	  block_msg.data.push_back(target_3d2.x);
	  block_msg.data.push_back(target_3d2.y);
	}else{
	  //点1坐标
	  block_msg.data.push_back(target_3d2.x);
	  block_msg.data.push_back(target_3d2.y);
	  //点2坐标
	  block_msg.data.push_back(target_3d1.x);
	  block_msg.data.push_back(target_3d1.y);
	}
	 //中心点坐标
	block_msg.data.push_back(target_3d3.x);
	block_msg.data.push_back(target_3d3.y);
	//装甲板号码
	block_msg.data.push_back(target_3d1.z);
	if(target_3d1.x!=0 && target_3d1.y!=0)
	  block_info_pub.publish(block_msg);
      }else{//camera2检测到车辆
	if(num==0)//camera1未检测到车
	{
	  std::cout<<"camera2检测到车辆,camera1未检测到车"<<std::endl;
 	  float dis1=pow((car1_x-Point1_x),2)+pow((car1_y-Point1_y),2);
    float dis2=pow((car1_x-Point2_x),2)+pow((car1_y-Point2_y),2);
	  if (dis1<dis2)
	  {
	    //点1坐标
	    block_msg.data.push_back(Point1_x);
	    block_msg.data.push_back(Point1_y);
	    //点2坐标
	    block_msg.data.push_back(Point2_x);
	    block_msg.data.push_back(Point2_y);
	  }else{
	    //点1坐标
	    block_msg.data.push_back(Point2_x);
	    block_msg.data.push_back(Point2_y);
	    //点2坐标
	    block_msg.data.push_back(Point1_x);
	    block_msg.data.push_back(Point1_y);
	  }
	  //中心点坐标
	  block_msg.data.push_back(Point3_x);
	  block_msg.data.push_back(Point3_y);
	  //装甲板号码
	  block_msg.data.push_back(id2);

	  if(Point1_x!=0 && Point2_x!=0)
	    block_info_pub.publish(block_msg);
	}else{//camera1检测到车辆
	  if(num==num2)
	  {	   
 	    float dis1=pow((car1_x-target_3d1.x),2)+pow((car1_y-target_3d1.y),2);
 	    float dis2=pow((car1_x-target_3d2.x),2)+pow((car1_x-target_3d2.x),2);
	    if (dis1<dis2)
	    {
	      //点1坐标
	      block_msg.data.push_back(target_3d1.x);
	      block_msg.data.push_back(target_3d1.y);
	      //点2坐标
	      block_msg.data.push_back(target_3d2.x);
	      block_msg.data.push_back(target_3d2.y);
	    }else{
	      //点1坐标
	      block_msg.data.push_back(target_3d2.x);
	      block_msg.data.push_back(target_3d2.y);
	      //点2坐标
	      block_msg.data.push_back(target_3d1.x);
	      block_msg.data.push_back(target_3d1.y);
	    }
	    
	    //中心点坐标
	    block_msg.data.push_back(target_3d3.x);
	    block_msg.data.push_back(target_3d3.x);
	    
	    //装甲板号码
	    block_msg.data.push_back(target_3d1.z);

	    if(target_3d1.x!=0 && target_3d1.y!=0)
	      block_info_pub.publish(block_msg);
	  }else{//camera1和camera2 分别检测到不同区域的车辆
	    if(num!=0 &&num2!=0)
	    {
	      return;
	    }	    
	  }	  
	}
      } 
    }else if(node_state_ == NodeState::PAUSE){
      std::unique_lock<std::mutex> lock(mutex_);
      condition_var_.wait(lock);
    }
  }
}

void ArmorDetectionNode::PublishMsgs() 
{
  enemy_info_pub_.publish(gimbal_angle_);
}

void ArmorDetectionNode::StartThread() 
{
  ROS_INFO("Armor detection node started!");
  running_ = true;
  armor_detection_thread_.resize(camera_num_);
  for(unsigned int i=0;i<camera_num_;i++)
  {     
    armor_detector_[i]->SetThreadState(true);
    if(node_state_ == NodeState::IDLE)
    {
      armor_detection_thread_[i] = std::thread(&ArmorDetectionNode::ExecuteLoop, this,i);
      std::cout<<"success"<<std::endl;
    }
  }
  node_state_ = NodeState::RUNNING;
  condition_var_.notify_one();
}

void ArmorDetectionNode::PauseThread() 
{
  ROS_INFO("Armor detection thread paused!");
  node_state_ = NodeState::PAUSE;
}

void ArmorDetectionNode::StopThread() 
{
  node_state_ = NodeState::IDLE;
  running_ = false;
  for(int i=0;i<camera_num_;i++)
  {
    armor_detector_[i]->SetThreadState(false);
    if (armor_detection_thread_[i].joinable()) 
    {
      armor_detection_thread_[i].join();
    }
  }
}

ArmorDetectionNode::~ArmorDetectionNode() 
{
  StopThread();
}
} //namespace roborts_detection

void SignalHandler(int signal)
{
  if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown())
  {
    ros::shutdown();
  }
}

int main(int argc, char **argv) 
{
  signal(SIGINT, SignalHandler);
  signal(SIGTERM,SignalHandler);
  ros::init(argc, argv, "armor_detection_node", ros::init_options::NoSigintHandler);
  roborts_detection::ArmorDetectionNode armor_detection;
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  
  //------------------------------------------[ADD]---------------------------------------------//
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> ac("armor_detection_node_action", true);
  ac.waitForServer();
  roborts_msgs::ArmorDetectionGoal goal;
  goal.command = 1;
  ac.sendGoal(goal);
  //------------------------------------------[END]---------------------------------------------//

  ros::waitForShutdown();
  armor_detection.StopThread();
  return 0;
}


