/****************************************************************************
 *@brief		检测装甲板节点
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
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "armor_detection_node.h"
#include "roborts_msgs/ArmorDetectionAction.h"
namespace roborts_detection {

ArmorDetectionNode::ArmorDetectionNode():
    node_state_(roborts_common::IDLE),
    demensions_(3),
    initialized_(false),
    detected_enemy_(false),
    undetected_count_(0),
    as_(nh_, "armor_detection_node1_action", boost::bind(&ArmorDetectionNode::ActionCB, this, _1), false)
{
  initialized_ = false;
  enemy_nh_ = ros::NodeHandle();
  if (Init().IsOK()) 
  {
		ROS_ERROR("armor_detection_node1 initalized successed!");
    initialized_ = true;
    node_state_ = roborts_common::IDLE;
  } else {
    ROS_ERROR("armor_detection_node1 initalized failed!");
    node_state_ = roborts_common::FAILURE;
  } 
  as_.start();
}
//-------------------------------------------------------[ADD]----------------------------------------------//

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
float game_state;
int   stricken_armor_id;
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

ErrorInfo ArmorDetectionNode::Init() 
{
 
  enemy_info_pub_ = enemy_nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 100);
  chatter_pub = enemy_nh_.advertise<std_msgs::Float64MultiArray>("chatter", 1000);  
  vision_pub = enemy_nh_.advertise<std_msgs::Float64MultiArray>("vision_tomcu", 1000);
  decision_pub = enemy_nh_.advertise<std_msgs::Int16>("IsAngleAchieved", 1000); 
  firecommand_pub = enemy_nh_.advertise<std_msgs::Int16>("FireCmd", 1000); 
  sub_vision=enemy_nh_.subscribe("vision_tonuc", 1000,&ArmorDetectionNode::sendCallback, this);  
  sub_decision=enemy_nh_.subscribe("ChassisFollowGimbal", 1,&ArmorDetectionNode::decision_sendCallback, this); 
  sub_gamestate=enemy_nh_.subscribe("referee_system", 1,&ArmorDetectionNode::gamestate_sendCallback, this);
  image_transport::ImageTransport video_it(enemy_nh_);
  video_pubs_ = video_it.advertise("video_img", 1, true);
 
  //ArmorDetectionAlgorithms armor_detection_param;
  Amors armor_detection_param;
  std::string file_name = ros::package::getPath("roborts_detection") + "/armor_detection/config/armor_detection1.prototxt"; 
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
  last_being_hit=0;
  last_laser_identify = 0;
  final_chassis_angle = 0;
  rotated_angle = 0;
  stricken_armor_id = 0;
  vice_being_hit = 0;
  vice_laser_identify = 0; 
	
	return ErrorInfo(ErrorCode::OK);
    
}

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
  while(ros::ok()) 
  {
    if(as_.isPreemptRequested()) 
    {
      as_.setPreempted();
      return;
    }
#if 1
		std::lock_guard<std::mutex> guard(mutex_);
		if (undetected_count_ != 0) 
		{
			feedback.detected = true;
			std::cout<<" feedback.detected = true; "<<std::endl;
			feedback.error_code = error_info_.error_code();
			feedback.error_msg = error_info_.error_msg();

			feedback.enemy_pos.header.frame_id = "camera0";
			feedback.enemy_pos.header.stamp    = ros::Time::now();

			feedback.enemy_pos.pose.position.x = x_;
			feedback.enemy_pos.pose.position.y = y_;
			feedback.enemy_pos.pose.position.z = z_;
			feedback.enemy_pos.pose.orientation.w = 1;
			as_.publishFeedback(feedback);
			undetected_msg_published = false;
		} else if(!undetected_msg_published) {
			feedback.detected = false;
			std::cout<<" feedback.detected = false; "<<std::endl;
			feedback.error_code = error_info_.error_code();
			feedback.error_msg = error_info_.error_msg();

			feedback.enemy_pos.header.frame_id = "camera0";
			feedback.enemy_pos.header.stamp    = ros::Time::now();

			feedback.enemy_pos.pose.position.x = 0;
			feedback.enemy_pos.pose.position.y = 0;
			feedback.enemy_pos.pose.position.z = 0;
			feedback.enemy_pos.pose.orientation.w = 1;
			as_.publishFeedback(feedback);
			undetected_msg_published = true;
		}
	}
	rate.sleep();
#endif 
}

void ArmorDetectionNode::ExecuteLoop(const unsigned int index) 
{
  undetected_count_ = undetected_armor_delay_[index];
  while(running_) 
  {
    usleep(1);
    if (node_state_ == NodeState::RUNNING) 
    {
      cv::Point3f target_3d;
      float yaw_output,pitch_output;
      bool firecommand;
      float distance_out;
			int armor_id;//装甲板id，发给决策
      ErrorInfo error_info = armor_detector_[index]->DetectArmor(video_img, detected_enemy_, target_3d, msg1, pitch_receive, yaw_receive, yaw_output, pitch_output, firecommand, distance_out,armor_id);
      sensor_msgs::ImagePtr video_msg;
      video_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", video_img).toImageMsg();
      video_pubs_.publish(video_msg);  

			std::lock_guard<std::mutex> guard(mutex_);
			x_ = target_3d.x;
			y_ = target_3d.y;
			z_ = target_3d.z;
			error_info_ = error_info;

      if(detected_enemy_) 
      {
				last_being_hit = 0;
				last_laser_identify = 0;
				vice_being_hit = 0;
				vice_laser_identify = 0;
				gimbal_tdt_new.data.push_back(yaw_output);
				gimbal_tdt_new.data.push_back(pitch_output);
				if(game_state==4)
				{
					gimbal_tdt_new.data.push_back(firecommand);
					if(firecommand==1)
					{
						firecmd.data = 1;
						firecommand_pub.publish(firecmd);	    
					}else{
						firecmd.data = 0;
						firecommand_pub.publish(firecmd);	    
					}	  
				}else{
					gimbal_tdt_new.data.push_back(0);
				}
				gimbal_tdt_new.data.push_back(1);
				if(vision_permit==1)
					gimbal_tdt_new.data.push_back(1);
				else
					gimbal_tdt_new.data.push_back(0);
				gimbal_tdt_new.data.push_back(distance_out);
				gimbal_tdt_new.data.push_back(0);
				vision_pub.publish(gimbal_tdt_new);
				gimbal_tdt_new.data.clear();
				
				float pitch, yaw;
				gimbal_control_[index].Transform(target_3d, pitch, yaw);
				gimbal_angle_.yaw_mode = true;
				gimbal_angle_.pitch_mode = true;
				
				gimbal_angle_.yaw_angle = yaw * 0.7;
				gimbal_angle_.pitch_angle = pitch;
				std::lock_guard<std::mutex> guard(mutex_);
				undetected_count_ = undetected_armor_delay_[index];
				PublishMsgs();
				chatter_pub.publish(msg1);
      }else if(undetected_count_ != 0) {
				std::cout<<"undetected_count_ != 0"<<std::endl;	
				gimbal_tdt_new.data.push_back(0);
				gimbal_tdt_new.data.push_back(0);
				gimbal_tdt_new.data.push_back(0);
				gimbal_tdt_new.data.push_back(0);
				gimbal_tdt_new.data.push_back(0);
				gimbal_tdt_new.data.push_back(0);
				gimbal_tdt_new.data.push_back(0);
				vision_pub.publish(gimbal_tdt_new);
				gimbal_tdt_new.data.clear();	
				firecmd.data = 0;
				firecommand_pub.publish(firecmd);	
				gimbal_angle_.yaw_mode = true;
				gimbal_angle_.pitch_mode = true;
				gimbal_angle_.yaw_angle = 0;
				gimbal_angle_.pitch_angle = 0;
				undetected_count_--;
				PublishMsgs();
				chatter_pub.publish(msg1);
      }else{
				firecmd.data = 0;
				firecommand_pub.publish(firecmd);
				if(vision_permit==1)
				{	
					float cal_rotated_angle;	
					if(being_hit==1&&last_being_hit==0)//The First Time being Hit
					{
						vice_being_hit = 1;
						float cal_final_chassis_angle;
						if(yaw_receive>=0)//yaw_receive>0
						{
							switch(stricken_armor_id)
							{
							case 0:
								cal_rotated_angle=yaw_receive*(-1);
								rotated_angle = cal_rotated_angle;
								cal_final_chassis_angle = chassis_angle+yaw_receive+cal_rotated_angle;
								final_chassis_angle = cal_final_chassis_angle;
								break;
							case 1:
								cal_rotated_angle=(90-yaw_receive);
								rotated_angle = cal_rotated_angle;
								cal_final_chassis_angle = chassis_angle+yaw_receive+cal_rotated_angle;
								final_chassis_angle = cal_final_chassis_angle;
								break;
							case 2:
								cal_rotated_angle=(180-yaw_receive);
								rotated_angle = cal_rotated_angle;
								cal_final_chassis_angle = chassis_angle+yaw_receive+cal_rotated_angle;
								final_chassis_angle = cal_final_chassis_angle;
								break;
							case 3:
								cal_rotated_angle=(yaw_receive+90)*(-1);
								rotated_angle = cal_rotated_angle;
								cal_final_chassis_angle = chassis_angle+yaw_receive+cal_rotated_angle;
								final_chassis_angle = cal_final_chassis_angle;
								break;			
							}
						}else{	//yaw_receive<0
							switch(stricken_armor_id)
							{
							case 0:
								cal_rotated_angle=yaw_receive*(-1);
								rotated_angle = cal_rotated_angle;
								cal_final_chassis_angle = chassis_angle+yaw_receive+cal_rotated_angle;
								final_chassis_angle = cal_final_chassis_angle;
								break;
							case 1:
								cal_rotated_angle=(90-yaw_receive);
								rotated_angle = cal_rotated_angle;
								cal_final_chassis_angle = chassis_angle+yaw_receive+cal_rotated_angle;
								final_chassis_angle = cal_final_chassis_angle;
								break;
							case 2:
								cal_rotated_angle=(180+yaw_receive)*(-1);
								rotated_angle = cal_rotated_angle;
								cal_final_chassis_angle = chassis_angle+yaw_receive+cal_rotated_angle;
								final_chassis_angle = cal_final_chassis_angle;
								break;
							case 3:
								cal_rotated_angle=(yaw_receive+90)*(-1);
								rotated_angle = cal_rotated_angle;
								cal_final_chassis_angle = chassis_angle+yaw_receive+cal_rotated_angle;
								final_chassis_angle = cal_final_chassis_angle;
								break;
							}
						}
						gimbal_tdt_new.data.push_back(0);
						gimbal_tdt_new.data.push_back(0);
						gimbal_tdt_new.data.push_back(0);
						gimbal_tdt_new.data.push_back(0);
						gimbal_tdt_new.data.push_back(1);
						gimbal_tdt_new.data.push_back(0);
						gimbal_tdt_new.data.push_back(0);
						vision_pub.publish(gimbal_tdt_new);
						gimbal_tdt_new.data.clear();	
						std::cout<<"case 0"<<std::endl;
						last_being_hit = 1;					
					}
					else if(being_hit==1&&last_being_hit==1&&vice_being_hit==1)
					{
						std::cout<<" final_chassis_angle is "<<final_chassis_angle<<std::endl;
						cal_rotated_angle = final_chassis_angle-(chassis_angle+yaw_receive);
						std::cout<<" cal_rotated_angle is "<<cal_rotated_angle<<std::endl;
						if(std::abs(cal_rotated_angle)>180)
						{
						if(cal_rotated_angle>0)
							cal_rotated_angle=cal_rotated_angle-360;
						else
							cal_rotated_angle=cal_rotated_angle+360;
						}
						if(std::abs(cal_rotated_angle)<12)
						{
							gimbal_tdt_new.data.push_back(0); // linear.x
							gimbal_tdt_new.data.push_back(0); // linear.y
							gimbal_tdt_new.data.push_back(0); // linear.z
							gimbal_tdt_new.data.push_back(0); // angular.x
							gimbal_tdt_new.data.push_back(1); // angular.y
							gimbal_tdt_new.data.push_back(0); // angular.z
							gimbal_tdt_new.data.push_back(0);
							vision_pub.publish(gimbal_tdt_new);
							gimbal_tdt_new.data.clear();
							std::cout<<"case 2"<<std::endl;
							vice_being_hit = 0;
							last_being_hit = 1;
						}else{
							gimbal_tdt_new.data.push_back(cal_rotated_angle);
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(1);
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(0);
							vision_pub.publish(gimbal_tdt_new);
							gimbal_tdt_new.data.clear();

							std::cout<<"case 1"<<std::endl;
							vice_being_hit = 1;
							last_being_hit = 1;
						}
					}
					else if(being_hit==1&&last_being_hit==1&&vice_being_hit==0)
					{
						if(std::abs(yaw_receive)<8)
						{
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(0);
							vision_pub.publish(gimbal_tdt_new);
							gimbal_tdt_new.data.clear();
							last_being_hit = 0;
							std_msgs::Int16 decision_command;
							decision_command.data = 1;
							decision_pub.publish(decision_command);
							being_hit = 0;
							std::cout<<"case 4"<<std::endl;
						}
						gimbal_tdt_new.data.push_back(0);
						gimbal_tdt_new.data.push_back(0);
						gimbal_tdt_new.data.push_back(0);
						gimbal_tdt_new.data.push_back(0);
						gimbal_tdt_new.data.push_back(1);
						gimbal_tdt_new.data.push_back(0);
						gimbal_tdt_new.data.push_back(0);
						vision_pub.publish(gimbal_tdt_new);
						gimbal_tdt_new.data.clear();
						std::cout<<"case 3"<<std::endl;
						last_being_hit = 1;
					}							
					if(being_hit!=1)
					{
						if(laser_identify==1&&last_laser_identify==0)//The First Time laser identify
						{
							vice_laser_identify = 1;
							final_chassis_angle = laser_target_angle;
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(1);
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(0);
							vision_pub.publish(gimbal_tdt_new);
							gimbal_tdt_new.data.clear();

							std::cout<<"case a"<<std::endl;
							last_laser_identify = 1;
						}
						else if(laser_identify==1&&last_laser_identify==1&&vice_laser_identify==1)
						{
							cal_rotated_angle = final_chassis_angle-(chassis_angle+yaw_receive);
							if(std::abs(cal_rotated_angle)>180)
							{
							if(cal_rotated_angle>0)
								cal_rotated_angle=cal_rotated_angle-360;
							else
								cal_rotated_angle=cal_rotated_angle+360;
							}
							if(std::abs(cal_rotated_angle)<12)
							{
								gimbal_tdt_new.data.push_back(0);
								gimbal_tdt_new.data.push_back(0);
								gimbal_tdt_new.data.push_back(0);
								gimbal_tdt_new.data.push_back(0);
								gimbal_tdt_new.data.push_back(1);
								gimbal_tdt_new.data.push_back(0);
								gimbal_tdt_new.data.push_back(0);
								vision_pub.publish(gimbal_tdt_new);
								gimbal_tdt_new.data.clear();

								vice_laser_identify = 0;
								last_laser_identify = 1;
								std::cout<<"case c"<<std::endl;
							}else{
								gimbal_tdt_new.data.push_back(cal_rotated_angle);
								gimbal_tdt_new.data.push_back(0);
								gimbal_tdt_new.data.push_back(0);
								gimbal_tdt_new.data.push_back(0);
								gimbal_tdt_new.data.push_back(1);
								gimbal_tdt_new.data.push_back(0);
								gimbal_tdt_new.data.push_back(0);
								vision_pub.publish(gimbal_tdt_new);
								gimbal_tdt_new.data.clear();

								std::cout<<"case b"<<std::endl;
								vice_laser_identify = 1;
								last_laser_identify = 1;
							}
						}
						else if(laser_identify==1&&last_laser_identify==1&&vice_laser_identify==0)
						{
							if(std::abs(yaw_receive)<8)
							{
								gimbal_tdt_new.data.push_back(0);
								gimbal_tdt_new.data.push_back(0);
								gimbal_tdt_new.data.push_back(0);
								gimbal_tdt_new.data.push_back(0);
								gimbal_tdt_new.data.push_back(0);
								gimbal_tdt_new.data.push_back(0);
								gimbal_tdt_new.data.push_back(0);
								vision_pub.publish(gimbal_tdt_new);
								gimbal_tdt_new.data.clear();
								last_laser_identify = 0;
								laser_identify = 0;
								std_msgs::Int16 decision_command;
								decision_command.data = 1;
								decision_pub.publish(decision_command);
								std::cout<<"case e"<<std::endl;

							}else{
								gimbal_tdt_new.data.push_back(0);
								gimbal_tdt_new.data.push_back(0);
								gimbal_tdt_new.data.push_back(0);
								gimbal_tdt_new.data.push_back(0);
								gimbal_tdt_new.data.push_back(1);
								gimbal_tdt_new.data.push_back(0);
								gimbal_tdt_new.data.push_back(0);
								vision_pub.publish(gimbal_tdt_new);
								gimbal_tdt_new.data.clear();

								last_laser_identify = 1;
								std::cout<<"case d"<<std::endl;
							}					
						}
						else if(laser_identify==0&&last_laser_identify==0)
						{
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(0);
							gimbal_tdt_new.data.push_back(0);
							vision_pub.publish(gimbal_tdt_new);
							gimbal_tdt_new.data.clear();
							last_laser_identify = 0;
							std::cout<<"case 5"<<std::endl;
						}	
					}
				}else{
					gimbal_tdt_new.data.push_back(0);
					gimbal_tdt_new.data.push_back(0);
					gimbal_tdt_new.data.push_back(0);
					gimbal_tdt_new.data.push_back(0);
					gimbal_tdt_new.data.push_back(0);
					gimbal_tdt_new.data.push_back(0);
					gimbal_tdt_new.data.push_back(0);
					vision_pub.publish(gimbal_tdt_new);
					gimbal_tdt_new.data.clear();
					last_being_hit = 0;
					vice_being_hit = 0;
					last_laser_identify = 0;
					vice_laser_identify = 0;
					std::cout<<"case 6"<<std::endl;
				}				
			} 
		}else if (node_state_ == NodeState::PAUSE) {
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
  ros::init(argc, argv, "armor_detection_node1", ros::init_options::NoSigintHandler);
  roborts_detection::ArmorDetectionNode armor_detection;
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> ac("armor_detection_node1_action", true);
  ac.waitForServer();
  roborts_msgs::ArmorDetectionGoal goal;
  goal.command = 1;
  ac.sendGoal(goal);
  ros::waitForShutdown();
  armor_detection.StopThread();
  return 0;
}


