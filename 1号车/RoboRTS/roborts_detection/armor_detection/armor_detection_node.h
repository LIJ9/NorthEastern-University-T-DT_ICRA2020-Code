/****************************************************************************
 *@brief		检测视觉标签节点检测装甲板节点头文件
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

#ifndef ROBORTS_DETECTION_ARMOR_DETECTION_NODE_H
#define ROBORTS_DETECTION_ARMOR_DETECTION_NODE_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include <geometry_msgs/Twist.h>
#include "actionlib/server/simple_action_server.h"
#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/GimbalRate.h"
#include "roborts_msgs/ArmorDetectionAction.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Int16.h"

#include "alg_factory/algorithm_factory.h"
#include "io/io.h"
#include "state/node_state.h"

#include "cv_toolbox.h"

#include "armor_detection_base.h"
#include "proto/armor_detection.pb.h"
#include "armor_detection_algorithms.h"
#include "gimbal_control.h"

namespace roborts_detection {

using roborts_common::NodeState;
using roborts_common::ErrorInfo;

class ArmorDetectionNode {
 public:
  explicit ArmorDetectionNode();
  void MultiThread();
  /**
   * @brief Initializing armor detection algorithm.
   * @return Return the error information.
   */
  ErrorInfo Init();
  /**
   * @brief Actionlib server call back function.
   * @param data Command for control the algorithm thread.
   */
  void ActionCB(const roborts_msgs::ArmorDetectionGoal::ConstPtr &data);
  /**
   * @brief Starting the armor detection thread.
   */
  void StartThread();
  /**
   * @brief Pausing the armor detection thread when received command 2 in action_lib callback function.
   */
  void PauseThread();
  /**
   * @brief Stopping armor detection thread.
   */
  void StopThread();
  /**
   * @brief Executing the armor detection algorithm.
   */
  void ExecuteLoop(const unsigned int index);
  void sendCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void decision_sendCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void gamestate_sendCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  
  inline void set_last_being_hit(float being_hit);
  inline bool get_last_being_hit()const;
  inline void set_rotated_angle(float cal_rotated_angle);
  inline bool get_rotated_angle()const;
  inline void set_final_chassis_angle(float cal_final_chassis_angle);
  inline bool get_final_chassis_angle()const;
	
  /**
   * @brief Publishing enemy pose information that been calculated by the armor detection algorithm.
   */
  void PublishMsgs();
  ~ArmorDetectionNode();
 protected:
 private:
  std::vector<std::shared_ptr<ArmorDetectionBase> > armor_detector_;
  std::vector<std::thread > armor_detection_thread_;
  unsigned int max_rotating_fps_;
  unsigned int min_rotating_detected_count_;
  //unsigned int undetected_armor_delay_;
  std::vector<unsigned int> undetected_armor_delay_;

  //! state and error
  NodeState node_state_;
  ErrorInfo error_info_;
  bool initialized_;
  bool running_;
  std::mutex mutex_;
  std::condition_variable condition_var_;
  unsigned int undetected_count_;

  //! enemy information
  double x_;
  double y_;
  double z_;
  bool   detected_enemy_;
  bool   detected_markers_;//视觉标签
  unsigned long demensions_;

  std_msgs::Float64MultiArray msg1;
  geometry_msgs::Twist gimbal_tdt;
  
  std_msgs::Float64MultiArray gimbal_tdt_new;

  std_msgs::Float64MultiArray location_msg;

  std_msgs::Float64MultiArray decision_msg;
  
  cv::Mat video_img;
  cv::Mat video_img_;//视觉标签
  bool last_being_hit;
  bool last_laser_identify;
  float rotated_angle;
  float final_chassis_angle;
  bool vice_being_hit;
  bool vice_laser_identify;
  
  
  //ROS info
  ros::NodeHandle nh_;
  ros::NodeHandle enemy_nh_;
	
  ros::Publisher enemy_info_pub_;
  ros::Publisher marker_info_pub_;
  ros::Publisher chatter_pub;
  ros::Publisher vision_pub;
  ros::Publisher location_pub;
  ros::Publisher decision_pub;
  ros::Publisher firecommand_pub;
	
  ros::Subscriber sub_vision;
  ros::Subscriber sub_decision;//being hit!!!
  ros::Subscriber sub_gamestate;
	
  image_transport::Publisher video_pubs_;

  std::vector<std::shared_ptr<CVToolbox> >  cv_toolbox_;
  actionlib::SimpleActionServer<roborts_msgs::ArmorDetectionAction> as_;
  roborts_msgs::GimbalAngle gimbal_angle_;

   //! control model
   //GimbalContrl gimbal_control_;
   std::vector<GimbalContrl> gimbal_control_;
  
  //[ADD] threads
  std::vector<std::thread>detect_threads_;
  int camera_num_;
  std::vector<std::string> selected_algorithm;
};
} //namespace roborts_detection

#endif //ROBORTS_DETECTION_ARMOR_DETECTION_NODE_H
