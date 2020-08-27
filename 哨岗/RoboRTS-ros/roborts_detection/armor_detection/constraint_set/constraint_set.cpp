/****************************************************************************
 *@brief		装甲板检测算法
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
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include "constraint_set.h"
#include "math.h"
#include "timer/timer.h"
#include "io/io.h"
#include "../../../roborts_decision/my_file.h"
#include <iostream>
#include <fstream>

int cut_i=1;

namespace roborts_detection {
#define img_width 1440 //图像宽度
#define img_heigth 1080 //图像高度

ConstraintSet::ConstraintSet(std::shared_ptr<CVToolbox> cv_toolbox):ArmorDetectionBase(cv_toolbox)
{
  filter_x_count_ = 0;
  filter_y_count_ = 0; 
  filter_z_count_ = 0;
  filter_distance_count_ = 0;
  filter_pitch_count_ = 0;
  filter_yaw_count_ = 0;
  read_index_ = -1;
  detection_time_ = 0;
  thread_running_ = false;

  LoadParam();
  error_info_ = ErrorInfo(roborts_common::OK);
    
}

void ConstraintSet::LoadParam() 
{
  //read parameters
  ConstraintSetConfig constraint_set_config_;
  std::string file_name = ros::package::getPath("roborts_detection") + \
      "/armor_detection/constraint_set/config/constraint_set.prototxt";
  bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &constraint_set_config_);
  ROS_ASSERT_MSG(read_state, "Cannot open %s", file_name.c_str());

  enable_debug_ = constraint_set_config_.enable_debug();
  enemy_color_ = constraint_set_config_.enemy_color();
  using_hsv_ = constraint_set_config_.using_hsv();

  //armor info
  float armor_width = constraint_set_config_.armor_size().width();
  float armor_height = constraint_set_config_.armor_size().height();
  SolveArmorCoordinate(armor_width, armor_height);
  roborts_decision::RobotConfig ROBOT_CONFIG;

  //algorithm threshold parameters80
  light_max_aspect_ratio_ = constraint_set_config_.threshold().light_max_aspect_ratio();
  light_min_area_ = constraint_set_config_.threshold().light_min_area();
  light_max_angle_ = constraint_set_config_.threshold().light_max_angle();
  light_max_angle_diff_ = constraint_set_config_.threshold().light_max_angle_diff();
  armor_max_angle_ = constraint_set_config_.threshold().armor_max_angle();
  armor_min_area_ = constraint_set_config_.threshold().armor_min_area();
  armor_max_aspect_ratio_ = constraint_set_config_.threshold().armor_max_aspect_ratio();
  armor_max_pixel_val_ = constraint_set_config_.threshold().armor_max_pixel_val();
  armor_max_stddev_ = constraint_set_config_.threshold().armor_max_stddev();
  armor_max_mean_   = constraint_set_config_.threshold().armor_max_mean();

  color_thread_ = constraint_set_config_.threshold().color_thread();
  blue_thread_ = constraint_set_config_.threshold().blue_thread();
  red_thread_ = constraint_set_config_.threshold().red_thread(); 
//------------------------------------[ADD]----------------------------------------//    
  std::vector<cv::Point2f> init_points;
  float a=0;
  cv::Point2f init_2f(a,a);
  init_points.push_back(init_2f);
  init_points.push_back(init_2f);
  init_points.push_back(init_2f);
  init_points.push_back(init_2f);
  cv::RotatedRect init_rotated_Rect(Point2f(0, 0), cv::Size2f(100, 50), 30);
  bool init_bool=0;
  cv::Rect init_tracking_rect(0, 0, 0, 0);
  float init_armor_stddev = 0.0;
  int init_armor_id = 0;
  float armor_stddev = 0;
  //last_armor_(init_rotated_Rect, init_points, init_bool, init_armor_id, init_tracking_rect, armor_stddev);
    
  last_armor_.rect = init_rotated_Rect;
  last_armor_.vertex = init_points;
  last_armor_.stddev = armor_stddev;
  last_armor_.single = init_bool;
  last_armor_.id = init_armor_id;
  last_armor_.tracking_rect = init_tracking_rect;
    
  offset_=cv::Point(0,0);
  roi_rect_.x = 0;
  roi_rect_.x = 0;
  roi_rect_.width = img_width;
  roi_rect_.height = img_heigth;
	
//------------------------------------[END]----------------------------------------//
  int get_intrinsic_state = -1;
  int get_distortion_state = -1;
  while ((get_intrinsic_state < 0) || (get_distortion_state < 0))
  {
    ROS_WARN("Wait for camera driver launch %d", get_intrinsic_state);
    usleep(50000);
    ros::spinOnce();
    get_intrinsic_state = cv_toolbox_->GetCameraMatrix(intrinsic_matrix_);
    get_distortion_state = cv_toolbox_->GetCameraDistortion(distortion_coeffs_);
  }
}


ErrorInfo ConstraintSet::DetectArmor_00(cv::Mat &result_img, bool &detected, cv::Point3f &target_3d1, cv::Point3f &target_3d2,cv::Point3f &target_3d3,int &num,std_msgs::Float64MultiArray &msg1)
{
  std::vector<cv::RotatedRect> lights;
  std::vector<cv::RotatedRect> lights_copy;
  std::vector<ArmorInfo> armors;
  //ROI
  RoiFilter(src_img_, offset_);  
  auto img_begin = std::chrono::high_resolution_clock::now();
  bool sleep_by_diff_flag = true;
  while (true) 
	{
    // Ensure exit this thread while call Ctrl-C
    if (!thread_running_) 
    {
      ErrorInfo error_info(ErrorCode::STOP_DETECTION);
      return error_info;
    }
    read_index_ = cv_toolbox_->NextImage(src_img_);
    if (read_index_ < 0) 
    {
      // Reducing lock and unlock when accessing function 'NextImage'
      if (detection_time_ == 0) {
        usleep(20000);
        continue;
      } else {
        double capture_time = 0;
        cv_toolbox_->GetCaptureTime(capture_time);
        if (capture_time == 0) {
          // Make sure the driver is launched and the image callback is called
          usleep(20000);
          continue;
        } else if (capture_time > detection_time_ && sleep_by_diff_flag) {
          //ROS_WARN("time sleep %lf", (capture_time - detection_time_));
          usleep((unsigned int)(capture_time - detection_time_));
          sleep_by_diff_flag = false;
          continue;
        } else {
          //For real time request when image call back called, the function 'NextImage' should be called.
          usleep(500);
          continue;
        }
      }
    } else {
      break;
    }
  }
  auto detection_begin = std::chrono::high_resolution_clock::now(); 
  if (enable_debug_) {
    show_lights_before_filter_ = src_img_.clone();
    show_lights_after_filter_ = src_img_.clone();
    show_armors_befor_filter_ = src_img_.clone();
    show_armors_after_filter_ = src_img_.clone();
    // cv::waitKey(1);9-1
  }
  // just_write=src_img_.clone();//910
  DetectLights(src_img_, lights, lights_copy);
  FilterLights(lights,lights_copy);
  PossibleArmors(lights, armors, lights_copy);
  //FilterArmors(armors);
  if(!armors.empty()) {
    float absolute_pitch_;
    float absolute_yaw_;
    detected = true;
    ArmorInfo final_armor = SlectFinalArmor(armors);
    if(enable_debug_) {	
      cv_toolbox_->DrawRotatedRect(src_img_, armors[0].rect, cv::Scalar(0, 255, 0), 2);
      cv::circle(src_img_, Point(armors[0].rect.center.x, armors[0].rect.center.y), 3, cv::Scalar(0,0,255), 1, 8);
      cv::line(src_img_,  Point(320, 240),
		Point(armors[0].rect.center.x, armors[0].rect.center.y),
		cv::Scalar(0,255,255));
    }
    Target output_target;     
    //CalcControlInfo(final_armor, target_3d, pitch_now, yaw_now, yaw_output, pitch_output, absolute_pitch_, absolute_yaw_, output_target, distance_out);
    //armor_ID = final_armor.id;
    blockFun0(final_armor,target_3d1,target_3d2,target_3d3,num);     
    //Get(final_armor, yaw_output, pitch_output, absolute_pitch_, absolute_yaw_, firecommand);      
  }else{
    detected = false;
    std::vector<cv::Point2f> tmp_points;
    float a=0;
    cv::Point2f tmp_2f(a,a);
    tmp_points.push_back(tmp_2f);
    tmp_points.push_back(tmp_2f);
    tmp_points.push_back(tmp_2f);
    tmp_points.push_back(tmp_2f);
    cv::RotatedRect tmp_rotated_Rect(Point2f(0, 0), cv::Size2f(100, 50), 30);
    int tmp_bool = 0;
    cv::Rect tmp_tracking_rect(0, 0, 0, 0);
    float tmp_thre = 0;
    float tmp_armor_stddev = 0.0;
    ArmorInfo tmp_armor(tmp_rotated_Rect, tmp_points, tmp_bool, 0, tmp_tracking_rect, tmp_thre, tmp_armor_stddev);
    set_last_armor(tmp_armor);
  }
  if(enable_debug_) {
    cv::imshow("result_img_", src_img_);
  }
  msg1=msg;
  lights.clear();
  lights_copy.clear();                                
  armors.clear();
  cv_toolbox_->ReadComplete(read_index_);
  //ROS_INFO("read complete");
  detection_time_ = std::chrono::duration<double, std::ratio<1, 1000000>>
      (std::chrono::high_resolution_clock::now() - detection_begin).count();
  msg.data.clear();
  result_img=src_img_;
  //delete the internal memory
  std::vector<cv::RotatedRect>().swap(lights);  
  std::vector<cv::RotatedRect>().swap(lights_copy);  
  std::vector<ArmorInfo>().swap(armors);  
  return error_info_;
}
ErrorInfo ConstraintSet::DetectArmor_01(cv::Mat &result_img, bool &detected, cv::Point3f &target_3d1, cv::Point3f &target_3d2,cv::Point3f &target_3d3,int &num,std_msgs::Float64MultiArray &msg1)
{

  std::vector<cv::RotatedRect> lights;
  std::vector<cv::RotatedRect> lights_copy;
  std::vector<ArmorInfo> armors;
  //ROI
  RoiFilter(src_img_, offset_);  
  auto img_begin = std::chrono::high_resolution_clock::now();
  bool sleep_by_diff_flag = true;
  while (true) 
	{
    // Ensure exit this thread while call Ctrl-C
    if (!thread_running_) 
    {
      ErrorInfo error_info(ErrorCode::STOP_DETECTION);
      return error_info;
    }
    read_index_ = cv_toolbox_->NextImage(src_img_);
    if (read_index_ < 0) 
    {
      // Reducing lock and unlock when accessing function 'NextImage'
      if (detection_time_ == 0) {
        usleep(20000);
        continue;
      } else {
        double capture_time = 0;
        cv_toolbox_->GetCaptureTime(capture_time);
        if (capture_time == 0) {
          // Make sure the driver is launched and the image callback is called
          usleep(20000);
          continue;
        } else if (capture_time > detection_time_ && sleep_by_diff_flag) {
          //ROS_WARN("time sleep %lf", (capture_time - detection_time_));
          usleep((unsigned int)(capture_time - detection_time_));
          sleep_by_diff_flag = false;
          continue;
        } else {
          //For real time request when image call back called, the function 'NextImage' should be called.
          usleep(500);
          continue;
        }
      }
    } else {
      break;
    }
  }
  auto detection_begin = std::chrono::high_resolution_clock::now(); 
  if (enable_debug_) {
    show_lights_before_filter_ = src_img_.clone();
    show_lights_after_filter_ = src_img_.clone();
    show_armors_befor_filter_ = src_img_.clone();
    show_armors_after_filter_ = src_img_.clone();
    // cv::waitKey(1);9-1
  }
  // just_write=src_img_.clone();//910
  DetectLights(src_img_, lights, lights_copy);
  FilterLights(lights,lights_copy);
  PossibleArmors(lights, armors, lights_copy);
  //FilterArmors(armors);
  if(!armors.empty()) {
    float absolute_pitch_;
    float absolute_yaw_;
    detected = true;
    ArmorInfo final_armor = SlectFinalArmor(armors);
    if(enable_debug_) {	
      cv_toolbox_->DrawRotatedRect(src_img_, armors[0].rect, cv::Scalar(0, 255, 0), 2);
      cv::circle(src_img_, Point(armors[0].rect.center.x, armors[0].rect.center.y), 3, cv::Scalar(0,0,255), 1, 8);
      cv::line(src_img_,  Point(320, 240),
		Point(armors[0].rect.center.x, armors[0].rect.center.y),
		cv::Scalar(0,255,255));
    }
    Target output_target;     
    //CalcControlInfo(final_armor, target_3d, pitch_now, yaw_now, yaw_output, pitch_output, absolute_pitch_, absolute_yaw_, output_target, distance_out);
    //armor_ID = final_armor.id;
    blockFun1(final_armor,target_3d1,target_3d2,target_3d3,num);     
    //Get(final_armor, yaw_output, pitch_output, absolute_pitch_, absolute_yaw_, firecommand);      
  }else{
    detected = false;
    std::vector<cv::Point2f> tmp_points;
    float a=0;
    cv::Point2f tmp_2f(a,a);
    tmp_points.push_back(tmp_2f);
    tmp_points.push_back(tmp_2f);
    tmp_points.push_back(tmp_2f);
    tmp_points.push_back(tmp_2f);
    cv::RotatedRect tmp_rotated_Rect(Point2f(0, 0), cv::Size2f(100, 50), 30);
    int tmp_bool = 0;
    cv::Rect tmp_tracking_rect(0, 0, 0, 0);
    float tmp_thre = 0;
    float tmp_armor_stddev = 0.0;
    ArmorInfo tmp_armor(tmp_rotated_Rect, tmp_points, tmp_bool, 0, tmp_tracking_rect, tmp_thre, tmp_armor_stddev);
    set_last_armor(tmp_armor);
  }
  if(enable_debug_) {
    cv::imshow("result_img_", src_img_);
  }
  msg1=msg;
  lights.clear();
  lights_copy.clear();                                
  armors.clear();
  cv_toolbox_->ReadComplete(read_index_);
  //ROS_INFO("read complete");
  detection_time_ = std::chrono::duration<double, std::ratio<1, 1000000>>
      (std::chrono::high_resolution_clock::now() - detection_begin).count();
  msg.data.clear();
  result_img=src_img_;
  //delete the internal memory
  std::vector<cv::RotatedRect>().swap(lights);  
  std::vector<cv::RotatedRect>().swap(lights_copy);  
  std::vector<ArmorInfo>().swap(armors);  
  return error_info_;
}
bool ConstraintSet::JudgeColor(cv::RotatedRect single_light,Mat &src) 
{
	bool flag= false;
	cv::Rect judge_rect=single_light.boundingRect();
	if(RectSafety(judge_rect, src.rows, src.cols)==false)
	  return flag;
	cv::Scalar jud=sum(src(judge_rect));
	switch(enemy_color_)
	{
	  case BLUE:
	    if(jud[0]>jud[2])
	    {
	      flag= true;
	    }
	    break;
	  case RED:
	    if(jud[2]>jud[0])
	    {
	      flag= true;
	    }
	    break;
	}
	return flag;
}

void ConstraintSet::DetectLights(const cv::Mat &src_, std::vector<cv::RotatedRect> &lights, std::vector<cv::RotatedRect> &lights_copy)
{
  cv::Mat src;
  src = src_(roi_rect_);
  if(enable_debug_)
  cv::imshow("ROI",src);

  cv::cvtColor(src, gray_img_, CV_BGR2GRAY);
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::Mat binary_brightness_img, binary_light_img, binary_color_img;
  if(using_hsv_) 
  {
    binary_color_img = cv_toolbox_->DistillationColor(src, enemy_color_, using_hsv_);
    cv::threshold(gray_img_, binary_brightness_img, color_thread_, 255, CV_THRESH_BINARY);
  }else {
    auto light = cv_toolbox_->DistillationColor(src, enemy_color_, using_hsv_);
    cv::threshold(gray_img_, binary_brightness_img, color_thread_, 255, CV_THRESH_BINARY);
    float thresh;
    if (enemy_color_ == BLUE)
      thresh = blue_thread_;
    else
      thresh = red_thread_;
    cv::threshold(light, binary_color_img, thresh, 255, CV_THRESH_BINARY);
    if(enable_debug_)
      cv::imshow("light", light);
  }
  if (enable_debug_) {
    cv::imshow("binary_brightness_img", binary_brightness_img);
    cv::imshow("binary_color_img", binary_color_img);
  }
  auto contours_light = cv_toolbox_->FindContours_Chao(binary_color_img, offset_);
  auto contours_brightness = cv_toolbox_->FindContours_Chao(binary_brightness_img, offset_);
  lights.reserve(contours_light.size());
  lights_copy.reserve(contours_light.size());
  lights_info_.reserve(contours_light.size());
  // TODO: To be optimized

	for (unsigned int j = 0; j < contours_light.size(); j++) 
	{
		cv::RotatedRect single_light = cv::minAreaRect(contours_light[j]);
		cv::Point2f vertices_point[4];
		single_light.points(vertices_point);
		LightInfo light_info(vertices_point);
		cv::RotatedRect tmp_light = cv::minAreaRect(contours_light[j]);
		if (enable_debug_)
		{
			cv_toolbox_->DrawRotatedRect(show_lights_before_filter_, single_light, cv::Scalar(0, 255, 0), 2, light_info.angle_);
			cv::rectangle(show_lights_before_filter_, roi_rect_, cv::Scalar(0, 0, 255), 2);
		}
		lights_copy.push_back(single_light);
		single_light.angle = light_info.angle_;
		lights.push_back(single_light);
	}
  if (enable_debug_)
  {
		cv::imshow("show_lights_before_filter", show_lights_before_filter_);
  }
  auto c = cv::waitKey(1);
  if (c == 'a') 
  {
    cv::waitKey(0);
  }
}

float ConstraintSet::getDistance(cv::Point pointO, cv::Point pointA) 
{
    float distance;
    distance = pow((pointO.x - pointA.x), 2) + pow((pointO.y - pointA.y), 2);
    distance = sqrt(distance);
    return distance;
}

bool ConstraintSet::get_eligibility(const cv::RotatedRect &left_lightbar, const cv::RotatedRect &right_lightbar) 
{
  if(left_lightbar.center.x>right_lightbar.center.x)
  {
    return false;
  }
  float tan_angle;//两个矩形框质心连线与水平线夹角的正切
  int left_lightbar_height = std::max(left_lightbar.size.height,left_lightbar.size.width);
  int right_lightbar_height = std::max(right_lightbar.size.height,right_lightbar.size.width);
  float left_lightbar_angle = left_lightbar.angle;
  float right_lightbar_angle = right_lightbar.angle;
  cv::Point left_lightbar_center = left_lightbar.center;
  cv::Point right_lightbar_center = right_lightbar.center;
  int height_min = std::min(left_lightbar_height, right_lightbar_height);
  int height_max = std::max(left_lightbar_height, right_lightbar_height);
  extern cv::Mat sr;   
  if (right_lightbar_center.x == left_lightbar_center.x) 
  {//防止除0的尴尬情况
    tan_angle = 100;
  }else{
    tan_angle = fabs((right_lightbar_center.y - left_lightbar_center.y) /
    static_cast<float>((right_lightbar_center.x - left_lightbar_center.x)));    
  }
  float grade = 200;
  if (fabs(left_lightbar_angle - right_lightbar_angle) > 20)
	{    
    if (fabs(left_lightbar_angle - right_lightbar_angle) <= 160)
		{
      return false;      
    }           
  }
  float point_distance = getDistance(left_lightbar_center, right_lightbar_center);
  if (point_distance > 3 * height_max || point_distance < height_min) 
	{ 
    return false;         
  }
  if(height_min==0)
	{ 
    return false;         
  }
  if (height_max / height_min > 3) 
	{ 
      return false;           
  }
  if (fabs(fabs(left_lightbar_angle) - fabs(right_lightbar_angle)) > 160)
	{
    grade = grade - 2 * (180 - fabs(fabs(left_lightbar_angle) - fabs(right_lightbar_angle)));    
  }else {
        grade = grade - 2 * fabs(fabs(left_lightbar_angle) - fabs(right_lightbar_angle));    
  }
  grade = grade - atan(tan_angle) * 180 * 0.6;
  if (grade > 165) { 
    return true;    
  }else { 
    return false;           
  }
}

void ConstraintSet::FilterLights(std::vector<cv::RotatedRect> &lights, std::vector<cv::RotatedRect> &lights_copy) 
{
  std::vector<cv::RotatedRect> rects;
  std::vector<cv::RotatedRect> rects_copy;                                                                                          
  rects.reserve(lights.size());
  rects_copy.reserve(lights.size());                                                                                                
  for (unsigned int i =0; i < lights.size(); i++) 
	{
    float angle;
    cv::RotatedRect light = lights[i];
    cv::RotatedRect light_copy = lights_copy[i];
    float angle_copy;
    auto light_aspect_ratio_copy =
        std::max(light_copy.size.width, light_copy.size.height) / std::min(light_copy.size.width, light_copy.size.height);
	  //https://stackoverflow.com/questions/15956124/minarearect-angles-unsure-about-the-angle-returned/21427814#21427814
    if(light.size.width < light.size.height) {
      angle =light.angle; // -light.angle
    } else{
      angle =light.angle; // light.angle + 90
    }
	  rects.push_back(light);
          rects_copy.push_back(light_copy);
	  if (enable_debug_)
		{
      cv_toolbox_->DrawRotatedRect(show_lights_after_filter_, light, cv::Scalar(0, 255, 0), 2, angle);
      cv_toolbox_->DrawRotatedRect(show_lights_after_filter_, light, cv::Scalar(0, 255, 0), 2, light.angle);
    }
  }
  if (enable_debug_)
    cv::imshow("lights_after_filter", show_lights_after_filter_);
  lights = rects;
  lights_copy = rects_copy;
}


void ConstraintSet::GetCorrectPointOder(const cv::RotatedRect &rotatedRect ,cv::Point2f &bl,cv::Point2f &tl,cv::Point2f &tr,cv::Point2f &br)
{ 
    cv::Point2f vertices[4];                                                                                                            
    rotatedRect.points(vertices);                                                                                                       
    float max_y=0;                                                                                                                      
    float sec_y=0;                                                                                                                     
    int max_y_index=0;                                                                                                                  
    int sec_y_index=0;                                                                                                                
    int t_max_x_index=0;                                                                                                                
    float t_max_x=0;                                                                                                                    
    for(int i=0;i<4;i++)                                                                                                                
    {                                                                                                                                   
			if(vertices[i].y>=max_y)                                                                                                        
			{                                                                                                                               
				max_y=vertices[i].y;                                                                                                        
				max_y_index=i;                                                                                                              
			}                                                                                                                               
    }                                                                                                                                   
    for(int i=0;i<4;i++)                                                                                                                
    {                                                                                                                                   
			if(i!=max_y_index)                                                                                                             
			{                                                                                                                               
				if(vertices[i].y>=sec_y)                                                                                                    
				{                                                                                                                           
					sec_y=vertices[i].y;                                                                                                    
					sec_y_index=i;                                                                                                          
				}                                                                                                                           
			}                                                                                                                             
    }                                                                                                                                   
    if(vertices[max_y_index].x<vertices[sec_y_index].x)                                                                                 
    {                                                                                                                                   
			bl=vertices[max_y_index];                                                                                                       
			br=vertices[sec_y_index];                                                                                                       
    } else {                                                                                                                            
			bl = vertices[sec_y_index];                                                                                                    
			br = vertices[max_y_index];                                                                                                     
    }                                                                                                                                   
    for(int i=0;i<4;i++)                                                                                                                
    {                                                                                                                                   
			if(i!=sec_y_index&&i!=max_y_index)                                                                                              
			{                                                                                                                               
				if(vertices[i].x>=t_max_x)                                                                                                  
				{                                                                                                                           
					t_max_x=vertices[i].x;                                                                                                 
					t_max_x_index=i;                                                                                                        
				}                                                                                                                           
			}                                                                                                                               
    }                                                                                                                                   
    tr=vertices[t_max_x_index];                                                                                                         
    for(int i=0;i<4;i++)                                                                                                                
    {                                                                                                                                   
			if(i!=max_y_index&&i!=t_max_x_index&&i!=sec_y_index)                                                                            
				tl=vertices[i];                                                                                                             
    }                                                                                                                                   
} 


bool ConstraintSet::RectSafety(cv::Rect &brect, int rows, int cols)
{                                                                   
	cv::Rect out_rect=cv::Rect(0,0,cols,rows);                                                                                          
	brect=brect&out_rect;                                                                                                               
	if(brect.width == 0 || brect.height == 0){                                                                                          
		return false;                                                                                                                   
	} else {                                                                                                                            
		return true;                                                                                                                    
	}                                                                                                                                   
} 

bool ConstraintSet::RectSafety_1(cv::Rect &brect,cv::Size size) 
{                                                                       
	cv::Rect out_rect=cv::Rect(0,0,size.width,size.height);                                                                             
	brect=brect&out_rect;                                                                                                               
	if(brect.width == 0 || brect.height == 0){                                                                                          
		return false;                                                                                                                   
	} else {                                                                                                                            
		return true;                                                                                                                    
	}                                                                                                                                   
}  


cv::RotatedRect ConstraintSet::get_single_rotatedrect(const cv::RotatedRect &lightBar,bool right_or_left,cv::Point2f &bl,cv::Point2f &tl,cv::Point2f &tr,cv::Point2f &br, const float light0_width, const float light0_height, const float light0_angle_0)
{   //right=1,left=0           //
	cv::RotatedRect search_rotatedrect;                                                                                                //
	GetCorrectPointOder(lightBar,bl,tl,tr,br);                                                                                         //
	if(right_or_left)                                                                                                                  //
	{                                                                                                                                  //
		search_rotatedrect=cv::RotatedRect(cv::Point2f(lightBar.center.x+0.7*light0_width+ 0.8 *light0_height,          //
																							((tr.y+tl.y)/2+(br.y+bl.y)/2)/2),                                                      //
											cv::Size2f(2.3*light0_height,1.2*light0_height),light0_angle_0);                              //
	}else{
		search_rotatedrect=cv::RotatedRect(cv::Point2f(lightBar.center.x-0.7*light0_width - 0.8*light0_height,          //
																							((tr.y+tl.y)/2+(br.y+bl.y)/2)/2),                                                      //
											cv::Size2f(2.3*light0_height,1.2*light0_height),light0_angle_0);                                //
	}                                                                                                                                  //
	return search_rotatedrect;                                                                                                         //
}                                                                                                                                       

void ConstraintSet::MLHogSvm(cv::Mat &src,cv::RotatedRect search_rotatedrect_raw,const int enemy_color_,int single)
{  
  const cv::Size kWinSize=cv::Size(24, 24);                                                                                           
  const cv::Size kBlockSize=cv::Size(8, 8);                                                                                           
  const cv::Size kBlockStride = cv::Size(4, 4);                                                                                       
  const cv::Size kBellSize =  cv::Size(4, 4);                                                                                         
  cv::Mat img = src.clone();                                                                                                          
  cv::Mat armorimg;                                                                                                                  
  cv::RotatedRect search_rotatedrect;
  if(single==0)
  {
    search_rotatedrect=search_rotatedrect_raw;    
  }
  else
  {
    if(search_rotatedrect_raw.angle<90)
    {
      search_rotatedrect=cv::RotatedRect(search_rotatedrect_raw.center,cv::Size2f(search_rotatedrect_raw.size.height,search_rotatedrect_raw.size.width),search_rotatedrect_raw.angle-90);     
    }else{
      search_rotatedrect=cv::RotatedRect(search_rotatedrect_raw.center,cv::Size2f(search_rotatedrect_raw.size.width,search_rotatedrect_raw.size.height),search_rotatedrect_raw.angle-180);     
    }    
  }	 
  cv::RotatedRect roi = search_rotatedrect;                                                                                      
  cv::Rect roi_mat =roi.boundingRect();                                                                                          
  if(RectSafety(roi_mat,src.rows,src.cols)==false)
    return;                                                                                         
  cv::Mat roiimg = src(roi_mat);                                                                                                 
  cv::Point center = roi.center;                                                                                                 
  float angle = 0;                                                                                                               
  if(search_rotatedrect.angle<-45){
    angle = search_rotatedrect.angle+90;    
  }else{
    angle =search_rotatedrect.angle;    
  }
  cv::Mat rotationmat = cv::getRotationMatrix2D(center-roi_mat.tl(),angle, 1);	     
  if(rotationmat.empty())
    return;
  if(roiimg.empty())
    return; 	    
  cv::warpAffine(roiimg,armorimg,rotationmat,roiimg.size());  
  cv::Rect realsizerect;
  float dis_x,dis_y;
  switch(single)
  {
	case 0:
		dis_x = cvRound((roi_mat.width-search_rotatedrect.size.width)/2);
		dis_y = cvRound((roi_mat.height-search_rotatedrect.size.height)/2);
		if(search_rotatedrect.angle>-45)
		{
			realsizerect.x = center.x -roi_mat.x-cvRound(roi_mat.width/2)+dis_x;
			realsizerect.y = center.y -roi_mat.y-cvRound(roi_mat.height/2)+dis_y;
			realsizerect.width = search_rotatedrect.size.width;
			realsizerect.height = search_rotatedrect.size.height;	
		}else{
			realsizerect.x = center.x -roi_mat.x-cvRound(roi_mat.width/2);//+dis_y;
			realsizerect.y = center.y -roi_mat.y-cvRound(roi_mat.height/2);//+dis_x;
			realsizerect.width = search_rotatedrect.size.height;
			realsizerect.height = search_rotatedrect.size.width;	
		}
		break;
	case 1:
		dis_x = cvRound((roi_mat.width-search_rotatedrect.size.width)/2);
		dis_y = cvRound((roi_mat.height-search_rotatedrect.size.height)/2);
		if(search_rotatedrect.angle>-45)
		{
			realsizerect.x = center.x -roi_mat.x-cvRound(roi_mat.width/2);
			realsizerect.y = center.y -roi_mat.y-cvRound(roi_mat.height/2);
			realsizerect.width = search_rotatedrect.size.width;
			realsizerect.height = search_rotatedrect.size.height;	
		}else{
			realsizerect.x = center.x -roi_mat.x-cvRound(roi_mat.width/2);
			realsizerect.y = center.y -roi_mat.y-cvRound(roi_mat.height/2);
			realsizerect.width = search_rotatedrect.size.height;
			realsizerect.height = search_rotatedrect.size.width;	
		}
		break;
	case 2:
		dis_x = cvRound((roi_mat.width-search_rotatedrect.size.width)/2);
		dis_y = cvRound((roi_mat.height-search_rotatedrect.size.height)/2);
		if(search_rotatedrect.angle>-45)
		{
			realsizerect.x = center.x -roi_mat.x-cvRound(roi_mat.width/2)+dis_x;
			realsizerect.y = center.y -roi_mat.y-cvRound(roi_mat.height/2)+dis_y;
			realsizerect.width = search_rotatedrect.size.width;
			realsizerect.height = search_rotatedrect.size.height;	
		}else{
			realsizerect.x = center.x -roi_mat.x-cvRound(roi_mat.width/2);
			realsizerect.y = center.y -roi_mat.y-cvRound(roi_mat.height/2);
			realsizerect.width = search_rotatedrect.size.height;
			realsizerect.height = search_rotatedrect.size.width;	
		}
		break;
	default:
		break;   
  }
	if(RectSafety(realsizerect, armorimg.rows,armorimg.cols)==false)
		return;
	armorimg = armorimg(realsizerect);   
	std::vector<cv::Mat> rgb;                                                                                                     
	if(armorimg.empty())
	return;
	cv::split(armorimg,rgb);
	if(enemy_color_==0)
		armorimg = rgb[0];                                                                                                
	else 
		armorimg = rgb[2];                                                                                                 
	if(!armorimg.empty())
	{                                                                                                         

		if(enable_debug_)
		{
			cv::namedWindow("A",CV_WINDOW_AUTOSIZE);                                                                                     
			cv::imshow("A",armorimg);
			cv_toolbox_->DrawRotatedRect(img, search_rotatedrect, cv::Scalar(0, 0, 255), 2);
		}
	}else
		return;
	int Bin = 9;                                                                                                                       
	cv::HOGDescriptor HogDescriptor(kWinSize,kBlockSize,kBlockStride,kBellSize,Bin);                                                   
	std::vector<float> ArmorHog;                                                                                                       
	const cv::Size kWinStride = cv::Size(60, 80);                                                                                      
	cv::resize(armorimg, armorimg, cv::Size(24, 24), CV_INTER_CUBIC);                                                                  
	HogDescriptor.compute(armorimg, ArmorHog, kWinStride);                                                                             
	static cv::Ptr<cv::ml::SVM> svm_hog_1 = cv::ml::SVM::load("/home/tdt/catkin_ws/src/RoboRTS/roborts_detection/armor_detection/constraint_set/model/SVM(NEW_SMALL).xml");
	cv::Mat armorpredict(1, ArmorHog.size(), CV_32FC1); 																																																														
	for (int j = 0; j < ArmorHog.size(); j++) 
	{                                                                                        
		armorpredict.at<float>(j) = ArmorHog[j];                                                                                      
	}                                                                                                                                  
	int result_1;
	result_1 = svm_hog_1->predict(armorpredict);
	svmresult=0;
	float tmp_height,tmp_width,tmp_ratio;
	if(search_rotatedrect.size.height>search_rotatedrect.size.width)
	{
		tmp_height = search_rotatedrect.size.height;
		tmp_width = search_rotatedrect.size.width;
	}else{
		tmp_height = search_rotatedrect.size.width;
		tmp_width = search_rotatedrect.size.height;
	}
	tmp_ratio = tmp_height/tmp_width;
	if(tmp_ratio>=2&&tmp_ratio<=2.5)
	{
		svmresult =1;
		armor_id =1;
	};
	if(tmp_ratio>=1.4&&tmp_ratio<=1.8)
	{
		svmresult =1;
		armor_id =2;
	};

	switch(result_1)
	{
	case 1:                                                                            
		svmresult=1;                                                                                                                   
		armor_id=1;
		break;
	case 2:
		svmresult=1;                                                                                                                   
		armor_id=2;
		break; 
	case 3:                                                                        
		svmresult=1;                                                                                                                   
		armor_id=3;
		break;
	default:
		break;
	}
	img.release();
	armorimg.release();
	roiimg.release();
	rotationmat.release();
	armorpredict.release();
	std::vector<cv::Mat>().swap(rgb);     
}  

void ConstraintSet::PossibleArmors(const std::vector<cv::RotatedRect> &lights, std::vector<ArmorInfo> &armors, std::vector<cv::RotatedRect> &lights_copy) 
{
  //std::cout << "********************************************PossibleArmors********************************************" << std::endl;  
  std::vector<bool> see(lights.size(),0); 
//******双灯条******//
#if 1
	for (unsigned int i = 0; i < lights.size(); i++) 
	{
		if(see[i]==1)
			continue;
		for (unsigned int j = i + 1; j < lights.size(); j++) 
		{
			svmresult=0;
			armor_id=0;
			if(see[j]==1)
				continue;
			cv::RotatedRect light1 = lights[i];
			cv::RotatedRect light2 = lights[j];
			cv::RotatedRect light1_copy = lights_copy[i];
			cv::RotatedRect light2_copy = lights_copy[j];
			auto edge1 = std::minmax(light1.size.width, light1.size.height);
			auto edge2 = std::minmax(light2.size.width, light2.size.height);
			auto lights_dis = std::sqrt((light1.center.x - light2.center.x) * (light1.center.x - light2.center.x) +
					(light1.center.y - light2.center.y) * (light1.center.y - light2.center.y));
			auto center_angle = std::atan(std::abs(light1.center.y - light2.center.y) / std::abs(light1.center.x - light2.center.x)) * 180 / CV_PI;
			center_angle = center_angle > 90 ? 180 - center_angle : center_angle;
			cv::RotatedRect rect;
			rect.angle = static_cast<float>(center_angle);
			rect.center.x = (light1.center.x + light2.center.x) / 2;
			rect.center.y = (light1.center.y + light2.center.y) / 2;
			float armor_width = std::abs(static_cast<float>(lights_dis) - std::max(edge1.first, edge2.first));
			float armor_height = std::max<float>(edge1.second, edge2.second);

			rect.size.width = std::max<float>(armor_width, armor_height);
			rect.size.height = std::min<float>(armor_width, armor_height);

			float light1_angle = light1.angle;
			float light2_angle = light2.angle;
			auto angle_diff = std::abs(light1_angle - light2_angle);
			if (angle_diff > 175) 
			{
				angle_diff = 180 -angle_diff;
			}
			float left_lightbar_width,left_lightbar_height,right_lightbar_width,right_lightbar_height,rotatedrect_angle;
			float light1_angle_1,light2_angle_2;
			int height_min_rotated,width,height_min_rect;
			cv::Point rotatedrect_center,search_rect_tl,search_rect_br;
			cv::RotatedRect search_rotatedrect;
			cv::Rect search_rect;
			cv::Point2f left_bl, left_tl, left_tr, left_br;
			cv::Point2f right_bl, right_tl, right_tr, right_br;
			cv::RotatedRect Numb_rotatedrect;
			cv::Point2f Numb_rects_center;
			float height_tmp, width_tmp;
			cv::RotatedRect lightbar_tmp;
			float search_rect_width, search_rect_height;
			std::vector<cv::Rect> Numb_rects;
			cv::Rect Numb_rect;
			cv::Point2f Numb_bl, Numb_tl, Numb_tr, Numb_br;
			std::vector<cv::Point2f> Numb_points;
			if (light1.center.x < light2.center.x) 
			{
				if(get_eligibility(light1, light2) )
				{
					if (light1_copy.size.width < light1_copy.size.height) 
					{
						left_lightbar_width = light1_copy.size.width;
						left_lightbar_height = light1_copy.size.height;
						light1_angle_1 = light1_copy.angle+90;
					}else {
						left_lightbar_width = light1_copy.size.height;
						left_lightbar_height = light1_copy.size.width;
						light1_angle_1 = light1_copy.angle+180;
					}
					if (light2_copy.size.width < light2_copy.size.height) 
					{
						right_lightbar_width = light2_copy.size.width;
						right_lightbar_height = light2_copy.size.height;
						light2_angle_2 = light2_copy.angle+90;
					}else {
						right_lightbar_width = light2_copy.size.height;
						right_lightbar_height = light2_copy.size.width;
						light2_angle_2 = light2_copy.angle+180;

					}
					height_min_rotated = 2.5*std::max(left_lightbar_height,right_lightbar_height);
					width = abs(abs(light1_copy.center.x-light2_copy.center.x)-3*std::max(left_lightbar_width,right_lightbar_width));
					rotatedrect_center=(light1_copy.center+light2_copy.center)/2;
					rotatedrect_angle=(left_lightbar_height>right_lightbar_height?
					light1_copy.angle:light2_copy.angle);
					if(rotatedrect_angle<-45)
					{	
						search_rotatedrect=cv::RotatedRect(rotatedrect_center,cv::Size2f(height_min_rotated,width),rotatedrect_angle);
					}else{
						search_rotatedrect=cv::RotatedRect(rotatedrect_center,cv::Size2f(width,height_min_rotated),rotatedrect_angle);
					}
					height_min_rect = std::max(left_lightbar_height,right_lightbar_height);
					GetCorrectPointOder(light1_copy, left_bl, left_tl, left_tr, left_br);
					GetCorrectPointOder(light2_copy, right_bl, right_tl, right_tr, right_br);
					search_rect_tl.x=std::max(left_br.x,left_tr.x)+6;
					search_rect_tl.y= (light1_copy.center.y+light2_copy.center.y)/2 - 
					height_min_rect-abs(light1_copy.center.y-light2_copy.center.y);
					search_rect_tl.y= (light1_copy.center.y+light2_copy.center.y)/2 - 
					height_min_rect-abs(light1_copy.center.y-light2_copy.center.y);
					search_rect_br.x=std::min(right_bl.x,right_tl.x)-6;
					search_rect_br.y=(light1_copy.center.y+light2_copy.center.y)/2 + 
					height_min_rect+abs(light1_copy.center.y-light2_copy.center.y);
					search_rect_br.y=(light1_copy.center.y+light2_copy.center.y)/2 + 
					height_min_rect+abs(light1_copy.center.y-light2_copy.center.y);
					cv::Rect search_rect=cv::Rect(search_rect_tl,search_rect_br);
					bool safety=RectSafety(search_rect,img_heigth,img_width);
					if(safety== false)
						continue;
					cv::Mat image_numb=src_img_(search_rect);
					std::vector<cv::Mat> rgb;
					split(image_numb,rgb);
					if(enemy_color_==RED)
						image_numb=rgb[0];
					else
						image_numb=rgb[2];
					double threshold_value=cv::threshold(image_numb,image_numb,0,255,CV_THRESH_OTSU);
					cv::threshold(image_numb,image_numb,threshold_value,255,CV_THRESH_BINARY);
					std::vector<std::vector<cv::Point>>contours;
					cv::findContours(image_numb,contours,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
					for(std::vector<cv::Point> const &contour : contours)
					{
						if (fabs(contourArea(contour, 1)) < 5)
							continue;
						Numb_rect = boundingRect(contour);
						if(RectSafety(Numb_rect,img_heigth,img_width)==false)
							continue;
						if (Numb_rect.height == 0 || Numb_rect.width == 0)
							continue;
						if (Numb_rect.area() < 0.1 * search_rect.area())
							continue;
						Numb_rects.push_back(Numb_rect);
					}
					if(Numb_rects.empty())
						continue;
					for(int z=1;z<Numb_rects.size();z++)
					{
						Numb_rects[0]=Numb_rects[0].area()>Numb_rects[i].area() ? Numb_rects[0]:Numb_rects[i];
					}
					search_rect_width=search_rect.size().width < search_rect.size().height?search_rect.size().width:search_rect.size().height;
					search_rect_height=search_rect.size().width > search_rect.size().height?search_rect.size().width:search_rect.size().height;
					lightbar_tmp=light1_angle_1>light2_angle_2?light1_copy:light2_copy;
					width_tmp=lightbar_tmp.angle>-45?Numb_rects[0].width:Numb_rects[0].height;
					height_tmp=lightbar_tmp.angle<-45?Numb_rects[0].width:Numb_rects[0].height;
					Numb_rects_center=(Numb_rects[0].tl()+Numb_rects[0].br())/2;
					if(width_tmp>5 && height_tmp>5 && Numb_rects_center.x >=0 && Numb_rects_center.y>=0)
					{
						Numb_rotatedrect = cv::RotatedRect(Numb_rects_center + search_rotatedrect.center -
																									cv::Point2f(search_rect_width / 2, search_rect_height / 2),
																									cv::Size2f(width_tmp, height_tmp), lightbar_tmp.angle);
					}else
						continue;
					if(Numb_rotatedrect.size.area()<10 || Numb_rotatedrect.center.x < 0
								|| Numb_rotatedrect.center.y < 0)
						continue;
					if(last_armor_.id==0)
					{
						MLHogSvm(src_img_,Numb_rotatedrect,enemy_color_,0);
						if(svmresult)
						{
							GetCorrectPointOder(Numb_rotatedrect, Numb_bl, Numb_tl, Numb_tr, Numb_br);
							Numb_points.push_back(Numb_tl);
							Numb_points.push_back(Numb_tr);
							Numb_points.push_back(Numb_br);
							Numb_points.push_back(Numb_bl);
							cv::Rect trackingrect;
							trackingrect=search_rotatedrect.boundingRect();
							if(RectSafety(trackingrect,img_heigth,img_width)==false)
								continue;
							int thre=cvRound(threshold(image_numb,image_numb,0,255,CV_THRESH_OTSU));
							armors.emplace_back(ArmorInfo(Numb_rotatedrect, Numb_points, 0, armor_id, trackingrect, thre));
							see[i]=1;
							see[j]=1;
						}else
						{
							std::cout<<" <<<MLHogSvm didn't passed "<<std::endl;
						}
						if (enable_debug_)
						{
							cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, rect, cv::Scalar(0, 255, 0), 2);
							cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, Numb_rotatedrect, cv::Scalar(0, 100, 100), 2);
						}
						Numb_points.clear();
						std::vector<cv::Rect>().swap(Numb_rects);
						std::vector<cv::Point2f>().swap(Numb_points);
						std::vector<cv::Mat>().swap(rgb);
						std::vector<std::vector<cv::Point>>().swap(contours);					
					}else{
						cv::Rect trackingrect;
						trackingrect=search_rotatedrect.boundingRect();
						if(RectSafety(trackingrect,img_heigth,img_width)==false)
							continue;
						GetCorrectPointOder(Numb_rotatedrect, Numb_bl, Numb_tl, Numb_tr, Numb_br);
						Numb_points.clear();
						std::vector<cv::Point2f>().swap(Numb_points);
						Numb_points.push_back(Numb_tl);
						Numb_points.push_back(Numb_tr);
						Numb_points.push_back(Numb_br);
						Numb_points.push_back(Numb_bl);
						int thre=cvRound(threshold(image_numb,image_numb,0,255,CV_THRESH_OTSU));
						ArmorInfo tmp_armor(Numb_rotatedrect, Numb_points, 0, last_armor_.id, trackingrect, thre);
						SetMlRoi(tmp_armor,src_img_);
						if(FastTargetLock(tmp_armor,last_armor_)==true)
						{
							std::vector<ArmorInfo> tmp_armors;
							tmp_armors.emplace_back(ArmorInfo(Numb_rotatedrect, Numb_points, 0, last_armor_.id, trackingrect, thre));
							armors=tmp_armors;
							cv_toolbox_->DrawRotatedRect(src_img_, Numb_rotatedrect, cv::Scalar(0, 0, 255), 2);
							tmp_armors.clear();
							std::vector<ArmorInfo>().swap(tmp_armors);
							return;
						}else{
							MLHogSvm(src_img_,Numb_rotatedrect,enemy_color_,0);
							if(svmresult)
							{
								int thre=cvRound(threshold(image_numb,image_numb,0,255,CV_THRESH_OTSU));
								armors.emplace_back(ArmorInfo(Numb_rotatedrect, Numb_points, 0, armor_id, trackingrect, thre));
								see[i]=1;
								see[j]=1;
							}else{
								std::cout<<"<<<MLHogSvm didn't passed "<<std::endl;
							}
							if (enable_debug_)
							{
								cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, rect, cv::Scalar(0, 255, 0), 2);
								cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, Numb_rotatedrect, cv::Scalar(0, 100, 100), 2);

							}
							Numb_points.clear();
							std::vector<cv::Rect>().swap(Numb_rects);
							std::vector<cv::Point2f>().swap(Numb_points);
							std::vector<cv::Mat>().swap(rgb);
							std::vector<std::vector<cv::Point>>().swap(contours);						
						}					
					}			
				}else{
					std::cout<<" <<<get_eligibility didn't passed "<<std::endl;				
				}			
			}else{
				if(get_eligibility(light2, light1) )
				{
					if (light1_copy.size.width < light1_copy.size.height)
					{
						right_lightbar_width = light1_copy.size.width;
						right_lightbar_height = light1_copy.size.height;
					} else {
						right_lightbar_width = light1_copy.size.height;
						right_lightbar_height = light1_copy.size.width;
					}
					if (light2_copy.size.width < light2_copy.size.height) 
					{
						left_lightbar_width = light2_copy.size.width;
						left_lightbar_height = light2_copy.size.height;
					} else{
						left_lightbar_width = light2_copy.size.height;
						left_lightbar_height = light2_copy.size.width;
					}
					height_min_rotated = 2.5*std::max(left_lightbar_height,right_lightbar_height);
					width = abs(abs(light2_copy.center.x-light1_copy.center.x)-3*std::max(left_lightbar_width,right_lightbar_width));
					rotatedrect_center=(light2_copy.center+light1_copy.center)/2;
					rotatedrect_angle=(left_lightbar_height>right_lightbar_height?
					light2_copy.angle:light1_copy.angle);
					if(rotatedrect_angle<-45)
					{
					search_rotatedrect=cv::RotatedRect(rotatedrect_center,cv::Size2f(height_min_rotated,width),rotatedrect_angle);

					}
					else
					{
					search_rotatedrect=cv::RotatedRect(rotatedrect_center,cv::Size2f(width,height_min_rotated),rotatedrect_angle);

					}
					height_min_rect = std::max(left_lightbar_height,right_lightbar_height);
					GetCorrectPointOder(light2_copy, left_bl, left_tl, left_tr, left_br);
					GetCorrectPointOder(light1_copy, right_bl, right_tl, right_tr, right_br);
					search_rect_tl.x=std::max(left_br.x,left_tr.x)+6;
					search_rect_tl.y= (light1_copy.center.y+light2_copy.center.y)/2 - 
					height_min_rect-abs(light2_copy.center.y-light1_copy.center.y);
					search_rect_tl.y= (light1_copy.center.y+light2_copy.center.y)/2 - 
					height_min_rect-abs(light2_copy.center.y-light1_copy.center.y);
					search_rect_br.x=std::min(right_bl.x,right_tl.x)-6;
					search_rect_br.y=(light1_copy.center.y+light2_copy.center.y)/2 + 
					height_min_rect+abs(light2_copy.center.y-light1_copy.center.y);
					search_rect_br.y=(light1_copy.center.y+light2_copy.center.y)/2 + 
					height_min_rect+abs(light2_copy.center.y-light1_copy.center.y);
					cv::Rect search_rect=cv::Rect(search_rect_tl,search_rect_br);
					bool safety=RectSafety(search_rect,img_heigth,img_width);
					if(safety== false)
					continue;
					if(search_rect.width==1||search_rect.height==1)
					{
						continue;
					}
					cv::Mat image_numb=src_img_(search_rect);
					std::vector<cv::Mat> rgb;
					split(image_numb,rgb);
					if(enemy_color_==RED)
						image_numb=rgb[0];
					else
						image_numb=rgb[2];
					double threshold_value=cv::threshold(image_numb,image_numb,0,255,CV_THRESH_OTSU);
					cv::threshold(image_numb,image_numb,threshold_value,255,CV_THRESH_BINARY);
					std::vector<std::vector<cv::Point>>contours;
					cv::findContours(image_numb,contours,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
					for(std::vector<cv::Point> const &contour : contours)
					{
						if (fabs(contourArea(contour, 1)) < 5)
							continue;
						Numb_rect = boundingRect(contour);
						if (Numb_rect.height == 0 || Numb_rect.width == 0)
							continue;
						if (Numb_rect.area() < 0.1 * search_rect.area())
							continue;
						Numb_rects.push_back(Numb_rect);
					}
					if(Numb_rects.empty())
						continue;
					for(int z=1;z<Numb_rects.size();z++)
					{
						Numb_rects[0]=Numb_rects[0].area()>Numb_rects[i].area() ? Numb_rects[0]:Numb_rects[i];
					}
					search_rect_width=search_rect.size().width < search_rect.size().height?search_rect.size().width:search_rect.size().height;
					search_rect_height=search_rect.size().width > search_rect.size().height?search_rect.size().width:search_rect.size().height;
					lightbar_tmp=light2_angle_2>light1_angle_1?light2_copy:light1_copy;
					width_tmp=lightbar_tmp.angle>-45?Numb_rects[0].width:Numb_rects[0].height;
					height_tmp=lightbar_tmp.angle<-45?Numb_rects[0].width:Numb_rects[0].height;
					Numb_rects_center=(Numb_rects[0].tl()+Numb_rects[0].br())/2;
					if(width_tmp>5 && height_tmp>5 && Numb_rects_center.x >=0 && Numb_rects_center.y>=0)
					{
						Numb_rotatedrect = cv::RotatedRect(Numb_rects_center + search_rotatedrect.center - 
						cv::Point2f(search_rect_width / 2, search_rect_height / 2),
						cv::Size2f(width_tmp, height_tmp), lightbar_tmp.angle);

					}else
						continue;
					if(Numb_rotatedrect.size.area()<10 || Numb_rotatedrect.center.x < 0|| Numb_rotatedrect.center.y < 0)
						continue;
					if(last_armor_.id==0)
					{			
						std::cout<<" >>>last_armor_.id==0 "<<std::endl;
						MLHogSvm(src_img_,Numb_rotatedrect,enemy_color_,0);
						if(svmresult)
						{
							GetCorrectPointOder(Numb_rotatedrect, Numb_bl, Numb_tl, Numb_tr, Numb_br);
							Numb_points.push_back(Numb_tl);
							Numb_points.push_back(Numb_tr);
							Numb_points.push_back(Numb_br);
							Numb_points.push_back(Numb_bl);
							cv::Rect trackingrect;
							trackingrect=search_rotatedrect.boundingRect();
							if(RectSafety(trackingrect,img_heigth,img_width)==false)
							continue;
							int thre=cvRound(threshold(image_numb,image_numb,0,255,CV_THRESH_OTSU));
							armors.emplace_back(ArmorInfo(Numb_rotatedrect, Numb_points, 0, armor_id, trackingrect, thre));
							see[i]=1;
							see[j]=1;
						}else{
							std::cout<<" >>>MLHogSvm didn't passed "<<std::endl;
						}
						if (enable_debug_)
						{
							cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, rect, cv::Scalar(0, 255, 0), 2);
							cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, Numb_rotatedrect, cv::Scalar(0, 100, 100), 2);
						}
						Numb_points.clear();
						std::vector<cv::Rect>().swap(Numb_rects);
						std::vector<cv::Point2f>().swap(Numb_points);
						std::vector<cv::Mat>().swap(rgb);
						std::vector<std::vector<cv::Point>>().swap(contours);
					}else{
						cv::Rect trackingrect;
						trackingrect=search_rotatedrect.boundingRect();
						if(RectSafety(trackingrect,img_heigth,img_width)==false)
							continue;
						GetCorrectPointOder(Numb_rotatedrect, Numb_bl, Numb_tl, Numb_tr, Numb_br);
						Numb_points.clear();
						std::vector<cv::Point2f>().swap(Numb_points);
						Numb_points.push_back(Numb_tl);
						Numb_points.push_back(Numb_tr);
						Numb_points.push_back(Numb_br);
						Numb_points.push_back(Numb_bl);
						int thre=cvRound(threshold(image_numb,image_numb,0,255,CV_THRESH_OTSU));
						ArmorInfo tmp_armor(Numb_rotatedrect, Numb_points, 0, last_armor_.id, trackingrect, thre);
						SetMlRoi(tmp_armor,src_img_);
						if(FastTargetLock(tmp_armor,last_armor_)==true)
						{
							std::vector<ArmorInfo> tmp_armors;
							tmp_armors.emplace_back(ArmorInfo(Numb_rotatedrect, Numb_points, 0, last_armor_.id, trackingrect, thre));
							armors=tmp_armors;
							cv_toolbox_->DrawRotatedRect(src_img_, Numb_rotatedrect, cv::Scalar(0, 0, 255), 2);
							tmp_armors.clear();
							std::vector<ArmorInfo>().swap(tmp_armors);
							return;
						}else{
							MLHogSvm(src_img_,Numb_rotatedrect,enemy_color_,0);
							if(svmresult)
							{
								int thre=cvRound(threshold(image_numb,image_numb,0,255,CV_THRESH_OTSU));
								armors.emplace_back(ArmorInfo(Numb_rotatedrect, Numb_points, 0, armor_id, trackingrect, thre));
								see[i]=1;
								see[j]=1;
							}else{
								std::cout<<" >>>MLHogSvm didn't passed "<<std::endl;
							}
							if (enable_debug_)
							{
								cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, rect, cv::Scalar(0, 255, 0), 2);
								cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, Numb_rotatedrect, cv::Scalar(0, 100, 100), 2);
							}
							Numb_points.clear();
							std::vector<cv::Rect>().swap(Numb_rects);
							std::vector<cv::Point2f>().swap(Numb_points);
							std::vector<cv::Mat>().swap(rgb);
							std::vector<std::vector<cv::Point>>().swap(contours);								
						}				
					}
				}else{
					std::cout<<">>>get_eligibility didn't passed "<<std::endl;
				}		
			}							
		}			
	}
#endif

#if 1
//******单灯条******//
	cv::Point2f Numb_bl, Numb_tl, Numb_tr, Numb_br;
	std::vector<cv::Point2f> Numb_points;
	for(unsigned int k = 0; k < lights.size(); k++) 
	{
		svmresult=0;
		armor_id=0;
		cv::RotatedRect light0 = lights_copy[k];
		float light0_angle_0;
		float light0_width,light0_height;
		if (light0.size.width < light0.size.height) 
		{
		light0_angle_0 = light0.angle+90;
		light0_width = light0.size.width;
		light0_height = light0.size.height;
		}else{
			light0_angle_0 = light0.angle+180;
			light0_width = light0.size.height;
			light0_height = light0.size.width;
		}
		cv::RotatedRect left_search_rect, right_search_rect;
		cv::Point2f bl,tl,tr,br;
		if(see[k])
			continue;
		else {
			if(fabs(light0_angle_0) > 90) 
			{
				left_search_rect = get_single_rotatedrect(light0, false,bl,tl,tr,br,light0_width,light0_height,light0_angle_0);
			}
			if(fabs(light0_angle_0) < 90)
			{
				right_search_rect = get_single_rotatedrect(light0, true,bl,tl,tr,br,light0_width,light0_height,light0_angle_0);
			}
		}
		cv::RotatedRect search_rect_0;
		for(int lr=0;lr<2;lr++) 
		{
			if(lr==0)
			{
				search_rect_0 = left_search_rect;
			}else {
				search_rect_0 = right_search_rect;
			}
			if(search_rect_0.boundingRect().area()<600)
				continue;
			cv::Rect rectsaft=search_rect_0.boundingRect();
			bool safety=RectSafety(rectsaft,img_heigth,img_width);
			if(safety== false)
				continue;
			cv::Mat image_roi;
			cv::Mat tmp=src_img_.clone();
			cv::Mat image_roi_tmp=tmp(rectsaft);
			cv::Mat matri;
			if(lr==1)
				matri=cv::getRotationMatrix2D(search_rect_0.center-(cv::Point2f)search_rect_0.boundingRect().tl(),light0.angle,1);
			else
				matri=cv::getRotationMatrix2D(search_rect_0.center-(cv::Point2f)search_rect_0.boundingRect().tl(),light0.angle+90,1);
			if(image_roi_tmp.empty())
				continue;
			cv::warpAffine(image_roi_tmp,image_roi,matri,rectsaft.size());
			float search_rect_width=search_rect_0.size.width < search_rect_0.size.height?search_rect_0.size.width:search_rect_0.size.height;
			float search_rect_height=search_rect_0.size.width > search_rect_0.size.height?search_rect_0.size.width:search_rect_0.size.height;
			float numb_point_x=image_roi.cols/2 - search_rect_width/2;
			float numb_point_y=image_roi.rows/2 - search_rect_height/2;
			cv::Rect last_roi(numb_point_x,numb_point_y,search_rect_width,search_rect_height);
			bool safety_1 = RectSafety_1(last_roi,image_roi.size());
			if(safety_1==false)
				continue;
			cv::Mat last_image=image_roi(last_roi);
			cv::Mat otus_image;
			cv::Rect matri_rect;
			bool safety_otus;
			if(lr==0)
			{
				matri_rect=cv::Rect(cv::Point(0.65*last_image.cols,0.35*last_image.rows),cv::Point(0.95*last_image.cols,0.65*last_image.rows));
				safety_otus = RectSafety(matri_rect, last_image.rows, last_image.cols);
				if(safety_otus==false)
				continue;
				otus_image=last_image(matri_rect);
			}else{
				matri_rect=cv::Rect(cv::Point(0.05*last_image.cols,0.35*last_image.rows),cv::Point(0.45*last_image.cols,0.65*last_image.rows));
				safety_otus = RectSafety(matri_rect, last_image.rows, last_image.cols);
				if(safety_otus==false)
				continue;
				otus_image=last_image(matri_rect);
			}
			if(otus_image.empty())
				continue;
			std::vector<cv::Mat> rgb1;
			std::vector<cv::Mat> rgb2;
			split(otus_image, rgb1);
			split(last_image,rgb2);
			if(enemy_color_==RED)
			{
				otus_image= rgb1[0];
				last_image=rgb2[0];
			}else{
				otus_image=rgb1[2];
				last_image=rgb2[2];
			}
			int thre;
			if(!region_otsu_threshold(last_image,last_image,thre, bool(lr))) continue;
			double threshold_value=cv::threshold(otus_image,otus_image,0,255,CV_THRESH_OTSU);
			cv::threshold(last_image,last_image,threshold_value,255,cv::THRESH_BINARY);
			std::vector<std::vector<cv::Point>>contours;
			cv::findContours(last_image,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
			std::vector<cv::RotatedRect> Numb_rects;
			for(std::vector<cv::Point> const &contour : contours) 
			{
				if (fabs(contourArea(contour, 1)) < 5)
					continue;
				cv::RotatedRect Numb_rect = minAreaRect(contour);
				if (Numb_rect.size.height == 0 || Numb_rect.size.width == 0)
					continue;
				if (Numb_rect.size.area() < 0.1 * last_roi.size().area())
					continue;
				Numb_rects.push_back(Numb_rect);
			}
			if(Numb_rects.empty())
				continue;
			for(int i=0;i<2;i++)
			{
				float max_area=0;
				int max_i=0;
				for(int j=i;j<Numb_rects.size();j++)
				{
					if(Numb_rects[j].size.area()>max_area)
					{
						max_area=Numb_rects[j].size.area();
						max_i=j;
					}
					cv::RotatedRect tmp_rect=Numb_rects[i];
					Numb_rects[i]=Numb_rects[max_i];
					Numb_rects[max_i]=tmp_rect;
				}		
			}
			if(lr==1)
			{
			if(Numb_rects.size()>1)
			{
					Numb_rects[0]=Numb_rects[0].center.x<Numb_rects[1].center.x? Numb_rects[0]:Numb_rects[1];
			}
			}else{
				if(Numb_rects.size()>1)
				{
					Numb_rects[0]=Numb_rects[0].center.x>Numb_rects[1].center.x? Numb_rects[0]:Numb_rects[1];
				}
			}
			float width;
			float heigth;
			if(light0.angle>-45) 
			{
				width = Numb_rects[0].size.width < Numb_rects[0].size.height ?
								Numb_rects[0].size.width : Numb_rects[0].size.height;
				heigth = Numb_rects[0].size.width > Numb_rects[0].size.height ?
								Numb_rects[0].size.width : Numb_rects[0].size.height;
			}
			if(light0.angle<-45)
			{
				width = Numb_rects[0].size.width > Numb_rects[0].size.height ?
								Numb_rects[0].size.width : Numb_rects[0].size.height;
				heigth = Numb_rects[0].size.width < Numb_rects[0].size.height ?
								Numb_rects[0].size.width : Numb_rects[0].size.height;
			}
			cv::Point2f Numb_rects_center=Numb_rects[0].center;
			cv::RotatedRect Numb_rotatedrect;
			if(width>5 && heigth>5 && Numb_rects_center.x >=0 && Numb_rects_center.y>=0) 
			{
				Numb_rotatedrect = cv::RotatedRect(Numb_rects_center + search_rect_0.center -
																			cv::Point2f(search_rect_width / 2, search_rect_height / 2),
																			cv::Size2f(width, heigth), light0.angle);
			}
			else
				continue;
			if(Numb_rotatedrect.size.area()<10 || Numb_rotatedrect.center.x < 0
				|| Numb_rotatedrect.center.y < 0)
				continue;
			if(lr==0)
				MLHogSvm(src_img_,Numb_rotatedrect,enemy_color_,1);
			else
				MLHogSvm(src_img_,Numb_rotatedrect,enemy_color_,2);
			if(svmresult)
			{
				GetCorrectPointOder(Numb_rotatedrect, Numb_bl, Numb_tl, Numb_tr, Numb_br);
				Numb_points.push_back(Numb_tl);
				Numb_points.push_back(Numb_tr);
				Numb_points.push_back(Numb_br);
				Numb_points.push_back(Numb_bl);
				cv::Rect trackingrect;
				trackingrect=search_rect_0.boundingRect();
				if(RectSafety(trackingrect,img_heigth,img_width)==false)
					continue;
				armors.emplace_back(ArmorInfo(Numb_rotatedrect, Numb_points, 1, armor_id, trackingrect, thre));
				Numb_points.clear();
				std::vector<cv::Point2f>().swap(Numb_points);
				std::vector<cv::RotatedRect>().swap(Numb_rects);
				std::vector<std::vector<cv::Point>>().swap(contours);
				see[k]=1;
			}else
				continue;
			if (enable_debug_)
				{
					cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, Numb_rotatedrect, cv::Scalar(0, 100, 100), 2);
					cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, light0, cv::Scalar(255, 0, 0), 2);
				}		
		}			
	}
#endif

	if (enable_debug_)
    cv::imshow("armors_before_filter", show_armors_befor_filter_);
  std::vector<bool>().swap(see);
}

void ConstraintSet::FilterArmors(std::vector<ArmorInfo> &armors) 
{
  cv::Mat mask = cv::Mat::zeros(gray_img_.size(), CV_8UC1);
  for (auto armor_iter = armors.begin(); armor_iter != armors.end();) 
	{
    cv::Point pts[4];
    for (unsigned int i = 0; i < 4; i++)
		{
      pts[i].x = (int) armor_iter->vertex[i].x;
      pts[i].y = (int) armor_iter->vertex[i].y;
    }
    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255), 8, 0);
    cv::Mat mat_mean;
    cv::Mat mat_stddev;
    cv::meanStdDev(gray_img_, mat_mean, mat_stddev, mask);
    auto stddev = mat_stddev.at<double>(0, 0);
    auto mean = mat_mean.at<double>(0, 0);
    if (stddev > armor_max_stddev_ || mean > armor_max_mean_) 
		{
      armor_iter = armors.erase(armor_iter);
    } else {
      armor_iter++;
    }
  }
  std::vector<bool> is_armor(armors.size(), true);
  for (int i = 0; i < armors.size() && is_armor[i] == true; i++) 
	{
    for (int j = i + 1; j < armors.size() && is_armor[j]; j++) 
		{
      float dx = armors[i].rect.center.x - armors[j].rect.center.x;
      float dy = armors[i].rect.center.y - armors[j].rect.center.y;
      float dis = std::sqrt(dx * dx + dy * dy);
      if (dis < armors[i].rect.size.width + armors[j].rect.size.width) 
			{
        if (armors[i].rect.angle > armors[j].rect.angle)
				{
          is_armor[i] = false;
        } else {
          is_armor[j] = false;
        }
      }
    }
  }
  for (unsigned int i = 0; i < armors.size(); i++) 
	{
    if (!is_armor[i]) 
		{
      armors.erase(armors.begin() + i);
      is_armor.erase(is_armor.begin() + i);
    } else if (enable_debug_) 
		{
      cv_toolbox_->DrawRotatedRect(show_armors_after_filter_, armors[i].rect, cv::Scalar(0, 255, 0), 2);
    }
  }
  if (enable_debug_)
  {
    cv::imshow("armors_after_filter", show_armors_after_filter_);
  }
}

void ConstraintSet::set_last_armor(ArmorInfo last_armor)
{
    //lock_guard<mutex> guard(last_armor_mutex_);//互斥锁，构造上锁析构解锁
    last_armor_=last_armor;
    //test = 1;
}
ArmorInfo ConstraintSet::get_last_armor()const
{
    //lock_guard<mutex> guard(last_armor_mutex_);//互斥锁，构造上锁析构解锁
    return last_armor_;
}

void ConstraintSet::SetMlRoi(ArmorInfo &armor,const cv::Mat &src) 
{
	cv::Mat armorimg;
	cv::RotatedRect roi=armor.rect;
	cv::Rect roi_mat=roi.boundingRect();
	if(RectSafety(roi_mat,img_heigth,img_width)==false)
		return;
	cv::Rect roi_matcopy=Rect(roi_mat);
	if(RectSafety_1(roi_matcopy,src.size()))
	{
		cv::Mat roiimg;
		if(roi_matcopy!=roi_mat)
		{
				roiimg=Mat::zeros(roi_mat.size(),CV_8UC3);
				src(roi_matcopy).copyTo(roiimg(Rect(Point(0,0),roi_matcopy.size())));
		} else{
				roiimg=src(roi_mat);
		}
		float angle;
		if (armor.rect.size.width < armor.rect.size.height) 
		{
			angle = armor.rect.angle;
		} else {
			angle = 90 + armor.rect.angle;
		}
		cv::Mat rotationmat = getRotationMatrix2D(Point(roi.center)-roi_mat.tl(),angle, 1);
		cv::warpAffine(roiimg,armorimg,rotationmat,roiimg.size());
		cv::Size size_;
		if(roi.angle>=-45)
		{
				size_=roi.size;
		} else{
				size_=Size(roi.size.height,roi.size.width);
		}
		cv::Rect realsizerect(Point(roi.center)-roi_mat.tl()-Point(size_)/2,size_);
		if(RectSafety_1(realsizerect,armorimg.size()))
		{  
			armorimg = armorimg(realsizerect);
		} else{
			armorimg = Mat::zeros(28,28,CV_8UC3);
		}
	}else{
		armorimg = Mat::zeros(28,28,CV_8UC3);
	}
	cv::Mat rgb[3];
	cv::split(armorimg,rgb);
	armorimg = rgb[2-enemy_color_];
	resize(armorimg,armorimg,Size(28,28));
	armor.set_ml_roi(armorimg);
}

float ConstraintSet::getSimilarity(const cv::Mat& first,const cv::Mat& second){
    double dotSum=first.dot(second);//内积
    double normFirst=cv::norm(first);//取模
    double normSecond=cv::norm(second);
    if(normFirst!=0 && normSecond!=0)
		{
        return dotSum/(normFirst*normSecond);
    }
}

bool ConstraintSet::FastTargetLock(ArmorInfo &armor,ArmorInfo &last_armor) 
{
	if(last_armor.id==0) 
	{
		return false;
	}
	static HOGDescriptor *hog = new HOGDescriptor(Size(28, 28), Size(14, 14), Size(7, 7), Size(7, 7), 9);
	vector<float> descriptors;
	if(armor.get_ml_roi().empty())
		return false;
	hog->compute(armor.get_ml_roi(), descriptors,Size(1, 1), Size(0, 0));
	Mat des=Mat(descriptors);
	armor.set_ml_hog(des);
	if(last_armor.get_ml_hog().empty())
	{
		if(armor.get_ml_roi().empty())
			return false;
		hog->compute(armor.get_ml_roi(), descriptors,Size(1, 1), Size(0, 0));
		des=Mat(descriptors);
		last_armor.set_ml_hog(des);
	}
	if(armor.get_ml_hog().empty())
		return false;
	RotatedRect r1=armor.rect;
	RotatedRect r2=last_armor.rect;
	float  similarity=getSimilarity(armor.get_ml_hog(),last_armor.get_ml_hog());
	similarity-=0.1*fabs(r1.size.area()-r2.size.area())/r2.size.area();
	similarity-=0.1*getDistance(r1.center,r2.center)/r2.size.height;
	if(similarity>0.86)//0.86)
	{
		last_armor = armor;
		return true;
	}else{
		return false;
	}
    
}

ArmorInfo ConstraintSet::SlectFinalArmor(std::vector<ArmorInfo> &armors) {
  if(armors.empty())
  {
    std::vector<cv::Point2f> tmp_points;
    float a=0;
    cv::Point2f tmp_2f(a,a);
    tmp_points.push_back(tmp_2f);
    tmp_points.push_back(tmp_2f);
    tmp_points.push_back(tmp_2f);
    tmp_points.push_back(tmp_2f);
    cv::RotatedRect tmp_rotated_Rect(Point2f(0, 0), cv::Size2f(100, 50), 30);
    int tmp_bool = 0;
    cv::Rect tmp_tracking_rect(0, 0, 0, 0);
    int tmp_thre = 0;
    float tmp_armor_stddev = 0.0;
    ArmorInfo tmp_armor(tmp_rotated_Rect, tmp_points, tmp_bool, 0, tmp_tracking_rect, tmp_thre, tmp_armor_stddev);//(Numb_rotatedrect, Numb_points, 1, armor_id, trackingrect)
    set_last_armor(tmp_armor);
    return tmp_armor;
  }else{
    std::vector<ArmorInfo> tmp_armors;
    for (const auto &armor : armors) 
		{
      tmp_armors.push_back(armor);
    }
    if(tmp_armors.empty())
    {
      std::sort(armors.begin(),armors.end(),[](const ArmorInfo &p1, const ArmorInfo &p2) { return p1.rect.size.area() > p2.rect.size.area(); });
    }else{
      armors=tmp_armors;
      std::sort(armors.begin(),armors.end(),[](const ArmorInfo &p1, const ArmorInfo &p2) { return p1.rect.size.area() > p2.rect.size.area(); });
    }
    cv::Rect boundRect = armors[0].tracking_rect;
    msg.data.push_back(boundRect.tl().x);
    msg.data.push_back(boundRect.tl().y);
    msg.data.push_back(boundRect.br().x);
    msg.data.push_back(boundRect.br().y);
    set_last_armor(armors[0]);
    cv_toolbox_->DrawRotatedRect(src_img_, armors[0].rect, cv::Scalar(255, 0, 0), 2);
    return armors[0];
  }
}

//-- No LightBar
bool ConstraintSet::region_otsu_threshold(const Mat &inputimage, Mat &outputimage, int &thre, bool lr) 
{
	bool ret= false;
	Mat sum_row,judge;
	Mat tmp;
	Rect orect;
	if(lr==0){
		orect=Rect(Point(cvRound(0.6*inputimage.cols),cvRound(0.35*inputimage.rows)),
									Point(cvRound(0.9*inputimage.cols),cvRound(0.65*inputimage.rows)));
	}else{
		orect=Rect(Point(cvRound(0.1*inputimage.cols),cvRound(0.35*inputimage.rows)),
									Point(cvRound(0.5*inputimage.cols),cvRound(0.65*inputimage.rows)));
	}
	thre=cvRound(threshold(inputimage(orect),tmp,0,255,THRESH_OTSU));
	threshold(inputimage,outputimage,thre,255,THRESH_BINARY);//将灰度图二值化 寻找灯条
	reduce(outputimage(Range(outputimage.rows*3/10,outputimage.rows*7/10),
											Range::all())/255,sum_row,1, REDUCE_SUM,CV_32F);
	threshold(sum_row,judge,2,1,THRESH_BINARY_INV);
	if (sum(judge)[0] <= 1) 
	{
		threshold(sum_row,judge,outputimage.cols*0.5,1,THRESH_BINARY);
		if (sum(judge)[0] < 0.5*sum_row.rows) 
		{
				ret= true;
		}
	}
	return ret;
}


bool ConstraintSet::rotatingCalipers( const Point2f* points, int n, float* out, float angle , float deviation) 
{
	float minarea = FLT_MAX, minarea_angle = FLT_MAX;
	float max_dist = 0;
	char  buffer[32] = {};
	char  buffer_angle[32] = {};
	int   i, k;
	
	AutoBuffer<float> abuf(n*3);
	
	float*   inv_vect_length = abuf;
	Point2f* vect = (Point2f*)(inv_vect_length + n);
	
	int   left = 0, bottom = 0, right = 0, top = 0;
	int   seq[4] = { -1, -1, -1, -1 };
	float orientation = 0;
	float base_a;
	float base_b = 0;
	float left_x, right_x, top_y, bottom_y;
	
	Point2f pt0 = points[0];

	left_x = right_x = pt0.x;
	top_y = bottom_y = pt0.y;

	for( i = 0; i < n; i++ ) 
	{
		double dx, dy;
		if( pt0.x < left_x )
				left_x = pt0.x, left = i;
		if( pt0.x > right_x )
				right_x = pt0.x, right = i;
		if( pt0.y > top_y )
				top_y = pt0.y, top = i;
		if( pt0.y < bottom_y )
				bottom_y = pt0.y, bottom = i;
		Point2f pt = points[(i+1) & (i+1 < n ? -1 : 0)];//pt0 下一个
		dx = pt.x - pt0.x;
		dy = pt.y - pt0.y;
		vect[i].x = (float)dx;
		vect[i].y = (float)dy;
		inv_vect_length[i] = (float)(1./std::sqrt(dx*dx + dy*dy));
		pt0 = pt;
	}
	double ax = vect[n-1].x;
	double ay = vect[n-1].y;
	for( i = 0; i < n; i++ ) 
	{
		double bx = vect[i].x;
		double by = vect[i].y;
		double convexity = ax * by - ay * bx;
		if( convexity != 0 ) 
		{
				orientation = (convexity > 0) ? 1.f : (-1.f);
				break;
		}
		ax = bx;
		ay = by;
	}
	CV_Assert( orientation != 0 );
	base_a = orientation;

	/*****************************************************************************************/
	/*                         init calipers position                                        */
	seq[0] = bottom;
	seq[1] = right;
	seq[2] = top;
	seq[3] = left;
	/*****************************************************************************************/
	/*                         Main loop - evaluate angles and rotate calipers               */

	/* all of edges will be checked while rotating calipers by 90 degrees */
	for( k = 0; k < n; k++ )
	{
		/* sinus of minimal angle */
		/*float sinus;*/

		/* compute cosine of angle between calipers side and polygon edge */
		/* dp - dot product */
		float dp[4] = {
					+base_a * vect[seq[0]].x + base_b * vect[seq[0]].y,
					-base_b * vect[seq[1]].x + base_a * vect[seq[1]].y,
					-base_a * vect[seq[2]].x - base_b * vect[seq[2]].y,
					+base_b * vect[seq[3]].x - base_a * vect[seq[3]].y,
		};
		float maxcos = dp[0] * inv_vect_length[seq[0]];
		/* number of calipers edges, that has minimal angle with edge */
		int main_element = 0;
		/* choose minimal angle */
		for ( i = 1; i < 4; ++i )
		{
			float cosalpha = dp[i] * inv_vect_length[seq[i]];
			if (cosalpha > maxcos) 
			{
				main_element = i;
				maxcos = cosalpha;
			}
		}
		/*rotate calipers*/
		//get next base
		int pindex = seq[main_element];
		float lead_x = vect[pindex].x*inv_vect_length[pindex];
		float lead_y = vect[pindex].y*inv_vect_length[pindex];
		switch( main_element )
		{
		case 0:
			base_a = lead_x;
			base_b = lead_y;
			break;
		case 1:
			base_a = lead_y;
			base_b = -lead_x;
			break;
		case 2:
			base_a = -lead_x;
			base_b = -lead_y;
			break;
		case 3:
			base_a = -lead_y;
			base_b = lead_x;
			break;
		default:
				;
		}
		/* change base point of main edge */
		seq[main_element] += 1;
		seq[main_element] = (seq[main_element] == n) ? 0 : seq[main_element];

		float height;
		float area;

		/* find vector left-right */
		float dx = points[seq[1]].x - points[seq[3]].x;
		float dy = points[seq[1]].y - points[seq[3]].y;

		/* dotproduct */
		float width = dx * base_a + dy * base_b;

		/* find vector left-right */
		dx = points[seq[2]].x - points[seq[0]].x;
		dy = points[seq[2]].y - points[seq[0]].y;

		/* dotproduct */

		height = -dx * base_b + dy * base_a;
		float tmp_angle;
		if(height>width) {
				tmp_angle =90.f - asin(base_b)*180.f/kPi;
		} else {
				tmp_angle = - asin(base_b)*180.f/kPi;
		}
		area = width * height;
		if(abs(tmp_angle-angle)<abs(deviation)) 
		{
			float *buf = (float *) buffer_angle;
			if(area <= minarea_angle) 
			{
					minarea_angle =area;
			}
			/* leftist point */
			((int *) buf)[0] = seq[3];
			buf[1] = base_a;
			buf[2] = width;
			buf[3] = base_b;
			buf[4] = height;
			/* bottom point */
			((int *) buf)[5] = seq[0];
			buf[6] = area;

		}
		if( area <= minarea ) 
		{
			float *buf = (float *) buffer;
			minarea = area;
			/* leftist point */
			((int *) buf)[0] = seq[3];
			buf[1] = base_a;
			buf[2] = width;
			buf[3] = base_b;
			buf[4] = height;
			/* bottom point */
			((int *) buf)[5] = seq[0];
			buf[6] = area;
		}
	}
	float *buf;
	if(minarea_angle!= FLT_MAX){
		buf = (float *) buffer_angle;
	}else{
		return false;
		buf = (float *) buffer;
	}
	float A1 = buf[1];
	float B1 = buf[3];
	float A2 = -buf[3];
	float B2 = buf[1];
	float C1 = A1 * points[((int *) buf)[0]].x + points[((int *) buf)[0]].y * B1;
	float C2 = A2 * points[((int *) buf)[5]].x + points[((int *) buf)[5]].y * B2;
	float idet = 1.f / (A1 * B2 - A2 * B1);
	float px = (C1 * B2 - C2 * B1) * idet;
	float py = (A1 * C2 - A2 * C1) * idet;
	out[0] = px;
	out[1] = py;
	out[2] = A1 * buf[2];
	out[3] = B1 * buf[2];
	out[4] = A2 * buf[4];
	out[5] = B2 * buf[4];
	return true;
}

cv::RotatedRect ConstraintSet::Chao_minAreaRect( InputArray _points, float angle, float deviation ) 
{
	Mat hull;
	Point2f out[3];
	RotatedRect box;
	cv::convexHull(_points, hull, true, true);
	if( hull.depth() != CV_32F ) 
	{
		Mat temp;
		hull.convertTo(temp, CV_32F);
		hull = temp;
	}
	int n = hull.checkVector(2);
	const Point2f* hpoints = hull.ptr<Point2f>();
	if( n > 2 ) 
	{
		if(deviation>0&&rotatingCalipers( hpoints, n, (float*)out , angle , deviation))
		{
			box.center.x = out[0].x + (out[1].x + out[2].x)*0.5f;
			box.center.y = out[0].y + (out[1].y + out[2].y)*0.5f;
			box.size.height = (float)std::sqrt((double)out[1].x *
																					out[1].x + (double)out[1].y * out[1].y);
			box.size.height = (float)std::sqrt((double)out[2].x *
																					out[2].x + (double)out[2].y * out[2].y);
			box.angle = (float)atan2( (double)out[1].y, (double)out[1].x );
		} else{
			angle=-angle;
			vector<Point> hullpoints;
			for(int i=0;i<hull.rows;i++)
			{
					float x1=hpoints[i].x;
					float y1=hpoints[i].y;
					float x2=cos(angle*CV_PI/180)*x1-sin(angle*CV_PI/180)*y1;
					float y2=sin(angle*CV_PI/180)*x1+cos(angle*CV_PI/180)*y1;
					hullpoints.push_back(Point(x2,y2));
			}
			sort(hullpoints.begin(),hullpoints.end(),
						[](Point a,Point b)->bool{ return a.x<b.x;});
			Point2f p1=hullpoints[0];
			Point2f p2=hullpoints[hullpoints.size()-1];
			sort(hullpoints.begin(),hullpoints.end(),
						[](Point a,Point b)->bool{ return a.y<b.y;});

			Point2f p3=hullpoints[0];
			Point2f p4=hullpoints[hullpoints.size()-1];
			box.size.width=p2.x-p1.x+1;
			box.size.height=p4.y-p3.y+1;
			Point2f center_=Point2f((p1.x+p2.x)/2,(p3.y+p4.y)/2);
			box.center.x =cos(-angle*CV_PI/180)*center_.x-sin(-angle*CV_PI/180)*center_.y;
			box.center.y=sin(-angle*CV_PI/180)*center_.x+cos(-angle*CV_PI/180)*center_.y;
			box.angle=-angle*CV_PI/180;
		}
	}else if( n == 2 ) {
		box.center.x = (hpoints[0].x + hpoints[1].x) * 0.5f;
		box.center.y = (hpoints[0].y + hpoints[1].y) * 0.5f;
		double dx = hpoints[1].x - hpoints[0].x;
		double dy = hpoints[1].y - hpoints[0].y;
		box.size.width = (float)std::sqrt(dx * dx + dy * dy);
		box.size.height = 0;
		box.angle = (float)atan2( dy, dx);
	}else {
		if( n == 1 )
			box.center = hpoints[0];
	}
	box.angle = (float)(box.angle*180/CV_PI);
	return box;
}


bool ConstraintSet::NoLightBar_Detect(cv::Mat &src, std::vector<ArmorInfo> &armors) 
{
	int last_threshold_=last_armor_.num_threshold;
	if(RectSafety(roi_rect_, src_img_.rows, src_img_.cols)==false)
		return false;
	cv::Mat src_roi = src(roi_rect_);
	std::vector<std::vector<Point>> contour;
	std::vector<Mat> split_mat;
	cv::split(src_roi,split_mat);
	src_roi=split_mat[2-enemy_color_];
	cv::threshold(src_roi,src_roi,max(20,last_threshold_),255,CV_THRESH_BINARY);
	cv::findContours(src_roi,contour,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE,roi_rect_.tl());
	for(size_t i=0;i<contour.size();i++)
	{
		if (contour[i].size() < 60) continue;
		float tmp_angle;
		if(fabs(last_armor_.rect.angle)<45)
			tmp_angle = last_armor_.rect.angle;
		else
			tmp_angle = 90+last_armor_.rect.angle;
		cv::RotatedRect rot_rect=Chao_minAreaRect(contour[i],tmp_angle,0);
		if(rot_rect.size.area()<50) continue;
		cv::Rect trackingrect;
		cv::Point2f Numb_bl, Numb_tl, Numb_tr, Numb_br;
		std::vector<cv::Point2f> Numb_points;
		trackingrect=rot_rect.boundingRect();
		if(RectSafety(trackingrect,img_heigth,img_width)==false)
			continue;
		GetCorrectPointOder(rot_rect, Numb_bl, Numb_tl, Numb_tr, Numb_br);
		Numb_points.push_back(Numb_tl);
		Numb_points.push_back(Numb_tr);
		Numb_points.push_back(Numb_br);
		Numb_points.push_back(Numb_bl);
		/*
		if(RectSafety(trackingrect, src_img_.rows, src_img_.cols)==false)
		continue;
		*/
		cv::Mat image_numb=src_img_(trackingrect);
		std::vector<cv::Mat> rgb;
		split(image_numb,rgb);
		if(enemy_color_==RED)
			image_numb=rgb[0];
		else
			image_numb=rgb[2];
		int thre=cvRound(threshold(image_numb,image_numb,0,255,CV_THRESH_OTSU));
		ArmorInfo tmp_armor(rot_rect, Numb_points, 0, last_armor_.id, trackingrect, thre);
		Numb_points.clear();
		std::vector<cv::Point2f>().swap(Numb_points);
		SetMlRoi(tmp_armor,src_img_);

#if 0
		MLHogSvm(src_img_,tmp_armor.rect,enemy_color_,0);
		if(svmresult)
		{
		std::vector<ArmorInfo> tmp_armors;
		tmp_armors.emplace_back(tmp_armor);
		armors=tmp_armors;
		cv_toolbox_->DrawRotatedRect(src_img_, tmp_armor.rect, cv::Scalar(0, 0, 255), 2);
		tmp_armors.clear();
		std::vector<ArmorInfo>().swap(tmp_armors);  
		return true;
		}
		else
		return false;
#endif 
		if(FastTargetLock(tmp_armor,last_armor_)==true)
		{
			std::vector<ArmorInfo> tmp_armors;
			tmp_armors.emplace_back(tmp_armor);
			armors=tmp_armors;
			cv_toolbox_->DrawRotatedRect(src_img_, tmp_armor.rect, cv::Scalar(0, 0, 255), 2);
			tmp_armors.clear();
			std::vector<ArmorInfo>().swap(tmp_armors);  
			return true;
		}			
	}
	return false;
}

void ConstraintSet::CalcControlInfo(const ArmorInfo & armor, cv::Point3f &target_3d, float pitch_now, float yaw_now, float &yaw_output, float &pitch_output, float &absolute_pitch_, float &absolute_yaw_, Target &output_target, float &distance_out) 
{
  cv::Mat rvec;
  cv::Mat tvec;
  std::vector<cv::Point3f> numb_points_;
  std::cout<<"armor.id is +++++++++++++++++++++++   "<<armor.id<<std::endl;
	switch(armor.id)
	{
	case 1:
		//width_1: 44 # mm
		//height_1: 102 # mm
		//width: 120 # mm
		//height: 60 # mm
		numb_points_.emplace_back(cv::Point3f(-2.2, 5.1,  0.0));//-2.2, 5.1,  0.0
		numb_points_.emplace_back(cv::Point3f(2.2,  5.1,  0.0));//2.2,  5.1,  0.0
		numb_points_.emplace_back(cv::Point3f(2.2,  -5.1, 0.0));//2.2,  -5.1, 0.0
		numb_points_.emplace_back(cv::Point3f(-2.2, -5.1, 0.0));//-2.2, -5.1, 0.0
		cv::solvePnP(numb_points_,armor.vertex,intrinsic_matrix_,distortion_coeffs_,rvec,tvec);
		target_3d = cv::Point3f(tvec);
		break;
	case 2:
		//width_1: 66 # mm
		//height_1: 110 # mm
		numb_points_.emplace_back(cv::Point3f(-3.3, 5.5,  0.0));//-3.3, 5.5,  0.0
		numb_points_.emplace_back(cv::Point3f(3.3,  5.5,  0.0));//3.3,  5.5,  0.0
		numb_points_.emplace_back(cv::Point3f(3.3,  -5.5, 0.0));//3.3,  -5.5, 0.0
		numb_points_.emplace_back(cv::Point3f(-3.3, -5.5, 0.0));//-3.3, -5.5, 0.0
		cv::solvePnP(numb_points_,armor.vertex,intrinsic_matrix_,distortion_coeffs_,rvec,tvec);
		target_3d = cv::Point3f(tvec);
		break;
	case 3:
		//width_1: 62 # mm
		//height_1: 102 # mm
		numb_points_.emplace_back(cv::Point3f(-3.15, 5,  0.0));
		numb_points_.emplace_back(cv::Point3f(3.15,  5,  0.0));
		numb_points_.emplace_back(cv::Point3f(3.15,  -5, 0.0));
		numb_points_.emplace_back(cv::Point3f(-3.15, -5, 0.0));
		cv::solvePnP(numb_points_,armor.vertex,intrinsic_matrix_,distortion_coeffs_,rvec,tvec);
		target_3d = cv::Point3f(tvec);
		break;
	case 5:
		//width_1: 62 # mm
		//height_1: 102 # mm
		numb_points_.emplace_back(cv::Point3f(-3.1, 5.1,  0.0));
		numb_points_.emplace_back(cv::Point3f(3.1,  5.1,  0.0));
		numb_points_.emplace_back(cv::Point3f(3.1,  -5.1, 0.0));
		numb_points_.emplace_back(cv::Point3f(-3.1, -5.1, 0.0));
	cv::solvePnP(numb_points_,armor.vertex,intrinsic_matrix_,distortion_coeffs_,rvec,tvec);
		target_3d = cv::Point3f(tvec);
		break;
	default:
		break;
	} 
	double s=-3.945;//相机炮台高度差//cm 
	double v=1700;//子弹射速 
	double kangle = 0.2742;//0.3077; 
	yaw_output=atan2(320-armor.rect.center.x,intrinsic_matrix_.at<double>(0,0))*57.3; 
	/*
	//std::cout<<"!!!"<<std::endl;
	//std::cout<<" yaw_output is "<<yaw_output<<std::endl;//顺时针变小
	//std::cout<<" yaw_now is "<<yaw_now<<std::endl;//逆时针变大
	//std::cout<<"!!!"<<std::endl;
	*/
	float disX,disY,disZ; 
	disX=tvec.at<double>(0); 
	disY=tvec.at<double>(1); 
	disZ=tvec.at<double>(2); 
		
	float distance = sqrt((disX*disX)+(disY*disY)+(disZ*disZ)); 
	if(distance<=100)
		v=1600;
	
	distance_out = distance;
	double pitch_angle=pitch_now/57.3;
	double pixel_angle=-atan2(240-armor.rect.center.y,intrinsic_matrix_.at<double>(0,0)); 
	
	/*
	//std::cout<<"!!!"<<std::endl;
	//std::cout<<" pixel_angle is "<<pixel_angle<<std::endl;//抬头变大
	//std::cout<<" pitch_now is "<<pitch_now<<std::endl;//抬头变小
	//std::cout<<"!!!"<<std::endl;
	*/
	//TODO:+-Problem!!!
	double real_angle=pitch_angle+pixel_angle; 
	double dis_x = distance*cos(-real_angle)+(-s)/sin(kangle)*sin(1.57-pitch_angle-kangle); 
	double dis_y = distance*sin(real_angle)+(-s)/sin(kangle)*cos(1.57-pitch_angle-kangle); 
	double distance_y=dis_y; 
	distance = dis_x; 
	double tanx1=(-distance+sqrt(distance*distance-4*(-0.5*g*100*(distance*distance/(v*v))*(-0.5*g*100*(distance*distance/(v*v))+distance_y))))/(-g*100*(distance*distance/(v*v))); 
	double tanx2=(-distance-sqrt(distance*distance-4*(-0.5*g*100*(distance*distance/(v*v))*(-0.5*g*100*(distance*distance/(v*v))+distance_y))))/(-g*100*(distance*distance/(v*v))); 
	double best_tanx=tanx1; 
	if(fabs(tanx2)<fabs(tanx1))
	{ 
		best_tanx=tanx2; 
	} 
    
	float tmp_measure; 
	pitch_output=(-1)*atan2(240-armor.rect.center.y,intrinsic_matrix_.at<double>(0,0))*57.3; 
	tmp_measure=atan(best_tanx)*57.3; 
	absolute_pitch_=pitch_now+pitch_output;
	absolute_yaw_=yaw_now-yaw_output;
	/************Predictor*************/
	////////////////////////////////////////////////////////////////////////
	cv::Mat rotMatrix(3, 3, CV_32FC1);
	cv::Rodrigues( rvec,rotMatrix);
	pnp_assess=PnpAssess(intrinsic_matrix_,distortion_coeffs_,rotMatrix,tvec,numb_points_,armor.vertex);
	float rotsqrt = sqrt(rotMatrix.at<double>(0,0) * rotMatrix.at<double>(0,0) +  rotMatrix.at<double>(1,0) * rotMatrix.at<double>(1,0) );
	bool singular = rotsqrt < 1e-6; // If
	//转换为欧拉角 逆时针方向的旋转是正、顺时针方向的旋转是负
	float angle_x, angle_y, angle_z;
	if (!singular)
	{
		angle_x = atan2(rotMatrix.at<double>(2,1) , rotMatrix.at<double>(2,2))-kPi;
		angle_y = atan2(-rotMatrix.at<double>(2,0), rotsqrt);
		angle_z = atan2(rotMatrix.at<double>(1,0), rotMatrix.at<double>(0,0));
	}else{
		angle_x = atan2(-rotMatrix.at<double>(1,2), rotMatrix.at<double>(1,1));
		angle_y = atan2(-rotMatrix.at<double>(2,0), rotsqrt);
		angle_z = 0;
  }
	float tmp_distance = sqrt((disX*disX)+(disY*disY)+(disZ*disZ));
	float predict_absolute_pitch_,predict_absolute_yaw_;
	CoordinateTransformation(disX, disY, disZ, tmp_distance, armor.rect.center.x, yaw_now, pitch_now, predict_absolute_pitch_, predict_absolute_yaw_);
	float tmp_height,tmp_width;
	tmp_height = std::max(armor.rect.size.height,armor.rect.size.width);
	tmp_width = std::min(armor.rect.size.height,armor.rect.size.width);
	Target output_target1(disX,disY,disZ,tmp_height,tmp_width,angle_x,angle_y,angle_z,armor.rect.center.x,armor.rect.center.y,tmp_distance, pnp_assess);
	output_target=output_target1;
    
}

double ConstraintSet::PnpAssess(Mat matrix,Mat dist_coeffs,Mat rot_matrix,Mat tran_vector,vector<Point3f> worldPoint,vector<Point2f> imagePoint)
{
	vector<Point3f> world_point;
	vector<Point2f> image_point;
	projectPoints(worldPoint,rot_matrix,tran_vector,matrix,dist_coeffs,image_point);
	float dis=0;
	for(int i=0;i<image_point.size();i++)
	{
		dis=dis+(image_point[i].x-imagePoint[i].x)*(image_point[i].x-imagePoint[i].x)+(image_point[i].y-imagePoint[i].y)*(image_point[i].y-imagePoint[i].y);      
	}
	dis=sqrt(dis);
	return dis;
}


void ConstraintSet::CoordinateTransformation(float &disX, float &disY, float &disZ, float target_distance, int armor_rect_center_x, float yaw_now, float pitch_now, float &absolute_pitch_, float &absolute_yaw_)
{
	float s=-3.945;
	float kangle=0.2742;
	float length=0;
	float distance=target_distance+length;
	float yaw_pixel_angle=atan2((320-armor_rect_center_x),intrinsic_matrix_.at<double>(0,0));
	float absolute_yaw_angle=yaw_pixel_angle-yaw_now;
	double dis_x = distance*sin(yaw_pixel_angle);
	double dis_z = distance*cos(yaw_pixel_angle);
	float pitch_pixel_angle=atan2((240-armor_rect_center_x),intrinsic_matrix_.at<double>(0,0));
	float absolute_pitch_angle=pitch_pixel_angle-pitch_now;
	double dis_y = distance*sin(-absolute_pitch_angle)+(-s)/sin(kangle)*sin(-pitch_now+kangle);
	disX = dis_x;
	disY = dis_y;
	disZ = dis_z;
	absolute_yaw_= absolute_yaw_angle;
	absolute_pitch_ = absolute_pitch_angle;
}

cv::Rect ConstraintSet::rectCenterScale(cv::Rect rect, cv::Size2f size)
{
	cv::Size arg=cv::Size(cvRound((size.width-1) *rect.size().width),  cvRound((size.height-1) *rect.size().height));
	rect= rect + arg;
	cv::Point pt;
	pt.x = cvRound(arg.width/2.0);
	pt.y = cvRound(arg.height/2.0);
	return (rect-pt);
}


void ConstraintSet::RoiFilter(Mat &input, cv::Point &offset) 
{
	if(last_armor_.id!=0)
	{
		roi_rect_=last_armor_.tracking_rect;
		Size enlarge_size =last_armor_.single == 0?cv::Size(3.1,4.1):cv::Size(4.1,5.1);
		roi_rect_=Rect(cv::Point(0,0),cv::Size(img_width,img_heigth));//Rect(cv::Point(0,0),cv::Size(img_width,img_heigth));//rectCenterScale(roi_rect_,enlarge_size);
		//cv::rectangle(src_img_, roi_rect_, cv::Scalar(0, 0, 255), 3);
		if (RectSafety(roi_rect_, src_img_.rows, src_img_.cols)==false)
		{
			roi_rect_=Rect(cv::Point(0,0),cv::Size(img_width,img_heigth));
		}
	}else{
		roi_rect_=Rect(cv::Point(0,0),cv::Size(img_width,img_heigth));
	}
	//roi_rect_=Rect(Point(0,0),input.size());//TODO:!!!Strange!!!
	offset = roi_rect_.tl();  
}

bool ConstraintSet::Get(const ArmorInfo & armor, float yaw, float pitch, float absolute_pitch_, float absolute_yaw_, bool &firecommand) 
{
	if(std::abs(pitch)<=3&&std::abs(yaw)<=3)
		firecommand = 1;
	else {
		firecommand = 0;
	}     
}
void ConstraintSet::CalcArmorInfo(std::vector<cv::Point2f> &armor_points,
                                 cv::RotatedRect left_light,
                                 cv::RotatedRect right_light)
{
  cv::Point2f left_points[4], right_points[4];
  left_light.points(left_points);
  right_light.points(right_points);

  cv::Point2f right_lu, right_ld, lift_ru, lift_rd;
  std::sort(left_points, left_points + 4, [](const cv::Point2f &p1, const cv::Point2f &p2) { return p1.x < p2.x; });
  std::sort(right_points, right_points + 4, [](const cv::Point2f &p1, const cv::Point2f &p2) { return p1.x < p2.x; });
  if (right_points[0].y < right_points[1].y) {
    right_lu = right_points[0];
    right_ld = right_points[1];
  } else {
    right_lu = right_points[1];
    right_ld = right_points[0];
  }

  if (left_points[2].y < left_points[3].y) {
    lift_ru = left_points[2];
    lift_rd = left_points[3];
  } else {
    lift_ru = left_points[3];
    lift_rd = left_points[2];
  }
  armor_points.push_back(lift_ru);
  armor_points.push_back(right_lu);
  armor_points.push_back(right_ld);
  armor_points.push_back(lift_rd);

}

void ConstraintSet::SolveArmorCoordinate(const float width,
                                         const float height)
{
  armor_points_.emplace_back(cv::Point3f(-width/2, height/2,  0.0));
  armor_points_.emplace_back(cv::Point3f(width/2,  height/2,  0.0));
  armor_points_.emplace_back(cv::Point3f(width/2,  -height/2, 0.0));
  armor_points_.emplace_back(cv::Point3f(-width/2, -height/2, 0.0));
}

void ConstraintSet::SignalFilter(double &new_num, double &old_num, unsigned int &filter_count, double max_diff) 
{
  if(fabs(new_num - old_num) > max_diff && filter_count < 2) 
	{
    filter_count++;
    new_num += max_diff;
  } else {
    filter_count = 0;
    old_num = new_num;
  }
}
void ConstraintSet::SetThreadState(bool thread_state)
{
  thread_running_ = thread_state;
}

/********************************************************[ADD]************************************************/
void ConstraintSet::blockFun0(const ArmorInfo & armor,cv::Point3f &target_3d1,cv::Point3f &target_3d2,cv::Point3f &target_3d3,int &num)
{
  std::cout<<"camera0 begain!!!!!!!!!"<<std::endl;
  float x;//装甲板中点x坐标值
  float y;//装甲板中点y坐标值
  float fun_value_1;//约束函数1的函数值
  float fun_value_2;//约束函数2的函数值
  float fun_value_3;//约束函数3的函数值
  float fun_value_4;//约束函数4的函数值
  
  x=armor.rect.center.x;
  //std::cout<<"x="<<x<<std::endl;
  y=armor.rect.center.y;  
  //std::cout<<"y="<<y<<std::endl;

#if 0
1.41566          -47.4096
-0.142276          351.423
0.571429          -182.429
-0.195008          632.293
#endif
  fun_value_1 = y-1.41566  *x +47.4096;
  fun_value_2 = y+0.142276 *x-351.423;
  fun_value_3 = y-0.571429*x +182.429;
  fun_value_4 = y+0.195008 *x - 632.293;

  
  //block 6
  if(fun_value_2<0 && fun_value_3>0 )
  {
    target_3d1.x=1500;
    target_3d1.y=1695;
    target_3d1.z=armor.id;
    
    target_3d2.x=1500;
    target_3d2.y=3120;
    target_3d2.z=armor.id;
    
    target_3d3.x=750;
    target_3d3.y=2400;
    
    num = 6;
    std::cout<<"block num is:"<<num <<std::endl;
   
  }
  //block 2
  if( fun_value_3<0)
  {
    target_3d1.x=3250;
    target_3d1.y=4100;
    target_3d1.z=armor.id;
    
    target_3d2.x=5450;
    target_3d2.y=4100;
    target_3d2.z=armor.id;
    
    target_3d3.x=4000;
    target_3d3.y=4600;
    
    num = 2;
    std::cout<<"block num is:"<<num <<std::endl;
  }
  //block 3
  if(fun_value_1<0 &&fun_value_2>0 && fun_value_3>0 &&  fun_value_4<0)
  {
    target_3d1.x=3250;
    target_3d1.y=4100;
    target_3d1.z=armor.id;
    
    target_3d2.x=4000;
    target_3d2.y=1700;
    target_3d2.z=armor.id;
    
    target_3d3.x=3250;
    target_3d3.y=2400;
    
    num = 3;
    std::cout<<"block num is:"<<num <<std::endl;
  }
  //block 4
  if(fun_value_1<0 && fun_value_3>0&& fun_value_4>0)
  {
    target_3d1.x=5450;
    target_3d1.y=1000;
    target_3d1.z=armor.id;
    
    target_3d2.x=4000;
    target_3d2.y=3100;
    target_3d2.z=armor.id;
    target_3d3.x=5450;
    target_3d3.y=2400;
    
    num = 4;
    std::cout<<"block num is:"<<num <<std::endl;
  }
  //block 5
  if( fun_value_1>0&&fun_value_2>0 )
  {
    target_3d1.x=3250;
    target_3d1.y=1000;
    target_3d1.z=armor.id;
    
    target_3d2.x=5450;
    target_3d2.y=1000;
    target_3d2.z=armor.id;
    
    target_3d3.x=4000;
    target_3d3.y=500;
    
    num=5;
    std::cout<<"block num is:"<<num<<std::endl;
  }
}

void ConstraintSet::blockFun1(const ArmorInfo & armor,cv::Point3f &target_3d1,cv::Point3f &target_3d2,cv::Point3f &target_3d3,int &num)
{
  std::cout<<"camera1 begin!!!!!!!"<<std::endl;
  float x;//装甲板中点x坐标值
  float y;//装甲板中点y坐标值
  float fun_value_1;//约束函数1的函数值
  float fun_value_2;//约束函数2的函数值
  float fun_value_3;//约束函数3的函数值
  float fun_value_4;//约束函数4的函数值
  
  x=armor.rect.center.x;
  //std::cout<<"x="<<x<<std::endl;
  y=armor.rect.center.y;  
 // std::cout<<"y="<<y<<std::endl;
  #if 0
1.8254          -121.46
-0.12963          175.704
-0.5          312
-0.923567          469.567
1.5625          -127
-0.0734694          317.984
0.520635          -134.724
-0.189147          629.712
#endif
  fun_value_1 = y-1.5625 *x +127;
  fun_value_2 = y+0.0734694 *x-317.984;
  fun_value_3 = y-0.52063 *x +134.724;
  fun_value_4 = y+0.189147 *x - 629.712;
  //block 2
  if(fun_value_1>0 && fun_value_2>0)
  {
    target_3d1.x=3250;
    target_3d1.y=4100;
    target_3d1.z=armor.id;
    
    target_3d2.x=5450;
    target_3d2.y=4100;
    target_3d2.z=armor.id;
    
    target_3d3.x=4000;
    target_3d3.y=4600;
    
    num = 2;
    std::cout<<"block num is:"<<num <<std::endl;
  }
  //block 3
 // if( fun_value_1<0 && fun_value_2>0 && fun_value_3>0&& fun_value_4<0)
  if( fun_value_1<0 && fun_value_4>0 && fun_value_3>0)
  {
    target_3d1.x=3250;
    target_3d1.y=4100;
    target_3d1.z=armor.id;
    
    target_3d2.x=4000;
    target_3d2.y=1700;
    target_3d2.z=armor.id;
    
    target_3d3.x=3250;
    target_3d3.y=2400;
    
    num = 3;
    std::cout<<"block3 num is:"<<num <<std::endl;
  }
  //block 4
  if( fun_value_1<0 && fun_value_2>0 && fun_value_3>0&& fun_value_4<0)  
  {
    target_3d1.x=5450;
    target_3d1.y=1000;
    target_3d1.z=armor.id;
    
    target_3d2.x=4000;
    target_3d2.y=3100;
    target_3d2.z=armor.id;
    
    target_3d3.x=5450;
    target_3d3.y=2400;
    
    num = 4;
    std::cout<<"block4 num is:"<<num <<std::endl;
  }
  //block 5 
  if(fun_value_3<0)
  {
    target_3d1.x=3250;
    target_3d1.y=1000;
    target_3d1.z=armor.id;
    
    target_3d2.x=5450;
    target_3d2.y=1000;
    target_3d2.z=armor.id;
    
    target_3d3.x=4000;
    target_3d3.y=500;
    
    num = 5;
    std::cout<<"block num is:"<<num <<std::endl;
  }
  //block 6
  if(fun_value_2<0 && fun_value_3>0)
  {
    target_3d1.x=6600;
    target_3d1.y=1700;
    target_3d1.z=armor.id;
    
    target_3d2.x=6600;
    target_3d2.y=3100;
    target_3d2.z=armor.id;
    
    target_3d3.x=7350;
    target_3d3.y=2400;
    
    num=1;
    std::cout<<"block num is:"<<num<<std::endl;
  }
}

#if 0
void ConstraintSet::blockFun0(const ArmorInfo & armor,cv::Point3f &target_3d1,cv::Point3f &target_3d2,cv::Point3f &target_3d3,int &num)
{
  std::cout<<"camera0 begain!!!!!!!!!"<<std::endl;
  float x;//装甲板中点x坐标值
  float y;//装甲板中点y坐标值
  float fun_value_1;//约束函数1的函数值
  float fun_value_2;//约束函数2的函数值
  float fun_value_3;//约束函数3的函数值
  float fun_value_4;//约束函数4的函数值
  
  x=armor.rect.center.x;
  //std::cout<<"x="<<x<<std::endl;
  y=armor.rect.center.y;  
  //std::cout<<"y="<<y<<std::endl;
  
  fun_value_1 = y+ 0.1446*x -307;
  fun_value_2 = y-0.635*x +479;
  fun_value_3 = y-1.1628*x + 27;
  fun_value_4=y+0.174*x - 656;
  //block 1
  if(fun_value_1<0 && fun_value_2>0 && x <1012)
  {
    target_3d1.x=1500;
    target_3d1.y=1695;
    target_3d1.z=armor.id;
    
    target_3d2.x=1500;
    target_3d2.y=3120;
    target_3d2.z=armor.id;
    
    target_3d3.x=750;
    target_3d3.y=2400;
    
    num = 1;
    std::cout<<"block num is:"<<num <<std::endl;
   
  }
  //block 2
  if( fun_value_2<0)
  {
    target_3d1.x=3250;
    target_3d1.y=4100;
    target_3d1.z=armor.id;
    
    target_3d2.x=5450;
    target_3d2.y=4100;
    target_3d2.z=armor.id;
    
    target_3d3.x=4000;
    target_3d3.y=4600;
    
    num = 2;
    std::cout<<"block num is:"<<num <<std::endl;
  }
  //block 3
  if(fun_value_1>0 && fun_value_2>0 && fun_value_3<0 && fun_value_4<0)
  {
    target_3d1.x=3250;
    target_3d1.y=4100;
    target_3d1.z=armor.id;
    
    target_3d2.x=4000;
    target_3d2.y=1700;
    target_3d2.z=armor.id;
    
    target_3d3.x=3250;
    target_3d3.y=2400;
    
    num = 3;
    std::cout<<"block num is:"<<num <<std::endl;
  }
  //block 4
  if(fun_value_3<0 && fun_value_4>0)
  {
    target_3d1.x=5450;
    target_3d1.y=1000;
    target_3d1.z=armor.id;
    
    target_3d2.x=4000;
    target_3d2.y=3100;
    target_3d2.z=armor.id;
    target_3d3.x=5450;
    target_3d3.y=2400;
    
    num = 4;
    std::cout<<"block num is:"<<num <<std::endl;
  }
  //block 5
  if(fun_value_1>0 && fun_value_3>0)
  {
    target_3d1.x=3250;
    target_3d1.y=1000;
    target_3d1.z=armor.id;
    
    target_3d2.x=5450;
    target_3d2.y=1000;
    target_3d2.z=armor.id;
    
    target_3d3.x=4000;
    target_3d3.y=500;
    
    num=5;
    std::cout<<"block num is:"<<num<<std::endl;
  }
}
void ConstraintSet::blockFun1(const ArmorInfo & armor,cv::Point3f &target_3d1,cv::Point3f &target_3d2,cv::Point3f &target_3d3,int &num)
{
  std::cout<<"camera1 begin!!!!!!!"<<std::endl;
  float x;//装甲板中点x坐标值
  float y;//装甲板中点y坐标值
  float fun_value_1;//约束函数1的函数值
  float fun_value_2;//约束函数2的函数值
  float fun_value_3;//约束函数3的函数值
  float fun_value_4;//约束函数4的函数值
  
  x=armor.rect.center.x;
  //std::cout<<"x="<<x<<std::endl;
  y=armor.rect.center.y;  
 // std::cout<<"y="<<y<<std::endl;
  
  fun_value_1 = y-1.5*x +142.5;
  fun_value_2 = y+0.107*x-264;
  fun_value_3 = y-0.5192*x +420;
  fun_value_4 = y+0.1673*x -714;
  //block 2
  if(fun_value_1>0 && fun_value_2>0)
  {
    target_3d1.x=3250;
    target_3d1.y=4100;
    target_3d1.z=armor.id;
    
    target_3d2.x=5450;
    target_3d2.y=4100;
    target_3d2.z=armor.id;
    
    target_3d3.x=4000;
    target_3d3.y=4600;
    
    num = 2;
    std::cout<<"block num is:"<<num <<std::endl;
  }
  //block 3
 // if( fun_value_1<0 && fun_value_2>0 && fun_value_3>0&& fun_value_4<0)
  if( fun_value_1<0 && fun_value_4>0)
  {
    target_3d1.x=3250;
    target_3d1.y=4100;
    target_3d1.z=armor.id;
    
    target_3d2.x=4000;
    target_3d2.y=1700;
    target_3d2.z=armor.id;
    
    target_3d3.x=3250;
    target_3d3.y=2400;
    
    num = 3;
    std::cout<<"block3 num is:"<<num <<std::endl;
  }
  //block 4
  if( fun_value_1<0 && fun_value_2>0 && fun_value_3>0&& fun_value_4<0)  
  {
    target_3d1.x=5450;
    target_3d1.y=1000;
    target_3d1.z=armor.id;
    
    target_3d2.x=4000;
    target_3d2.y=3100;
    target_3d2.z=armor.id;
    
    target_3d3.x=5450;
    target_3d3.y=2400;
    
    num = 4;
    std::cout<<"block4 num is:"<<num <<std::endl;
  }
  //block 5 
  if(fun_value_3<0)
  {
    target_3d1.x=3250;
    target_3d1.y=1000;
    target_3d1.z=armor.id;
    
    target_3d2.x=5450;
    target_3d2.y=1000;
    target_3d2.z=armor.id;
    
    target_3d3.x=4000;
    target_3d3.y=500;
    
    num = 5;
    std::cout<<"block num is:"<<num <<std::endl;
  }
  //block 6
  if(fun_value_2<0 && fun_value_3>0)
  {
    target_3d1.x=6600;
    target_3d1.y=1700;
    target_3d1.z=armor.id;
    
    target_3d2.x=6600;
    target_3d2.y=3100;
    target_3d2.z=armor.id;
    
    target_3d3.x=7350;
    target_3d3.y=2400;
    
    num=6;
    std::cout<<"block num is:"<<num<<std::endl;
  }
}
#endif 
/********************************************[END0]***************************************************/
ConstraintSet::~ConstraintSet() {

}
} //namespace roborts_detection
