/****************************************************************************
 *@brief		装甲板、视觉标签检测算法
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

#include <opencv2/ml/ml.hpp>

#include "constraint_set.h"
#include "math.h"
#include "timer/timer.h"
#include "io/io.h"
#include "../../../roborts_decision/my_file.h"



namespace roborts_detection {
#define img_width 640 //图像宽度
#define img_heigth 480 //图像高度
#define HALF_LENGTH 75  //视觉标签
#define NUM_ 1000  //视觉标签
static int num_=0;
/***********************************************[constraint_set函数]*************************************************
 * @ brief:
 * *****************************************************************************************************************/
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
/*************************************************[LoadParam函数]****************************************************
 * @ brief:
 * *****************************************************************************************************************/
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


  //algorithm threshold parameters
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
   //感兴趣区域
  roi_rect_.x=0;
  roi_rect_.y=0;
  roi_rect_.width=640;//1440;
  roi_rect_.height=480;//1080;
  //初始化装甲板  
  std::vector<cv::Point2f>init_points;
  float a=0;
  cv::Point2f init_2f(a,a);
  init_points.push_back(init_2f);
  init_points.push_back(init_2f);
  init_points.push_back(init_2f);
  init_points.push_back(init_2f);
  cv::RotatedRect init_rotated_Rect(Point2f(0,0),cv::Size2f(100,50),30);
  bool		init_bool=0;
  cv::Rect	init_tracking_rect(0,0,0,0);
  int 		init_armor_id=0;
  float		armor_stddev=0;
  last_armor_.rect=init_rotated_Rect;
  last_armor_.vertex=init_points;
  last_armor_.stddev=armor_stddev;
  last_armor_.single=init_bool;
  last_armor_.id=init_armor_id;
  last_armor_.tracking_rect=init_tracking_rect;
  //偏移量
  offset_=cv::Point(0,0); 
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
/***********************************************[  DetectArmor函数  ]*************************************************
 *@ brief: 
**********************************************************************************************************************/
ErrorInfo ConstraintSet::DetectArmor(cv::Mat &result_img, bool &detected, cv::Point3f &target_3d, std_msgs::Float64MultiArray &msg1, float pitch_now, float yaw_now, float &yaw_output, float &pitch_output, bool &firecommand, float &distance_out,int &armor_id) 
{
  std::vector<cv::RotatedRect> lights;
  std::vector<cv::RotatedRect> lights_copy;
  std::vector<ArmorInfo> armors;
  std::vector<cv::RotatedRect> marker;
  //ROI
  roiFilter(src_img_, offset_);  
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
    gammaTransform(src_img_, src_img_);
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
  if (enable_debug_) 
  {
    show_lights_after_filter_ = src_img_.clone();
    show_lights_before_filter_ = src_img_.clone();
    show_armors_befor_filter_ = src_img_.clone();
    show_armors_after_filter_ = src_img_.clone();
   
  }
  DetectLights(src_img_, lights);
  FilterLights(lights);
  sortLights(lights);
  PossibleArmors(lights, armors);
  FilterArmors(armors);
  if(!armors.empty()) 
  {
    float absolute_pitch_;
    float absolute_yaw_;
    detected = true;
    ArmorInfo final_armor = SlectFinalArmor(armors);
    if(enable_debug_) 
    {
      cv_toolbox_->DrawRotatedRect(src_img_, armors[0].rect, cv::Scalar(0, 255, 0), 2);
      cv::circle(src_img_, Point(armors[0].rect.center.x, armors[0].rect.center.y), 3, cv::Scalar(0,0,255), 1, 8);    
      cv::line(src_img_,  Point(320, 240),Point(armors[0].rect.center.x, armors[0].rect.center.y),cv::Scalar(0,255,255));   
    }
    Target output_target;
    CalcControlInfo(final_armor, target_3d, pitch_now, yaw_now, yaw_output, pitch_output, absolute_pitch_, absolute_yaw_, output_target, distance_out);          
    armor_id=2;
    Get(final_armor, yaw_output, pitch_output, absolute_pitch_, absolute_yaw_, firecommand);      
    }else{     
      detected = false;
      yaw_output = 0;
      pitch_output = 0;
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
      setLastArmor(tmp_armor);
    }
    if(enable_debug_) 
    {
      cv::imshow("result_img_", src_img_);
    }
    msg1=msg;
    lights.clear();
    lights_copy.clear();                                                                                      
    armors.clear();
    cv_toolbox_->ReadComplete(read_index_);
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

ErrorInfo ConstraintSet::DetectMarkers(cv::Mat &result_img, bool &detected, cv::Point3f &target_3d,float &dex)
{
  //ROI
  roiFilter(src_img_, offset_);  
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
    gammaTransform(src_img_, src_img_);
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
  if(enable_debug_)
  {
     show_marker_before_filter_ = src_img_.clone();
  }
  DetectionVisionMarks(src_img_,target_3d,detected,dex);
  //color_Thead(src_img_);
  cv_toolbox_->ReadComplete(read_index_);
  //ROS_INFO("read complete");
  detection_time_ = std::chrono::duration<double, std::ratio<1, 1000000>>
      (std::chrono::high_resolution_clock::now() - detection_begin).count();
  return error_info_;
}


bool ConstraintSet::JudgeColor(cv::RotatedRect single_light,Mat &src) 
     {
	bool flag= false;
	cv::Rect judge_rect=single_light.boundingRect();
	if(rectSafety(judge_rect, src.rows, src.cols)==false)
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

/************************************[   DetectLights函数  ]******************************* 
 *@ brief:主线1：通道分离、通道相减、阈值分隔；
 *@ brief:主线2：灰度化、阈值分割；
 *@ brief：1和2的重合部分为检测到的灯条，利用 LightInfo类求解灯条矩形的信息；
****************************************************************************************/
void ConstraintSet::DetectLights(const cv::Mat &src_, std::vector<cv::RotatedRect> &lights)
{
  cv::Mat src;
  src = src_(roi_rect_);
  cv::cvtColor(src, gray_img_, CV_BGR2GRAY);
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::Mat binary_brightness_img, binary_light_img, binary_color_img;
  if(using_hsv_) 
  {
    binary_color_img = cv_toolbox_->DistillationColor(src, enemy_color_, using_hsv_);
    cv::threshold(gray_img_, binary_brightness_img, color_thread_, 255, CV_THRESH_BINARY);
  }else{
    auto light = cv_toolbox_->DistillationColor(src, enemy_color_, using_hsv_);
    cv::threshold(gray_img_, binary_brightness_img, color_thread_, 255, CV_THRESH_BINARY);
    float thresh;
    if (enemy_color_ == BLUE)
      thresh = blue_thread_;
    else
      thresh = red_thread_;
    cv::threshold(light, binary_color_img, thresh, 255, CV_THRESH_BINARY);
    dilate(binary_color_img, binary_color_img, element);//膨胀
  }
  if (enable_debug_) 
  {
    cv::imshow("binary_brightness_img", binary_brightness_img);
    cv::imshow("binary_color_img", binary_color_img);
  }
  auto contours_light = cv_toolbox_->FindContours_Chao(binary_color_img, offset_);
  auto contours_brightness = cv_toolbox_->FindContours_Chao(binary_brightness_img, offset_);
  lights.reserve(contours_light.size());
  // TODO: To be optimized
  for (unsigned int i = 0; i < contours_brightness.size(); ++i) 
  {
		for (unsigned int j = 0; j < contours_light.size(); ++j) 
		{
			if (cv::pointPolygonTest(contours_light[j], contours_brightness[i][0], false) >= 0.0) 
			{
				//IF contours_brightness‘point is in the contours_light
				cv::RotatedRect single_light = cv::minAreaRect(contours_brightness[i]);//[-90,0]
				LightBar lightbar(single_light);
				if (enable_debug_)
				{
					cv_toolbox_->DrawRotatedRect(show_lights_before_filter_, single_light, cv::Scalar(0, 255, 0), 1,single_light.angle);
					cv::rectangle(show_lights_before_filter_, roi_rect_, cv::Scalar(0, 0, 255), 2);	  
				}
				lights.push_back(single_light);//lighs_copy的角度[-90,0]
				break;	
			}     
		}   
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

/**************************************[FilterLights]************************************
*@ brief:根据一定条件过滤不满足条件的灯条
*@param:lights  所有检测到灯条旋转矩形
****************************************************************************************/
void ConstraintSet::FilterLights(std::vector<cv::RotatedRect> &lights) 
{
  std::vector<cv::RotatedRect> rects;                                                                                         
  rects.reserve(lights.size());                                                                                             
  for (unsigned int i =0; i < lights.size(); i++) 
  {
    float angle;
    float angle_copy;
    cv::RotatedRect light = lights[i];
    auto light_aspect_ratio_copy =
        std::max(light.size.width, light.size.height) / std::min(light.size.width, light.size.height);
		//https://stackoverflow.com/questions/15956124/minarearect-angles-unsure-about-the-angle-returned/21427814#21427814
    if(light.size.width < light.size.height) 
    {
      angle =light.angle; // -light.angle
    } else{
      angle =light.angle; // light.angle + 90
    }
    rects.push_back(light);
    if (enable_debug_)
    {
      cv_toolbox_->DrawRotatedRect(show_lights_after_filter_, light, cv::Scalar(0, 255, 0), 1, light.angle);  
    }   
  }
  if (enable_debug_)
    cv::imshow("lights_after_filter", show_lights_after_filter_);
  lights = rects;

}

/**************************************[ PossibleArmors函数 ]******************************
*@ brief:
*@param:lights  所有检测到灯条旋转矩形
*@param:armors  所有装甲板的信息
*@param:lights_copy  
****************************************************************************************/
void ConstraintSet::PossibleArmors(const std::vector<cv::RotatedRect> &lights, std::vector<ArmorInfo> &armors) 
{
  std::vector<bool> see(lights.size(),0);
//******双灯条******//
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
			//左侧灯条参数：
			float left_lightbar_width,left_lightbar_height;//长、宽
			float light1_angle;//角度

			//右侧灯条参数
			float right_lightbar_width,right_lightbar_height;//长、宽
			float light2_angle;//角度

			//search_rotatedrect参数
			cv::RotatedRect search_rotatedrect;
			int height_min_rotated,width_min_rotated;
			cv::Point rotatedrect_center;
			float rotatedrect_angle;

			//search_rect参数  框出数字的矩形
			cv::Rect search_rect;	
			float search_rect_width, search_rect_height;
			cv:Point search_rect_tl,search_rect_br;
			int height_min_rect;

			//数字的旋转矩形参数
			cv::RotatedRect Numb_rotatedrect;
			cv::Point2f Numb_rects_center;

			//数字矩形参数
			std::vector<cv::Rect> Numb_rects;//数字边缘轮廓
			cv::Rect Numb_rect;
			cv::Point2f Numb_bl, Numb_tl, Numb_tr, Numb_br;
			std::vector<cv::Point2f> Numb_points;

			//lifht1_copy矩形角点
			cv::Point2f left_bl, left_tl, left_tr, left_br;	
			//lift2_copy矩形角点
			cv::Point2f right_bl, right_tl, right_tr, right_br;

			//临时旋转矩形，选择light1_copy和light2_copy时的中间变量
			cv::RotatedRect lightbar_tmp;
			float height_tmp, width_tmp;	  	 	  
			if(getEligibility(light1, light2) )
			{	    	    
				if (light1.size.width < light1.size.height) 
				{
					left_lightbar_width = light1.size.width;
					left_lightbar_height = light1.size.height;
					light1_angle = light1.angle+90;			  	      
				}else {
					left_lightbar_width = light1.size.height;
					left_lightbar_height = light1.size.width;
					light1_angle = light1.angle+180;
				}
				if (light2.size.width < light2.size.height) {
					right_lightbar_width = light2.size.width;
					right_lightbar_height = light2.size.height;
					light2_angle = light2.angle+90;	      
				}else {
					right_lightbar_width = light2.size.height;
					right_lightbar_height = light2.size.width;
					light2_angle = light2.angle+180;	      
				}
				height_min_rotated = 2.5*std::max(left_lightbar_height,right_lightbar_height);
				width_min_rotated = abs(abs(light1.center.x-light2.center.x)-3*std::max(left_lightbar_width,right_lightbar_width));
				rotatedrect_center=(light1.center+light2.center)/2;
				rotatedrect_angle=(left_lightbar_height>right_lightbar_height ? light1.angle:light2.angle);
				if(rotatedrect_angle<-45)
				{
					search_rotatedrect=cv::RotatedRect(rotatedrect_center,cv::Size2f(height_min_rotated,width_min_rotated),rotatedrect_angle);	      
				}else{
					search_rotatedrect=cv::RotatedRect(rotatedrect_center,cv::Size2f(width_min_rotated,height_min_rotated),rotatedrect_angle);	      
				}
				height_min_rect = std::max(left_lightbar_height,right_lightbar_height);
				getCorrectPointOder(light1, left_bl, left_tl, left_tr, left_br);
				getCorrectPointOder(light2, right_bl, right_tl, right_tr, right_br);
				search_rect_tl.x=std::max(left_br.x,left_tr.x)+6;
				search_rect_tl.y= (light1.center.y+light2.center.y)/2 - height_min_rect-abs(light1.center.y-light2.center.y);
				search_rect_br.x=std::min(right_bl.x,right_tl.x)-6;
				search_rect_br.y=(light1.center.y+light2.center.y)/2 + 	height_min_rect+abs(light1.center.y-light2.center.y);
				cv::Rect search_rect=cv::Rect(search_rect_tl,search_rect_br);
				bool safety=rectSafety(search_rect,img_heigth,img_width);
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
				std::vector<std::vector<cv::Point>>contours;//
				cv::findContours(image_numb,contours,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
				for(std::vector<cv::Point> const &contour : contours)
				{
					if (fabs(contourArea(contour, 1)) < 5)
					continue;
					Numb_rect = boundingRect(contour);
					if(rectSafety(Numb_rect,img_heigth,img_width)==false)
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
					Numb_rects[0]=Numb_rects[0].area()>Numb_rects[i].area() ? Numb_rects[0]:Numb_rects[i];	//取轮廓面积最大的      
				}
				search_rect_width=search_rect.size().width < search_rect.size().height?search_rect.size().width:search_rect.size().height; //取值小的为宽
				search_rect_height=search_rect.size().width > search_rect.size().height?search_rect.size().width:search_rect.size().height;//取值的的为高
				lightbar_tmp=light1_angle>light2_angle?light1:light2;//选出角度大的
				width_tmp=lightbar_tmp.angle>-45?Numb_rects[0].width:Numb_rects[0].height;
				height_tmp=lightbar_tmp.angle<-45?Numb_rects[0].width:Numb_rects[0].height;
				Numb_rects_center=(Numb_rects[0].tl()+Numb_rects[0].br())/2;
				if(width_tmp>5 && height_tmp>5 && Numb_rects_center.x >=0 && Numb_rects_center.y>=0)
					Numb_rotatedrect = cv::RotatedRect(Numb_rects_center + search_rotatedrect.center - cv::Point2f(search_rect_width / 2, search_rect_height / 2),
														cv::Size2f(width_tmp, height_tmp),
														lightbar_tmp.angle);	      
				else
					continue;
				if(Numb_rotatedrect.size.area()<10 || Numb_rotatedrect.center.x < 0|| Numb_rotatedrect.center.y < 0)
					continue;	    	    
				if(last_armor_.id==0)
				{	     	      
					MLHogSVM(src_img_,Numb_rotatedrect,enemy_color_,0);
					if(svmresult)
					{	
						//std::cout<<" <<<MLHogSvm passed "<<std::endl;	
						getCorrectPointOder(Numb_rotatedrect, Numb_bl, Numb_tl, Numb_tr, Numb_br);
						Numb_points.push_back(Numb_tl);
						Numb_points.push_back(Numb_tr);
						Numb_points.push_back(Numb_br);
						Numb_points.push_back(Numb_bl);
						cv::Rect trackingrect;
						trackingrect=search_rotatedrect.boundingRect();
						if(rectSafety(trackingrect,img_heigth,img_width)==false)
						continue;
						int thre=cvRound(threshold(image_numb,image_numb,0,255,CV_THRESH_OTSU));
						armors.emplace_back(ArmorInfo(Numb_rotatedrect, Numb_points, 0, armor_id, trackingrect, thre));
						see[i]=1;
						see[j]=1;							
					}else{
						std::cout<<" <<<MLHogSvm didn't passed "<<std::endl;					
					}
					if (enable_debug_)
					{
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
					if(rectSafety(trackingrect,img_heigth,img_width)==false)
						continue;
					getCorrectPointOder(Numb_rotatedrect, Numb_bl, Numb_tl, Numb_tr, Numb_br);
					Numb_points.clear();
					std::vector<cv::Point2f>().swap(Numb_points);
					Numb_points.push_back(Numb_tl);
					Numb_points.push_back(Numb_tr);
					Numb_points.push_back(Numb_br);
					Numb_points.push_back(Numb_bl);
					int thre=cvRound(threshold(image_numb,image_numb,0,255,CV_THRESH_OTSU));
					ArmorInfo tmp_armor(Numb_rotatedrect, Numb_points, 0, last_armor_.id, trackingrect, thre);
					setMlRoi(tmp_armor,src_img_);

					if(fastTargetLock(tmp_armor,last_armor_)==true)
					{		
						//std::cout<<"<<<Fast passed "<<std::endl;	
						std::vector<ArmorInfo> tmp_armors;
						tmp_armors.emplace_back(ArmorInfo(Numb_rotatedrect, Numb_points, 0, last_armor_.id, trackingrect, thre));
						armors=tmp_armors;
						cv_toolbox_->DrawRotatedRect(src_img_, Numb_rotatedrect, cv::Scalar(0, 0, 255), 2);
						tmp_armors.clear();
						std::vector<ArmorInfo>().swap(tmp_armors);
						return;							
					}else{
						//std::cout<<"<<<Fast didn't passed "<<std::endl;		
						MLHogSVM(src_img_,Numb_rotatedrect,enemy_color_,0);
						if(svmresult)
						{		  
							//std::cout<<"<<<MLHogSvm passed "<<std::endl;		  
							int thre=cvRound(threshold(image_numb,image_numb,0,255,CV_THRESH_OTSU));
							armors.emplace_back(ArmorInfo(Numb_rotatedrect, Numb_points, 0, armor_id, trackingrect, thre));
							see[i]=1;
							see[j]=1;		  			  		  
						}else{
							std::cout<<"<<<MLHogSvm didn't passed "<<std::endl;			  		  
						}
						if (enable_debug_)
						{
							cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, Numb_rotatedrect, cv::Scalar(0, 100, 100), 2);		  		  		  
						}
						Numb_points.clear();
						std::vector<cv::Rect>().swap(Numb_rects);
						std::vector<cv::Point2f>().swap(Numb_points);
						std::vector<cv::Mat>().swap(rgb);
						std::vector<std::vector<cv::Point>>().swap(contours);								      	      
					}	      	      	    
				}				    	  
			}	  		  	 			      
		}     	          
  }   
//******单灯条******//
  cv::Point2f Numb_bl, Numb_tl, Numb_tr, Numb_br;
  std::vector<cv::Point2f> Numb_points;
  for(unsigned int k = 0; k < lights.size(); k++)
  {
		svmresult=0;
		armor_id=0;
		cv::RotatedRect light0 = lights[k];
		float light0_angle_0;
		float light0_width,light0_height;
		if (light0.size.width < light0.size.height) 
		{
			light0_angle_0 = light0.angle+90;
			light0_width = light0.size.width;
			light0_height = light0.size.height;	  
		} else {
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
				left_search_rect = getSingleRotatedRect(light0, false,bl,tl,tr,br,light0_width,light0_height,light0_angle_0);		
			}
			if(fabs(light0_angle_0) < 90) 
			{
				right_search_rect = getSingleRotatedRect(light0, true,bl,tl,tr,br,light0_width,light0_height,light0_angle_0);	    
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
			bool safety=rectSafety(rectsaft,img_heigth,img_width);
			if(safety== false)
			continue;
			cv::Mat image_roi;
			cv::Mat tmp=src_img_.clone();
			cv::Mat image_roi_tmp=tmp(rectsaft);
			cv::Mat matri;	     
				//TODO:TMP!!!     
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
			bool safety_1 = rectSafety_1(last_roi,image_roi.size());
			if(safety_1==false)
				continue;
			cv::Mat last_image=image_roi(last_roi);
			cv::Mat otus_image;
			cv::Rect matri_rect;
			bool safety_otus;     
			if(lr==0)
			{
				matri_rect=cv::Rect(cv::Point(0.65*last_image.cols,0.35*last_image.rows),cv::Point(0.95*last_image.cols,0.65*last_image.rows));
				safety_otus = rectSafety(matri_rect, last_image.rows, last_image.cols);
				if(safety_otus==false)
					continue;
				otus_image=last_image(matri_rect);		
			} else{
				matri_rect=cv::Rect(cv::Point(0.05*last_image.cols,0.35*last_image.rows),cv::Point(0.45*last_image.cols,0.65*last_image.rows));
				safety_otus = rectSafety(matri_rect, last_image.rows, last_image.cols);
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
			if(!regionOtsuThreshold(last_image,last_image,thre, bool(lr))) 
				continue;	    
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
				width = Numb_rects[0].size.width < Numb_rects[0].size.height ? Numb_rects[0].size.width : Numb_rects[0].size.height;
				heigth = Numb_rects[0].size.width > Numb_rects[0].size.height ? Numb_rects[0].size.width : Numb_rects[0].size.height;		
			}
			if(light0.angle<-45)
			{
				width = Numb_rects[0].size.width > Numb_rects[0].size.height ? Numb_rects[0].size.width : Numb_rects[0].size.height;
				heigth = Numb_rects[0].size.width < Numb_rects[0].size.height ? Numb_rects[0].size.width : Numb_rects[0].size.height;		
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
			if(Numb_rotatedrect.size.area()<10 || Numb_rotatedrect.center.x < 0 || Numb_rotatedrect.center.y < 0)
				continue;
			if(lr==0)
				MLHogSVM(src_img_,Numb_rotatedrect,enemy_color_,1);
			else
				MLHogSVM(src_img_,Numb_rotatedrect,enemy_color_,2);
			if(svmresult)
			{
				getCorrectPointOder(Numb_rotatedrect, Numb_bl, Numb_tl, Numb_tr, Numb_br);
				Numb_points.push_back(Numb_tl);
				Numb_points.push_back(Numb_tr);
				Numb_points.push_back(Numb_br);
				Numb_points.push_back(Numb_bl);
				cv::Rect trackingrect;
				trackingrect=search_rect_0.boundingRect();
				if(rectSafety(trackingrect,img_heigth,img_width)==false)
					continue;
				armors.emplace_back(ArmorInfo(Numb_rotatedrect, Numb_points, 1, armor_id, trackingrect, thre));		
				Numb_points.clear();	  
				std::vector<cv::Point2f>().swap(Numb_points);
				std::vector<cv::RotatedRect>().swap(Numb_rects);
				std::vector<std::vector<cv::Point>>().swap(contours);
				see[k]=1;					    
			}
			else
				continue;
			if (enable_debug_)
			{
				cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, Numb_rotatedrect, cv::Scalar(0, 100, 100), 2);
				cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, light0, cv::Scalar(255, 0, 0), 2);	  	    
			}	  	  		
		}	    
	}
  if (enable_debug_)
    cv::imshow("armors_before_filter", show_armors_befor_filter_);
  std::vector<bool>().swap(see);  
}

//-----------------------------------[  FilterArmors函数  ]-------------------------------------//
//@ brief: 
//----------------------------------------------------------------------------------------------//
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
  // nms
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
    } else if (enable_debug_) {
      cv_toolbox_->DrawRotatedRect(show_armors_after_filter_, armors[i].rect, cv::Scalar(0, 255, 0), 2);
    }
  }
  if (enable_debug_)
    cv::imshow("armors_after_filter", show_armors_after_filter_);
}


//-------------------------------------------------[SlectFinalArmor函数]------------------------------------------------//
//@brief:
//----------------------------------------------------------------------------------------------------------------------//
ArmorInfo ConstraintSet::SlectFinalArmor(std::vector<ArmorInfo> &armors)
{
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
    setLastArmor(tmp_armor);
    return tmp_armor;
  }else{
    std::vector<ArmorInfo> tmp_armors;
    for (const auto &armor : armors) {
      tmp_armors.push_back(armor);
    }
    if(tmp_armors.empty())
    {
      std::sort(armors.begin(),
		armors.end(),
		[](const ArmorInfo &p1, const ArmorInfo &p2) { return p1.rect.size.area() > p2.rect.size.area(); });
    }else{
      armors=tmp_armors;
      std::sort(armors.begin(),
		armors.end(),
		[](const ArmorInfo &p1, const ArmorInfo &p2) { return p1.rect.size.area() > p2.rect.size.area(); });
    }
    cv::Rect boundRect = armors[0].tracking_rect;
    msg.data.push_back(boundRect.tl().x);
    msg.data.push_back(boundRect.tl().y);
    msg.data.push_back(boundRect.br().x);
    msg.data.push_back(boundRect.br().y);
    setLastArmor(armors[0]);
    cv_toolbox_->DrawRotatedRect(src_img_, armors[0].rect, cv::Scalar(255, 0, 0), 2);
    return armors[0];
  }
}

bool ConstraintSet::rotatingCalipers( const Point2f* points, int n, float* out, float angle , float deviation)
{
	float minarea = FLT_MAX, minarea_angle = FLT_MAX;
	float max_dist = 0;
	char  buffer[32] = {};
	char  buffer_angle[32] = {};
	int   i, k;
	
	AutoBuffer<float> abuf(n*3);
	float* inv_vect_length = abuf;
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
//-- find convex hull orientation
#if 1
	double ax = vect[n-1].x;
	double ay = vect[n-1].y;
	for( i = 0; i < n; i++ )
	{
		double bx = vect[i].x;
		double by = vect[i].y;

		double convexity = ax * by - ay * bx;

		if( convexity != 0 ) {
			orientation = (convexity > 0) ? 1.f : (-1.f);
			break;
		}
		ax = bx;
		ay = by;
	}
	CV_Assert( orientation != 0 );
#endif 
	
	base_a = orientation;
	//-- init calipers position                                        
	seq[0] = bottom;
	seq[1] = right;
	seq[2] = top;
	seq[3] = left;
	
	/*           Main loop - evaluate angles and rotate calipers          */

	/* all of edges will be checked while rotating calipers by 90 degrees */
	for( k = 0; k < n; k++ ) 
	{	
#if 0
		sinus of minimal angle
		float sinus
		compute cosine of angle between calipers side and polygon edge 
		dp - dot product 
#endif 
		float dp[4] = {
						+base_a * vect[seq[0]].x + base_b * vect[seq[0]].y,
						-base_b * vect[seq[1]].x + base_a * vect[seq[1]].y,
						-base_a * vect[seq[2]].x - base_b * vect[seq[2]].y,
						+base_b * vect[seq[3]].x - base_a * vect[seq[3]].y,
		};
		float maxcos = dp[0] * inv_vect_length[seq[0]];
		
		// number of calipers edges, that has minimal angle with edge 
		int main_element = 0;

		//-- choose minimal angle 
		for ( i = 1; i < 4; ++i ) 
		{
			float cosalpha = dp[i] * inv_vect_length[seq[i]];
			if (cosalpha > maxcos) 
			{
				main_element = i;
				maxcos = cosalpha;
			}
		}
		
		//-- rotate calipers
#if 1
		//-- get next base
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
		
#endif 
		//-- change base point of main edge 
		seq[main_element] += 1;
		seq[main_element] = (seq[main_element] == n) ? 0 : seq[main_element];
		float height;
		float area;
		// find vector left-right
		float dx = points[seq[1]].x - points[seq[3]].x;
		float dy = points[seq[1]].y - points[seq[3]].y;

		// dotproduct 
		float width = dx * base_a + dy * base_b;

		// find vector left-right 
		dx = points[seq[2]].x - points[seq[0]].x;
		dy = points[seq[2]].y - points[seq[0]].y;

		// dotproduct 
		height = -dx * base_b + dy * base_a;
		float tmp_angle;
		if(height>width) 
		{
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
	} else {
		buf = (float *) buffer;
		return false;
		
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
		}else{
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

	}
	else if( n == 2 ) 
	{
		box.center.x = (hpoints[0].x + hpoints[1].x) * 0.5f;
		box.center.y = (hpoints[0].y + hpoints[1].y) * 0.5f;
		double dx = hpoints[1].x - hpoints[0].x;
		double dy = hpoints[1].y - hpoints[0].y;
		box.size.width = (float)std::sqrt(dx * dx + dy * dy);
		box.size.height = 0;
		box.angle = (float)atan2( dy, dx);
	}
	else 
	{
		if( n == 1 )
			box.center = hpoints[0];
	}
	box.angle = (float)(box.angle*180/CV_PI);
	return box;
}


bool ConstraintSet::NoLightBar_Detect(cv::Mat &src, std::vector<ArmorInfo> &armors) 
{
	int last_threshold_=last_armor_.num_threshold;
	if(rectSafety(roi_rect_, src_img_.rows, src_img_.cols)==false)
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
		if(rectSafety(trackingrect,img_heigth,img_width)==false)
		continue;
		getCorrectPointOder(rot_rect, Numb_bl, Numb_tl, Numb_tr, Numb_br);
		Numb_points.push_back(Numb_tl);
		Numb_points.push_back(Numb_tr);
		Numb_points.push_back(Numb_br);
		Numb_points.push_back(Numb_bl);
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
		setMlRoi(tmp_armor,src_img_);
		if(fastTargetLock(tmp_armor,last_armor_)==true)
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
//***********pnp解算*********************// 
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
	}
	else
	{
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
//-------------------------------------------------[PnpAssess函数]--------------------------------------------------------------//
//@brief:世界坐标转换成图像坐标，求图像坐标的计算量与实际量的误差，评定pnp算法的准确性
//-----------------------------------------------------------------------------------------------------------------------------//
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

//-------------------------------------------------[CoordinateTransformation函数]----------------------------------------------//
//@brief:
//-----------------------------------------------------------------------------------------------------------------------------//
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

#if 0
void ConstraintSet::PredictControlInfo(const ArmorInfo & armor, cv::Point3f &target_3d, float pitch_now, float yaw_now, float &yaw_output, float &pitch_output, float &absolute_pitch_, float &absolute_yaw_, struct timeval time)
{
  cv::Mat rvec;
  cv::Mat tvec;
  std::vector<cv::Point3f> numb_points_;
    switch(armor.id)
    {
      case 1:
	//width_1: 44 # mm
	//height_1: 102 # mm
	//width: 120 # mm
        //height: 60 # mm
	numb_points_.emplace_back(cv::Point3f(-22, 51,  0.0));
        numb_points_.emplace_back(cv::Point3f(22,  51,  0.0));
        numb_points_.emplace_back(cv::Point3f(22,  -51, 0.0));
        numb_points_.emplace_back(cv::Point3f(-22, -51, 0.0));
	cv::solvePnP(numb_points_,
               armor.vertex,
               intrinsic_matrix_,
               distortion_coeffs_,
               rvec,
               tvec);
        target_3d = cv::Point3f(tvec);
	break;
      case 2:
	//width_1: 66 # mm
	//height_1: 110 # mm
	numb_points_.emplace_back(cv::Point3f(-33, 55,  0.0));
        numb_points_.emplace_back(cv::Point3f(33,  55,  0.0));
        numb_points_.emplace_back(cv::Point3f(33,  -55, 0.0));
        numb_points_.emplace_back(cv::Point3f(-33, -55, 0.0));
	cv::solvePnP(numb_points_,
               armor.vertex,
               intrinsic_matrix_,
               distortion_coeffs_,
               rvec,
               tvec);
        target_3d = cv::Point3f(tvec);
	break;
	case 5:
	//width_1: 62 # mm
	//height_1: 102 # mm
	numb_points_.emplace_back(cv::Point3f(-31, 51,  0.0));
        numb_points_.emplace_back(cv::Point3f(31,  51,  0.0));
        numb_points_.emplace_back(cv::Point3f(31,  -51, 0.0));
        numb_points_.emplace_back(cv::Point3f(-31, -51, 0.0));
	cv::solvePnP(numb_points_,
               armor.vertex,
               intrinsic_matrix_,
               distortion_coeffs_,
               rvec,
               tvec);
        target_3d = cv::Point3f(tvec);
	break;
      default:
	break;
    }
    
    
    
    float absolute_pitch_,absolute_yaw_;
    double s=-3.945;//相机炮台高度差//cm
    double v=2000;//子弹射速
    double kangle = 0.233;
    yaw_output=atan2(320-armor.rect.center.x,intrinsic_matrix_.at<double>(0,0))*57.3;*/

    ////std::cout<<" yaw_now is "<<yaw_now<<std::endl;
    ////std::cout<<" yaw_output is "<<yaw_output<<std::endl;

 
    float disX,disY,disZ;
    disX=target_3d.x;
    disY=target_3d.y;
    disZ=target_3d.z;

    disX=tvec.at<double>(0);
    disY=tvec.at<double>(1);
    disZ=tvec.at<double>(2);

    float distance = sqrt((disX*disX)+(disY*disY)+(disZ*disZ));
    ////std::cout<<"11111111111"<<std::endl;
    ////std::cout<<" distance is "<<distance<<std::endl;
    ////std::cout<<"11111111111"<<std::endl;

  //predictor.Getpredict(disX, disY, disZ, armor.rect.center.x, distance, yaw_now, pitch_now, intrinsic_matrix_, pitch_output, yaw_output, absolute_pitch_, absolute_yaw_, time);
	
}
#endif 

cv::Rect ConstraintSet::rectCenterScale(cv::Rect rect, cv::Size2f size) {
	cv::Size arg=cv::Size(cvRound((size.width-1) *rect.size().width),  cvRound((size.height-1) *rect.size().height));
	rect= rect + arg;
	cv::Point pt;
	pt.x = cvRound(arg.width/2.0);
	pt.y = cvRound(arg.height/2.0);
	return (rect-pt);
}

/**************************************[ADD]*********************************************/
bool ConstraintSet::Get(const ArmorInfo & armor, float yaw, float pitch, float absolute_pitch_, float absolute_yaw_, bool &firecommand) {

#if 0
	float target_radius_=armor.rect.size.width>armor.rect.size.height?armor.rect.size.height/2:armor.rect.size.width/2;
    int distance;
    float yaw_diff=tan(fabs(yaw-absolute_yaw_)/57.3)*intrinsic_matrix_.at<double>(0,0);
    float pitch_diff=tan(fabs(pitch-absolute_pitch_)/57.3)*intrinsic_matrix_.at<double>(1,1);
    distance=sqrt(yaw_diff*yaw_diff+pitch_diff*pitch_diff);
    if(distance<target_radius_+20)
        firecommand = 1;
    else {
        firecommand = 0;
    }
#endif 
	if(std::abs(pitch)<=3&&std::abs(yaw)<=3)
		firecommand = 1;
	else 
		firecommand = 0;     
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
  }else{
    filter_count = 0;
    old_num = new_num;
  }
}
void ConstraintSet::SetThreadState(bool thread_state) 
{
  thread_running_ = thread_state;
}

/**************************************[sort]********************************************
 * @ brief 根据灯条矩形中心点的x值从小到大排序
 * *************************************************************************************/
void ConstraintSet::sortLights(std::vector<cv::RotatedRect> &lights)
{
  for(int i=0;i<lights.size();i++)  
  { 
    for(int j=i+1;j<lights.size();j++)
    {     
      cv::RotatedRect light_tmp;
      if(lights[i].center.x>lights[j].center.x)
      {
				light_tmp=lights[i];
				lights[i]=lights[j];
				lights[j]=light_tmp;
      }
    }   
  }
}
/**************************************[rectSafety]**************************************
 * @ brief 
 * *************************************************************************************/
bool ConstraintSet::rectSafety(cv::Rect &brect, int rows, int cols)
{                                                                   
	cv::Rect out_rect=cv::Rect(0,0,cols,rows);                                                                                          
	brect=brect&out_rect;                                                                                                               
	if(brect.width == 0 || brect.height == 0)
	{                                                                                          
		return false;                                                                                                                   
	} else { 	
		return true;                                                                                                                    
	}                                                                                                                                   
} 
/**************************************[rectSafety_1]************************************
 * @ brief 
 * *************************************************************************************/
bool ConstraintSet::rectSafety_1(cv::Rect &brect,cv::Size size) 
{                                                                       
	cv::Rect out_rect=cv::Rect(0,0,size.width,size.height);                                                                             
	brect=brect&out_rect;                                                                                                               
	if(brect.width == 0 || brect.height == 0){                                                                                          
		return false;                                                                                                                   
	} else {                                                                                                                            
		return true;                                                                                                                    
	}                                                                                                                                   
}  
/**************************************[RoiFilter]***************************************
 * @ brief 
 * *************************************************************************************/
void ConstraintSet::roiFilter(Mat &input, cv::Point &offset) 
{ 
  if(last_armor_.id!=0)
  {
    roi_rect_=last_armor_.tracking_rect;
    Size enlarge_size =last_armor_.single == 0?cv::Size(3.1,4.1):cv::Size(4.1,5.1);
    roi_rect_=Rect(cv::Point(0,0),cv::Size(img_width,img_heigth));//Rect(cv::Point(0,0),cv::Size(640,480));//rectCenterScale(roi_rect_,enlarge_size);
    //cv::rectangle(src_img_, roi_rect_, cv::Scalar(0, 0, 255), 3);
    if (rectSafety(roi_rect_, src_img_.rows, src_img_.cols)==false)
      roi_rect_=Rect(cv::Point(0,0),cv::Size(img_width,img_heigth));	  
  }else{
    roi_rect_=Rect(cv::Point(0,0),cv::Size(img_width,img_heigth));
  }            
  offset = roi_rect_.tl();    
}
/**************************************[getDistance]***************************************
 * @ brief 解算两点欧式距离
 * *************************************************************************************/
float ConstraintSet::getDistance(cv::Point pointO, cv::Point point1) 
{
	float distance;
	distance = pow((pointO.x - point1.x), 2) + pow((pointO.y - point1.y), 2);
	distance = sqrt(distance);
	return distance;
}
/************************************[getEligibility]************************************
 * @ brief 从角度和距离两个条件判断灯条是否满足条件
 * *************************************************************************************/
bool ConstraintSet::getEligibility(const cv::RotatedRect &left_lightbar, const cv::RotatedRect &right_lightbar) 
{
  if(left_lightbar.center.x>right_lightbar.center.x)
    return false;
  float		tan_angle;//两个矩形框质心连线与水平线的夹角的正切
  int			left_lightbar_height = std::max(left_lightbar.size.height,left_lightbar.size.width);//左矩形的高
  int			right_lightbar_height = std::max(right_lightbar.size.height,right_lightbar.size.width);//右矩形的高
  float		left_lightbar_angle = left_lightbar.angle;
  float		right_lightbar_angle = right_lightbar.angle;
	
  cv::Point	left_lightbar_center = left_lightbar.center;
  cv::Point	right_lightbar_center = right_lightbar.center;
	
  int		height_min = std::min(left_lightbar_height,right_lightbar_height);
  int		height_max = std::max(left_lightbar_height,right_lightbar_height); 
  
  //中心距判据
  float point_distance = getDistance(left_lightbar_center, right_lightbar_center);
  if (point_distance > 3 * height_max || point_distance < height_min) 
    return false;         
  if(height_min==0)
    return false;         
  if (height_max / height_min > 3)
      return false;  
  //角度判据
  if (right_lightbar_center.x == left_lightbar_center.x) 
    tan_angle = 100;//防止除0的尴尬情况
  else
    tan_angle = fabs((right_lightbar_center.y - left_lightbar_center.y) / static_cast<float>((right_lightbar_center.x - left_lightbar_center.x)));    
  float grade = 200;
  if (fabs(left_lightbar_angle - right_lightbar_angle) > 20)
  {    
    if (fabs(left_lightbar_angle - right_lightbar_angle) <= 160)
      return false;               
  }        
  if (fabs(fabs(left_lightbar_angle) - fabs(right_lightbar_angle)) > 160)
    grade = grade - 2 * (180 - fabs(fabs(left_lightbar_angle) - fabs(right_lightbar_angle)));    
  else
    grade = grade - 2 * fabs(fabs(left_lightbar_angle) - fabs(right_lightbar_angle));    
  grade = grade - atan(tan_angle) * 180 * 0.6;
  if (grade > 165)  
    return true;    
  else
    return false;           
}
/***************************[ getCorrectPointOrder函]*************************************
*@ brief: 确定矩形的br、bl、tl、tr;
*@流程：
* 	1、通过for循环确定y值最大的点 和 第二大的点;
* 	2、通过比较步骤1两个点的x值 x值大的为br 、另一个点为bl;
* 	3、通过x值的大小确定另外两个点，大的为tr,小的为tl;
*****************************************************************************************/
void ConstraintSet::getCorrectPointOder(const cv::RotatedRect &rotatedRect ,cv::Point2f &bl,cv::Point2f &tl,cv::Point2f &tr,cv::Point2f &br)
{
  cv::Point2f vertices[4];                                                                                                            
  rotatedRect.points(vertices);                                                                                                       
  float	max_y=0;                                                                                                                      
  float	sec_y=0;                                                                                                                     
  int		max_y_index=0;      //y值最大的点                                                                                                            
  int		sec_y_index=0;      //y值第二大的点                                                                                                        
  int		t_max_x_index=0;                                                                                                                
  float	t_max_x=0;                                                                                                                    
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
/************************************[MLHogSVM]******************************************
 * @ brief 
 * *************************************************************************************/
void ConstraintSet::MLHogSVM(cv::Mat &src,cv::RotatedRect search_rotatedrect_raw,const int enemy_color_,int single)
{
  const cv::Size	kWinSize=cv::Size(24,24);
  const cv::Size	kBlockSize=cv::Size(8,8);
  const cv::Size 	kBlockStride=cv::Size(4,4);
  const cv::Size  	kBellSize=cv::Size(4,4);
  int			Bin=9;
  cv::Mat		img=src.clone();
  cv::Mat 		armorimg;
  cv::RotatedRect 	numb_rotatedrect;
  
  if(single==0)
    numb_rotatedrect=search_rotatedrect_raw;    
  else
  {
    if(search_rotatedrect_raw.angle<90)
      numb_rotatedrect=cv::RotatedRect(search_rotatedrect_raw.center,cv::Size2f(search_rotatedrect_raw.size.height,search_rotatedrect_raw.size.width),search_rotatedrect_raw.angle-90);     
    else
      numb_rotatedrect=cv::RotatedRect(search_rotatedrect_raw.center,cv::Size2f(search_rotatedrect_raw.size.width,search_rotatedrect_raw.size.height),search_rotatedrect_raw.angle-180);         
  }	 
  cv::RotatedRect roi = numb_rotatedrect;                                                                                      
  cv::Rect roi_mat =roi.boundingRect();                                                                                          
  if(rectSafety(roi_mat,src.rows,src.cols)==false)
    return;                                                                                         
  cv::Mat roiimg = src(roi_mat);                                                                                                 
  cv::Point center = roi.center;                                                                                                 
  float angle = 0;                                                                                                               
  if(numb_rotatedrect.angle<-45)
    angle = numb_rotatedrect.angle+90;    
  else
    angle =numb_rotatedrect.angle;    
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
		dis_x = cvRound((roi_mat.width-numb_rotatedrect.size.width)/2);
		dis_y = cvRound((roi_mat.height-numb_rotatedrect.size.height)/2);
		if(numb_rotatedrect.angle>-45)
		{
			realsizerect.x = center.x -roi_mat.x-cvRound(roi_mat.width/2)+dis_x;
			realsizerect.y = center.y -roi_mat.y-cvRound(roi_mat.height/2)+dis_y;
			realsizerect.width = numb_rotatedrect.size.width;
			realsizerect.height = numb_rotatedrect.size.height;	
		}else{
			realsizerect.x = center.x -roi_mat.x-cvRound(roi_mat.width/2);//+dis_y;
			realsizerect.y = center.y -roi_mat.y-cvRound(roi_mat.height/2);//+dis_x;
			realsizerect.width = numb_rotatedrect.size.height;
			realsizerect.height = numb_rotatedrect.size.width;	
		}
		break;
	case 1:
		dis_x = cvRound((roi_mat.width-numb_rotatedrect.size.width)/2);
		dis_y = cvRound((roi_mat.height-numb_rotatedrect.size.height)/2);
		if(numb_rotatedrect.angle>-45)
		{
			realsizerect.x = center.x -roi_mat.x-cvRound(roi_mat.width/2);
			realsizerect.y = center.y -roi_mat.y-cvRound(roi_mat.height/2);
			realsizerect.width = numb_rotatedrect.size.width;
			realsizerect.height = numb_rotatedrect.size.height;	
		}else{
			realsizerect.x = center.x -roi_mat.x-cvRound(roi_mat.width/2);
			realsizerect.y = center.y -roi_mat.y-cvRound(roi_mat.height/2);
			realsizerect.width = numb_rotatedrect.size.height;
			realsizerect.height = numb_rotatedrect.size.width;	
		}
		break;
	case 2:
		dis_x = cvRound((roi_mat.width-numb_rotatedrect.size.width)/2);
		dis_y = cvRound((roi_mat.height-numb_rotatedrect.size.height)/2);
		if(numb_rotatedrect.angle>-45)
		{
			realsizerect.x = center.x -roi_mat.x-cvRound(roi_mat.width/2)+dis_x;
			realsizerect.y = center.y -roi_mat.y-cvRound(roi_mat.height/2)+dis_y;
			realsizerect.width = numb_rotatedrect.size.width;
			realsizerect.height = numb_rotatedrect.size.height;	
		}else{
			realsizerect.x = center.x -roi_mat.x-cvRound(roi_mat.width/2);
			realsizerect.y = center.y -roi_mat.y-cvRound(roi_mat.height/2);
			realsizerect.width = numb_rotatedrect.size.height;
			realsizerect.height = numb_rotatedrect.size.width;	
		}
		break;
	default:
		break;   
  }	     
  if(rectSafety(realsizerect, armorimg.rows,armorimg.cols)==false)
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
      cv_toolbox_->DrawRotatedRect(img, numb_rotatedrect, cv::Scalar(0, 0, 255), 2);     
    }	      
  }                                                                                                    
  else
    return;                                                                                                                      
  cv::HOGDescriptor HogDescriptor(kWinSize,kBlockSize,kBlockStride,kBellSize,Bin);                                                   
  std::vector<float> ArmorHog;                                                                                                       
  const cv::Size kWinStride = cv::Size(60, 80);                                                                                      
  cv::resize(armorimg, armorimg, cv::Size(24, 24), CV_INTER_CUBIC);                                                                  
  HogDescriptor.compute(armorimg, ArmorHog, kWinStride);                                                                            
  static cv::Ptr<cv::ml::SVM> svm_hog_1 = cv::ml::SVM::load("/home/tdt/catkin_ws/src/RoboRTS-ros/roborts_detection/armor_detection/constraint_set/model/SVM(NEW_SMALL).xml");
  cv::Mat armorpredict(1, ArmorHog.size(), CV_32FC1);                                                                                                                                                                                                                        
  for (int j = 0; j < ArmorHog.size(); j++)
  {                                                                                        
    armorpredict.at<float>(j) = ArmorHog[j];                                                                                         
  }                                                                                                                                  
  int result_1;
  result_1 = svm_hog_1->predict(armorpredict);   
  svmresult=0;  
  float tmp_height,tmp_width,tmp_ratio;
  if(numb_rotatedrect.size.height>numb_rotatedrect.size.width)
  {
    tmp_height = numb_rotatedrect.size.height;
    tmp_width = numb_rotatedrect.size.width;   
  }else{
    tmp_height = numb_rotatedrect.size.width;
    tmp_width = numb_rotatedrect.size.height;  
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
  //cv::waitKey(0);
}
/************************************[setMlRoi]******************************************
 * @ brief 
 * *************************************************************************************/
void ConstraintSet::setMlRoi(ArmorInfo &armor,const cv::Mat &src)
{
  cv::Mat armorimg;
  cv::RotatedRect roi=armor.rect;
  cv::Rect roi_mat=roi.boundingRect();
  if(rectSafety(roi_mat,img_heigth,img_width)==false)
    return;
  cv::Rect roi_matcopy=Rect(roi_mat);
  if(rectSafety_1(roi_matcopy,src.size()))
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
    if(roi.angle>=-45){
			size_=roi.size;
    } else{
			size_=Size(roi.size.height,roi.size.width);
    }
    cv::Rect realsizerect(Point(roi.center)-roi_mat.tl()-Point(size_)/2,size_);
    if(rectSafety_1(realsizerect,armorimg.size()))
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
/************************************[fastTargetLock]************************************
 * @ brief 
 * *************************************************************************************/

//bool ConstraintSet::fastTargetLock1(cv::Mat input,cv::Mat mask) 
float ConstraintSet::fastTargetLock1(cv::Mat input,cv::Mat mask) 
{
  static HOGDescriptor *hog = new HOGDescriptor(Size(28, 28), Size(14, 14), Size(7, 7), Size(7, 7), 9);
  vector<float> descriptors_input;
  vector<float> descriptors_mask;
  if(input.empty())
  {
    return false;
  }
  if(mask .empty())
  {
    return false;
  }
  if(input.cols<15||input.rows<15)
  {
    return false;
  }
  hog->compute(input, descriptors_input,Size(1, 1), Size(0, 0));
  Mat des_input=Mat(descriptors_input);
  hog->compute(mask, descriptors_mask,Size(1, 1), Size(0, 0));
  Mat des_mask=Mat(descriptors_mask);
  if(des_input.empty()||des_mask.empty())
    return false;
  float similarity=getSimilarity(des_input,des_mask);
  return similarity;    
}

bool ConstraintSet::fastTargetLock(ArmorInfo &armor,ArmorInfo &last_armor) 
{
  if(last_armor.id==0) 
    return false;
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
  if(similarity>0.86)
  {
    last_armor = armor;
    return true;
  } else{
    return false;
  }    
}
/************************************[getSimilarity]*************************************
 * @ brief 
 * *************************************************************************************/
float ConstraintSet::getSimilarity(const cv::Mat& first,const cv::Mat& second)
{
	double	dotSum=first.dot(second);//内积
	double	normFirst=cv::norm(first);//取模
	double	normSecond=cv::norm(second);
	if(normFirst!=0 && normSecond!=0)
		return dotSum/(normFirst*normSecond);   
}
/*********************************[getSingleRotatedRect]*********************************
 * @ brief 
 * *************************************************************************************/
cv::RotatedRect ConstraintSet::getSingleRotatedRect(const cv::RotatedRect &lightBar,bool right_or_left,cv::Point2f &bl,cv::Point2f &tl,cv::Point2f &tr,cv::Point2f &br, const float light0_width, const float light0_height, const float light0_angle_0)
{   
	cv::RotatedRect search_rotatedrect;                                                                                                
	getCorrectPointOder(lightBar,bl,tl,tr,br);                                                                                         
	if(right_or_left)                                                                                                                  
	{                                                                                                                                  
		search_rotatedrect=cv::RotatedRect(cv::Point2f(lightBar.center.x+0.7*light0_width+ 0.8 *light0_height,          
																			((tr.y+tl.y)/2+(br.y+bl.y)/2)/2),                                                      
				cv::Size2f(2.3*light0_height,1.2*light0_height),light0_angle_0);                              
	}else{
		search_rotatedrect=cv::RotatedRect(cv::Point2f(lightBar.center.x-0.7*light0_width - 0.8*light0_height,          
																			((tr.y+tl.y)/2+(br.y+bl.y)/2)/2),                                                      
				cv::Size2f(2.3*light0_height,1.2*light0_height),light0_angle_0);                                
	}                                                                                                                                
	return search_rotatedrect;                                                                                                         
}  
/*********************************[regionOtsuThreshold]*********************************
 * @ brief 
 * *************************************************************************************/
bool ConstraintSet::regionOtsuThreshold(const Mat &inputimage, Mat &outputimage, int &thre, bool lr) 
{
  bool	ret= false;
  Mat	sum_row,judge;
  Mat	tmp;
  Rect	orect;
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
  if (sum(judge)[0] <= 1) {
    threshold(sum_row,judge,outputimage.cols*0.5,1,THRESH_BINARY);
    if (sum(judge)[0] < 0.5*sum_row.rows) 
	ret= true;
  }
  return ret;
}
/*********************************[setLastArmor]*****************************************
 * @ brief 
 * *************************************************************************************/
void ConstraintSet::setLastArmor(ArmorInfo last_armor)
{
    last_armor_=last_armor;
}
/*********************************[getLastArmor]*****************************************
 * @ brief 
 * *************************************************************************************/
ArmorInfo ConstraintSet::getLastArmor()const
{
    return last_armor_;
}
/********************************[gammaTransform]****************************************
 * @ brief 
 * *************************************************************************************/
void  ConstraintSet::gammaTransform(cv::Mat &src, cv::Mat &dist)
{
  Mat imageGamma;
  //灰度归一化
  src.convertTo(imageGamma, CV_64F, 1.0 / 255, 0);
  //伽马变换
  double gamma = 0.5;
  pow(imageGamma, gamma, dist);//dist 要与imageGamma有相同的数据类型
  dist.convertTo(dist, CV_8U, 255, 0);
}
/********************************[readImg]****************************************
 * @ brief:读取文件夹中所有的图片
 * *************************************************************************************/
void ConstraintSet::readImg(std::vector<cv::Mat> &pattern_imgs)
{
  //std::string pattern="/home/tdt/catkin_ws/src/RoboRTS-ros/roborts_detection/armor_detection/constraint_set/picture/*.jpg";
   //std::string pattern = ros::package::getPath("roborts_detection") +"/armor_detection/constraint_set/picture/*.png";
   std::string image_name;
   int i=0;
   int NUM=43;
   while(i<NUM)
   {
     stringstream ss;//int 转换成string
     std::string str;
     ss<<i;
     ss>>str;
     image_name=str;
     image_name= ros::package::getPath("roborts_detection") +"/armor_detection/constraint_set/picture/"+image_name+".png";
     Mat image =cv::imread(image_name);
     pattern_imgs.push_back(image);
     i++;
   }
#if 0
  std::vector<cv::String> result;//注意：类型必须是cv::String
  cv::glob(pattern,result);//void glob(String pattern, std::vector<String>& result, bool recursive = false);
			  //false只遍历指定文件夹符合条件的
			  //true会遍历文件夹下的子文件夹
  if(result.size()==0)
    std::cout<<"No Image Files[png]"<<std::endl;
  for (int i=0;i<result.size();++i)
  {
    Mat image =cv::imread(result[i]);
    pattern_imgs.push_back(image);
  }
#endif 
}
int  ConstraintSet::MLHogSVM(cv::Mat &roi_image)
{
  const cv::Size	kWinSize=cv::Size(128,128);
  const cv::Size	kBlockSize=cv::Size(16,16);
  const cv::Size 	kBlockStride=cv::Size(8,8);
  const cv::Size  	kBellSize=cv::Size(8,8);
  int			Bin=3;
  int                   result_1=100;
  
  cv::HOGDescriptor HogDescriptor(kWinSize,kBlockSize,kBlockStride,kBellSize,Bin);                                                   
  std::vector<float> ArmorHog;                                                                                                       
  //const cv::Size kWinStride = cv::Size(60, 80);                                                                                      
  cv::resize(roi_image, roi_image, cv::Size(128, 128), CV_INTER_CUBIC); 
  threshold(roi_image, roi_image, 0, 255, CV_THRESH_OTSU);//阈值化
  cv:imshow("roi_image",roi_image);
 // HogDescriptor.compute(roi_image, ArmorHog, kWinStride);  
   HogDescriptor.compute(roi_image, ArmorHog);  
  static cv::Ptr<cv::ml::SVM> svm_hog_1 = cv::ml::SVM::load(ros::package::getPath("roborts_detection") +"/armor_detection/constraint_set/model/SVMxin3(marker2).xml");
  // static cv::Ptr<cv::ml::SVM> svm_hog_1 = cv::ml::SVM::load("/home/tdt/SVMxin3(marker2).xml");
  cv::Mat armorpredict(1, ArmorHog.size(), CV_32FC1,Scalar::all(0));                                                                                                                                                                                                                        
  for (int j = 0; j < ArmorHog.size(); j++)
  {                                                                                        
    armorpredict.at<float>(0,j) = ArmorHog[j];                                                                                         
  }                                                                                                                                  
  result_1 = svm_hog_1->predict(armorpredict); 
  return result_1;
}
/****************************************[DetectionVisionMarks]************************************
 * 
 * 
 * ***********************************************************************************************/
void ConstraintSet::DetectionVisionMarks(const cv::Mat &src_,cv::Point3f &target_3d,bool &detected,float &dex)
{
 
  std::vector<cv::Mat> pattern_imgs;//样本图片
  readImg(pattern_imgs); 
  cv::Mat src;
  src_(roi_rect_).copyTo(src);
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
//--红色
#if 0
  cv::Mat split_std_img;//通道相减处理后灰度图
  cv::Mat threshold_std_img;//阈值化处理后的二值图
  std::vector<cv::Mat> bgr;//bgr三通道
  cv::split(src,bgr);//通道分离
  cv::imshow("bgr[0]",bgr[0]);
  cv::imshow("bgr[1]",bgr[1]);
  cv::imshow("bgr[2]",bgr[2]);
  
  cv::subtract(bgr[2],bgr[1],split_std_img);//通道相减
  cv::imshow("split_std_img",split_std_img);
#endif 
  
//-- yellow 
#if 0
  cv::Mat img_hsv;
  cv::cvtColor(src,img_hsv,CV_BGR2HSV);
  cv::Mat img_hsv_yellow;
  cv::Mat split_std_img;
  cv::Mat threshold_std_img;
  img_hsv_yellow=img_hsv.clone();
//   cv::Mat yellow_low(cv::Scalar(30,140,105));
//   cv::Mat yellow_higher(cv::Scalar(50,255,255));
  cv::Mat yellow_low(cv::Scalar(0,0,0));
  cv::Mat yellow_higher(cv::Scalar(180,178,255));
  //cv::inRange(threshold_std_img,yellow_low,yellow_higher,img_hsv_yellow);
  cv::inRange(img_hsv_yellow,yellow_low,yellow_higher,threshold_std_img);
#endif 

  //--orange 
#if 1
  cv::Mat img_hsv;
  vector<cv::Mat> hsv_split;
  cv::cvtColor(src,img_hsv,CV_BGR2HSV); //Convert the captured frame from BGR to HSV 
  split(img_hsv, hsv_split);
  equalizeHist(hsv_split[2],hsv_split[2]);
  merge(hsv_split,img_hsv);  
  cv::Mat threshold_std_img;
  cv::Mat yellow_low(cv::Scalar(0,50,0));
  cv::Mat yellow_higher(cv::Scalar(20,255,255));
  cv::inRange(img_hsv,yellow_low,yellow_higher,threshold_std_img);
   //开操作 (去除一些噪点)
   morphologyEx(threshold_std_img, threshold_std_img, MORPH_OPEN, element);
   //闭操作 (连接一些连通域)
   morphologyEx(threshold_std_img, threshold_std_img, MORPH_CLOSE, element);
   cv::imshow("threshold_std_img",threshold_std_img);
#endif 

  vector<vector<Point>> contours;
  vector<vector<Point>> contours_std;
  vector<Vec4i> hierarchy;
  vector<Rect> labelRects;
  findContours(threshold_std_img, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
  
  int count      = 0; //识别到的标签数量
  int final_size = 0; //最终选取的标签的尺寸
  int final_count;    //选取的面积最大的标签的标号
  for (unsigned int i = 0; i < contours.size(); i++) 
  {
    Rect label = boundingRect(contours[i]);
    if (hierarchy[i][3] == -1 && label.width >35 && label.height >35 && 0.9<label.width/label.height<1.1&&0.9<label.height/label.width<1.1)
    {
      labelRects.push_back(label);
      contours_std.push_back(contours[i]);
      if(contours[i].size()>final_size)
      {
				final_size = contours[i].size();
				final_count = count;
      }
      count = count+1;
    }
  }//寻找轮廓
  //cv::drawContours(src,contours_std,-1,cv::Scalar(0,255,0));
	vector<vector<Point2d>> labels;
	int cornerDetectSensibility = 50;//通道相减差值多大可认为是红色
	for (unsigned int iter = 0; iter < labelRects.size(); iter++) 
	{
		vector<Point2d> points;
		Point availablePoint[100];
		int x, y, cnt;
		cnt = 0;
		for (int j = 0; j < min(labelRects[iter].width, labelRects[iter].height) && cnt == 0; j++)
		for (int i = 0; i < j; i++) 
		{
			Point p(labelRects[iter].tl().x + i, labelRects[iter].tl().y + j - i);
			if ((int)threshold_std_img.at<uchar>(p.y, p.x) > cornerDetectSensibility) availablePoint[cnt++] = p;
		}
		x = min(availablePoint[0].x, availablePoint[cnt - 1].x);
		y = min(availablePoint[0].y, availablePoint[cnt - 1].y);
		points.push_back(Point2d(x, y)); //左上

		cnt = 0;
		for (int j = 0; j < min(labelRects[iter].width, labelRects[iter].height) && cnt == 0; j++)
			for (int i = 0; i < j; i++) 
			{
				Point p(labelRects[iter].br().x - i, labelRects[iter].tl().y + j - i);
				if ((int)threshold_std_img.at<uchar>(p.y, p.x) > cornerDetectSensibility) availablePoint[cnt++] = p;
			}
		x = max(availablePoint[0].x, availablePoint[cnt - 1].x);
		y = min(availablePoint[0].y, availablePoint[cnt - 1].y);
		points.push_back(Point2d(x, y)); //右上
		
		cnt = 0;
		for (int j = 0; j < min(labelRects[iter].width, labelRects[iter].height) && cnt == 0; j++)
			for (int i = 0; i < j; i++) 
			{
				Point p(labelRects[iter].br().x - i, labelRects[iter].br().y - j + i);
				if ((int)threshold_std_img.at<uchar>(p.y, p.x) > cornerDetectSensibility) availablePoint[cnt++] = p;
			}
		x = max(availablePoint[0].x, availablePoint[cnt - 1].x);
		y = min(availablePoint[0].y, availablePoint[cnt - 1].y);
		points.push_back(Point2d(x, y)); //右下

		cnt = 0;
		for (int j = 0; j < min(labelRects[iter].width, labelRects[iter].height) && cnt == 0; j++)
			for (int i = 0; i < j; i++) 
			{
				Point p(labelRects[iter].tl().x + i, labelRects[iter].br().y - j + i);
				if ((int)threshold_std_img.at<uchar>(p.y, p.x) > cornerDetectSensibility) availablePoint[cnt++] = p;
			}
		x = min(availablePoint[0].x, availablePoint[cnt - 1].x);
		y = max(availablePoint[0].y, availablePoint[cnt - 1].y);
		points.push_back(Point2d(x, y)); //左下

		for (int i = 0; i < 4; i++) 
		{
			cout << points[i] << endl;
			cv::line(show_marker_before_filter_, points[i], points[(i + 1) % 4], Scalar(0,0,255), 2);
		}
		labels.push_back(points);
		cout << endl;
	}
	imshow("ans", src);
	cv::imshow("show_marker_before_filter_",show_marker_before_filter_); 
  if(!labels.empty())
  {
    float P0_x=labels[final_count][0].x;
    float P0_y=labels[final_count][0].y;
    
    float P1_x=labels[final_count][1].x;
    float P1_y=labels[final_count][1].y;
    
    float P2_x=labels[final_count][2].x;
    float P2_y=labels[final_count][2].y;
    
    float P3_x=labels[final_count][3].x;
    float P3_y=labels[final_count][3].y;
    float width_1=P1_x-P0_x;
    float height_1=P3_y-P0_y;
    float width_2=P2_x-P3_x;
    float height_2=P2_y-P1_y;
    
    if(height_1<0.9*width_1)
    {
      labels[final_count][3].x=labels[final_count][0].x;
      labels[final_count][3].y=labels[final_count][0].y+width_1;         
    }
    
    if(height_2<0.9*width_2)
    {
      labels[final_count][2].x=labels[final_count][1].x;
      labels[final_count][2].y=labels[final_count][1].y+width_1;         
    }
    cv::Mat warp_img;
    Point2f srcPoints[3];//原图中的三点 ,一个包含三维点（x，y）的数组，其中x、y是浮点型数
    Point2f dstPoints[3];//目标图中的三点
    srcPoints[0] = labels[final_count][0];
    srcPoints[1] = labels[final_count][1];
    srcPoints[2] = labels[final_count][2];
    dstPoints[0] = Point2f(0, 0);
    dstPoints[1] = Point2f(src_img_.cols, 0);
    dstPoints[2] = Point2f(src_img_.cols, src_img_.rows);
    Mat M1 = getAffineTransform(srcPoints, dstPoints);//由三个点对计算变换矩阵
    warpAffine(src_img_, warp_img, M1, src.size());//仿射变换
    resize(warp_img,warp_img,Size(28,28));
    cv::imshow("experiment",warp_img);
    
    float max=0;//相似度最大的值
    int   dex_=0;//相似度最大标签的数字
    for(int i=0;i<pattern_imgs.size();i++)
    {
       resize(pattern_imgs[i],pattern_imgs[i],Size(28,28));
       float tmp=fastTargetLock1(warp_img,pattern_imgs[i]);
       if(tmp>max)
       {
	 max=tmp;
	 dex_=i;
       }
    }
    std::cout<<"相似度："<<max<<std::endl;
    if(max>0.5)
    {
      dex=dex_;
    }
    std::cout<<"num is:"<<dex_<<std::endl;
    std::cout<<"max is:"<<max<<std::endl;
    cv::waitKey(1);
  } 
  vector<Point3d> obj = vector<Point3d> {
      cv::Point3d(-HALF_LENGTH, HALF_LENGTH, 0), //tl
      cv::Point3d(HALF_LENGTH, HALF_LENGTH, 0),  //tr
      cv::Point3d(HALF_LENGTH, -HALF_LENGTH, 0) ,  //br
      cv::Point3d(-HALF_LENGTH, -HALF_LENGTH, 0)   //bl
  };
  
  if(!labels.empty())
  {
    cv::Mat rVec = cv::Mat::zeros(3, 1, CV_32FC1);//init rvec
    cv::Mat tVec = cv::Mat::zeros(3, 1, CV_32FC1);//init tvec
    solvePnP(obj, labels[final_count],  intrinsic_matrix_,distortion_coeffs_, rVec, tVec, false, SOLVEPNP_ITERATIVE);
    target_3d=cv::Point3f(tVec);
    detected = 1;
    //ROS_ERROR_STREAM("detected = 1");
    float tmp_x=target_3d.x;
    float tmp_y=target_3d.y;
    float tmp_z=target_3d.z;
		
    //------------------------------[旋转矩阵转换成欧拉角]------------------------------------//
#if 0
    cv::Mat R;
    cv::Rodrigues(rVec,R);
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    bool singular = sy < 1e-6; // If
    float x, y, z;
    if (!singular) {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    } else {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    std::cout<<"绕x轴转角："<<x<<std::endl;
    std::cout<<"绕y轴转角："<<y<<std::endl;
    std::cout<<"绕z轴转角："<<z<<std::endl;
#endif 
		
    //------------------------------[截取感兴趣区域并保存]------------------------------------//
    Mat roi_image;   
    vector <float> xs;
    vector <float> ys;
    for(int i=0;i<4;i++)
    {
       xs.push_back(labels[final_count][i].x);
       ys.push_back(labels[final_count][i].y);
    }
    float x_min=xs[0];
    float x_max=xs[0];
    float y_min=ys[0];
    float y_max=ys[0];
    for(int j=1;j<4;j++)
    {
			if (x_min>xs[j])
				x_min=xs[j];
			if(x_max<xs[j])
				x_max=xs[j];
			if (y_min>ys[j])
				y_min=ys[j];
			if(y_max<ys[j])
				y_max=ys[j];    
    }
    roi_image=src(Rect(x_min,y_min,x_max-x_min,y_max-y_min));
    float raio=float(roi_image.cols)/float(roi_image.rows);
    cv::cvtColor(roi_image,roi_image,CV_BGR2GRAY);
    int  a=MLHogSVM(roi_image);
    dex=(float)a;
		//------------------------------[保存样本]------------------------------------//
#if 0
    if(num_<NUM_)
    {
      if(0.7<raio && raio<1.3)
      {
      string s="/home/tdt/"+to_string(num_)+".png";
      cv::imwrite(s,roi_image);
      num_++;
      std::cout<<"num:"<<num_<<std::endl;
     }
    }else{
			ROS_ERROR_STREAM("NX 同学 图片收集满，请关闭节点！！！！！！！！！！！！！！！");
      }
#endif 
     
  } 
  auto c = cv::waitKey(1);
  if (c == 'a') 
  {
    cv::waitKey(0);
  }  
}

void ConstraintSet::color_Thead(const cv::Mat &src_)
{
  namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

  int iLowH = 0;
  int iHighH = 20;

  int iLowS = 50; 
  int iHighS = 255;

  int iLowV = 0;
  int iHighV = 255;

  //Create trackbars in "Control" window
  cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
  cvCreateTrackbar("HighH", "Control", &iHighH, 179);

  cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
  cvCreateTrackbar("HighS", "Control", &iHighS, 255);

  cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
  cvCreateTrackbar("HighV", "Control", &iHighV, 255);

  Mat src;
  src_(roi_rect_).copyTo(src);
  cv::imshow("src",src);
  cv::Mat img_hsv;
  vector<Mat>hsv_split;
  cv::cvtColor(src,img_hsv,CV_BGR2HSV);
  split(img_hsv,hsv_split);
  equalizeHist(hsv_split[2],hsv_split[2]);
  merge(hsv_split,img_hsv);
  cv::Mat threshold_std_img;
 
  cv::inRange(img_hsv,Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV),threshold_std_img);
  	//开操作 (去除一些噪点)
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(threshold_std_img, threshold_std_img, MORPH_OPEN, element);      
	//闭操作 (连接一些连通域)
	morphologyEx(threshold_std_img, threshold_std_img, MORPH_CLOSE, element);  
  cv::imshow("threshold_std_img",threshold_std_img);
	
#if 0
  Mat imgOriginal;
	src_(roi_rect_).copyTo(imgOriginal);
	Mat imgHSV;
	vector<Mat> hsvSplit;
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV     
	//因为我们读取的是彩色图，直方图均衡化需要在HSV空间做
	split(imgHSV, hsvSplit);
	equalizeHist(hsvSplit[2],hsvSplit[2]);
	merge(hsvSplit,imgHSV);
	Mat imgThresholded;    
	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image      
	//开操作 (去除一些噪点)
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);      
	//闭操作 (连接一些连通域)
	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);      
	imshow("Thresholded Image", imgThresholded); //show the thresholded image
	imshow("Original", imgOriginal); //show the original image    
#endif 
	auto c = cv::waitKey(1);
	if (c == 'a') 
	{
	  cv::waitKey(0);
	}
}
/**************************************[END]*********************************************/
ConstraintSet::~ConstraintSet() {

}
} //namespace roborts_detection
