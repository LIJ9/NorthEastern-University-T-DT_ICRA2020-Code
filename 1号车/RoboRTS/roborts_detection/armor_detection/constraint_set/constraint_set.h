/****************************************************************************
 *@brief		装甲板、视觉标签检测算法头文件
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

#ifndef ROBORTS_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_H
#define ROBORTS_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_H

#include <vector>
#include <list>


#include <opencv2/opencv.hpp>

#include "alg_factory/algorithm_factory.h"
#include "state/error_code.h"

#include "cv_toolbox.h"
#include "std_msgs/Float64MultiArray.h"

#include "../armor_detection_base.h"

#include "proto/constraint_set.pb.h"
//---------------------[ADD]----------------//
#include "predict.h"
#include "macro.h"
#include <fstream> //读取文件
//---------------------[END]----------------//

#include "constraint_set.h"
namespace roborts_detection {

using roborts_common::ErrorCode;
using roborts_common::ErrorInfo;

enum State {
  INITIALIZED = 0,
  RUNNING = 1,
  FAILED = 2,
  STOPED = 3
};

struct LightInfo {
                                                                                                                                 
  explicit LightInfo(cv::Point2f vertices[]) {
    auto edge_1 = std::pow(vertices[0].x - vertices[1].x, 2) +
        std::pow(vertices[0].y - vertices[1].y, 2);
    auto edge_2 = std::pow(vertices[1].x - vertices[2].x, 2) +
        std::pow(vertices[1].y - vertices[2].y, 2);

    if (edge_1 > edge_2)
    {
      width_  = (float)std::sqrt(edge_1);
      height_ = (float)std::sqrt(edge_2);
      if (vertices[0].y < vertices[1].y)
      {
        angle_ = std::atan2(vertices[1].y - vertices[0].y, vertices[1].x - vertices[0].x);
      } else {
        angle_ = std::atan2(vertices[0].y - vertices[1].y, vertices[0].x - vertices[1].x);
      }
    } else {
      width_  = (float)std::sqrt(edge_2);
      height_ = (float)std::sqrt(edge_1);

      if (vertices[2].y < vertices[1].y) 
      {
        angle_ = std::atan2(vertices[1].y - vertices[2].y, vertices[1].x - vertices[2].x);
      } else {
        angle_ = std::atan2(vertices[2].y - vertices[1].y, vertices[2].x - vertices[1].x);
      }
    }

    angle_ = (float)(angle_*180.0 / 3.1415926);
    area_ = width_ * height_;
    aspect_ratio_ = width_ / height_;
    center_.x = (vertices[1].x + vertices[3].x) / 2;
    center_.y = (vertices[1].y + vertices[3].y) / 2;
    vertices_.push_back(vertices[0]);
    vertices_.push_back(vertices[1]);
    vertices_.push_back(vertices[2]);
    vertices_.push_back(vertices[3]);
  }

 public:
  //! Light area
  float area_;
  //! Light angle, come from the long edge's slope
  float angle_;
  //! Light center
  cv::Point2f center_;
  //! Light aspect ratio = width_/height_
  float aspect_ratio_;
  //! Light width
  float width_;
  //! Light height
  float height_;
  //! Light vertices
  std::vector<cv::Point2f> vertices_;
};

/**
 *  This class describes the armor information, including maximum bounding box, vertex, standard deviation.
 */
class ArmorInfo {
 public:
   ArmorInfo()
   {        
  }
  ArmorInfo(cv::RotatedRect armor_rect, std::vector<cv::Point2f> armor_vertex, bool armor_single,
	    int armor_id, cv::Rect armor_tracking_rect,int armor_num_threshold, float armor_stddev = 0.0) 
  {
    rect = armor_rect;
    vertex = armor_vertex;
    stddev = armor_stddev;
    single = armor_single;
    id = armor_id;
    tracking_rect = armor_tracking_rect;
    num_threshold = armor_num_threshold;
  }
  inline cv::Mat get_ml_roi() const { return ml_roi_; }
  inline void set_ml_roi(cv::Mat &ml_roi) { ml_roi.copyTo(ml_roi_); }
  inline cv::Mat get_ml_hog() const { return ml_hog_; }
  inline void set_ml_hog(cv::Mat &ml_hog) { ml_hog.copyTo(ml_hog_); }
 public:
  cv::RotatedRect rect;
  std::vector<cv::Point2f> vertex;
  float stddev;
  bool single;
  int id;
  cv::Rect tracking_rect;
  cv::Mat ml_roi_;
  cv::Mat ml_hog_;
  int num_threshold;
};

/**
 * @brief This class achieved functions that can help to detect armors of RoboMaster vehicle.
 */
class ConstraintSet : public ArmorDetectionBase
{
 public:
  ConstraintSet(std::shared_ptr<CVToolbox> cv_toolbox);
  void LoadParam() override;
  ErrorInfo DetectArmor(cv::Mat &result_img, bool &detected, cv::Point3f &target_3d, std_msgs::Float64MultiArray &msg1, float pitch_now, float yaw_now, float &yaw_output, float &pitch_output, bool &firecommand, float &distance_out,int &armor_id) override;
  ErrorInfo DetectMarkers(cv::Mat &result_img, bool &detected, cv::Point3f &target_3d,float &dex) override;
  ArmorInfo SlectFinalArmor(std::vector<ArmorInfo> &armors);
  void DetectLights(const cv::Mat &src_tmp, std::vector<cv::RotatedRect> &lights);
  void FilterLights(std::vector<cv::RotatedRect> &lights);
  void PossibleArmors(const std::vector<cv::RotatedRect> &lights, std::vector<ArmorInfo> &armors);
  void FilterArmors(std::vector<ArmorInfo> &armors);
  void CalcControlInfo(const ArmorInfo & armor, cv::Point3f &target_3d, float pitch_now, float yaw_now, float &yaw_output, float &pitch_output, float &absolute_pitch_, float &absolute_yaw_, Target &output_target, float &distance_out);
  void CalcArmorInfo(std::vector<cv::Point2f> &armor_points, cv::RotatedRect left_light, cv::RotatedRect right_light);
  void SolveArmorCoordinate(const float width, const float height);
  void SignalFilter(double &new_num, double &old_num,unsigned int &filter_count, double max_diff);
  void SetThreadState(bool thread_state) override;
  
  /************************************[ADD]********************************************/
  void	sortLights(std::vector<cv::RotatedRect> &lights);
  bool	rectSafety(cv::Rect &brect, int rows, int cols);                                                                                    //
  bool	rectSafety_1(cv::Rect &brect,cv::Size size);
  void	roiFilter(Mat &input, cv::Point &offset);
  float	getDistance(cv::Point pointO, cv::Point point1);  
  bool	getEligibility(const cv::RotatedRect &left_lightbar, const cv::RotatedRect &right_lightbar);  
  void  getCorrectPointOder(const cv::RotatedRect &rotatedRect ,cv::Point2f &bl,cv::Point2f &tl,cv::Point2f &tr,cv::Point2f &br);    
  void  MLHogSVM(cv::Mat &src,cv::RotatedRect search_rotatedrect,const int enemy_color_,int single);
  int   MLHogSVM(cv::Mat &roi_image);
  void  setMlRoi(ArmorInfo &armor,const cv::Mat &src);
  float  fastTargetLock1(cv::Mat input,cv::Mat mask); 
  bool  fastTargetLock(ArmorInfo &armor,ArmorInfo &last_armor);
  float getSimilarity(const cv::Mat& first,const cv::Mat& second);
  bool  regionOtsuThreshold(const Mat &inputimage, Mat &outputimage, int &thre, bool lr);
  void  gammaTransform(cv::Mat &src, cv::Mat &dist);
  bool  JudgeColor(cv::RotatedRect single_light,Mat &src);
  bool  Get(const ArmorInfo & armor,float yaw, float pitch,  float absolute_pitch_, float absolute_yaw_, bool &firecommand);
  bool  rotatingCalipers( const Point2f* points, int n, float* out, float angle , float deviation);
  bool  NoLightBar_Detect(Mat &src, std::vector<ArmorInfo> &armors);
  void  CoordinateTransformation(float &disX, float &disY, float &disZ, float target_distance, int armor_rect_center_x, float yaw_now, float pitch_now, float &absolute_pitch_, float &absolute_yaw_);
  double PnpAssess(Mat matrix,Mat dist_coeffs,Mat rot_matrix,Mat tran_vector,vector<Point3f> worldPoint,vector<Point2f> imagePoint);
  inline void      setLastArmor(ArmorInfo last_armor);
  inline ArmorInfo getLastArmor()const;
  cv::RotatedRect  getSingleRotatedRect(const cv::RotatedRect &lightBar,bool right_or_left,cv::Point2f &bl,cv::Point2f &tl,cv::Point2f &tr,cv::Point2f &br, const float light0_width, const float light0_height, const float light0_angle_0);
  cv::RotatedRect  Chao_minAreaRect( InputArray _points, float angle, float deviation );
  cv::Rect   	   rectCenterScale(cv::Rect rect, cv::Size2f size); 
 //ErrorInfo DetectMarkers(cv::Mat &result_img,bool &detected,cv::Point3f &target_3d) override;
  void  DetectionVisionMarks(const cv::Mat &src_,cv::Point3f &target_3d,bool &detected,float &dex);
  void readImg(std::vector<cv::Mat> &pattern_imgs);
  void color_Thead(const cv::Mat &src_);
  //void PredictControlInfo(const ArmorInfo & armor, cv::Point3f &target_3d, float pitch_now, float yaw_now, float &yaw_output, float &pitch_output, float &absolute_pitch_, float &absolute_yaw_, struct timeval time);
  /************************************[END]********************************************/
  ~ConstraintSet() final;
  
 private:
  ErrorInfo error_info_;
  unsigned int filter_x_count_;
  unsigned int filter_y_count_;
  unsigned int filter_z_count_;
  unsigned int filter_distance_count_;
  unsigned int filter_pitch_count_;
  unsigned int filter_yaw_count_;

  cv::Mat src_img_;
  cv::Mat src_img_1;
  cv::Mat gray_img_;
  //!  Camera intrinsic matrix
  cv::Mat intrinsic_matrix_;
  //! Camera distortion Coefficient
  cv::Mat distortion_coeffs_;
  //! Read image index
  int read_index_;
  //! detection time
  double detection_time_;

  // Parameters come form .prototxt file
  bool enable_debug_;
  bool using_hsv_;
  unsigned int enemy_color_;

  //! Use for debug
  cv::Mat show_lights_before_filter_;
  cv::Mat show_lights_after_filter_;
  cv::Mat show_armors_befor_filter_;
  cv::Mat show_armors_after_filter_;
  cv::Mat show_marker_before_filter_;

  //! armor info
  std::vector<cv::Point3f> armor_points_;

  //! Filter lights
  std::vector<LightInfo> lights_info_;
  float light_max_aspect_ratio_;
  float light_min_area_;
  float light_max_angle_;
  float light_max_angle_diff_;

  //! Filter armor
  float armor_max_angle_;
  float armor_min_area_;
  float armor_max_aspect_ratio_;
  float armor_max_pixel_val_;
  float armor_max_mean_;
  float armor_max_stddev_;

  float color_thread_;
  float blue_thread_;
  float red_thread_;

  bool thread_running_;

  //ros
  ros::NodeHandle nh;

  const float g=9.8;
	
  int armor_id;
	
  std_msgs::Float64MultiArray msg;
	
  bool svmresult;
	
  std_msgs::Float64MultiArray rev_msg;

  float pnp_assess;
  const float kPi = 3.141592653;

  cv::Rect roi_rect_;
  cv::Point offset_;
 
  Predictor predictor;
  ArmorInfo last_armor_;

};

roborts_common::REGISTER_ALGORITHM(ArmorDetectionBase, "constraint_set", ConstraintSet, std::shared_ptr<CVToolbox>);

} //namespace roborts_detection


#endif // AOTO_PILOT_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_H
