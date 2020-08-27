/****************************************************************************
 *@brief		
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

#ifndef ROBORTS_DETECTION_ARMOR_DETECTION_BASE_H
#define ROBORTS_DETECTION_ARMOR_DETECTION_BASE_H

#include <vector>
#include "state/error_code.h"
#include "../util/cv_toolbox.h"

namespace roborts_detection {

using roborts_common::ErrorInfo;

class ArmorDetectionBase {
 public:
  ArmorDetectionBase(std::shared_ptr<CVToolbox> cv_toolbox) : cv_toolbox_(cv_toolbox)
  {  };
  virtual void LoadParam() = 0;
  virtual ErrorInfo DetectArmor(cv::Mat &result_img, bool &detected, cv::Point3f &target_3d,std_msgs::Float64MultiArray &msg1, float pitch_now, float yaw_now, float &yaw_output, float &pitch_output, bool &firecommand, float &distance,int &armor_id) = 0;
  virtual ErrorInfo DetectMarkers(cv::Mat &result_img, bool &detected, cv::Point3f &target_3d,float &dex) = 0;
  virtual void SetThreadState(bool thread_state) = 0;
  virtual ~ArmorDetectionBase() = default;
 protected:
  std::shared_ptr<CVToolbox> cv_toolbox_;
};
} //namespace roborts_detection

#endif //ROBORTS_DETECTION_ARMOR_DETECTION_BASE_H