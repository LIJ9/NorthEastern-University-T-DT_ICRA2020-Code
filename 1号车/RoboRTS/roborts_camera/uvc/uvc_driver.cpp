/****************************************************************************
 *@brief	constraint_set.cpp
 *@version	1.0.0.0
 *@author	Way
 * 
 *@Change History:
 * -------------------------------------------------------------------------
 * <Date>	|<version>	|<Author>	|<Description>
 * -------------------------------------------------------------------------
 * 2019/10/22   |1.0.0.1       |Way		|
 *-------------------------------------------------------------------------- 
 *
 ***************************************************************************/

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <linux/videodev2.h>
#include <sys/ioctl.h>

#include "uvc_driver.h"

namespace roborts_camera {
UVCDriver::UVCDriver(CameraInfo camera_info):
    CameraBase(camera_info){
}

void UVCDriver::StartReadCamera(cv::Mat &img,int index) {
  if(!camera_initialized_){
    camera_info_.cap_handle.open(camera_info_.camera_path);
    SetCameraExposure(camera_info_.camera_path, camera_info_.exposure_value);
    ROS_ASSERT_MSG(camera_info_.cap_handle.isOpened(), "Cannot open %s .", cameras_[camera_num].video_path.c_str());
    camera_initialized_ = true;
  }
  else {
    camera_info_.cap_handle >> img;
  }
}

void UVCDriver::StopReadCamera() {
  //TODO: To be implemented
}

void UVCDriver::SetCameraExposure(std::string id, int val)
{
  int cam_fd;
  if ((cam_fd = open(id.c_str(), O_RDWR)) == -1) {
    std::cerr << "Camera open error" << std::endl;
  }

  struct v4l2_control control_s;
  control_s.id = V4L2_CID_AUTO_WHITE_BALANCE;
  control_s.value = camera_info_.auto_white_balance;
  ioctl(cam_fd, VIDIOC_S_CTRL, &control_s);

  control_s.id = V4L2_CID_EXPOSURE_AUTO;
  control_s.value = V4L2_EXPOSURE_MANUAL;
  ioctl(cam_fd, VIDIOC_S_CTRL, &control_s);

  // Set exposure value
  control_s.id = V4L2_CID_EXPOSURE_ABSOLUTE;
  control_s.value = val;
  ioctl(cam_fd, VIDIOC_S_CTRL, &control_s);
  close(cam_fd);
}

UVCDriver::~UVCDriver() {
}

} //namespace roborts_camera
