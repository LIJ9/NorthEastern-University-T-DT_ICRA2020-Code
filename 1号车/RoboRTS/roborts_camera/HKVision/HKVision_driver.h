/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#include "ImageProcess/MvCameraControl.h"
#include "ImageProcess/MvGigEDevice.h"

#ifndef ROBORTS_CAMERA_HKVISION_DRIVER_H
#define ROBORTS_CAMERA_HKVISION_DRIVER_H



#include <thread>

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "actionlib/server/simple_action_server.h"



#include "../camera_param.h"
#include "../camera_base.h"
#include "alg_factory/algorithm_factory.h"
#include "io/io.h"





namespace roborts_camera {
  /**
 * @brief HKVision Camera class, product of the camera factory inherited from CameraBase
 */
class HKVisionDriver: public CameraBase {
  public:
  /**
   * @brief Constructor of HKVisionDriver
   * @param camera_info  Information and parameters of camera
   */
  explicit HKVisionDriver(CameraInfo camera_info);
  /**
   * @brief Start to read HKVision camera
   * @param img Image data in form of cv::Mat to be read
   */
  void StartReadCamera(cv::Mat &img,int index) override;
  /**
   * @brief Stop to read HKVision camera
   */
  void StopReadCamera();
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool RestartCam();
    bool get_mat(cv::Mat &img);    //获取Opencv Mat图像

    bool set_exposure(bool auto_exp, int t);    //设置曝光
    bool DoExposure();    //单帧自动曝光
    bool set_gain(bool auto_gain,int val);    //设置增益
    bool set_brightness(int val);    //设置亮度

    bool set_white_balance(bool auto_whitebalance, int r,int g,int b);    //设置白平衡
    bool DoWhiteBalance();    //单帧自动白平衡
    bool set_hue(int val);    //设置色调
    bool set_saturation(int val);    //设置饱和度

    bool set_gamma(int selector = MV_GAMMA_SELECTOR_SRGB,int val = 0);    //设置伽玛
    bool set_sharpness(int val);    //设置锐度
    std::string get_camguid();

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  
  ~HKVisionDriver() override;
  
  private:
  /**
   * @brief Set camera exposure
   * @param id Camera path
   * @param val Camera exposure value
   */
  void SetCameraExposure(std::string id, int val);
  //! Initialization of camera read
  bool read_camera_initialized_;
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   bool InitCam(int nDeviceIndex);
    bool set_format(unsigned int fps,unsigned int width,unsigned int height,int pixelformat = PixelType_Gvsp_RGB8_Packed);
    bool StartGrabbing();
    bool CloseGrabbing();
    
    
    
    void* handle_ = NULL;
    int n_ret_ = -1;
    unsigned char * p_data_ = NULL;
    int n_data_size_ = 0;
    MV_FRAME_OUT_INFO_EX st_img_info_ = {0};
    int cam_index_ = -1;
    std::string cam_guid_;
    cv::VideoWriter outputVideo;
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
};
roborts_common::REGISTER_ALGORITHM(CameraBase, "HKVision", HKVisionDriver, CameraInfo);

} //namespace roborts_camera
#endif //ROBORTS_CAMERA_HKVision_DRIVER_H