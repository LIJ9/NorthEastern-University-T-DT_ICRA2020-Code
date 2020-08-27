/****************************************************************************
 *@brief	constraint_set.cpp
 *@version	1.0.0.0
 *@author	Way
 * 
 *@Change History:
 * -------------------------------------------------------------------------
 * <Date>	|<version>	|<Author>	|<Description>
 * -------------------------------------------------------------------------
 * 2019/10/22   |1.0.0.1       |Way		|修改代码  驱动双相机
 *-------------------------------------------------------------------------- 
 *
 ***************************************************************************/


#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <linux/videodev2.h>
#include <sys/ioctl.h>

#include "HKVision_driver.h"

namespace roborts_camera {
HKVisionDriver::HKVisionDriver(CameraInfo camera_info):CameraBase(camera_info)
{
}

void HKVisionDriver::StartReadCamera(cv::Mat &img,int index) 
{
  if(!camera_initialized_)
  {
    InitCam(index);   
    set_format(camera_info_.fps,640,480);
    //set_format(camera_info_.fps,1440,1080);
    StartGrabbing();
    SetCameraExposure(camera_info_.camera_path, camera_info_.exposure_value);
    camera_initialized_ = true;
  }else {
    SetCameraExposure(camera_info_.camera_path, camera_info_.exposure_value);   
    n_ret_ = MV_CC_GetImageForBGR(handle_, p_data_, n_data_size_, &st_img_info_, 1000);

    cv::Mat frame(st_img_info_.nHeight, st_img_info_.nWidth, CV_8UC3, p_data_);//读入图片到frame
    img=frame;//赋给Camera_Output，注意img和frame共享一个矩阵
  }
}

void HKVisionDriver::StopReadCamera() 
{
  //TODO: To be implemented
}

void HKVisionDriver::SetCameraExposure(std::string id, int val)
{
  set_exposure(camera_info_.auto_exposure, val);
}
bool HKVisionDriver::InitCam(int nDeviceIndex)
{
    MV_CC_DEVICE_INFO_LIST m_stDevList = {0};
    n_ret_ = MV_CC_EnumDevices(MV_USB_DEVICE, &m_stDevList);//枚举设备
    if (MV_OK != n_ret_){
        printf("MV_CC_EnumDevices fail [%x]\n", n_ret_);
    }
    if (m_stDevList.nDeviceNum == 0){
        printf("no camera found!!!!!!!!!!!!!!!!!!!!\n");
    }
    //-------------------------------[ADD]-----------------------------------//
    std::cout<<"nDeviceNum="<<m_stDevList.nDeviceNum<<std::endl;
    //-------------------------------[END]-----------------------------------// 
    //printf("%d\n", m_stDevList.nDeviceNum);
    //选择查找到的一台在线设备，创建设备句柄  
    MV_CC_DEVICE_INFO m_stDevInfo = {0};
    memcpy(&m_stDevInfo, m_stDevList.pDeviceInfo[nDeviceIndex], sizeof(MV_CC_DEVICE_INFO));
    n_ret_ = MV_CC_CreateHandle(&handle_, &m_stDevInfo);
    if (MV_OK != n_ret_){
        printf("MV_CC_CreateHandle_ fail [%x]\n", n_ret_);
    }
    cam_index_ = nDeviceIndex;
    cam_guid_ = std::string((char*)m_stDevInfo.SpecialInfo.stUsb3VInfo.chDeviceGUID);
    //连接设备
    unsigned int nAccessMode = MV_ACCESS_Exclusive;
    unsigned short nSwitchoverKey = 0;
    n_ret_ = MV_CC_OpenDevice(handle_, nAccessMode, nSwitchoverKey);
    if (MV_OK != n_ret_){
        printf("MV_CC_OpenDevice fail [%x]\n", n_ret_);
    }
}

bool HKVisionDriver::set_format(unsigned int fps, unsigned int width,unsigned int height, int pixelformat)
{
    n_ret_ = MV_CC_SetPixelFormat(handle_, pixelformat);
    if (MV_OK != n_ret_){
        printf("MV_CC_SetPixelFormat fail! n_ret_ [%x]\n", n_ret_);
        return false;
    }
    n_ret_ = MV_CC_SetWidth(handle_,width);
    if (MV_OK != n_ret_){
        printf("MV_CC_SetWidth fail! n_ret_ [%x]\n", n_ret_);
        return false;
    }

    n_ret_ = MV_CC_SetHeight(handle_,height);
    if (MV_OK != n_ret_){
        printf("MV_CC_SetHeight fail! n_ret_ [%x]\n", n_ret_);
        return false;
    }

    n_ret_ = MV_CC_SetFrameRate(handle_,fps);
    if (MV_OK != n_ret_){
        printf("MV_CC_SetFrameRate fail! n_ret_ [%x]\n", n_ret_);
        return false;
    }
    return true;
}

bool HKVisionDriver::StartGrabbing(){
    n_ret_ = MV_CC_StartGrabbing(handle_);
    if (MV_OK != n_ret_){
        printf("MV_CC_StartGrabbing fail! n_ret_ [%x]\n", n_ret_);
        MV_CC_StopGrabbing(handle_);
        MV_CC_DestroyHandle(handle_);
        return false;
    }
    MVCC_INTVALUE stIntvalue = {0};
    n_ret_ = MV_CC_GetIntValue(handle_, "PayloadSize", &stIntvalue);
    if (n_ret_ != MV_OK){
        printf("MV_CC_GetIntValue failed! n_ret_ [%x]\n", n_ret_);
        return false;
    }
    n_data_size_ = stIntvalue.nCurValue + 2048; //一帧数据大小+预留字节(用于SDK内部处理)

    p_data_ =  (unsigned char*)malloc(n_data_size_);
    memset(&st_img_info_, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    return true;
}

bool HKVisionDriver::CloseGrabbing()
{
    n_ret_ = MV_CC_StopGrabbing(handle_);
    if (MV_OK != n_ret_){
        printf("MV_CC_StopGrabbing fail! n_ret_ [%x]\n", n_ret_);
        return -1;
    }
    return true;
}

bool HKVisionDriver::RestartCam()
{
    n_ret_ = MV_CC_DestroyHandle(handle_);
    if (MV_OK != n_ret_){
        printf("MV_CC_DestroyHandle_ fail! n_ret_ [%x]\n", n_ret_);
    }

    MV_CC_DEVICE_INFO_LIST m_stDevList = {0};
    n_ret_ = MV_CC_EnumDevices(MV_USB_DEVICE, &m_stDevList);
    if (MV_OK != n_ret_){
        printf("MV_CC_EnumDevices fail [%x]\n", n_ret_);
    }

    int i = 0;
    if (m_stDevList.nDeviceNum == 0){
        printf("no camera found!?????????????????\n");
    }

    //选择查找到的一台在线设备，创建设备句柄
    MV_CC_DEVICE_INFO m_stDevInfo = {0};
    memcpy(&m_stDevInfo, m_stDevList.pDeviceInfo[cam_index_], sizeof(MV_CC_DEVICE_INFO));

    n_ret_ = MV_CC_CreateHandle(&handle_, &m_stDevInfo);

    if (MV_OK != n_ret_){
        printf("MV_CC_CreateHandle_ fail [%x]\n", n_ret_);
    }

    //连接设备
    unsigned int nAccessMode = MV_ACCESS_Exclusive;
    unsigned short nSwitchoverKey = 0;

    n_ret_ = MV_CC_OpenDevice(handle_, nAccessMode, nSwitchoverKey);
    if (MV_OK != n_ret_){
        printf("MV_CC_OpenDevice fail [%x]\n", n_ret_);
    }
    StartGrabbing();
    return true;
}

bool HKVisionDriver::get_mat(cv::Mat &img){
    n_ret_ = MV_CC_GetImageForBGR(handle_, p_data_, n_data_size_, &st_img_info_, 1000);
    cv::Mat frame(st_img_info_.nHeight, st_img_info_.nWidth, CV_8UC3, p_data_);//读入图片到frame
    img=frame;//赋给Camera_Output，注意img和frame共享一个矩阵
    /*if (save_res_ == 1){
        outputVideo << img;
    }*/
    return true;
}
bool HKVisionDriver::set_exposure(bool auto_exp, int t){
  if (auto_exp){
        n_ret_ = MV_CC_SetEnumValue(handle_,"ExposureAuto",MV_EXPOSURE_AUTO_MODE_CONTINUOUS);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetExposureAutoMode fail! n_ret_ [%x]\n", n_ret_);
            MVCC_ENUMVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetEnumValue(handle_,"ExposureAuto",&CurrentValue);
            printf("current value: %d\n",CurrentValue.nCurValue);
            return -1;
        }
    }
    else{
        n_ret_ = MV_CC_SetEnumValue(handle_,"ExposureAuto",MV_EXPOSURE_AUTO_MODE_OFF);
        if (MV_OK != n_ret_){
	    //std::cout<<"1111!!!!!!!!!!"<<std::endl;
            //printf("MV_CC_SetExposureAutoMode fail! n_ret_ [%x]\n", n_ret_);
            MVCC_ENUMVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetEnumValue(handle_,"ExposureAuto",&CurrentValue);
          //  printf("current value: %d\n",CurrentValue.nCurValue);
            return -1;
        }

        n_ret_ = MV_CC_SetFloatValue(handle_,"ExposureTime",t);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetExposureTime fail! n_ret_ [%x]\n", n_ret_);
            MVCC_FLOATVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetFloatValue(handle_,"ExposureTime",&CurrentValue);
            printf("current value: %f\n",CurrentValue.fCurValue);
            return -1;
        }
    }
    return true;
}


bool HKVisionDriver::DoExposure(){
    n_ret_ = MV_CC_SetExposureAutoMode(handle_,MV_EXPOSURE_AUTO_MODE_ONCE);
    if (MV_OK != n_ret_){
        printf("MV_CC_SetExposureAutoMode fail! n_ret_ [%x]\n", n_ret_);
        return -1;
    }
    return true;
}



bool HKVisionDriver::set_gain(bool auto_gain,int val){
    if (auto_gain){
        n_ret_ = MV_CC_SetGainMode(handle_,MV_GAIN_MODE_CONTINUOUS);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetGainMode fail! n_ret_ [%x]\n", n_ret_);
            return -1;
        }
    }
    else{
        n_ret_ = MV_CC_SetGainMode(handle_,MV_GAIN_MODE_OFF);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetGainMode fail! n_ret_ [%x]\n", n_ret_);
            return -1;
        }

        n_ret_ = MV_CC_SetGain(handle_, val);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetGain fail! n_ret_ [%x]\n", n_ret_);
            return -1;
        }
    }
    return true;
}



bool HKVisionDriver::set_brightness(int val){
    n_ret_ = MV_CC_SetBrightness(handle_,val);
    if (MV_OK != n_ret_){
        printf("MV_CC_SetBrightness fail! n_ret_ [%x]\n", n_ret_);
        return -1;
    }

    return true;
}



bool HKVisionDriver::set_white_balance(bool auto_whitebalance, int r,int g,int b){
    if (auto_whitebalance){
        n_ret_ = MV_CC_SetBalanceWhiteAuto(handle_,MV_BALANCEWHITE_AUTO_CONTINUOUS);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetBalanceWhiteAuto fail! n_ret_ [%x]\n", n_ret_);
            return -1;
        }
    }
    else{
        n_ret_ = MV_CC_SetBalanceWhiteAuto(handle_,MV_BALANCEWHITE_AUTO_OFF);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetBalanceWhiteAuto fail! n_ret_ [%x]\n", n_ret_);
            return -1;
        }

        n_ret_ = MV_CC_SetBalanceRatioRed(handle_,r);
        if (MV_OK != n_ret_) {
            printf("MV_CC_SetBalanceRatioRed fail! n_ret_ [%x]\n", n_ret_);
            return -1;
        }

        n_ret_ = MV_CC_SetBalanceRatioGreen(handle_,g);
        if (MV_OK != n_ret_) {
            printf("MV_CC_SetBalanceRatioGreen fail! n_ret_ [%x]\n", n_ret_);
            return -1;
        }

        n_ret_ = MV_CC_SetBalanceRatioBlue(handle_,b);
        if (MV_OK != n_ret_) {
            printf("MV_CC_SetBalanceRatioBlue fail! n_ret_ [%x]\n", n_ret_);
            return -1;
        }
    }
    return true;
}



bool HKVisionDriver::DoWhiteBalance(){
    n_ret_ = MV_CC_SetBalanceWhiteAuto(handle_,MV_BALANCEWHITE_AUTO_ONCE);
    if (MV_OK != n_ret_){
        printf("MV_CC_SetBalanceWhiteAuto fail! n_ret_ [%x]\n", n_ret_);
        return -1;
    }
    return true;
}



bool HKVisionDriver::set_hue(int val){
    n_ret_ = MV_CC_SetHue(handle_,val);
    if (MV_OK != n_ret_){
        printf("MV_CC_SetHue fail! n_ret_ [%x]\n", n_ret_);
        return -1;
    }
    return true;
}



bool HKVisionDriver::set_saturation(int val){
    n_ret_ = MV_CC_SetSaturation(handle_,val);
    if (MV_OK != n_ret_){
        printf("MV_CC_SetSaturation fail! n_ret_ [%x]\n", n_ret_);
        return -1;
    }
    return true;
}



bool HKVisionDriver::set_gamma(int selector,int val){
n_ret_ = MV_CC_SetGammaSelector(handle_,MV_GAMMA_SELECTOR_USER);
if (MV_OK != n_ret_){
    printf("MV_CC_SetGammaSelector fail! n_ret_ [%x]\n", n_ret_);
    return -1;
}

n_ret_ = MV_CC_SetGamma(handle_,val);
if (MV_OK != n_ret_) {
    printf("MV_CC_SetGamma fail! n_ret_ [%x]\n", n_ret_);
    return -1;
}

if (selector == MV_GAMMA_SELECTOR_SRGB){
    n_ret_ = MV_CC_SetGammaSelector(handle_,MV_GAMMA_SELECTOR_SRGB);
    if (MV_OK != n_ret_){
        printf("MV_CC_SetGammaSelector fail! n_ret_ [%x]\n", n_ret_);
        return -1;
    }
}
return true;
}



bool HKVisionDriver::set_sharpness(int val){
    n_ret_ = MV_CC_SetSharpness(handle_,val);
    if (MV_OK != n_ret_){
        printf("MV_CC_SetSharpness fail! n_ret_ [%x]\n", n_ret_);
        return -1;
    }
    return true;
}



std::string HKVisionDriver::get_camguid(){
    return cam_guid_;
}

HKVisionDriver::~HKVisionDriver() {
  CloseGrabbing();
    n_ret_ = MV_CC_DestroyHandle(handle_);
    if (MV_OK != n_ret_){
        printf("MV_CC_DestroyHandle_ fail! n_ret_ [%x]\n", n_ret_);
    }
}
} //namespace roborts_camera
