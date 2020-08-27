#include "MvCameraControl.h"
#include"iostream"



bool setBalance(void *handle, unsigned int value);  //自动白平衡
bool setExposure(void *handle, unsigned int value); //自动曝光
bool setBrightness(void *handle, unsigned int value); //图像亮度
bool setFrameRate(void *handle, float value); //采集帧率
bool setGain(void *handle, float value);  //增益值
bool setGainMode(void *handle, unsigned int value); //增益模式
bool SetExposureTime(void *handle,const float nValue);

bool setBalance(void *handle, unsigned int value){
  /*
    value = 0 关闭
    value = 1 连续
    value = 2 一次
  */
  int nRet;
  nRet = MV_CC_SetBalanceWhiteAuto(handle, value);
  if(nRet != 0){
    return false;
  }
  return true;
}                                                                                                                                                                                                                                                                                                                                    

bool setExposure(void *handle, unsigned int value){
  /*
    value = 0 关闭
    value = 1 一次
    value = 2 连续
  */
  int nRet;
  nRet = MV_CC_SetExposureAutoMode(handle, value);
  if(nRet != 0){
    return false;
  }
  return true;
}



bool SetExposureTime(void *handle,const float nValue){
    int nRet;
    nRet = MV_CC_SetExposureTime(handle, nValue);
    if (MV_OK != nRet)
    {
        printf("error: SetExposureTime fail [%x]\n", nRet);
        return false;
    }
    return true;
}

bool setBrightness(void *handle, unsigned int value){
  int nRet;
  nRet = MV_CC_SetBrightness(handle, value);
  if(nRet != 0){
    return false;
  }
  return true;
}

bool setFrameRate(void *handle, float value){
  int nRet;
  nRet = MV_CC_SetFrameRate(handle, value);
  if(nRet != 0){
    return false;
  }
  return true;
}

bool setGain(void *handle, float value){
  int nRet;
  nRet = MV_CC_SetGain(handle, value);
  if(nRet != 0){
    return false;
  }
  return true;
}

bool setGainMode(void *handle, unsigned int value){
  /*
    value = 0 关闭
    value = 1 一次
    value = 2 连续
  */
  int nRet;
  nRet = MV_CC_SetGainMode(handle, value);
  if(nRet != 0){
    return false;
  }
  return true;
}
