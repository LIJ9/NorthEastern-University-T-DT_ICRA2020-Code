/****************************************************************************
 *@brief		云台控制头文件
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
#ifndef ROBORTS_DETECTION_GIMBAL_CONTROL_H
#define ROBORTS_DETECTION_GIMBAL_CONTROL_H
#include <iostream>
#include <opencv2/opencv.hpp>

namespace roborts_detection{

const double PI = 3.1415926535;
const float GRAVITY = 9.78;


/**
 * @brief The class can make a transformation: the 3D position of enemy -->  pitch,yaw angle of gimbal.
 * For more derails, see projectile_model.pdf
 * TODO: add enemy motion estimation
 */

class GimbalContrl
{
 private:
  /**
   * @brief Calculate the actual y value with air resistance
   * @param x the distanc
   * @param v Projectile velocity
   * @param angle Pitch angle
   * @return The actual y value in the gimbal coordinate
   */
  float BulletModel(float x,float v,float angle);
  /**
   * @brief Get the gimbal control angle
   * @param x Distance from enemy(the armor selected to shoot) to gimbal
   * @param y Value of y in gimbal coordinate.
   * @param v Projectile velocity
   * @return Gimbal pitch angle
   */
  float GetPitch(float x,float y,float v);

 public:
  /**
   * @brief Init the Transformation matrix from camera to gimbal //TODO: write in ros tf
   * @param x Translate x
   * @param y Translate y
   * @param z Translate z
   * @param pitch Rotate pitch
   * @param yaw Rotate yaw
   */
  void Init(float x,float y,float z,float pitch,float yaw, float init_v, float init_k);
  /**
   * @brief Get the gimbal control info.
   * @param postion Enemy position(actually it should be the target armor).
   * @param pitch Input and output actual pitch angle
   * @param yaw Input and output actual yaw angle
   */
  void Transform(cv::Point3f &postion,float &pitch,float &yaw);

 private:
  //! Transformation matrix between camera coordinate system and gimbal coordinate system.
  //! Translation unit: cm
  cv::Point3f offset_;
  //! Rotation matrix unit: degree
  float offset_pitch_;
  float offset_yaw_;

  //! Initial value
  float init_v_;
  float init_k_;

};

} //namespace roborts_detection

#endif //ROBORTS_DETECTION_GIMBAL_CONTROL_H
