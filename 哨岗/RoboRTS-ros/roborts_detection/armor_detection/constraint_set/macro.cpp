/*****************************************************************************
 * @brief        工程的宏定义声明
 * @version      1.0.0.1
 *
 * @author
 * @qq
 *
 *----------------------------------------------------------------------------
 * Change History :
 * <Date>     | <Version> | <Author> | <Description>
 *----------------------------------------------------------------------------
 * 2019/02/20 | 1.0.0.1   | 黎容熙   | 代码规范
 *----------------------------------------------------------------------------
 * 2019/11/26 | 1.0.0.2   | Way      |添加类LightBar
 * ---------------------------------------------------------------------------
 *
*****************************************************************************/

#include "macro.h"
#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/ml.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/stitching/detail/warpers_inl.hpp>
/**********************************
 * @函数名 minAreaRect
 * @作者
 * @联系方式
 * @简介
 * @传入参数
 * @传出参数 vector<Points>,float angle,float deiation
 **********************************/


Target::Target(float x,float y,float z,float height,float width,float angle_x,float angle_y,float angle_z,int image_x,int image_y,float distance,float pnp_assess) {
    x_=x;
    y_=y;
    z_=z;
    height_=height;
    width_=width;
    angle_x_=angle_x;
    angle_y_=angle_y;
    angle_z_=angle_z;
    image_center_x_=image_x;
    image_center_y_=image_y;
    distance_=distance;
    empty_=0;
    pnp_assess_=pnp_assess;
}
Target::Target(float x,float y,float z,float height,float width,float angle_x,float angle_y,float angle_z,int image_x,int image_y,float distance) {
    x_=x;
    y_=y;
    z_=z;
    height_=height;
    width_=width;
    angle_x_=angle_x;
    angle_y_=angle_y;
    angle_z_=angle_z;
    image_center_x_=image_x;
    image_center_y_=image_y;
    distance_=distance;
    empty_=0;
}

/*Target::Target(float x,float y,float z,float height,float width,float angle_x,float angle_y,float angle_z,int image_x,int image_y,float distance,float pnp_assess){
    x_=x;
    y_=y;
    z_=z;
    height_=height;
    width_=width;
    angle_x_=angle_x;
    angle_y_=angle_y;
    angle_z_=angle_z;
    image_center_x_=image_x;
    image_center_y_=image_y;
    distance_=distance;
    empty_=0;
    pnp_assess_=pnp_assess;
}*/

Target::Target(){
    empty_=1;
}




LightBar::LightBar(RotatedRect rotrect)
{
  rotated_rect_=rotrect;
  width_=static_cast<int>(rotated_rect_.size.width);
  height_=static_cast<int>(rotated_rect_.size.height);
  ratio_=width_/(height_+0.1); 
}
LightBar::~LightBar()=default;



/**************************************[VisionMarkers类]**********************************
 * @ brief:视觉标签成员函数实现
 * *************************************************************************************/
VisionMarkers::VisionMarkers(RotatedRect rotrect, Color color) 
{
  empty_=false;
  color_=color;
  rotated_rect_=rotrect;
  center_ = rotated_rect_.center;
  area_=static_cast<int>(rotated_rect_.size.area());
  if(fabs(rotated_rect_.angle)<45){
      angle_ = rotated_rect_.angle;
      width_ = static_cast<int>(rotated_rect_.size.width);
      height_ = static_cast<int>(rotated_rect_.size.height);
      center_ = rotated_rect_.center;
      Point2f vertices[4];
      rotated_rect_.points(vertices);
      bl_ = vertices[0];
      tl_ = vertices[1];
      tr_ = vertices[2];
      br_ = vertices[3];
  }else{
      angle_ = 90 + rotated_rect_.angle;
      width_ = static_cast<int>(rotated_rect_.size.height);
      height_ = static_cast<int>(rotated_rect_.size.width);
      Point2f vertices[4];
      rotated_rect_.points(vertices);
      center_ = rotated_rect_.center;
      br_ = vertices[0];
      bl_ = vertices[1];
      tl_ = vertices[2];
      tr_ = vertices[3];
  }
  top_point_=(tl_+tr_)/2;
  bottom_point_=(bl_+br_)/2;
  ratio_width_to_height_=width_/height_;
  judge_search_=0;
  bounding_rect_ = rotated_rect_.boundingRect();
}
VisionMarkers::VisionMarkers() 
{
  empty_=true;
  angle_ = 0;
  width_ = 0;
  height_ = 0;
  judge_search_=0;
  area_=0;
  angle_=0;
  center_=Point(0,0);
  tl_=Point2f(0,0);
  tr_=Point2f(0,0);
  bl_=Point2f(0,0);
  br_=Point2f(0,0);

  top_point_=Point2f(0,0);//灯条上端点
  bottom_point_=Point2f(0,0);//灯条下端点
  bounding_rect_=Rect(Point(0,0),Point(0,0));
  rotated_rect_=RotatedRect(Point2f(0,0),Size2f(0,0),0);
  color_=UNKNOW;
  judge_search_=0;
  ratio_width_to_height_=0;

}

VisionMarkers::~VisionMarkers() = default;
void VisionMarkers::operator=(const VisionMarkers &a)
{
  //contour_=a.contour_;
  area_=a.area_;
  angle_=a.angle_;
  width_=a.width_;
  height_=a.height_;
  center_=a.center_;
  tl_=a.tl_;
  tr_=a.tr_;
  bl_=a.bl_;
  br_=a.br_;
  top_point_=a.top_point_;//灯条上端点
  bottom_point_=a.bottom_point_;//灯条下端点
  bounding_rect_=a.bounding_rect_;
  rotated_rect_=a.rotated_rect_;
  color_=a.color_;
  judge_search_=a.judge_search_;
  ratio_width_to_height_=a.ratio_width_to_height_;
}










