/*****************************************************************************
 * @brief        工程的宏定义声明
 * @version      1.0.0.2
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
 * 2019/11/26 | 1.0.0.2   | 魏广伟   |添加类LightBar
 * ---------------------------------------------------------------------------
 *
*****************************************************************************/

#ifndef ROBORTS_DETECTION_ARMOR_DETECTION_MACRO_H
#define ROBORTS_DETECTION_ARMOR_DETECTION_MACRO_H

#include <opencv2/core/types.hpp>
#include <opencv2/core.hpp>
#include "sys/time.h"

#include <iostream>

using namespace cv;
using namespace std;
enum Color{BLUE=0,RED=2,YELLOW=3,UNKNOW};//视觉标签颜色
/**
 * @brief 
 * @author 2019icra代码
 */
class Target{
public:
    Target();
    Target(float x,float y,float z,float height,float width,float angle_x,float angle_y,float angle_z,int image_x,int image_y,float distance,float pnp_assess);
    //Target(float x,float y,float z,float height,float width,float angle_x,float angle_y,float angle_z,int image_x,int image_y,float distance,float pnp_assess);
    Target(float x,float y,float z,float height,float width,float angle_x,float angle_y,float angle_z,int image_x,int image_y,float distance);
    inline float get_x(void) const {return x_;}
    inline float get_y(void) const {return y_;}
    inline float get_z(void) const {return z_;}
    inline float get_distance(void) const { return distance_;}
    inline float get_angle_x(void) const { return angle_x_;}
    inline float get_angle_y(void) const { return angle_y_;}
    inline float get_angle_z(void) const { return angle_z_;}
    inline void set_absolute_yaw_angle(float yaw){absolute_yaw_angle_=yaw;}
    inline void set_absolute_pitch_angle(float pitch){absolute_pitch_angle_=pitch;}
    inline void set_distance(float distance){distance_=distance;}
    inline float get_absolute_yaw_angle(void){return absolute_yaw_angle_;}
    inline float get_absolute_pitch_angle(void){return absolute_pitch_angle_;}
    inline float get_distance(void){return distance_;}
    inline int set_image_x(float image_x){image_center_x_=image_x;}
    inline int set_image_y(float image_y){image_center_y_=image_y;}
    inline bool empty(){ return empty_;}
    inline int get_image_x(void){return image_center_x_;}
    inline int get_image_y(void){return image_center_y_;}
    inline void set_yaw(float yaw){yaw_=yaw;}
    inline float get_yaw(void){return yaw_;}
    inline void set_pitch(float pitch){pitch_=pitch;}
    inline float get_pitch(void){return pitch_;}
    inline void set_x(float x){x_=x;}
    inline void set_y(float y){y_=y;}
    inline void set_z(float z){z_=z;}
    inline float get_pnp_assess(void){return pnp_assess_;}

    inline Rect set_armor_rect(Rect rect){armor_rect_=rect;}
    inline Rect get_armor_rect(){ return armor_rect_;}
private:
    float x_;
    float y_;
    float z_;
    float height_;
    float width_;
    float angle_x_;
    float angle_y_;
    float angle_z_;
    int image_center_x_;
    int image_center_y_;
    float distance_=0;
    bool empty_;
    float yaw_;
    float pitch_;
    float absolute_yaw_angle_;
    float absolute_pitch_angle_;
    float pnp_assess_;
    Rect armor_rect_;

};

/**
 * @brief 灯条
 * @author Way
 */
class LightBar
{
public:
  LightBar(RotatedRect rotrect);
  LightBar();
  ~LightBar();
   inline int get_height() const { return height_; }
  inline int get_width() const { return width_; }
  inline float get_angle() const {return angle_;}
  inline float get_ratio() const {return ratio_;}
private:
  RotatedRect rotated_rect_;
  float angle_;
  float ratio_;
  int width_;
  int  height_;
};


class VisionMarkers
{
public:
  VisionMarkers(RotatedRect rotrect,Color color);
  VisionMarkers();
  ~VisionMarkers();
  //旋转矩形的各个顶点
  inline  Point tl() const { return static_cast<Point>(tl_); }
  inline  Point tr() const { return static_cast<Point>(tr_); }
  inline  Point bl() const { return static_cast<Point>(bl_); }
  inline  Point br() const { return static_cast<Point>(br_); }
  
  inline Color get_color() const { return color_; }
  inline int get_height() const { return height_; }
  inline int get_width() const { return width_; }
  inline float get_angle() const { return angle_; }
  inline Point get_center() const { return center_; }
  inline Rect get_rect() const { return bounding_rect_; }
  inline void set_judge_search(int judge_search) { judge_search_=judge_search;}
  inline int get_judge_search() const { return judge_search_; }
  inline RotatedRect get_rotatedrect() const {return rotated_rect_;}
  inline int get_area() const {return area_;}
  inline Point2f get_top_point(void) const {return top_point_;}
  inline Point2f get_bottom_point(void) const {return bottom_point_;}
  inline Rect get_bounding_rect(void) const {return bounding_rect_;}
  inline float get_ratio(void) const{return ratio_width_to_height_;}
  inline bool empty(void) const {return empty_;}

public:
    void operator=(const VisionMarkers &a);

private:
    ////增加成员变量记得修改=赋值符的重载++++
    int area_;
    float angle_;
    float width_;
    float height_;
    Point center_;
    Point2f tl_;
    Point2f tr_;
    Point2f bl_;
    Point2f br_;
    Point2f top_point_;//灯条上端点
    Point2f bottom_point_;//灯条下端点
    Rect bounding_rect_;
    RotatedRect rotated_rect_;
    Color color_;
    int judge_search_;//(0:none -1:left 1:right 2:both) have been search
    float ratio_width_to_height_;
    bool empty_;
};
#endif
