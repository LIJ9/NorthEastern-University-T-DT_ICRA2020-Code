/*************************************************************************
 * @brief          预测
 * @version        1.0.0.1
 * @authors        郭梓楠
 * -----------------------------------------------------------------------
 *   Change Hisitory ：
 *   <Date>     | <Verision> | <Author> |<Descripition>
 * -----------------------------------------------------------------------
 *   2019/02/17 |  1.0.0.1  |   郭梓楠   | 代码规范
 *
 ************************************************************************/

#ifndef ROBORTS_DETECTION_ARMOR_DETECTION_PREDICT_H
#define ROBORTS_DETECTION_ARMOR_DETECTION_PREDICT_H



#include <opencv/cv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/core/types.hpp"
#include <iostream>
#include "macro.h"


using namespace cv;
using namespace std;

/**
 * @brief         计算x的y次幂
 */
double RelatePow(vector<double> vx, int n, int ex);
/**
 * @brief         x的ex次方与y的乘积的累加
 */
double RelateMutiXY(vector<double> vx, vector<double> vy, int n, int ex);
/**
 *  @brief    最小二乘法拟合函数
 *  @param    vx  x的数组
 *  @param    vy  云台信息
 *  @param    n  拟合数据数量
 *  @param    ex  拟合函数次数
 *  @param   coefficient  系数矩阵
 *  @return   coefficient 系数矩阵
 *
 *  @author   郭梓楠
 *  @qq       3426445602
 **/
void EMatrix(vector<double> vx, vector<double> vy, int n, int ex, double coefficient[]);
/**
 * @brief         求解方程
 */
void CaleQuation(int exp, double coefficient[]);
/**
 * @brief         供CalEquation函数调用
 */
double F(double c[],int l,int m);

/**
 * @brief         队列节点
 */
struct PointNode
{
    Target tmp_target;
    Point point;
    int distance;
    float yaw_angle_now;  //云台yaw角度
    float pitch_angle_now;
    float rotation_angle;  //需旋转角度
    int  pixel_distance;  //像素差
    PointNode *next;
    struct timeval time_now;  //此刻时间
};
/**
 * @brief         预测
 */
class Predictor
{
public:
    /***
     *  @brief    预测函数
     *  @input    input_target  目标装甲板
     *  @input    input_shoot_platform  云台信息
     *  @input    time  时间结构体
     *  @output   target  以陀螺仪yaw pitch为0处为坐标系的目标点坐标
     *
     *  @author   郭梓楠
     *  @qq       3426445602
     ***/
    void predict_Get(Target &target,cv::Mat intrinsic_matrix_, float yaw_now, float pitch_now, struct timeval time);
    inline int GetData(void){return data_n_;};  //得到此刻有效数据的数量
    inline void Add(void){data_n_++;};  //有效数据加一
    inline void Init(void){data_n_=1;};  //有效数据初始化
    RotatedRect Predict_Output;
    struct PointNode* start_point=(struct PointNode*)malloc(sizeof(struct PointNode));  //队列节点
    struct PointNode* mid_point=(struct PointNode*)malloc(sizeof(struct PointNode));
    struct PointNode* new_point;
    struct PointNode* end_point=(struct PointNode*)malloc(sizeof(struct PointNode));
    struct PointNode* tem_point=(struct PointNode*)malloc(sizeof(struct PointNode));

private:
    int data_n_=1;
    float best_height_;
    float best_pnp_assess;
    float last_best_height_;
    float last_best_pnp_assess_;

};

#endif
