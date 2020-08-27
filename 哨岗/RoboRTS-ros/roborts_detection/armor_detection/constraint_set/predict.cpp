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
#include "iostream"
#include "predict.h"
#include  <stdio.h>
#include <sys/time.h>
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ml/ml.hpp"
#include "vector"
#include "opencv2/core/core.hpp"
#include "unistd.h"
using namespace std;
using namespace cv;

Point Position;
double em[6][4]             ;

double RelatePow(vector<double> vx, int n, int ex){
    double resum=0;
    for (int i=0; i<n; i++){
        resum+=pow(vx[i],ex);
    }
    return resum;

}

double RelateMutiXY(vector<double> vx, vector<double> vy, int n, int ex){
    double dremulti_sum=0;
    for (int i=0; i<n; i++){
        dremulti_sum+=pow(vx[i],ex)*vy[i];
    }
    return dremulti_sum;
}

void EMatrix(vector<double> vx, vector<double> vy, int n, int ex, double coefficient[]){//预测函数只用到这一个函数 其余函数为最小二乘法计算过程
    for (int i=1; i<=ex; i++)
    {
        for (int j=1; j<=ex; j++){
            em[i][j]=RelatePow(vx,n,i+j-2);
        }
        em[i][ex+1]=RelateMutiXY(vx,vy,n,i-1);
    }
    em[1][1]=n;
    CaleQuation(ex,coefficient);
}

void CaleQuation(int exp, double coefficient[]){
    for(int k=1;k<exp;k++)
    {
        for(int i=k+1;i<exp+1;i++){
            double p1=0;
            if(em[k][k]!=0){
                p1=em[i][k]/em[k][k];
            }
            for(int j=k;j<exp+2;j++){
                em[i][j]=em[i][j]-em[k][j]*p1;
            }
        }
    }
    coefficient[exp]=em[exp][exp+1]/em[exp][exp];
    for(int l=exp-1;l>=1;l--){
        coefficient[l]=(em[l][exp+1]-F(coefficient,l+1,exp))/em[l][l];
    }
}

double F(double c[],int l,int m){
    double sum=0;
    for(int i=l;i<=m;i++){
        sum+=em[l-1][i]*c[i];
    }
    return sum;
}


//预测
void Predictor::predict_Get(Target &target,cv::Mat intrinsic_matrix_, float yaw_now, float pitch_now, struct timeval time){
    start: if(GetData()==1){
    start_point->time_now=time;
    start_point->tmp_target=target;
    mid_point=start_point;
    Add();
}
    if(GetData()>1&&GetData()<5){
        new_point=(struct PointNode*)malloc(sizeof(struct PointNode));
        new_point->tmp_target=target;
        new_point->time_now=time;
        if(((new_point->time_now.tv_usec/1000+new_point->time_now.tv_sec*1000)-(mid_point->time_now.tv_usec/1000+mid_point->time_now.tv_sec*1000))>40){
            Init();
            goto start;
        }
        mid_point->next=new_point;
        mid_point=new_point;
        Add();
    }
    if(GetData()==5){
        end_point=(struct PointNode*)malloc(sizeof(struct PointNode));
        end_point->time_now=time;
        end_point->tmp_target=target;
        if(((end_point->time_now.tv_usec/1000+end_point->time_now.tv_sec*1000)-(mid_point->time_now.tv_usec/1000+mid_point->time_now.tv_sec*1000))>40){
            Init();
            goto start;
        }


        mid_point->next=end_point;
        Add();
        std::vector<cv::Point> Key_point;
        tem_point=start_point;
        float tmp_best_access=tem_point->tmp_target.get_pnp_assess();
        float tmp_best_height=tem_point->tmp_target.get_y();
        vector<double> D_T_point_h;
        vector<double> D_T_point_x;
        double tem_dis_h=0;
        for(int i=0;i<5;i++){
            if(tmp_best_access>tem_point->tmp_target.get_pnp_assess()){
                tmp_best_access=tem_point->tmp_target.get_pnp_assess();
                tmp_best_height=tem_point->tmp_target.get_y();
            }
            D_T_point_x.push_back((tem_point->time_now.tv_usec*1.0/1000+tem_point->time_now.tv_sec*1000)-(start_point->time_now.tv_usec*1.0/1000+start_point->time_now.tv_sec*1000));
            D_T_point_h.push_back(tem_point->tmp_target.get_y());
            tem_point=tem_point->next;
        }
        double coeff_h[5];
        memset(coeff_h,0,sizeof(double)*5);
        EMatrix(D_T_point_x,D_T_point_h,5,1,coeff_h);
        best_height_=0.3*coeff_h[0]+0.5*best_height_+0.2*target.get_y();

        double tem_dis_x=0;
        double tem_dis_z=0;
        vector<double> D_T_point_y;
        vector<double> D_T_point_z;
        tem_point=start_point;
        for(int i=0;i<5;i++){
            if(fabs(best_height_)>5) {
                double best_distance = best_height_ * 1.0 / sin(tem_point->tmp_target.get_absolute_pitch_angle());
                double best_x = sqrt(best_distance * best_distance - best_height_ * best_height_) *
                                sin(tem_point->tmp_target.get_absolute_yaw_angle());
                double best_z = sqrt(best_distance * best_distance - best_height_ * best_height_) *
                                cos(tem_point->tmp_target.get_absolute_yaw_angle());
                tem_point->tmp_target.set_distance(best_distance);
                tem_point->tmp_target.set_x(best_x);
                tem_point->tmp_target.set_y(best_height_);
                tem_point->tmp_target.set_z(best_z);
            }
            tem_dis_x=tem_point->tmp_target.get_x();
            tem_dis_z=tem_point->tmp_target.get_z();
            D_T_point_y.push_back(tem_dis_x);
            D_T_point_z.push_back(tem_dis_z);
            tem_point=tem_point->next;
        }

        double coeff[5];
        memset(coeff,0,sizeof(double)*5);
        EMatrix(D_T_point_x,D_T_point_y,5,2,coeff);

        double T;

        T=target.get_distance()/2;

        T=T+end_point->time_now.tv_usec/1000+end_point->time_now.tv_sec*1000-(start_point->time_now.tv_usec/1000+start_point->time_now.tv_sec*1000);

        target.set_x(coeff[1]+coeff[2]*T);
        EMatrix(D_T_point_x,D_T_point_z,5,2,coeff);
        target.set_z(coeff[1]+coeff[2]*T);
    }
    if(GetData()>5){

        struct PointNode* release;
        release=start_point;
        start_point=start_point->next;
        free(release);
        release=NULL;
        mid_point=end_point;
        end_point=(struct PointNode*)malloc(sizeof(struct PointNode));

        end_point->time_now=time;
        end_point->tmp_target=target;

        if(((end_point->time_now.tv_usec/1000+end_point->time_now.tv_sec*1000)-(mid_point->time_now.tv_usec/1000+mid_point->time_now.tv_sec*1000))>40){

            Init();
            goto start;
        }
        mid_point->next=end_point;
        tem_point=start_point;
        float tmp_best_access=tem_point->tmp_target.get_pnp_assess();
        float tmp_best_height=tem_point->tmp_target.get_y();
        vector<double> D_T_point_h;
        vector<double> D_T_point_x;
        double tem_dis_h=0;
        for(int i=0;i<5;i++){

            if(tmp_best_access>tem_point->tmp_target.get_pnp_assess()){
                tmp_best_access=tem_point->tmp_target.get_pnp_assess();
                tmp_best_height=tem_point->tmp_target.get_y();
            }
            D_T_point_x.push_back((tem_point->time_now.tv_usec*1.0/1000+tem_point->time_now.tv_sec*1000)-(start_point->time_now.tv_usec*1.0/1000+start_point->time_now.tv_sec*1000));
            D_T_point_h.push_back(tem_point->tmp_target.get_y());
            tem_point=tem_point->next;
        }
        double coeff_h[5];
        memset(coeff_h,0,sizeof(double)*5);
        EMatrix(D_T_point_x,D_T_point_h,5,1,coeff_h);
        best_height_=0.3*coeff_h[0]+0.5*best_height_+0.2*target.get_y();


        double tem_dis_x=0;
        double tem_dis_z=0;
        //x值数组
        vector<double> D_T_point_y;
        vector<double> D_T_point_z;
        tem_point=start_point;
        std::vector<cv::Point> Key_point;
        double tem_dis=0;

        for(int i=0;i<5;i++){

            if(fabs(best_height_)>5) {
                double best_distance = best_height_ * 1.0 / sin(tem_point->tmp_target.get_absolute_pitch_angle());
                double best_x = sqrt(best_distance * best_distance - best_height_ * best_height_) *
                                sin(tem_point->tmp_target.get_absolute_yaw_angle());
                double best_z = sqrt(best_distance * best_distance - best_height_ * best_height_) *
                                cos(tem_point->tmp_target.get_absolute_yaw_angle());
                tem_point->tmp_target.set_distance(best_distance);
                tem_point->tmp_target.set_x(best_x);
                tem_point->tmp_target.set_y(best_height_);
                tem_point->tmp_target.set_z(best_z);
            }
            tem_dis_x=tem_point->tmp_target.get_x();
            tem_dis_z=tem_point->tmp_target.get_z();
            D_T_point_y.push_back(tem_dis_x);
            D_T_point_z.push_back(tem_dis_z);
            tem_point=tem_point->next;
        }

        double coeff[5];
        memset(coeff,0,sizeof(double)*5);
        EMatrix(D_T_point_x,D_T_point_y,5,2,coeff);

        double T;

        T=100;
        T=T+end_point->time_now.tv_usec/1000+end_point->time_now.tv_sec*1000-(start_point->time_now.tv_usec/1000+start_point->time_now.tv_sec*1000);
        end_point->rotation_angle=-atan2((end_point->pixel_distance+(coeff[1]+coeff[2]*T-tem_dis)),intrinsic_matrix_.at<double>(0,0))*57.3;
        mid_point->next=end_point;
        target.set_x(coeff[1]+coeff[2]*T);
        EMatrix(D_T_point_x,D_T_point_z,5,2,coeff);
        target.set_z(coeff[1]+coeff[2]*T);
        target.set_yaw(end_point->rotation_angle);
        target.set_absolute_yaw_angle(yaw_now+end_point->rotation_angle);
    }

}