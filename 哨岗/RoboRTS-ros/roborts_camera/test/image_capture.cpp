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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <vector>
#include <thread>
#include <mutex>
#include <string>
#include <sstream>
#include<time.h>

#include "opencv2/opencv.hpp"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "message_filters/subscriber.h"
#include"message_filters/time_synchronizer.h"
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;
using namespace cv;

cv::Mat src_img;
cv::Mat src_img1;
cv::Mat result;
vector<Mat> images;
vector<Point2f> points;
#define R 255
#define G 0
#define B 255

#define R1 255
#define G1 0
#define B1 255
#define thick 2

//-- camera0
Point2f p01(404,274);
Point2f p02(1216,204);

Point2f p03(1160,168);
Point2f p04(1434,346);

Point2f p05(396,304);
Point2f p06(886,1078);

Point2f p07(584,618);
Point2f p08(1434,528);

//-- block center 
Point2f p1(600,180);
Point2f p2(1200,200);
Point2f p3(1000,370);
Point2f p4(1120,800);
Point2f p5(200,610);
Point2f p6(350,260);

//-- camera2
Point2f p11(234,256);
Point2f p12(1106,204);

Point2f p13(1040,160);
Point2f p14(1440,440);

Point2f p15(240,280);
Point2f p16(672,1080);

Point2f p17(420,640);
Point2f p18(1440,460);

void ManyImages(vector<Mat> Images, Mat& dst, int imgRows)
{
    int Num = Images.size();//得到Vector容器中图片个数
    //设定包含这些图片的窗口大小，这里都是BGR3通道，如果做灰度单通道，稍微改一下下面这行代码就可以
    Mat Window(480 * ((Num - 1) / imgRows + 2), 640 * (imgRows-1), CV_8UC3, Scalar(0, 0, 0));
    Mat Std_Image;//存放标准大小的图片
    Mat imageROI;//图片放置区域
    Size Std_Size = Size(640, 480);//每个图片显示大小300*300
    int x_Begin = 0;
    int y_Begin = 0;
    for (int i = 0; i < Num; i++)
    {
        x_Begin = (i / imgRows)*Std_Size.width;//每张图片起始坐标
        y_Begin = (i % imgRows)*Std_Size.height;
        resize(Images[i], Std_Image, Std_Size, 0, 0, INTER_LINEAR);//将图像设为标准大小
        //将其贴在Window上
        imageROI = Window(Rect(x_Begin, y_Begin, Std_Size.width, Std_Size.height));
        Std_Image.copyTo(imageROI);
    }
    dst = Window;
}

void DrawLine0(const cv::Mat &img)
{
	cv::line(img, p01, p02, Scalar(R,G,B), 3);
	cv::line(img, p03, p04, Scalar(R,G,B), 3);
	cv::line(img, p05, p06, Scalar(R,G,B), 3);
	cv::line(img, p07, p08, Scalar(R,G,B), 3); 
}
void DrawLine1(const cv::Mat &img)
{
	cv::line(img, p11, p12, Scalar(R,G,B), 3);
	cv::line(img, p13, p14, Scalar(R,G,B), 3);
	cv::line(img, p15, p16, Scalar(R,G,B), 3);
	cv::line(img, p17, p18, Scalar(R,G,B), 3);  
}
void DrawWarld0(const cv::Mat &img)
{
	int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 1.2;
	cv::putText(img, "[Block1]", p1, font_face, font_scale, cv::Scalar(R1,G1,B1),thick);
	cv::putText(img, "[Block2]", p2, font_face, font_scale, cv::Scalar(R1,G1,B1),thick);
	cv::putText(img, "[Block3]", p3, font_face, font_scale, cv::Scalar(R1,G1,B1),thick);
	cv::putText(img, "[Block4]", p4, font_face, font_scale, cv::Scalar(R1,G1,B1),thick);
	cv::putText(img, "[Block5]", p5, font_face, font_scale, cv::Scalar(R1,G1,B1),thick);
}
void DrawWarld1(const cv::Mat &img)
{
	int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 1.2;
	cv::putText(img, "[Block6] ",p1, font_face, font_scale, cv::Scalar(R1,G1,B1),thick);
	cv::putText(img, "[Block5]", p2, font_face, font_scale, cv::Scalar(R1,G1,B1),thick);
	cv::putText(img, "[Block4]", p3, font_face, font_scale, cv::Scalar(R1,G1,B1),thick);
	cv::putText(img, "[Block3]", p4, font_face, font_scale, cv::Scalar(R1,G1,B1),thick);
	cv::putText(img, "[Block2]", p5, font_face, font_scale, cv::Scalar(R1,G1,B1),thick);
}
void callback(const sensor_msgs::ImageConstPtr &msg1,const sensor_msgs::ImageConstPtr &msg2)
{
	src_img = cv_bridge::toCvShare(msg1, "bgr8")->image.clone();
	src_img1 = cv_bridge::toCvShare(msg2, "bgr8")->image.clone();
	DrawLine0(src_img);
	DrawLine1(src_img1);
	DrawWarld0(src_img);
	DrawWarld1(src_img1);
	images.push_back(src_img);
	images.push_back(src_img1);
	ManyImages(images,result,2);
	images.clear();
}



int main(int argc, char **argv) 
{
	
	ros::init(argc, argv, "image_capture");
	ros::NodeHandle nh;
	message_filters::Subscriber<Image> img_sub0(nh,"video0_img",1);
	message_filters::Subscriber<Image> img_sub1(nh,"video1_img",1);
	TimeSynchronizer<Image, Image> sync(img_sub0, img_sub1, 10);
	sync.registerCallback(boost::bind(&callback, _1, _2));
	while (nh.ok())
	{
		if(!result.empty())
			imshow("result",result);
		auto c = cv::waitKey(1);
		if (c == 'a') 
		{
			cv::waitKey(0);
		}
		ros::spinOnce();
	}
	return 0;
}



//-- 保存视频
#if 0

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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <vector>
#include <thread>
#include <mutex>
#include <string>
#include <sstream>
#include<time.h>

#include "opencv2/opencv.hpp"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

cv::Mat src_img;
void ReceiveImg(const sensor_msgs::ImageConstPtr &msg) {
  src_img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  std::cout<<"111"<<std::endl;
  //std::cout<<" real topic_name is "<<topic_name<<std::endl;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "image_capture");
  ros::NodeHandle nh;
  std::time_t now_time=time(NULL);  
  tm*  t_tm = localtime(&now_time);
  //std::printf("local time is    : %s\n", asctime(t_tm));
  std::string topic_name;
  topic_name = asctime(t_tm);
  cv::VideoWriter writer("/home/nuc/catkin_ws/src/RoboRTS-ros/roborts_debug/"+topic_name+"tdt_2.avi", CV_FOURCC('M', 'J', 'P', 'G'), 250.0, cv::Size(640, 480));
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/video_img", 20, boost::bind(&ReceiveImg, _1));
  while (nh.ok())
  {
    if(!src_img.empty())
      writer.write(src_img);
    ros::spinOnce();
  }
  return 0;
}
#endif