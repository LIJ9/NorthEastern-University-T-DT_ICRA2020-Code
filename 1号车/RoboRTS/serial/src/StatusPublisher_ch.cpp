/******************************************************************************
*
* The MIT License (MIT)
*
* Copyright (c) 2018 Bluewhale Robot
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Author: Xie fusheng, Randoms
*******************************************************************************/

#include "StatusPublisher.h"



namespace bw_auto_dock
{
StatusPublisher::StatusPublisher(double crash_distance,CallbackAsyncSerial* cmd_serial)
{
    mbUpdated = false;
    mdock_position_ = DOCK_POSITION::not_found;
    mcharge_status_ = CHARGE_STATUS::freed;
    sensor_status.left_sensor1 = 0;
    sensor_status.left_sensor2 = 0;
    sensor_status.right_sensor2 = 0;
    sensor_status.right_sensor1 = 0;
    crash_distance_ = crash_distance;//CallbackAsyncSerial cmd_serial;
    ///
    mcmd_serial_=cmd_serial;
    sub_vel = mNH.subscribe("cmd_vel", 1000,&StatusPublisher::sendCallback,this);//位置？
      //ROS_INFO_STREAM("wwwwwwwwwwwwwwwww");
    sub_vel1 = mNH.subscribe("cmd_vel1", 1000,&StatusPublisher::adjustCallback,this);//位置？
    sub_vision = mNH.subscribe("vision_tomcu", 1000,&StatusPublisher::visionCallback,this);//位置？
    
    sub_swing = mNH.subscribe("Swing_Cmd", 1,&StatusPublisher::swingCallback,this);//位置？
    
    ///
    mIRsensor1Pub = mNH.advertise<std_msgs::Int32>("bw_auto_dock/IRsensor1", 1, true);
    mIRsensor2Pub = mNH.advertise<std_msgs::Int32>("bw_auto_dock/IRsensor2", 1, true);
    mIRsensor3Pub = mNH.advertise<std_msgs::Int32>("bw_auto_dock/IRsensor3", 1, true);
    mIRsensor4Pub = mNH.advertise<std_msgs::Int32>("bw_auto_dock/IRsensor4", 1, true);
    mDockpostionPub = mNH.advertise<std_msgs::Int32>("bw_auto_dock/Dockpostion", 1, true);
    mChargestatusPub = mNH.advertise<std_msgs::Int32>("bw_auto_dock/Chargestatus", 1, true);
usart_send= mNH.advertise<std_msgs::Int16MultiArray>("usart_send", 1, true);

    mPowerPub = mNH.advertise<std_msgs::Float32>("bw_auto_dock/Chargepower", 1, true);
    mBatteryPowerPub = mNH.advertise<std_msgs::Float32>("bw_auto_dock/Batterypower", 1, true);
    mCurrentPub = mNH.advertise<std_msgs::Float32>("bw_auto_dock/Chargecurrent", 1, true);
    mCrashPub = mNH.advertise<std_msgs::Int32>("bw_auto_dock/Crashdetector", 1, true);
}
void  StatusPublisher::Append_CRC16_Check_Sum(uint8_t* pchMessage,uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return;
    }
    wCRC = Get_CRC16_Check_Sum ( (uint8_t *)pchMessage, dwLength-2, CRC_INIT );
    pchMessage[dwLength-2] = (uint8_t)(wCRC & 0x00ff);
    pchMessage[dwLength-1] = (uint8_t)((wCRC >> 8)& 0x00ff);
}

uint32_t StatusPublisher::Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength){
   uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);

}
uint16_t StatusPublisher::Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC){
   uint8_t chData;
    if (pchMessage == NULL)
    {
        return 0xFFFF;
    }
    while(dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}
void StatusPublisher::swingCallback(const std_msgs::Int16::ConstPtr&msg)
{
  swing=msg->data;
}
void StatusPublisher::sendCallback(const geometry_msgs::Twist::ConstPtr& msg){
  //ROS_INFO_STREAM("wwwwwwwwwwwwwwwww");
int x_vel=msg->linear.x*100;
int y_vel=-msg->linear.y*100*1.1;
int z_vel=msg->angular.z*100*1.3;
/*char cmd_str[6] = { (char)0xcd, (char)0xeb, (char)0xd7, (char)0x02, (char)0x4B, (char)0x00 };
    if (NULL != mcmd_serial_)
    {
        mcmd_serial_->write(cmd_str, 6);
    };
    */

//serial_send(0xfe,x_vel,y_vel,1,1,z_vel);
}
void StatusPublisher::serial_send(short head,int Yaw, int Pitch,bool Beat, bool NoObj, float Distance){

   unsigned char buff[18];
    buff[0]=head;
    buff[1]=(static_cast<short>((int)Yaw))>>8;
    buff[2]=(static_cast<short>((int)Yaw))&0x00ff;
    buff[3]=(static_cast<short>((int)Pitch))>>8;
    buff[4]=(static_cast<short>((int)Pitch))&0x00ff;
    buff[5]=(static_cast<short>((int)Distance))>>8;
    buff[6]=(static_cast<short>((int)Distance))&0x00ff;
    buff[7]=static_cast<char>(Beat);
    buff[8]=static_cast<char>(NoObj);
    buff[9]=static_cast<char>(swing);
    Append_CRC16_Check_Sum(buff,18);
   // char cmd_str[6] = { (char)0xcd, (char)0xeb, (char)0xd7, (char)0x02, (char)0x4B, (char)0x00 };
    if (NULL != mcmd_serial_)
    {
      
       // ROS_INFO_STREAM("11111111111111");
      //if(x_ad_vel==0)
        //mcmd_serial_->write(buff, 18);
     };
}

void StatusPublisher::adjustCallback(const geometry_msgs::Twist::ConstPtr& msg_ad)
{
 x_ad_vel=msg_ad->linear.x*100;
 int y_ad_vel=-msg_ad->linear.y*100*1.1;
 int z_ad_vel=msg_ad->angular.z*100*1.3; 
 //serial_send(0xfe,x_ad_vel,y_ad_vel,1,1,z_ad_vel);
}
void StatusPublisher::visionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
/*int x=msg->linear.x*100;//yaw
int y=msg->linear.y*100;//pitch
int z=msg->linear.z*100;//shoot
int w=msg->angular.x*100;//shoot
  *///serial_send(0xff,x,y,z,w,0);
}

void StatusPublisher::Update(const char data[], unsigned int len)
{
    int i = 0, j = 0;
std_msgs::Int16MultiArray Data;
Data.data.resize(10);
    unsigned char receive_byte[18];
    static unsigned char last_str[2] = { 0x00, 0x00 };
   // static unsigned char new_packed_ctr = DISABLE;  // ENABLE表示尚未处理完，DISABLE 表示新包开始；
//     static int new_packed_ok_len = 0;               //包的理论长度
    static int new_packed_ok_len = 18;               //包的理论长度
    static int new_packed_len = 0;                  //包的实际长度
    static unsigned char cmd_string_buf[512];
    unsigned char current_str = 0x00;
    const int cmd_string_max_size = 512;
    //receive_byte = (int*)&sensor_status;
    //ROS_INFO_STREAM("len"<<len);
    for (i = 0; i < len; i++)
    {
        current_str = data[i];
	
         unsigned int temp=(unsigned int)current_str;
        
	//ROS_INFO_STREAM("temp"<<temp);
        //判断是否有新包头
	if(!new_packed_ctr)
	{
	  // ROS_INFO_STREAM("temp_in"<<temp);
	  if (current_str == 255 || current_str == 254 || current_str == 253 || current_str == 170||current_str == 187)  //包头 
	  {
	      // ROS_INFO_STREAM("current_str"<<current_str);
	      new_packed_ctr =1;
	     // continue;
	  }
	}
        if (new_packed_ctr == 1)
        {
            //判断包当前大小
            if (new_packed_ok_len <= new_packed_len)
            {
                // std::cout<<"runup3 "<< new_packed_len<< new_packed_ok_len<<std::endl;
                //包长度已经大于等于理论长度，后续内容无效
                continue;
            }
            else
            {
	      //std::cout<<"new_packed_len"<<new_packed_len<<std::endl;
                //获取包内容
                new_packed_len++;
                cmd_string_buf[new_packed_len - 1] = current_str;
                if (new_packed_ok_len == new_packed_len && new_packed_ok_len > 0)
                {
                     //std::cout<<"runup4 "<<std::endl;
                    //当前包已经处理完成，开始处理
                    boost::mutex::scoped_lock lock(mMutex_sensor);
		    memcpy(&receive_byte[j], &cmd_string_buf, 18);
		     // ROS_INFO_STREAM("receive_byte"<<cmd_string_buf);
		      if (Verify_CRC16_Check_Sum(receive_byte,new_packed_ok_len)){
			    Data.data[0]=(short)receive_byte[0];
			    Data.data[1]=(short)(receive_byte[1] << 8 | receive_byte[2]);
			    Data.data[2]=(short)(receive_byte[3] << 8 | receive_byte[4]);
			    Data.data[3]=(short)(receive_byte[5] << 8 | receive_byte[6]);
			    Data.data[4]=(short)(receive_byte[7] << 8 | receive_byte[8]);
			    Data.data[6]=(short)(receive_byte[9] << 8 | receive_byte[10]);
			    Data.data[5]=(short)(receive_byte[11]);
			    Data.data[7]=(short)(receive_byte[12]);
			    Data.data[8]=(short)(receive_byte[13]<< 8 | receive_byte[14]);
			  //  Data.data[9]=(short)(receive_byte[15]);
			    usart_send.publish(Data);
		      }
		      new_packed_len = 0;
		      new_packed_ctr =0;
                    // ii++;new_packed_ok_len
                    // std::cout << ii << std::endl;
                
		}
            }
	}
    }
    return;
}

void StatusPublisher::Refresh()
{
    boost::mutex::scoped_lock lock1(mMutex_sensor);
    boost::mutex::scoped_lock lock2(mMutex_dock);
    boost::mutex::scoped_lock lock3(mMutex_charge);
    // std::cout<<"runR"<< mbUpdated<<std::endl;
    if (mbUpdated)
    {
        // Time
        ros::Time current_time = ros::Time::now();
        std_msgs::Int32 pub_data;
        pub_data.data = sensor_status.left_sensor1;
        mIRsensor1Pub.publish(pub_data);

        pub_data.data = sensor_status.left_sensor2;
        mIRsensor2Pub.publish(pub_data);

        pub_data.data = sensor_status.right_sensor2;
        mIRsensor3Pub.publish(pub_data);

        pub_data.data = sensor_status.right_sensor1;
        mIRsensor4Pub.publish(pub_data);

        //根据4个红外接收器状态判断dock方位
        if ((sensor_status.left_sensor1 + sensor_status.left_sensor2 + sensor_status.right_sensor2 +
             sensor_status.right_sensor1) == 0)
        {
            //没有发现
            mdock_position_ = DOCK_POSITION::not_found;
        }
        else
        {
            if ((sensor_status.left_sensor2 + sensor_status.right_sensor2) == 0)
            {
                //后面的传感器没有发现
                if (sensor_status.left_sensor1 == 0)
                {
                    //左边的传感器没有发现
                    switch (sensor_status.right_sensor1)
                    {
                        case 1:
                            mdock_position_ = DOCK_POSITION::right_up;
                            break;
                        case 2:
                            mdock_position_ = DOCK_POSITION::right_down;
                            break;
                        case 3:
                            mdock_position_ = DOCK_POSITION::right_center;
                            break;
                        case 4:
                            mdock_position_ = DOCK_POSITION::right;
                            break;
                        case 5:
                            mdock_position_ = DOCK_POSITION::right_up;
                            break;
                        case 6:
                            mdock_position_ = DOCK_POSITION::right_down;
                            break;
                        case 7:
                            mdock_position_ = DOCK_POSITION::right_center;
                            break;
                    }
                }
                else
                {
                    //右边的传感器没有发现
                    switch (sensor_status.left_sensor1)
                    {
                        case 1:
                            mdock_position_ = DOCK_POSITION::left_down;
                            break;
                        case 2:
                            mdock_position_ = DOCK_POSITION::left_up;
                            break;
                        case 3:
                            mdock_position_ = DOCK_POSITION::left_center;
                            break;
                        case 4:
                            mdock_position_ = DOCK_POSITION::left;
                            break;
                        case 5:
                            mdock_position_ = DOCK_POSITION::left_down;
                            break;
                        case 6:
                            mdock_position_ = DOCK_POSITION::left_up;
                            break;
                        case 7:
                            mdock_position_ = DOCK_POSITION::left_center;
                            break;
                    }
                }
            }
            else
            {
                if (sensor_status.left_sensor2 == 0)
                {
                    //后面的左边探测器没有探测
                    switch (sensor_status.right_sensor2)
                    {
                        case 1:
                            mdock_position_ = DOCK_POSITION::back_right;
                            break;
                        case 2:
                            mdock_position_ = DOCK_POSITION::back_right;
                            break;
                        case 3:
                            mdock_position_ = DOCK_POSITION::back_right;
                            break;
                        case 4:
                            mdock_position_ = DOCK_POSITION::back;
                            break;
                        case 5:
                            mdock_position_ = DOCK_POSITION::back_right;
                            break;
                        case 6:
                            mdock_position_ = DOCK_POSITION::back_right;
                            break;
                        case 7:
                            mdock_position_ = DOCK_POSITION::back_right;
                            break;
                    }
                }
                else
                {
                    //后面的右边探测器没有探测
                    switch (sensor_status.left_sensor2)
                    {
                        case 1:
                            if (sensor_status.right_sensor2 != 4 && sensor_status.right_sensor2 != 0)
                            {
                                mdock_position_ = DOCK_POSITION::back_center;
                            }
                            else
                            {
                                mdock_position_ = DOCK_POSITION::back_left;
                            }
                            break;
                        case 2:
                            if (sensor_status.right_sensor2 != 4 && sensor_status.right_sensor2 != 0 &&
                                sensor_status.right_sensor2 != 2 && sensor_status.right_sensor2 != 6)
                            {
                                mdock_position_ = DOCK_POSITION::back_center;
                            }
                            else
                            {
                                mdock_position_ = DOCK_POSITION::back_left;
                            }
                            break;
                        case 3:
                            if (sensor_status.right_sensor2 != 4 && sensor_status.right_sensor2 != 0)
                            {
                                mdock_position_ = DOCK_POSITION::back_center;
                            }
                            else
                            {
                                mdock_position_ = DOCK_POSITION::back_left;
                            }
                            break;
                        case 4:
                            if (sensor_status.right_sensor2 != 0 && sensor_status.right_sensor2 != 4)
                            {
                                mdock_position_ = DOCK_POSITION::back_right;
                            }
                            else
                            {
                                mdock_position_ = DOCK_POSITION::back;
                            }
                            break;
                        case 5:
                            if (sensor_status.right_sensor2 != 4 && sensor_status.right_sensor2 != 0)
                            {
                                mdock_position_ = DOCK_POSITION::back_center;
                            }
                            else
                            {
                                mdock_position_ = DOCK_POSITION::back_left;
                            }
                            break;
                        case 6:
                            if (sensor_status.right_sensor2 != 4 && sensor_status.right_sensor2 != 0 &&
                                sensor_status.right_sensor2 != 2 && sensor_status.right_sensor2 != 6)
                            {
                                mdock_position_ = DOCK_POSITION::back_center;
                            }
                            else
                            {
                                mdock_position_ = DOCK_POSITION::back_left;
                            }
                            break;
                        case 7:
                            if (sensor_status.right_sensor2 != 4 && sensor_status.right_sensor2 != 0)
                            {
                                mdock_position_ = DOCK_POSITION::back_center;
                            }
                            else
                            {
                                mdock_position_ = DOCK_POSITION::back_left;
                            }
                            break;
                    }
                }
            }
        }
        if (sensor_status.left_sensor1 == 3||sensor_status.left_sensor1 == 7)
        {
          //ROS_ERROR(" dock : %d, %d %d",(int)mdock_position_,sensor_status.left_sensor1,sensor_status.left_sensor2);
          mdock_position_ = DOCK_POSITION::left_center;
        }
        if (sensor_status.right_sensor1 == 3||sensor_status.right_sensor1 == 7)
        {
          //ROS_ERROR(" dock : %d, %d %d",(int)mdock_position_,sensor_status.left_sensor1,sensor_status.left_sensor2);
          mdock_position_ = DOCK_POSITION::right_center;
        }
        //发布dock位置
        pub_data.data = (int)mdock_position_;
        mDockpostionPub.publish(pub_data);
        //发布充电任务状态
        pub_data.data = (int)mcharge_status_;
        mChargestatusPub.publish(pub_data);

        if (sensor_status.distance1 <= this->crash_distance_&&sensor_status.distance1>0.1)
        {
            pub_data.data = 1;
        }
        else
        {
            pub_data.data = 0;
        }
        ROS_DEBUG("distance: %f %f",sensor_status.distance1,sensor_status.distance2);
        mCrashPub.publish(pub_data);

        std_msgs::Float32 pub_data2;
        pub_data2.data = sensor_status.power;
        mPowerPub.publish(pub_data2);

        pub_data2.data = sensor_status.battery;
        mBatteryPowerPub.publish(pub_data2);

        pub_data2.data = sensor_status.current;
        mCurrentPub.publish(pub_data2);

        mbUpdated = false;
    }
}

DOCK_POSITION StatusPublisher::get_dock_position()
{
    boost::mutex::scoped_lock lock(mMutex_dock);
    return mdock_position_;
}
void StatusPublisher::set_charge_status(CHARGE_STATUS charge_status)
{
    boost::mutex::scoped_lock lock(mMutex_charge);
    mcharge_status_ = charge_status;
}

CHARGE_STATUS StatusPublisher::get_charge_status()
{
    boost::mutex::scoped_lock lock(mMutex_charge);
    return mcharge_status_;
}

}  // namespace bw_auto_dock
