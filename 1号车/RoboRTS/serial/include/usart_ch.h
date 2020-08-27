/*************************************************************************
 * @brief          target resolver
 *                 用于使用pnp算法对目标的 x，y，z 距离的解算
 * @version        1.0.0.1
 * @authors        何武军   王钰深   郭梓楠
 * -----------------------------------------------------------------------
 *   Change Hisitory ：
 *   <Date>     | <Verision> | <Author> |<Descripition>
 * -----------------------------------------------------------------------
 *   2019/02/16 |  1.0.0.1  |   何武军   | 修改宏定义以及代码规范的命名格式
 *
 ************************************************************************/
#ifndef T_DT2019VISION_USART_H
#define T_DT2019VISION_USART_H
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> //文件控制定义
#include <termios.h>//终端控制定义
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <mutex>
#include <vector>
//设备名称
#define DEVICE "/dev/mych340"
const int  total_send_length = 18;
const int  receive_length = 18;
//#define S_TIMEOU
using namespace std;
class Class_Usart
{
public:
    static int serial_fd;
    static unsigned int reserved_send_length;
    void Usart_Init();
    int Usart_Send(short head,int my_position_x,int my_position_y,int enemy_position_x,int enemy_position_y,int bufflet,bool bufflet_sign,short buff_sign,int remain_blood);
    
    int Usart_Recv(volatile int *data);
    uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
    uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
    void Append_CRC16_Check_Sum(uint8_t  * pchMessage,uint32_t dwLength);



};

#endif //T_DT2019VISION_USART_H
