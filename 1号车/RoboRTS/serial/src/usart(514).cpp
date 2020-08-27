#include "usart.h"
#include <iostream>
#include <vector>

#include "sys/time.h"
extern int error1;
using namespace std;
uint16_t CRC_INIT = 0xffff;
#define DEBUG 1;
const uint16_t wCRC_Table[256] =
        {
                0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
                0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
                0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
                0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
                0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
                0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
                0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
                0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
                0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
                0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
                0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
                0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
                0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
                0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
                0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
                0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
                0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
                0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
                0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
                0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
                0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
                0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
                0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
                0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
                0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
                0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
                0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
                0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
                0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
                0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
                0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
                0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
        };
/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Class_Usart::Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
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
/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t  Class_Usart::Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}
/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void  Class_Usart::Append_CRC16_Check_Sum(uint8_t* pchMessage,uint32_t dwLength)
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


//打开串口并初始化设置
int Class_Usart::serial_fd=0;
void Class_Usart::Usart_Init()
{
//O_RDWR 读、写打开,
//O_NOCTTY 如果p a t h n a m e指的是终端设备，则不将此设备分配作为此进程的控制终端
//O_NDELAY非延时模式
//O_NONBLOCK非阻塞模式
 serial_fd = open(DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd < 0)

    {
         perror("Usart open is failed!");
    }

    //串口主要设置结构体termios <termios.h>
    struct termios options;

    /**1. tcgetattr函数用于获取与终端相关的参数。
    *参数fd为终端的文件描述符，返回的结果保存在termios结构体中
    */
    tcgetattr(serial_fd, &options);
    /**2. 修改所获得的参数*/
    options.c_cflag |= (CLOCAL | CREAD);//设置控制模式状态，本地连接，接收使能
    options.c_cflag &= ~CSIZE;//字符长度，设置数据位之前一定要屏掉这个位
    options.c_cflag &= ~CRTSCTS;//无硬件流控
    options.c_cflag |= CS8;//8位数据长度
    options.c_cflag &= ~CSTOPB;//1位停止位
    options.c_iflag |= IGNPAR;//无奇偶检验位
    options.c_oflag = 0; //输出模式
    options.c_iflag&=~(ICRNL|IXON);
    options.c_lflag = 0; //不激活终端模式
    cfsetospeed(&options, B115200);//设置波特率

    /**3. 设置新属性，TCSANOW：所有改变立即生效*/
    tcflush(serial_fd, TCIFLUSH);//溢出数据可以接收，但不读
    tcsetattr(serial_fd, TCSANOW, &options);
}
/**
*串口发送数据
*@fd:串口描述符
*@data:待发送数据
*@datalen:数据长度
*/
int Class_Usart::ff_Usart_Send(short head,int Yaw, int Pitch,bool Beat, bool NoObj, int Distance,int rotation,int swing)
{
    int len = 0;
    unsigned char buff[30];
    buff[0]=head;
    buff[1]=(static_cast<short>((int)Yaw))>>8;
    buff[2]=(static_cast<short>((int)Yaw))&0x00ff;
    buff[3]=(static_cast<short>((int)Pitch))>>8;
    buff[4]=(static_cast<short>((int)Pitch))&0x00ff;
    buff[5]=(static_cast<short>((int)Distance))>>8;
    buff[6]=(static_cast<short>((int)Distance))&0x00ff;
    buff[7]=static_cast<char>(Beat);
    buff[8]=static_cast<char>(NoObj);
    buff[9]=static_cast<char>(rotation);//地盘跟随;
    buff[10]=static_cast<char>(swing);//地盘摇摆;
    buff[11]=0;
    buff[12]=0;
    buff[13]=0;
    buff[14]=0;
   // buff[15]=(static_cast<short>((int)remain_blood))>>8;;
    //buff[16]=(static_cast<short>((int)remain_blood))&0x00ff;
//    buff[17]=0;
//    buff[18]=0;
//    buff[19]=0;
//    buff[20]=0;
//    buff[21]=0;
//    buff[22]=0;
//    buff[23]=0;
//    buff[24]=0;
//    buff[25]=0;
//    buff[26]=0;
//    buff[27]=0;        
//printf("%x,%x,%x,%x,%x,%x,%x,%x,%x;\n", buff[0], buff[1], buff[2],buff[3], buff[4], buff[5],buff[6], buff[7], buff[8]);
    Append_CRC16_Check_Sum(buff,total_send_length);
    len = write(Class_Usart::serial_fd, buff,total_send_length );//实际写入的长度
#ifdef DEBUG
    if(len == total_send_length)
    {

        printf("send finished total_send is %d\n",total_send_length);

    }
    else
    {
        tcflush(Class_Usart::serial_fd, TCOFLUSH);//TCOFLUSH刷新写入的数据但不传送
        return -1;
    }
#endif
    return 0;
}
int Class_Usart::fe_Usart_Send(short head,int linear_x, int linear_y,int angular_z,int add_num)
{
    int len = 0;
    unsigned char buff[30];
    buff[0]=head;
    buff[1]=(static_cast<short>((int)linear_x))>>8;
    buff[2]=(static_cast<short>((int)linear_x))&0x00ff;
    buff[3]=(static_cast<short>((int)linear_y))>>8;
    buff[4]=(static_cast<short>((int)linear_y))&0x00ff;
    buff[5]=(static_cast<short>((int)angular_z))>>8;
    buff[6]=(static_cast<short>((int)angular_z))&0x00ff;
    buff[7]=static_cast<char>(add_num);//申请补弹数
    buff[8]=0;//地盘跟随
    buff[9]=0;//地盘摇摆
    buff[10]=0;
    buff[11]=0;
    buff[12]=0;
    buff[13]=0;
    buff[14]=0;
   // buff[15]=(static_cast<short>((int)remain_blood))>>8;;
    //buff[16]=(static_cast<short>((int)remain_blood))&0x00ff;
//    buff[27]=0;        
//printf("%x,%x,%x,%x,%x,%x,%x,%x,%x;\n", buff[0], buff[1], buff[2],buff[3], buff[4], buff[5],buff[6], buff[7], buff[8]);
    Append_CRC16_Check_Sum(buff,total_send_length);
    len = write(Class_Usart::serial_fd, buff,total_send_length );//实际写入的长度
#ifdef DEBUG
    if(len == total_send_length)
    {

        printf("send finished total_send is %d\n",total_send_length);

    }
    else
    {
        tcflush(Class_Usart::serial_fd, TCOFLUSH);//TCOFLUSH刷新写入的数据但不传送
        return -1;
    }
#endif
    return 0;
}
int Class_Usart::bb_Usart_Send(short head,int my_position_x, int my_position_y,int enemy_position_x,int enemy_position_y,int bufflet,int bufflet_sign,int buff_sign)
{
    int len = 0;
    unsigned char buff[30];
    buff[0]=head;
    buff[1]=(static_cast<short>((int)my_position_x))>>8;
    buff[2]=(static_cast<short>((int)my_position_x))&0x00ff;
    buff[3]=(static_cast<short>((int)my_position_y))>>8;
    buff[4]=(static_cast<short>((int)my_position_y))&0x00ff;
    buff[5]=(static_cast<short>((int)enemy_position_x))>>8;
    buff[6]=(static_cast<short>((int)enemy_position_x))&0x00ff;
    buff[7]=static_cast<char>(bufflet_sign);//补弹标志为
    buff[8]=static_cast<char>(buff_sign);//加成标志为
    buff[9]=0;
    buff[10]=0;
    buff[11]=(static_cast<short>((int)enemy_position_y))>>8;
    buff[12]=(static_cast<short>((int)enemy_position_y))&0x00ff;
    buff[13]=(static_cast<short>((int)bufflet))>>8;;
    buff[14]=(static_cast<short>((int)bufflet))&0x00ff;
   // buff[15]=(static_cast<short>((int)remain_blood))>>8;;
    //buff[16]=(static_cast<short>((int)remain_blood))&0x00ff;
//    buff[27]=0;        
//printf("%x,%x,%x,%x,%x,%x,%x,%x,%x;\n", buff[0], buff[1], buff[2],buff[3], buff[4], buff[5],buff[6], buff[7], buff[8]);
    Append_CRC16_Check_Sum(buff,total_send_length);
    len = write(Class_Usart::serial_fd, buff,total_send_length );//实际写入的长度
#ifdef DEBUG
    if(len == total_send_length)
    {

        printf("send finished total_send is %d\n",total_send_length);

    }
    else
    {
        tcflush(Class_Usart::serial_fd, TCOFLUSH);//TCOFLUSH刷新写入的数据但不传送
        return -1;
    }
#endif
    return 0;
}
/**
*串口接收数据
*要求启动后，在pc端发送ascii文件
*/
int  Class_Usart::Usart_Recv(volatile int *data)
{
    //data.clear();
    short Yaw;short Pitch;short Chassis_Speed_x;short Chassis_Speed_y;
    unsigned char Receive_Data[18];//接受数据
    int  ret = 0;
// fd_set是一组文件描述字(fd)的集合，它用一位来表示一个fd（下面会仔细介绍）
    fd_set fs_read;
    struct timeval tv_timeout;
// FD_ZERO(fd_set *fdset);将指定的文件描述符集清空，在对文件描述符集合进行设置前，必须对其进行初始化，如果不清空，由于在系统分配内存空间后，通常并不作清空处理，所以结果是不可知的。
    FD_ZERO(&fs_read);
//FD_SET(fd_set *fdset);用于在文件描述符集合中增加一个新的文件描述符。 
    FD_SET(Class_Usart::serial_fd, &fs_read);
    //tv_timeout.tv_sec = (10 * 20 / 115200 + 2);
    tv_timeout.tv_sec =0;
    tv_timeout.tv_usec = 15000;
//TCIFLUSH输入缓冲区、TCOFLUSH输出缓冲区TCIOFLUSH输入输出缓冲区
   //tcflush(serial_fd, TCIFLUSH);//溢出数据可以接收，但不读
//nfds，一般设定为文件描述符+1,readset，用来检查可读性的一组文件描述字
    ret = select(Class_Usart::serial_fd + 1, &fs_read, NULL, NULL, &tv_timeout);
    //ret = select(Class_Usart::serial_fd + 1, &fs_read, NULL, NULL, NULL);
    //printf("ret = %d\n", ret);
#ifdef DEBUG
        printf("%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x;\n", Receive_Data[0], Receive_Data[1], Receive_Data[2],
               Receive_Data[3], Receive_Data[4], Receive_Data[5], Receive_Data[6],
               Receive_Data[7], Receive_Data[8], Receive_Data[9], Receive_Data[10],Receive_Data[11], Receive_Data[12],
	       Receive_Data[13], Receive_Data[14],Receive_Data[15],Receive_Data[16],Receive_Data[17]);
#endif      
    //如果返回0，代表在描述符状态改变前已超过timeout时间,错误返回-1
    if (FD_ISSET(Class_Usart::serial_fd, &fs_read)) {
        read(Class_Usart::serial_fd, Receive_Data, receive_length);
        struct timeval get_mes_time;
//gettimeofday()函数来得到时间。它的精度可以达到微妙
        gettimeofday(&get_mes_time,NULL);
 

           if (Verify_CRC16_Check_Sum(Receive_Data, receive_length)) {
    
 Yaw = (short)(Receive_Data[1] << 8 | Receive_Data[2]);
            Pitch = (short)(Receive_Data[3] << 8 | Receive_Data[4]);
            Chassis_Speed_x = (short)(Receive_Data[5] << 8 | Receive_Data[6]);
            Chassis_Speed_y = (short)(Receive_Data[7] << 8 | Receive_Data[8]);

//实际赋值,头帧，x轴速度，y轴速度
    data[0]=(int)Receive_Data[0];
    data[1]=(int)Yaw;
    data[2]=(int)Pitch;
    data[3]=(int)Chassis_Speed_x;
    data[4]=(int)Chassis_Speed_y;
    data[5]=(short)(Receive_Data[9] << 8 | Receive_Data[10]);
    data[6]=(short)(Receive_Data[11]<< 8 | Receive_Data[12]);
    data[7]=(short)(Receive_Data[13]);
    data[8]=(short)(Receive_Data[14]);
    data[9]=(short)(Receive_Data[15]);
    


            //printf("%f,%f,%f,%f;\n",Yaw/100.0,Pitch/100.0,Chassis_Speed_x/100.0,Chassis_Speed_y/100.0);
            // error1=0;
        } else {
            std::cout << "number data.error" << std::endl;
//printf("%d/n",999);
            // error1=1;

            return -1;
        }

        
/*
if (Receive_Data[0] == 0xaa &&Verify_CRC16_Check_Sum(Receive_Data, receive_length)) {
    
 Yaw = (short)(Receive_Data[1] << 8 | Receive_Data[2]);
            Pitch = (short)(Receive_Data[3] << 8 | Receive_Data[4]);
            Chassis_Speed_x = (short)(Receive_Data[5] << 8 | Receive_Data[6]);
            Chassis_Speed_y = (short)(Receive_Data[7] << 8 | Receive_Data[8]);
    data[0]=(int)Receive_Data[0];
    data[1]=(int)Yaw;
    data[2]=(int)Pitch;
    data[3]=(int)Chassis_Speed_x;
    data[4]=(int)Chassis_Speed_y;

            //printf("%f,%f,%f,%f;\n",Yaw/100.0,Pitch/100.0,Chassis_Speed_x/100.0,Chassis_Speed_y/100.0);
            // error1=0;
        } else {
            std::cout << "number data.error" << std::endl;
//printf("%d/n",999);
            // error1=1;

            return -1;
        }
*/
    } else {
        perror("select");
        std::cout << "select data.error" << std::endl;
        return -1;
    }


    //cout<<"iii "<<data[0]<<endl;

//    data[0]=Yaw;    data[1]=Pitch;    data[2]=Chassis_Speed_x;    data[3]=Chassis_Speed_y;
    //cout << "222 " <<  data[1] << endl;
    return 0;
}
