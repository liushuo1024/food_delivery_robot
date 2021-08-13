#include "robot_bringup/mbot_linux_serial.h"

using namespace std;
using namespace boost::asio;
//串口相关对象
boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/robot");
boost::system::error_code err;
/********************************************************
            串口发送接收相关常量、变量、共用体对象
********************************************************/
const unsigned char header[7] = {0x01, 0x10,0x20,0x88,0x00,0x02,0x04};

//发送左右轮速控制速度共用体,传感器的X，Z，Angle
union sendData
{
	short d;
	unsigned char data[2];
}leftVelSet,rightVelSet;

//接收数据（左轮速、右轮速、角度）共用体（-32767 - +32768）
union receiveData
{
	short d;
	unsigned char data[2];
}leftVelNow,rightVelNow,angleNow;

const double ROBOT_LENGTH = 385.00;  //mm
const double ROBOT_RADIUS = 192.50;  //两轮之间的半径长度mm
const double wheel_RADIUS = 69.5;  //车轮轮之间的半径长度mm
const double wheel_c = 436.46;  //车轮周长mm

/********************************************************
函数功能：串口参数初始化
入口参数：无
出口参数：
********************************************************/
void serialInit()
{
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));    
}
/********************************************************
函数功能：crc校验
入口参数：机器人线速度、角速度
出口参数：
********************************************************/
int modbus_crc(uint8_t buff[]){
    unsigned short tmp = 0xffff;
    unsigned short ret1 = 0;
  
    for(int n = 0; n < 11; n++){/*此处的11 -- 要校验的位数为11个*/
        tmp = buff[n] ^ tmp;
        for(int i = 0;i < 8;i++){  /*此处的8 -- 指每一个char类型又8bit，每bit都要处理*/
            if(tmp & 0x01){
                tmp = tmp >> 1;
                tmp = tmp ^ 0xa001;
            }   
            else{
                tmp = tmp >> 1;
            }   
        }   
    }   
    /*CRC校验后的值*/
    // printf("%X\n",tmp);
    /*将CRC校验的高低位对换位置*/
    ret1 = tmp >> 8;
    ret1 = ret1 | (tmp << 8); 
    // printf("ret: %X\n",ret1);
    return ret1;
}
void add_crc(uint8_t* buff){
    int crc;
    int crc_h,crc_l;
    crc = modbus_crc(buff);
    crc_h = crc >> 8;
    crc_l = crc & 0xFF;
    buff[11]=crc_h;
    buff[12]=crc_l;
}
/********************************************************
函数功能：使能电机
入口参数：
出口参数：
********************************************************/
bool motor_init()
{
    unsigned char buf[8] = {0x01,0x06,0x20,0x0E,0x00,0x08,0xE2,0x0F};
    unsigned char rec[8]={0};
    // 串口初始化连接
    boost::asio::write(sp, boost::asio::buffer(buf));
    read(sp,buffer(rec));		
    return true;
}
/********************************************************
函数功能：释放电机
入口参数：
出口参数：
********************************************************/
bool free_motor()
{
    //01 06 20 0E 00 07 A2 0B
    unsigned char buf[8] = {0x01,0x06,0x20,0x0E,0x00,0x07,0xA2,0x0B};
    unsigned char rec[8]={0};
    // 串口初始化连接
    boost::asio::write(sp, boost::asio::buffer(buf));
    read(sp,buffer(rec));		
    return true;
}

/********************************************************
函数功能：将机器人的线速度和角速度分解成左右轮子速度，打包发送给下位机
入口参数：机器人线速度、角速度
出口参数：
********************************************************/
void writeSpeed(double RobotV, double YawRate,unsigned char ctrlFlag)
{
    unsigned char buf[13] = {0};//
    unsigned char rec[8]={0};
    int i, length = 0;
    double r = RobotV / YawRate;//mm

    // 计算左右轮期望速度
    if(RobotV == 0)      //旋转
    {
        leftVelSet.d  = -(short)((-YawRate * ROBOT_RADIUS)*60/wheel_c);//RPM/min
        rightVelSet.d = (short)((YawRate * ROBOT_RADIUS)*60/wheel_c);//RPM/min
    } 
    else if(YawRate == 0)//直线
    {
        leftVelSet.d  = -(short)(RobotV*60*1000/wheel_c);//RPM/min
        rightVelSet.d = (short)(RobotV*60*1000/wheel_c);//RPM/min
        // std::cout<<"leftVelSet.d: "<<leftVelSet.d<<std::endl ;
        // std::cout<<"rightVelSet.d: "<<rightVelSet.d<<std::endl ;
    }
    else                //速度不一致
    {
        leftVelSet.d  = -(short)((RobotV-YawRate*(ROBOT_LENGTH/1000)/2)*60*1000/wheel_c);//RPM/min
        rightVelSet.d = (short)((RobotV+YawRate*(ROBOT_LENGTH/1000)/2)*60*1000/wheel_c);
        // leftVelSet.d  = -(short)((YawRate * (r - ROBOT_RADIUS)*60/wheel_c));//RPM/min
        // rightVelSet.d = (short)((YawRate * (r + ROBOT_RADIUS))*60/wheel_c);
    }

    // 设置消息头
    for(i = 0; i < 7; i++)
        buf[i] = header[i];             //buf[0] buf[1] buf[2] buf[3] buf[4] buf[5] buf[6]
    
    length = 5;
    // buf[2] = length;                    //buf[2]
    for(i = 0; i < 2; i++)
    {
        buf[i + 7] = leftVelSet.data[1-i];  //buf[7] buf[8]
        buf[i + 9] = rightVelSet.data[1-i]; //buf[9] buf[10]
    }
    add_crc(buf);
    ROS_INFO("write Left_v:%d\n",leftVelSet.d);
    ROS_INFO("write Right_v:%d\n",rightVelSet.d);
    boost::asio::write(sp, boost::asio::buffer(buf));// 通过串口下发数据
    read(sp,buffer(rec));
}
/********************************************************
函数功能：从下位机读取数据，解析出线速度、角速度、角度
入口参数：机器人线速度、角速度、角度，引用参数
出口参数：bool
********************************************************/
bool readSpeed(double &vx,double &vth,double &th,unsigned char &ctrlFlag)
{
    unsigned char search_speed[8] = {0x01,0x03,0x20,0xAB,0x00,0x02,0xBE,0x2B};//
    char i, length = 0;
    unsigned char checkSum;
    unsigned char buf[9]={0};
    //=========================================================
    //此段代码可以读数据的结尾，进而来进行读取数据的头部
    try
    {
        boost::asio::write(sp, boost::asio::buffer(search_speed));
        // std::cout<<"search_speed "<<std::endl ;
        boost::asio::streambuf response;
        read(sp,buffer(buf));
        // std::cout<<"get_speed "<<std::endl ; 
    }  
    catch(boost::system::system_error &err)
    {
        ROS_INFO("read_until error");
    } 
    //=========================================================        
    printf("read: %X %X %X %X %X %X %X %X %X\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8]);
    //检查信息头
    if (buf[0]!= search_speed[0] || buf[1] != search_speed[1])   //buf[0] buf[1]
    {
        ROS_ERROR("Received message header error!");
        return false;
    }

    // 读取速度值
    for(i = 0; i < 2; i++)
    {
        leftVelNow.data[i]  = buf[4-i]; //buf[3] buf[4]
        rightVelNow.data[i] = buf[6-i]; //buf[5] buf[6]
        // angleNow.data[i]    = buf[i + 7]; //buf[7] buf[8]
    }


    // 打印数据信息
    
    ROS_INFO("read Left_v:%d\n",leftVelNow.d);
    ROS_INFO("read Right_v:%d\n",rightVelNow.d);
    // ROS_INFO("Angle:%d\n",angleNow.d);
    // ROS_INFO("crtlFlag:%d\n",ctrlFlag);
  
    //===========================速度计算和Angle获取===========================================================
    // x方向速度，以及角速度
    vx  = (rightVelNow.d/10*wheel_c + (-leftVelNow.d/10*wheel_c))/2.0 /60/1000.0;        // m/s
    vth = (rightVelNow.d/10*wheel_c - (-leftVelNow.d/10*wheel_c))/ROBOT_LENGTH /60 ;       //rad/s
    //th  = angleNow.d*0.01745;//实时角度信息(rad)

    return true;
}

/********************************************************
函数功能：获得8位循环冗余校验值
入口参数：数组地址、长度
出口参数：校验值
********************************************************/

unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while(len--)
    {
        crc ^= *ptr++;
        for(i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc=(crc>>1)^0x8C;
            else 
                crc >>= 1;
        }
    }
    return crc;
}
