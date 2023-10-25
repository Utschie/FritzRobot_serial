#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <vector>
#include <string>

using namespace std;
using namespace ros;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;

    //创建一个serial对象
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyACM0");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //串口设置timeout
    sp.setTimeout(to);

    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyACM0 is opened.");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(200);
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t n = sp.available();
        if(n!=0)
        {
            uint8_t buffer[280];
            //读出数据
            n = sp.read(buffer, n);
            if (buffer[0]==97 && buffer[1]==99 && buffer[2]==99) {
                vector<uint8_t> vbuffer(buffer, buffer + sizeof(buffer));//array to vector for convenience
                string sAcc_x(vbuffer.begin() + 8, vbuffer.begin() + 17);
                string sAcc_y(vbuffer.begin() + 21, vbuffer.begin() + 30);
                string sAcc_z(vbuffer.begin() + 34, vbuffer.begin() + 43);
                string sOmega_x(vbuffer.begin() + 54, vbuffer.begin() + 63);
                string sOmega_y(vbuffer.begin() + 67, vbuffer.begin() + 76);
                string sOmega_z(vbuffer.begin() + 80, vbuffer.begin() + 89);
                string sQuaternion_w(vbuffer.begin() + 105, vbuffer.begin() + 114);
                string sQuaternion_x(vbuffer.begin() + 118, vbuffer.begin() + 127);
                string sQuaternion_y(vbuffer.begin() + 131, vbuffer.begin() + 140);
                string sQuaternion_z(vbuffer.begin() + 144, vbuffer.begin() + 153);
                string sWheelspeed_LF(vbuffer.begin() + 170, vbuffer.begin() + 179);
                string sWheelspeed_RF(vbuffer.begin() + 184, vbuffer.begin() + 193);
                string sWheelspeed_LB(vbuffer.begin() + 198, vbuffer.begin() + 207);
                string sWheelspeed_RB(vbuffer.begin() + 212, vbuffer.begin() + 221);
                string sVx(vbuffer.begin() + 229, vbuffer.begin() + 238);
                string sVy(vbuffer.begin() + 243, vbuffer.begin() + 252);
                string sVz(vbuffer.begin() + 257, vbuffer.begin() + 266);
                float fAcc_x = stof(sAcc_x);
                float fsAcc_y = stof(sAcc_y);
                float fAcc_z = stof(sAcc_z);
                float fOmega_x = stof(sOmega_x);
                float fOmega_y = stof(sOmega_y);
                float fOmega_z = stof(sOmega_z);
                float fQuaternion_w = stof(sQuaternion_w);
                float fQuaternion_x = stof(sQuaternion_x);
                float fQuaternion_y = stof(sQuaternion_y);
                float fQuaternion_z = stof(sQuaternion_z);
                float fWheelspeed_LF = stof(sWheelspeed_LF);
                float fWheelspeed_RF = stof(sWheelspeed_RF);
                float fWheelspeed_LB = stof(sWheelspeed_LB);
                float fWheelspeed_RB = stof(sWheelspeed_RB);
                float fVx = stof(sVx);
                float fVy = stof(sVy);
                float fVz = stof(sVz);
                std::cout <<std::fixed<<std::setprecision(6) << fAcc_x << " ";
            }



            /*
            for(int i=0; i<n; i++)
            {
                //16进制的方式打印到屏幕
                //std::cout << std::hex << (buffer[i] & 0xff) << " ";
                std::cout<<buffer[i];

            }
             */
            std::cout << n << " ";
            std::cout << std::endl;
        }
        loop_rate.sleep();
    }

    //关闭串口
    sp.close();

    return 0;
}
