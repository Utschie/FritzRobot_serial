#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <vector>
#include <string>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include "fritzrobot_serial/Wheelspeed.h"
using namespace std;
using namespace ros;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_driver");//初始化节点和名称
    ros::NodeHandle h;
    ros::Publisher imu_pub = h.advertise<sensor_msgs::Imu>("chassis/imu",1);
    ros::Publisher wheelspeed_pub=h.advertise<fritzrobot_serial::Wheelspeed>("chassis/wheelspeed",1);
    ros::Publisher vel_pub=h.advertise<geometry_msgs::TwistStamped>("chassis/vel",1);



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

    ros::Rate loop_rate(200);//设置节点的loop频率，这里是一秒转200次
    ros::Time current_time;
    while(ros::ok())
    {
        //获取缓冲区内的字节数

        current_time = ros::Time::now();
        size_t n = sp.available();
        if(n!=0)
        {
            uint8_t buffer[256];
            //读出数据
            if (n==203) {
                sp.read(buffer, n);
                if (buffer[0] == 97 && buffer[1] == 99 && buffer[2] == 99) {
                    vector<uint8_t> vbuffer(buffer, buffer + sizeof(buffer));//array to vector for convenience
                    string sAcc_x(vbuffer.begin() + 8, vbuffer.begin() + 17);
                    string sAcc_y(vbuffer.begin() + 21, vbuffer.begin() + 30);
                    string sAcc_z(vbuffer.begin() + 34, vbuffer.begin() + 43);
                    string sOmega_x(vbuffer.begin() + 54, vbuffer.begin() + 63);
                    string sOmega_y(vbuffer.begin() + 67, vbuffer.begin() + 76);
                    string sOmega_z(vbuffer.begin() + 80, vbuffer.begin() + 89);
                    string sWheelspeed_LF(vbuffer.begin() + 106, vbuffer.begin() + 115);
                    string sWheelspeed_RF(vbuffer.begin() + 120, vbuffer.begin() + 129);
                    string sWheelspeed_LB(vbuffer.begin() + 134, vbuffer.begin() + 143);
                    string sWheelspeed_RB(vbuffer.begin() + 148, vbuffer.begin() + 157);
                    string sVx(vbuffer.begin() + 165, vbuffer.begin() + 174);
                    string sVy(vbuffer.begin() + 179, vbuffer.begin() + 188);
                    string sVz(vbuffer.begin() + 193, vbuffer.begin() + 202);
                    float fAcc_x = stof(sAcc_x);
                    float fAcc_y = stof(sAcc_y);
                    float fAcc_z = stof(sAcc_z);
                    float fOmega_x = stof(sOmega_x);
                    float fOmega_y = stof(sOmega_y);
                    float fOmega_z = stof(sOmega_z);
                    float fWheelspeed_LF = stof(sWheelspeed_LF);
                    float fWheelspeed_RF = stof(sWheelspeed_RF);
                    float fWheelspeed_LB = stof(sWheelspeed_LB);
                    float fWheelspeed_RB = stof(sWheelspeed_RB);
                    float fVx = stof(sVx);
                    float fVy = stof(sVy);
                    float fVz = stof(sVz);
                    sensor_msgs::Imu msg_imu;
                    msg_imu.header.stamp=current_time;
                    msg_imu.header.frame_id="imu";
                    msg_imu.angular_velocity.x = fOmega_x;
                    msg_imu.angular_velocity.y = fOmega_y;
                    msg_imu.angular_velocity.z = fOmega_z;
                    msg_imu.linear_acceleration.x = fAcc_x;
                    msg_imu.linear_acceleration.y = fAcc_y;
                    msg_imu.linear_acceleration.z = fAcc_z;
                    geometry_msgs::TwistStamped msg_vel;
                    msg_vel.header.stamp=current_time;
                    msg_vel.twist.linear.x = fVx;
                    msg_vel.twist.linear.y = fVy;
                    msg_vel.twist.angular.z = fVz;
                    fritzrobot_serial::Wheelspeed msg_wl;
                    msg_wl.header.stamp=current_time;
                    msg_wl.vLF = fWheelspeed_LF;
                    msg_wl.vRF = fWheelspeed_RF;
                    msg_wl.vLB = fWheelspeed_LB;
                    msg_wl.vRB = fWheelspeed_RB;
                    imu_pub.publish(msg_imu);
                    vel_pub.publish(msg_vel);
                    wheelspeed_pub.publish(msg_wl);

                    //std::cout <<std::fixed<<std::setprecision(6) << fAcc_x << " ";
                }
            }else
            {
                sp.flushInput();//如果缓冲区的数据不对则清空缓冲区，避免出现段错误
            }
            //std::cout << n << " ";
            //std::cout << std::endl;
        }
        loop_rate.sleep();
    }

    //关闭串口
    sp.close();

    return 0;
}
