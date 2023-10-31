#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <vector>
#include <string>
#include <geometry_msgs/Twist.h>
using namespace std;
using namespace ros;

static serial::Serial sp;


void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg )
{
    float vx=msg->linear.x;
    float vy=msg->linear.y;
    float gz=msg->angular.z;
    char data[32];
    sprintf(data,"V: %f %f %f",vx,vy,gz);//
    sp.write(data);

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "downstream");//初始化节点和名称
    ros::NodeHandle h;
    ros::Subscriber cmd_sub = h.subscribe("cmd_vel", 1, cmdCallback);

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
     

    ros::spin();
    return 0;
}