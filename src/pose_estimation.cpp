#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <numeric>
#include <string>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <Eigen>
#include "ekf.h"
using namespace std;
using namespace ros;
using namespace Eigen;

static ros::Publisher pose_pub;
static ros::Subscriber imu_sub;
static EKF estimator(1.0,0.0,0.0,0.0);//initialize the orientation
static vector<float> g1,g2,g3;
static int ready_flag=0;
void ekfCallback(const sensor_msgs::Imu::ConstPtr& msg )
{


    float wx=msg->angular_velocity.x;
    float wy=msg->angular_velocity.y;
    float wz=msg->angular_velocity.z;
    float ax=msg->linear_acceleration.x;
    float ay=msg->linear_acceleration.y;
    float az=msg->linear_acceleration.z;

    if (g1.size()<300)
    {
       g1.push_back(ax);
       g2.push_back(ay);
       g3.push_back(az);
    }
    if(g1.size()==300)
    {
        float g1_sum= accumulate(g1.begin(),g1.end(),0.0);
        float g2_sum= accumulate(g2.begin(),g2.end(),0.0);
        float g3_sum= accumulate(g3.begin(),g3.end(),0.0);
        float gx=g1_sum/g1.size();
        float gy=g2_sum/g2.size();
        float gz=g3_sum/g3.size();
        estimator.setz(gx,gy,gz);
        ready_flag=1;
    }

    float dt = 0.02;
    //estimator.setz(ax,ay,az);

    if (ready_flag==1)
    {
        estimator.predict(wx,wy,wz,dt);
        estimator.update();
        float q0=estimator.state_vector(0);
        float q1=estimator.state_vector(1);
        float q2=estimator.state_vector(2);
        float q3=estimator.state_vector(3);
        //float q0=estimator.quat_est(0);
        //float q1=estimator.quat_est(1);
        //float q2=estimator.quat_est(2);
        //float q3=estimator.quat_est(3);
        sensor_msgs::Imu pub_msg;
        pub_msg.header.frame_id="base_link";
        pub_msg.orientation.w=q0;
        pub_msg.orientation.x=q1;
        pub_msg.orientation.y=q2;
        pub_msg.orientation.z=q3;
        //pub_msg.angular_velocity.x=wx;
        //pub_msg.angular_velocity.y=wy;
        //pub_msg.angular_velocity.z=wz;
        pose_pub.publish(pub_msg);
    }

    ros::Rate loop_rate(50);//设置节点的loop频率
    loop_rate.sleep();

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_estimation");//初始化节点和名称
    ros::NodeHandle h;
    pose_pub = h.advertise<sensor_msgs::Imu>("chassis/pose", 1);
    imu_sub = h.subscribe("chassis/imu", 1, ekfCallback);


    //while(ros::ok())
    //{
    //    ros::spinOnce();
    //}
    ros::spin();
    return 0;

}

