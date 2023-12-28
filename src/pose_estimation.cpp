#include <ros/ros.h>
#include <vector>
#include <numeric>
#include <sensor_msgs/Imu.h>
#include <Eigen>
#include "ekf.h"
using namespace std;
using namespace ros;
using namespace Eigen;

ros::Publisher* pose_pub= nullptr;
ros::Subscriber* imu_sub= nullptr;
ros::Time* current_time= nullptr;
ros::Time* last_time= nullptr;
static EKF estimator(1.0,0.0,0.0,0.0);//initialize the orientation
static vector<double> g1,g2,g3;
static int ready_flag=0;
void ekfCallback(const sensor_msgs::Imu::ConstPtr& msg )
{

    *current_time = ros::Time::now();
    double wx=msg->angular_velocity.x;
    double wy=msg->angular_velocity.y;
    double wz=msg->angular_velocity.z;
    double ax=msg->linear_acceleration.x;
    double ay=msg->linear_acceleration.y;
    double az=msg->linear_acceleration.z;

    if (g1.size()<300)
    {
       g1.push_back(ax);
       g2.push_back(ay);
       g3.push_back(az);
    }
    if(g1.size()==300)
    {
        double g1_sum= accumulate(g1.begin(),g1.end(),0.0);
        double g2_sum= accumulate(g2.begin(),g2.end(),0.0);
        double g3_sum= accumulate(g3.begin(),g3.end(),0.0);
        double gx=g1_sum/g1.size();
        double gy=g2_sum/g2.size();
        double gz=g3_sum/g3.size();
        estimator.setz(gx,gy,gz);
        ready_flag=1;
    }

    double dt = (*current_time - *last_time).toSec();
    //estimator.setz(ax,ay,az);

    if (ready_flag==1)
    {
        estimator.predict(wx,wy,wz,dt);
        estimator.update();
        double q0=estimator.state_vector(0);
        double q1=estimator.state_vector(1);
        double q2=estimator.state_vector(2);
        double q3=estimator.state_vector(3);
        //double q0=estimator.quat_est(0);
        //double q1=estimator.quat_est(1);
        //double q2=estimator.quat_est(2);
        //double q3=estimator.quat_est(3);
        sensor_msgs::Imu pub_msg;
        pub_msg.header.frame_id="base_link";
        pub_msg.orientation.w=q0;
        pub_msg.orientation.x=q1;
        pub_msg.orientation.y=q2;
        pub_msg.orientation.z=q3;
        //pub_msg.angular_velocity.x=wx;
        //pub_msg.angular_velocity.y=wy;
        //pub_msg.angular_velocity.z=wz;
        pose_pub->publish(pub_msg);
    }
    *last_time = *current_time;
    ros::Rate loop_rate(50);//设置节点的loop频率
    loop_rate.sleep();

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_estimation");//初始化节点和名称
    ros::NodeHandle h;
    pose_pub=new ros::Publisher;
    imu_sub=new ros::Subscriber;
    *pose_pub = h.advertise<sensor_msgs::Imu>("chassis/pose", 1);
    *imu_sub = h.subscribe("chassis/imu", 1, ekfCallback);
    *current_time= ros::Time::now();
    *last_time= ros::Time::now();
    //while(ros::ok())
    //{
    //    ros::spinOnce();
    //}
    ros::spin();
    delete pose_pub;
    delete imu_sub;
    return 0;

}

