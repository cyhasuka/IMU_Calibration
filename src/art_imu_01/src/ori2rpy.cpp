#include "tf/transform_datatypes.h"
#include "art_imu_01/rpy.h"
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <std_msgs/Empty.h>

using namespace std;
tf::Quaternion quat;

class ori2rpy
{
    public:
    ori2rpy()
    {
        sub = n.subscribe("imu_data", 1000, &ori2rpy::subCallback,this);
        pub = n.advertise<art_imu_01::rpy>("rpy_data",10);
    }
    void subCallback(const sensor_msgs::Imu& imudata)
    {
        tf::quaternionMsgToTF(imudata.orientation,quat);
        tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
        rpydata.R=roll*180/3.1415926;
        rpydata.P=pitch*180/3.1415926;
        rpydata.Y=yaw*180/3.1415926;
        pub.publish(rpydata);
    }
    private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    tf::Quaternion quat;
    
    double roll,pitch,yaw;
    art_imu_01::rpy rpydata;
};

int main(int argc,char **argv)
{
    ros::init(argc, argv, "rpy_pub");
    ori2rpy o;
    ros::spin();
    return 0;
}
