#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/art_imu_01/serial_to_imu.h"
#include "art_imu_01/rpy.h"
#include <sensor_msgs/Imu.h>

//订阅IMU RPY数据并打印
void subscriberCallback(const art_imu_01::rpy::ConstPtr& msg)
{
printf("I heard : \n R=%f\n P=%f\n Y=%f\n",msg->R,msg->P,msg->Y);
}
int main(int argc, char **argv)
{
        ros::Subscriber subscriber;
        ros::init(argc, argv,"sub");
        ros::NodeHandle nd;
        subscriber = nd.subscribe("/rpy_data_01", 5, &subscriberCallback);
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
               loop_rate.sleep();
                ros::spinOnce();
        }

        return 0;
}
