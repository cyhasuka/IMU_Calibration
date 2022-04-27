#include <ros/ros.h>

#include "imu_calib/do_calib.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "do_calib");

  imu_calib::DoCalib calib;
  while (ros::ok() && calib.running())
  {
    ros::spinOnce();
  }

  return 0;
}
