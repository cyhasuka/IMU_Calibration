#include <ros/ros.h>

#include "imu_calib/apply_calib.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "apply_calib");

  imu_calib::ApplyCalib calib;
  ros::spin();

  return 0;
}
