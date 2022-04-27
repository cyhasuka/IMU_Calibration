/**
 * \file do_calib.cpp
 * \Original author Daniel Koch <daniel.p.koch@gmail.com>
 * \Editor cyh <cyhasuka@gmail.com>
 *\Last edit date:2022.4.25
 * IMU初始校准程序
 */

#include "imu_calib/do_calib.h"

namespace imu_calib
{

DoCalib::DoCalib() :
  state_(START)
{
  ros::NodeHandle nh;
  imu_sub_ = nh.subscribe("imu_data", 1, &DoCalib::imuCallback, this); //raw_imu

  ros::NodeHandle nh_private("~");
  nh_private.param<int>("measurements", measurements_per_orientation_, 500);
  nh_private.param<double>("reference_acceleration", reference_acceleration_, 9.80665);
  nh_private.param<std::string>("output_file", output_file_, "imu_calib.yaml");

  orientations_.push(AccelCalib::XPOS);
  orientations_.push(AccelCalib::XNEG);
  orientations_.push(AccelCalib::YPOS);
  orientations_.push(AccelCalib::YNEG);
  orientations_.push(AccelCalib::ZPOS);
  orientations_.push(AccelCalib::ZNEG);

  orientation_labels_[AccelCalib::XPOS] = "X+ axis - Front side of the robot";
  orientation_labels_[AccelCalib::XNEG] = "X- axis - Rear side of the robot" ;
  orientation_labels_[AccelCalib::YPOS] = "Y+ axis - Right side of the robot";
  orientation_labels_[AccelCalib::YNEG] = "Y- axis - Left side of the robot";
  orientation_labels_[AccelCalib::ZPOS] = "Z+ axis - Top side of the robot";
  orientation_labels_[AccelCalib::ZNEG] = "Z- axis - Bottom side of the robot";
}

bool DoCalib::running()
{
  return state_ != DONE;
}

void DoCalib::imuCallback(art_msgs::Imu::ConstPtr imu)
{
  bool accepted;

  switch (state_)
  {
  case START:
    calib_.beginCalib(6*measurements_per_orientation_, reference_acceleration_);
    state_ = SWITCHING;
    break;


  case SWITCHING:
    if (orientations_.empty())
    {
      state_ = COMPUTING;
    }
    else
    {
      current_orientation_ = orientations_.front();

      orientations_.pop();
      measurements_received_ = 0;

      std::cout << "Orient IMU with " << orientation_labels_[current_orientation_] << " facing up. Press [ENTER] once done.";
      std::cin.ignore();
      std::cout << "Calibrating! This may take a while...." << std::endl;

      state_ = RECEIVING;
    }
    break;


  case RECEIVING:
    accepted = calib_.addMeasurement(current_orientation_,
                                     imu->linear_acceleration.x,
                                     imu->linear_acceleration.y,
                                     imu->linear_acceleration.z);

    measurements_received_ += accepted ? 1 : 0;
    if (measurements_received_ >= measurements_per_orientation_)
    {
      std::cout << " Done." << std::endl;
      state_ = SWITCHING;
    }
    break;


  case COMPUTING:
    std::cout << "Computing calibration parameters...";
    if (calib_.computeCalib())
    {
      std::cout << " Success!"  << std::endl;

      std::cout << "Saving calibration file...";
      if (calib_.saveCalib(output_file_))
      {
        std::cout << " Success!" << std::endl;
      }
      else
      {
        std::cout << " Failed." << std::endl;
      }
    }
    else
    {
      std::cout << " Failed.";
      ROS_ERROR("Calibration failed");
    }
    state_ = DONE;
    break;


  case DONE:
    break;
  }
}

} // namespace accel_calib
