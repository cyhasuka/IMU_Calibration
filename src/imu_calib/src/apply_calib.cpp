/**
 * \file apply_calib.cpp
  * \Original author Daniel Koch <daniel.p.koch@gmail.com>
 *  \Editor cyh <cyhasuka@gmail.com>
 * \Last edit date:2022.4.25
 * IMU 数据计算校准程序
 */
#include <ros/ros.h>
#include <iostream>
#include "std_msgs/String.h"
#include <std_msgs/Empty.h>
#include "imu_calib/apply_calib.h"
#include "tf/transform_datatypes.h"
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <time.h>

clock_t Corr_start,Corr_end;
double R_corr,P_corr,Y_corr,x_corr,y_corr,z_corr,w_corr;
bool ori_flag = true;
#define pi 3.1415926

namespace imu_calib
{

ApplyCalib::ApplyCalib() :
  gyro_sample_count_(0),
  gyro_bias_x_(0.0),
  gyro_bias_y_(0.0),
  gyro_bias_z_(0.0),
  acc_bias_x_(0.0),
  acc_bias_y_(0.0),
  acc_bias_z_(0.0)
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
//ros计时器
  Corr_start = ros::Time::now().toNSec();

  std::string calib_file;
  nh_private.param<std::string>("calib_file", calib_file, "imu_calib.yaml");
  if (!calib_.loadCalib(calib_file) || !calib_.calibReady())
  {
    ROS_FATAL("Calibration could not be loaded");
    ros::shutdown();
  }

  nh_private.param<bool>("calibrate_gyros", calibrate_gyros_, true);
  nh_private.param<int>("gyro_calib_samples", gyro_calib_samples_, 100);

  int queue_size;
  nh_private.param<int>("queue_size", queue_size, 5);

  raw_sub_ = nh.subscribe("imu_data", queue_size, &ApplyCalib::rawImuCallback, this); //raw_imu
  corrected_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data_raw", queue_size);
  mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("imu/mag", queue_size);
}

void ApplyCalib::rawImuCallback(art_msgs::Imu::ConstPtr raw)
{
     //等待IMU稳定计时器
    Corr_end = ros::Time::now().toNSec();
    double Corr_time = (Corr_end - Corr_start) / 10e8;

    sensor_msgs::Imu corrected;
    tf::Quaternion quat;
    
    corrected.header.stamp = ros::Time::now();
    corrected.header.frame_id = "IMU_link";

    if (calibrate_gyros_)
    {
      // 递归计算平均测量值
      gyro_sample_count_++;
      acc_bias_x_ = ((gyro_sample_count_ - 1) * acc_bias_x_ + raw->linear_acceleration.x) / gyro_sample_count_;
      acc_bias_y_ = ((gyro_sample_count_ - 1) * acc_bias_y_ + raw->linear_acceleration.y) / gyro_sample_count_;
      acc_bias_z_ = ((gyro_sample_count_ - 1) * acc_bias_z_ + raw->linear_acceleration.z) / gyro_sample_count_;

      if (gyro_sample_count_ >= gyro_calib_samples_)
      {
        ROS_INFO("Acc bias : [%.3f, %.3f, %.3f]", acc_bias_x_, acc_bias_y_, acc_bias_z_);
        calibrate_gyros_ = false;
      }
        return;
    }
    double raw_gyro[3];
    double corrected_gyro[3];
    raw_gyro[0] = raw->angular_velocity.x;
    raw_gyro[1] = raw->angular_velocity.y;
    raw_gyro[2] = raw->angular_velocity.z;
    calib_.applyCalib(raw_gyro, corrected_gyro);

    double raw_ori[4];
    double corrected_ori[4];
    raw_ori[0] = raw->orientation.x;
    raw_ori[1] = raw->orientation.y;
    raw_ori[2] = raw->orientation.z;
    raw_ori[3] = raw->orientation.w;
    calib_.applyCalib(raw_ori, corrected_ori);

    //不校准gyro
    corrected.angular_velocity.x = corrected_gyro[0];
    corrected.angular_velocity.y = corrected_gyro[1];
    corrected.angular_velocity.z = corrected_gyro[2];
    //不校准四元数ori
    corrected.orientation.x = raw_ori[0];
    corrected.orientation.y = raw_ori[1];
    corrected.orientation.z = raw_ori[2];
    corrected.orientation.w = raw_ori[3];

    if (Corr_time >= 17 && ori_flag)
    { 
        // x_corr = raw_ori[0];
        // y_corr = raw_ori[1];
        //z_corr = raw_ori[2];
        //w_corr = 1 - raw_ori[3];
        //ROS_INFO("Ori bias : [%.3f, %.3f, %.3f, %.3f]", x_corr, y_corr, z_corr,w_corr);
        ROS_INFO_STREAM_ONCE("All Calibration Completed. ");
        ori_flag = false;
    }
    //自动校准部分
    corrected.linear_acceleration.x = raw->linear_acceleration.x-acc_bias_x_;
    corrected.linear_acceleration.y = raw->linear_acceleration.y-acc_bias_y_;
    corrected.linear_acceleration.z = raw->linear_acceleration.z-acc_bias_z_ + 9.80665;
    //发布校准后IMU数据
    corrected_pub_.publish(corrected);

    //磁力计转换（备用） (miligauss to tesla)
    // sensor_msgs::MagneticField mag_msg;
    // mag_msg.header.stamp = ros::Time::now();
    // mag_msg.magnetic_field.x = raw->magnetic_field.x * 0.0000001;
    // mag_msg.magnetic_field.y = raw->magnetic_field.y * 0.0000001;
    // mag_msg.magnetic_field.z = raw->magnetic_field.z * 0.0000001;
    // mag_pub_.publish(mag_msg);

    //RPY转四元数（有奇点，备用）
    // double raw_ori[4]; 
    // double corrected_ori[4];
    // geometry_msgs::Quaternion q;
    // q = tf::createQuaternionMsgFromRollPitchYaw(raw_rpy[0],raw_rpy[1],raw_rpy[2]);
    // raw_ori[0] = q.x;
    // raw_ori[1] = q.y;
    // raw_ori[2] = q.z;
    // raw_ori[3] = q.w;
    // calib_.applyCalib(raw_ori, corrected_ori);
}
} // namespace accel_calib
