/*
art_imu version:1
date:2021.10

send_mode: 1:angle   2:full   3:raw   4:quaternion
init_imu : 1:init  0:Do not init
serial_rate: 1:200Hz 2:100Hz 3:50Hz 4:20Hz
*/

#include "../include/art_imu_01/serial_to_imu.h"
#include "tf/transform_datatypes.h"
#include "art_imu_01/rpy.h"
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include "std_msgs/String.h"
//#include <sensor_msgs/Imu.h>
#include <art_msgs/Imu.h>
#include <tf/tf.h>
#include <std_msgs/Empty.h>
#include <time.h>
using namespace std;

clock_t Corr_start,Corr_end;
bool imu_info = true;
double r_corr,p_corr,y_corr;

uint8_t buffer[35];

void write_callback(const std_msgs::String::ConstPtr& msg) 
{ 
    ROS_INFO_STREAM("Writing to serial port" <<msg->data); 
} 

int main(int argc, char **argv) 
{
  unsigned char Imu_Data[31];
  unsigned char Write_Data[9];
  ros::init(argc, argv, "imu_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
//ros计时器
  ros::init(argc, argv, "RecordTime");
  Corr_start = ros::Time::now().toNSec();
  std::string serial_port_;
  std::string frame_id;
  int baudrate_, serial_rate_, init_imu_, send_mode_,imu_rate_;
  nh_private.param<std::string>("serial_port", serial_port_, "/dev/ttyUSB0");
  //nh_private.param<int>("serial_rate",serial_rate_,1);
  nh_private.param<std::string>("frame_id", frame_id, "imu_link");
  //nh_private.param<int>("init_imu",init_imu_,1);
  //nh_private.param<int>("send_mode",send_mode_,2);
  nh_private.param<int>("baudrate",baudrate_,500000);
  nh_private.param<int>("imu_rate",imu_rate_,50); //50

  ros::Subscriber IMU_write_pub = nh.subscribe("imu_command", 10, write_callback); 
  serial_to_imu rsc(serial_port_,baudrate_);

  rsc.ser_write();//send_mode_,init_imu_,serial_rate_);

  // imu rate;
  ros::Rate loop_rate(imu_rate_);
  ROS_INFO("The rate of imu is %d",imu_rate_);
  int number=1;
  geometry_msgs::Quaternion q;

  //ros::Publisher Imu_Data_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);//10
  ros::Publisher Imu_Data_pub = nh.advertise<art_msgs::Imu>("imu_data", 10);//10
  ros::Publisher rpy_pub = nh.advertise<art_imu_01::rpy>("rpy_data_01",10);

  while (ros::ok()) 
  {
      double Corr_time = (Corr_end - Corr_start) / 10e8;
      size_t n = rsc.ser.available();
      n = rsc.ser.read(buffer, n);

      for(int i=0; i<35; i++)
      {
        if(buffer[i]==0xA5&&buffer[i+1]==0x0A&&buffer[i+2]==0x01&&buffer[i+29]==0x0D&&buffer[i+30]==0x0A)
        {
          //cout<<"ok  ";
          for(int j=0;j<31;j++)
          {
            Imu_Data[j]=buffer[i+j];
          }
        }
        else {continue;}
      }

      rsc.IMUDataConversion(Imu_Data);
      
      //sensor_msgs::Imu imu_data;
      art_msgs::Imu imu_data;
      imu_data.header.stamp = ros::Time::now();
      imu_data.header.frame_id = frame_id;
      imu_data.header.seq = number;

      imu_data.angular_velocity.x = rsc.ang_x;
      imu_data.angular_velocity.y = rsc.ang_y;
      imu_data.angular_velocity.z = rsc.ang_z;
      imu_data.linear_acceleration.x = rsc.lin_x;
      imu_data.linear_acceleration.y = rsc.lin_y;
      imu_data.linear_acceleration.z = rsc.lin_z;

      imu_data.rpy.x = rsc.R*180.0/3.1415926;
      imu_data.rpy.y = rsc.P*180.0/3.1415926;
      imu_data.rpy.z = rsc.Y*180.0/3.1415926;

      art_imu_01::rpy rpy_data;
      rpy_data.R=rsc.R*180.0/3.1415926 - r_corr;
      rpy_data.P=rsc.P*180.0/3.1415926 - p_corr;
      rpy_data.Y=rsc.Y*180.0/3.1415926 - y_corr;

      //等待IMU稳定计时器
      Corr_end = ros::Time::now().toNSec();
      //cout << "The run time is:" << Corr_time  << "s" << endl; //debug:打印运行时间
      ROS_INFO_STREAM_ONCE("Calibrating , DO NOT MOVE THE IMU !");
      ROS_INFO_STREAM_ONCE("Estimated calibration time: 17.00s");
      //发布IMU 数据话题
      if (Corr_time >= 16 && imu_info)
        {
            r_corr = rsc.R*180.0/3.1415926;
            p_corr = rsc.P*180.0/3.1415926;
            y_corr = rsc.Y*180.0/3.1415926;
            ROS_INFO("RPY bias :  [%.3f, %.3f, %.3f]", r_corr, p_corr, y_corr);
            imu_info = false;
        }
        //四元数（小车用,仅z/w）
        q = tf::createQuaternionMsgFromYaw(rsc.Y);
        imu_data.orientation.w=q.w;
        imu_data.orientation.x=q.x;
        imu_data.orientation.y=q.y;
        imu_data.orientation.z=q.z;
        // //四元数（无人机用）
        // q = tf::createQuaternionMsgFromRollPitchYaw(rsc.R,rsc.P,rsc.Y);
        // imu_data.orientation.w=rsc.q_w;
        // imu_data.orientation.x=rsc.q_x;
        // imu_data.orientation.y=rsc.q_y;
        // imu_data.orientation.z=rsc.q_z;
      Imu_Data_pub.publish(imu_data);
      rpy_pub.publish(rpy_data);

      number++;
      loop_rate.sleep();
  }
  rsc.ser.close();
  return 0;
}
