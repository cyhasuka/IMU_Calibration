#pragma once

#include <iostream>

///////////test
#include <serial/serial.h>
#include <ros/ros.h>

using namespace std;
class serial_to_imu
{
public:
    serial_to_imu(string port,int baudrate);
    serial::Serial ser;
    void ser_write();//int send, int init, int rate);
    void IMUDataConversion(unsigned char *IMUData);
    double q_x, q_y, q_z, q_w, ang_x, ang_y, ang_z, lin_x, lin_y, lin_z, R, P, Y;
    int openPort(string _port,int _baudrate);
// private:
	short acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, roll, pitch, yaw, q0, q1, q2, q3;
};

