#include "../include/art_imu_01/serial_to_imu.h"
#include "math.h"
//#include <kalman.h>
#include <serial/serial.h>

#define pi 3.1415926			//π

serial_to_imu::serial_to_imu(string port,int baudrate)
{
	ROS_INFO("create class succesed");
	openPort(port,baudrate);
}

void serial_to_imu::IMUDataConversion(unsigned char *IMUData)
{
	//acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, roll, pitch, yaw, q0, q1, q2, q3;
	acc_x = ((IMUData[3] & 0xff) << 8) | (IMUData[4] & 0xff); 		//加速度 m/s^2
	acc_y = ((IMUData[5] & 0xff) << 8) | (IMUData[6] & 0xff);
	acc_z = ((IMUData[7] & 0xff) << 8) | (IMUData[8] & 0xff);
	gyro_x = ((IMUData[9] & 0xff) << 8) | (IMUData[10] & 0xff);  		//角速度
	gyro_y = ((IMUData[11] & 0xff) << 8) | (IMUData[12] & 0xff);
	gyro_z = ((IMUData[13] & 0xff) << 8) | (IMUData[14] & 0xff);
	roll = ((IMUData[15] & 0xff) << 8) | (IMUData[16] & 0xff);      //RPY
	pitch = ((IMUData[17] & 0xff) << 8) | (IMUData[18] & 0xff);
	yaw = ((IMUData[19] & 0xff) << 8) | (IMUData[20] & 0xff);
	q0 = ((IMUData[21] & 0xff) << 8) | (IMUData[22] & 0xff);			//四元数
	q1 = ((IMUData[23] & 0xff) << 8) | (IMUData[24] & 0xff);
	q2 = ((IMUData[25] & 0xff) << 8) | (IMUData[26] & 0xff);
	q3 = ((IMUData[27] & 0xff) << 8) | (IMUData[28] & 0xff);
	//q_x, q_y, q_z, q_w, ang_x, ang_y, ang_z, lin_x, lin_y, lin_z, R, P, Y;
	lin_x = acc_x / 1000.0;//m/s^2
	lin_y = acc_y / 1000.0;
	lin_z = acc_z / 1000.0;
	ang_x = gyro_x/10.0/180.0*pi;//rad/s
	ang_y = gyro_y/10.0/180.0*pi;
	ang_z = gyro_z/10.0/180.0*pi;
	q_w = q0 / 10000.0;
	q_x = q1 / 10000.0;
	q_y = q2 / 10000.0;
	q_z = q3 / 10000.0;
	R = -roll/100.0/180.0*pi;//unit:rad
	P = -pitch/100.0/180.0*pi;
	Y = -yaw/100.0/180.0*pi;
	// R = roll/100;//unit:DEGREE
	// P = pitch/100;
	// Y = yaw/100;

}
void serial_to_imu::ser_write()//int send, int init, int rate)
{
	unsigned char write_data[9];
	write_data[0]=0xA5;
	write_data[1]=0x0A;
	write_data[2]=0x01;
	write_data[3]=0x02;//send;
	write_data[4]=0x04;
	write_data[5]=0x01;//init;
	write_data[6]=0x00;//rate;
	write_data[7]=0x0D;
	write_data[8]=0x0A;
	ser.write(write_data,9);
	ser.write(write_data,9);
}
int serial_to_imu::openPort(string _port,int _baudrate)
{
		try 
  {
    ser.setPort(_port);
    ser.setBaudrate(_baudrate);
    // serial::Timeout to = serial::Timeout::simpleTimeout(10000);
    // ser.setTimeout(to);
		ser.setTimeout(10000,10000,10000,10000,10000);

    ser.open();
  } 
  catch (serial::IOException &e)
  {
    ROS_ERROR_STREAM("Unable to open port ");
    return -1;
  }
 
  if (ser.isOpen()) 
  {
    ROS_INFO_STREAM("Serial Port initialized");
		ROS_INFO("The serial port is %s",_port.c_str());
  	ROS_INFO("The baudrate is %d",_baudrate);
  } 
  else 
  {
    return -1;
  }
}
