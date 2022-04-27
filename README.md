# IMU_Calibration

使用前请首先使用imu_calib/do_calib标定IMU  

-----
### 功能包简介：
art_imu_01: IMU主功能包     
art_msgs: msg文件包    
imu_calib: IMU数据校准功能包   
seriel: 串口功能包   

-----
## 使用方法：
### 1、IMU初始标定(使用环境、IMU未出现较大变动时仅需标定一次)  
    roslaunch art_imu_01 demo.launch
    rosrun imu_calib do_calib
Notes:
    标定时请严格遵循终端提示，将机器人(IMU)以水平面为基准，前->后->右->左->上->下顺序翻转，每翻转一次按下ENTER确认。按下确认后直到下一次提示请勿晃动机器人(IMU)，否则校准结果可能出现误差。

### 2、IMU标定效果Rviz演示、标定前后话题发布数据对比：
    roslaunch art_imu_01 imu_rviz.launch    
    rostopic echo /imu_data
    rostopic echo /imu/data_raw
Notes：/imu_data 为未修正原始数据，/imu/data_raw为修正后数据。

### 3、单独启动IMU：
    roslaunch art_imu_01 demo.launch
