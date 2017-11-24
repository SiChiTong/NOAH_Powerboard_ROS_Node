#include "../include/starline/imu_9250.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"
//#include "../include/starline/move.h"

#include <sstream>
#include <math.h>
#include <stdio.h>     
#include <stdlib.h>     
#include <unistd.h>     
#include <sys/types.h>  
#include <sys/stat.h>   
#include <fcntl.h>      
#include <termios.h>   
#include <errno.h>     
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>

extern void sendImuCmd(void);

ImuDataUnion getImuData;
unsigned char ImuDataPubFlag = 0;

void pub_baseImuData(ros::Publisher &imuData_pub)
{
  sensor_msgs::Imu imuData;
  imuData.header.frame_id = "odom";
  imuData.orientation.x = getImuData.ImuDataStruct.orientation_x;
  imuData.orientation.y = getImuData.ImuDataStruct.orientation_y;
  imuData.orientation.z = getImuData.ImuDataStruct.orientation_z;
  imuData.orientation.w = getImuData.ImuDataStruct.orientation_w;

  imuData.angular_velocity.x = getImuData.ImuDataStruct.angularVel_x;
  imuData.angular_velocity.y = getImuData.ImuDataStruct.angularVel_y;
  imuData.angular_velocity.z = getImuData.ImuDataStruct.angularVel_z;

  imuData.linear_acceleration.x = getImuData.ImuDataStruct.linearAcc_x;
  imuData.linear_acceleration.y = getImuData.ImuDataStruct.linearAcc_y;
  imuData.linear_acceleration.z = getImuData.ImuDataStruct.linearAcc_z;
  imuData_pub.publish(imuData);
}

void *GetImuDataThread(void *)
{
  ros::NodeHandle imu_handle;
  //20171122,Zero,for IMU
  ros::Publisher Imudata_pub = imu_handle.advertise<sensor_msgs::Imu>("noahbaseImu_tx",1);
  ros::Rate loop_rate(100);
  while(ros::ok())
  {
    sendImuCmd();
    if(ImuDataPubFlag)
    {
      pub_baseImuData(Imudata_pub);
      ImuDataPubFlag = 0;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
