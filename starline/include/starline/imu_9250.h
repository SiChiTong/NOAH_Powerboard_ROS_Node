#ifndef _IMU_9250_H
#define _IMU_9250_H
typedef union
{
  struct
  {
    //orientation
    float orientation_x;
    float orientation_y;
    float orientation_z;
    float orientation_w;
    //angular velocity
    float angularVel_x;
    float angularVel_y;
    float angularVel_z;
    //linear acceleration
    float linearAcc_x;
    float linearAcc_y;
    float linearAcc_z;
  }ImuDataStruct;
  unsigned char data[40];
}ImuDataUnion;

extern ImuDataUnion getImuData;
extern unsigned char ImuDataPubFlag;
extern void *GetImuDataThread(void *);

#endif
