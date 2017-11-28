/* 
 *  ultrasonic.cpp 
 *  Communicate with ultrasonics.
 *  Author: Kaka Xie 
 *  Create Date:2017/11/28
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h" 
#include "pthread.h"
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
#include <signal.h>
#include <iostream>
#include <vector>
#include "stdlib.h"
#include "cstdlib"
#include "string"
#include "sstream"
#include <mrobot_driver_msgs/vci_can.h>
#include <roscan/can_long_frame.h>
#include <ultrasonic.h>

void test_fun(void * arg)
{

}


int Ultrasonic::start_measurement(uint8_t ul_id)     
{
    int error = 0; 
    if((ul_id == 0) || (ul_id > 16))
    {
        return -1;
    }
    mrobot_driver_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_START_MEASUREMENT;
    id.CanID_Struct.SrcMACID = 1;//CAN_SUB_PB_SRC_ID;
    id.CanID_Struct.DestMACID = ULTRASONIC_CAN_SRC_MAC_ID_BASE+ ul_id - 1;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 3;
    can_msg.Data.resize(3);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = 0;
    can_msg.Data[2] = 0;
    //ROS_INFO("send CAN data");
    this->pub_to_can_node.publish(can_msg);
    return error;
}



void pub_json_msg_to_app( const nlohmann::json j_msg)
{
    std_msgs::String pub_json_msg;
    std::stringstream ss;

    ss.clear();
    ss << j_msg;
    pub_json_msg.data = ss.str();
    //this->noah_powerboard_pub.publish(pub_json_msg);
}
bool Ultrasonic::is_ultrasonic_can_id(CAN_ID_UNION id)
{
    if((id.CanID_Struct.SrcMACID >= 0x60)&&(id.CanID_Struct.SrcMACID <= 0x6f))//?????????????????????????????
    {
        return true ;
    }
    return false;
}

uint8_t Ultrasonic::parse_ultrasonic_id(CAN_ID_UNION id)
{
    if((id.CanID_Struct.SrcMACID >= 0x60)&&(id.CanID_Struct.SrcMACID <= 0x6f))//?????????????????????????????
    {
        return id.CanID_Struct.SrcMACID - ULTRASONIC_CAN_SRC_MAC_ID_BASE + 1 ;
    }
    return 0;
}
void Ultrasonic::rcv_from_can_node_callback(const mrobot_driver_msgs::vci_can::ConstPtr &c_msg)
{
    mrobot_driver_msgs::vci_can can_msg;
    mrobot_driver_msgs::vci_can long_msg;
    CAN_ID_UNION id;
    uint8_t ul_id;

    long_msg = this->long_frame.frame_construct(c_msg);
    mrobot_driver_msgs::vci_can* msg = &long_msg;
    if( msg->ID == 0 ) 
    {
        return;
    }
    if(this->is_log_on == true)
    {
        for(uint8_t i = 0; i < msg->DataLen; i++)
        {
            ROS_INFO("msg->Data[%d] = 0x%x",i,msg->Data[i]);
        }
    }
    can_msg.ID = msg->ID;
    id.CANx_ID = can_msg.ID;
    can_msg.DataLen = msg->DataLen;
    //if(id.CanID_Struct.SrcMACID != 0)//?????????????????????????????
    if(this->is_ultrasonic_can_id(id) == false)
    {
        ROS_INFO("not ultrasonic CAN id");
        return ;
    }

    if((ul_id = this->parse_ultrasonic_id(id)) == 0)
    {
        ROS_INFO("ultrasonic CAN id parse error");
        return ; 
    }

        ROS_INFO("get distancemark ");
    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_START_MEASUREMENT)//??????????????????????????//
    {
        ROS_INFO("get distance ");
        if(id.CanID_Struct.ACK == 1)
        {
            this->distance[ul_id - 1] = *(uint16_t *)&msg->Data[1];
            ROS_INFO("ultrasonic id: %3d,  distance: %3d",ul_id, this->distance[ul_id]);
            
            do
            {
            }while(0);

        }
    }
   

}
