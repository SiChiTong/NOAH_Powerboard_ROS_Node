/* 
 *  micro_laser.cpp 
 *  Communicate with micro lasers.
 *  Author: Kaka Xie 
 *  Date:2017/11/30
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
#include <laser.h>
#include <common.h>

void Laser::update_status(void)
{
    for(uint8_t i = 0; i < LASER_NUM_MAX; i++)
    {
        //ros::Duration duration = ros::Time::now() - start_measure_time[i];
        //if(duration.toSec() > 2)
        if(ros::Time::now() - start_measure_time[i] >= ros::Duration(2))
        {
            this->err_status[i] = LASER_ERR_COMMUNICATE_TIME_OUT;
            this->distance[i] = LASER_DISTANCE_ERR_TIME_OUT;
        }
    }
}

int Laser::start_measurement(uint8_t laser_id)     
{
    int error = 0; 
    if(laser_id > 15)
    {
        return -1;
    }
    mrobot_driver_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = LASER_CAN_SOURCE_ID_START_MEASUREMENT;//
    id.CanID_Struct.SrcMACID = 1;
    id.CanID_Struct.DestMACID = laser_id + LASER_CAN_SRC_MAC_ID_BASE;//
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = 0;
    this->pub_to_can_node.publish(can_msg);
    return error;
}


#if 0
void pub_json_msg_to_app( const nlohmann::json j_msg)
{
    std_msgs::String pub_json_msg;
    std::stringstream ss;

    ss.clear();
    ss << j_msg;
    pub_json_msg.data = ss.str();
    //this->noah_powerboard_pub.publish(pub_json_msg);
}
#endif

bool Laser::is_laser_can_id(CAN_ID_UNION id)
{
    if((id.CanID_Struct.SrcMACID >= 0x70)&&(id.CanID_Struct.SrcMACID <= 0x7f))
    {
        return true ;
    }
    return false;
}


#define NOT_LASER_ID      0xff
uint8_t Laser::parse_laser_id(CAN_ID_UNION id)
{
    if((id.CanID_Struct.SrcMACID >= 0x60)&&(id.CanID_Struct.SrcMACID <= 0x6f))
    {
        return id.CanID_Struct.SrcMACID - LASER_CAN_SRC_MAC_ID_BASE;
    }
    return NOT_LASER_ID;
}


void Laser::rcv_from_can_node_callback(const mrobot_driver_msgs::vci_can::ConstPtr &c_msg)
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

    if(this->is_laser_can_id(id) == false)
    {
        ROS_ERROR("not laser CAN id");
        return ;
    }

    if((ul_id = this->parse_laser_id(id)) == NOT_LASER_ID)
    {
        ROS_ERROR("laser CAN id not right");
        return ; 
    }

    if(id.CanID_Struct.SourceID == LASER_CAN_SOURCE_ID_START_MEASUREMENT)
    {
        if(id.CanID_Struct.ACK == 1)
        {
            this->start_measure_time[ul_id] = ros::Time::now();
            this->distance[ul_id] = msg->Data[0];
            this->distance[ul_id] += msg->Data[1]<<8;
            //if(this->is_log_on == true)
            {
                printf("\n");
                printf("\n");
                printf("\n");
                printf("laser:\n");
                for(uint8_t i = 0; i < LASER_NUM_MAX; i++)
                {
                    printf("%4d ",i + 1);
                }
                printf("\n");
                for(uint8_t i = 0; i <LASER_NUM_MAX; i++)
                {
                    printf("%4d ",this->distance[i]);
                }
                printf("\n");
            }
        }
    }
}

void Laser::pub_laser_data_to_navigation(uint16_t * ul_data)
{
    uint32_t en_laser = laser_en;
    static bool close_all_flag = 0;
    this->laser_data.header.stamp = ros::Time::now();
    this->laser_data.radiation_type = INFRARED;
    this->laser_data.field_of_view = 1;
    this->laser_data.min_range = 0.23;
    this->laser_data.max_range = 1.5;
    if(close_all_flag == 0)
    {
        for(int i=0;i<laser_real_num;i++)
        {
            if(en_laser == 0)
            {
                close_all_flag = 1;

                if(i >= 3)
                {
                    this->laser_data.min_range = 0.13;
                    this->laser_data.max_range = 1.2;
                }
                this->laser_data.header.frame_id = this->laser_frames[i];
                this->laser_data.range = 5.0;
                usleep(2000);
                this->pub_to_navigation.publish(this->laser_data);
            }
            else if(en_laser & (0x00000001<<i))
            {
                close_all_flag = 0;

                if(i >= 3)
                {
                    this->laser_data.min_range = 0.13;
                    this->laser_data.max_range = 1.2;
                }
                this->laser_data.header.frame_id = this->laser_frames[i];
                this->laser_data.range = this->distance[i];
                usleep(2000);
                this->pub_to_navigation.publish(this->laser_data);
            }
        }
    }

    if(en_laser == 0)
    {
        close_all_flag = 1;
    }
    else
    {
        close_all_flag = 0;
    }

}


