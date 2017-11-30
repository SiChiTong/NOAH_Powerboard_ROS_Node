#include "ros/ros.h"
#include "std_msgs/String.h"
#include "json.hpp"
#include <mrobot_driver_msgs/vci_can.h>
#include <roscan/can_long_frame.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud2.h>

#ifndef _LASER__H
#define _LASER__H

#define LASER_CAN_SRC_MAC_ID_BASE      0x70
#define LASER_CAN_SOURCE_ID_START_MEASUREMENT     0x80

#define LASER_NUM_MAX                 13 


#define LASER_DISTANCE_MAX                        120 
#define LASER_ERR_COMMUNICATE_TIME_OUT            1
#define LASER_DISTANCE_ERR_TIME_OUT               LASER_DISTANCE_MAX
#if 0
enum{
    INFRARED = 0,
    ULTRASOUND = 1,  
    HALL = 2,
};
#endif


class Laser 
{
    public:

        Laser(bool log_on = false)
        {
            is_log_on = log_on;

            pub_to_can_node = n.advertise<mrobot_driver_msgs::vci_can>("laser_to_can", 1000);
            sub_from_can_node = n.subscribe("can_to_laser", 1000, &Laser::rcv_from_can_node_callback, this);
            pub_to_navigation = n.advertise<sensor_msgs::Range>("laser_msg",20);
        }
        
        int start_measurement(uint8_t ul_id);
        void rcv_from_can_node_callback(const mrobot_driver_msgs::vci_can::ConstPtr &c_msg);
        void update_status(void);
        void pub_laser_data_to_navigation(uint16_t *data);
        bool is_log_on;
        can_long_frame  long_frame;
        ros::Time start_measure_time[LASER_NUM_MAX];
        uint8_t err_status[LASER_NUM_MAX];


        uint16_t distance[LASER_NUM_MAX] = {0};
        sensor_msgs::Range laser_data;
        ros::Publisher lasercloud_pub;
        ros::Publisher laser_pub;
        ros::Publisher pub_to_navigation;

    private:
        ros::NodeHandle n;
        ros::Subscriber laser_sub;
        ros::Subscriber sub_from_can_node;

        ros::Publisher  pub_to_can_node;
        std::string laser_frames[LASER_NUM_MAX] = {"laser_frame_0","laser_frame_1","laser_frame_2","laser_frame_3","laser_frame_4","laser_frame_5","laser_frame_6","laser_frame_7","laser_frame_8","laser_frame_9","laser_frame_10","laser_frame_11","laser_frame_12"};

        uint32_t laser_en = 0xffffffff;
        uint8_t laser_real_num = LASER_NUM_MAX;

        bool is_laser_can_id(CAN_ID_UNION id);
        uint8_t parse_laser_id(CAN_ID_UNION id);
};


#endif
