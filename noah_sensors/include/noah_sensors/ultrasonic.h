#include "ros/ros.h"
#include "std_msgs/String.h"
#include "json.hpp"
#include <mrobot_driver_msgs/vci_can.h>
#include <roscan/can_long_frame.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud2.h>

#ifndef _ULTRASONIC__H
#define _ULTRASONIC__H

#define ULTRASONIC_CAN_SRC_MAC_ID_BASE      0x60
#define CAN_SOURCE_ID_START_MEASUREMENT     0x80

#define ULTRASONIC_NUM_MAX                 14 

#define GROUP_PERIOD                        100//ms

#define DISTANCE_MAX                        400
#define ERR_COMMUNICATE_TIME_OUT            1
#define DISTANCE_ERR_TIME_OUT               255 



class Ultrasonic
{
    public:

        Ultrasonic(bool log_on = false)
        {
            is_log_on = log_on;
            //ultrasonic_pub = n.advertise<std_msgs::String>("ultrasonic_to_can",1000);

            pub_to_can_node = n.advertise<mrobot_driver_msgs::vci_can>("ultrasonic_to_can", 1000);
            sub_from_can_node = n.subscribe("can_to_ultrasonic", 1000, &Ultrasonic::rcv_from_can_node_callback, this);
            ultrasonic_pub_to_navigation = n.advertise<sensor_msgs::Range>("sonar_msg",20);
        }
        
        int start_measurement(uint8_t ul_id);
        void rcv_from_can_node_callback(const mrobot_driver_msgs::vci_can::ConstPtr &c_msg);
        void update_status(void);
        void pub_ultrasonic_data_to_navigation(uint16_t *data);
        bool is_log_on;
        can_long_frame  long_frame;
        ros::Time start_measure_time[ULTRASONIC_NUM_MAX];
        uint8_t err_status[ULTRASONIC_NUM_MAX];


        uint16_t distance[ULTRASONIC_NUM_MAX] = {0};
        uint8_t id_group[4][4] = 
        {
            {10,  11,   3,   7 },
            {0,   13,   5,   8 },
            {1,   9,  4,  0xff},
            {2,  6,  12,  0xff}
        };
        sensor_msgs::Range ultrasonic_data;
        ros::Publisher ultrasonic_pub_to_navigation;

    private:
        ros::NodeHandle n;
        ros::Subscriber noah_ultrasonic_sub;
        ros::Subscriber sub_from_can_node;

        ros::Publisher  ultrasonic_pub;
        ros::Publisher  pub_to_can_node;
        std::string ultrasonic_frames[ULTRASONIC_NUM_MAX] = {"sonar_frame_0","sonar_frame_1","sonar_frame_2","sonar_frame_3","sonar_frame_4","sonar_frame_5","sonar_frame_6","sonar_frame_7", "sonar_frame_8","sonar_frame_9","sonar_frame_10"};
        uint32_t ultrasonic_en = 0xffffffff;
        uint8_t ultrasonic_real_num = ULTRASONIC_NUM_MAX;

        bool is_ultrasonic_can_id(CAN_ID_UNION id);
        uint8_t parse_ultrasonic_id(CAN_ID_UNION id);
};


#endif
