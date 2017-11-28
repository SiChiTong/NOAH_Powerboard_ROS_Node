#include "ros/ros.h"
#include "std_msgs/String.h"
#include "json.hpp"
#include <mrobot_driver_msgs/vci_can.h>
#include <roscan/can_long_frame.h>

#ifndef _ULTRASONIC__H
#define _ULTRASONIC__H

#define ULTRASONIC_CAN_SRC_MAC_ID_BASE      0x60
#define CAN_SOURCE_ID_START_MEASUREMENT     0x80

#define ULTRASONIC_NUM_MAX                  20
class Ultrasonic
{
    public:

        Ultrasonic(bool log_on = false)
        {
            is_log_on = log_on;
            noah_ultrasonic_pub = n.advertise<std_msgs::String>("tx_noah_powerboard_node",1000);

            pub_to_can_node = n.advertise<mrobot_driver_msgs::vci_can>("noah_powerboard_to_can", 1000);
            sub_from_can_node = n.subscribe("can_to_noah_powerboard", 1000, &Ultrasonic::rcv_from_can_node_callback, this);
        }
        
        int start_measurement(uint8_t ul_id);
        bool is_log_on;
        void rcv_from_can_node_callback(const mrobot_driver_msgs::vci_can::ConstPtr &c_msg);
        can_long_frame  long_frame;

    private:
        ros::NodeHandle n;
        ros::Subscriber noah_ultrasonic_sub;
        ros::Subscriber sub_from_can_node;

        ros::Publisher  noah_ultrasonic_pub;
        ros::Publisher  pub_to_can_node;
        uint16_t distance[ULTRASONIC_NUM_MAX];
        bool is_ultrasonic_can_id(CAN_ID_UNION id);
        uint8_t parse_ultrasonic_id(CAN_ID_UNION id);
};


#endif

