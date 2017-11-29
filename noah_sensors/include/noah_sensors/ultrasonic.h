#include "ros/ros.h"
#include "std_msgs/String.h"
#include "json.hpp"
#include <mrobot_driver_msgs/vci_can.h>
#include <roscan/can_long_frame.h>

#ifndef _ULTRASONIC__H
#define _ULTRASONIC__H

#define ULTRASONIC_CAN_SRC_MAC_ID_BASE      0x60
#define CAN_SOURCE_ID_START_MEASUREMENT     0x80

#define ULTRASONIC_NUM_MAX                 15 

#define GROUP_PERIOD                        100//ms

#define DISTANCE_MAX                        200
#define ERR_COMMUNICATE_TIME_OUT            1
#define DISTANCE_ERR_TIME_OUT               DISTANCE_MAX
class Ultrasonic
{
    public:

        Ultrasonic(bool log_on = false)
        {
            is_log_on = log_on;
            //ultrasonic_pub = n.advertise<std_msgs::String>("ultrasonic_to_can",1000);

            pub_to_can_node = n.advertise<mrobot_driver_msgs::vci_can>("ultrasonic_to_can", 1000);
            sub_from_can_node = n.subscribe("can_to_ultrasonic", 1000, &Ultrasonic::rcv_from_can_node_callback, this);
        }
        
        int start_measurement(uint8_t ul_id);
        void rcv_from_can_node_callback(const mrobot_driver_msgs::vci_can::ConstPtr &c_msg);
        void update_status(void);
        bool is_log_on;
        can_long_frame  long_frame;
        ros::Time start_measure_time[ULTRASONIC_NUM_MAX];
        uint8_t err_status[ULTRASONIC_NUM_MAX];

        uint8_t id_group[4][4] = 
            {
                {1,   2,   3,   4 },
                {5,   6,   7,   8 },
                {9,   10,  11,  12},
                {13,  14,  15,  0}
            };

    private:
        ros::NodeHandle n;
        ros::Subscriber noah_ultrasonic_sub;
        ros::Subscriber sub_from_can_node;

        ros::Publisher  ultrasonic_pub;
        ros::Publisher  pub_to_can_node;
        uint16_t distance[ULTRASONIC_NUM_MAX] = {0};
        bool is_ultrasonic_can_id(CAN_ID_UNION id);
        uint8_t parse_ultrasonic_id(CAN_ID_UNION id);
};


#endif

