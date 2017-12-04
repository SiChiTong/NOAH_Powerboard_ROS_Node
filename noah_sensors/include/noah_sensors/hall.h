#include "ros/ros.h"
#include "std_msgs/String.h"
#include "json.hpp"
#include <mrobot_driver_msgs/vci_can.h>
#include <roscan/can_long_frame.h>
#include <sensor_msgs/Range.h>
#include <json.hpp>

#ifndef _HALL__H
#define _HALL__H

using json = nlohmann::json;

#define HALL_NUM    2
class Hall 
{
    public:

        Hall(bool log_on = false)
        {
            is_log_on = log_on;

            hall_pub = n.advertise<std_msgs::String>("hall_msg", 1000);
            hall_sub = n.subscribe("hall_to_starline_node", 1000, &Hall::hall_state_callback, this);
        }
        bool is_log_on;
        bool hall_state[HALL_NUM] = {false,false};
        void pub_hall_data(bool *state);

    private:
        ros::NodeHandle n;
        ros::Subscriber hall_sub;

        ros::Publisher  hall_pub;

        void pub_hall_msg(const nlohmann::json j_msg);
        void hall_state_callback(std_msgs::UInt8MultiArray data);



};


#endif
