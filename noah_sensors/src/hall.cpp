/* 
 *  hall.cpp 
 *  hall sensors proc.
 *  Author: Kaka Xie 
 *  Date:2017/12/04
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
//#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
//#include <sys/types.h>
//#include <sys/stat.h>
//#include <fcntl.h>
#include <termios.h>
//#include <errno.h>
#include <string.h>
//#include <time.h>
//#include <signal.h>
#include <iostream>
//#include <vector>
#include "stdlib.h"
//#include "cstdlib"
#include "string"
#include "sstream"
#include <common.h>
#include <hall.h>

void Hall::hall_state_callback(std_msgs::UInt8MultiArray data)
{
    uint8_t j;
    for(j = 0; j < HALL_NUM; j++)
    {
        this->hall_state[j] = data.data[j];
        ROS_INFO("hall %d state is %d",j, data.data[j]);
    }
    pub_hall_data(this->hall_state);
}

void Hall::pub_hall_msg(const nlohmann::json j_msg)
{
    std_msgs::String pub_json_msg;
    std::stringstream ss;

    ss.clear();
    ss << j_msg;
    pub_json_msg.data = ss.str();
    this->hall_pub.publish(pub_json_msg);
}
void Hall::pub_hall_data(bool *hall)
{
    
    static bool state[2] = {false, false};
    json j;

    if((state[0] != hall[0]) || (state[1] != hall[1]))
    {
        state[0] = (bool)hall[0];
        state[1] = (bool)hall[1];

        j.clear();
        j =
        {
            {"sensor_name","hall_sensor"},
            {"data",{

                        {"hall_1", (bool)hall[0]},
                        {"hall_2", (bool)hall[1]},
                    },
            },
        };
        this->pub_hall_msg(j);
    }

}

