/* 
 *  common.cpp 
 *  Author: Kaka Xie 
 *  Date:2017/12/01
 */


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
//#include "geometry_msgs/Twist.h"
//#include "geometry_msgs/PoseStamped.h"
//#include "geometry_msgs/TwistStamped.h"
//#include "tf/transform_broadcaster.h"
#include <signal.h>

//#include <sstream>
//#include <math.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <pthread.h>

#include <json.hpp>

#include <ultrasonic.h>
#include <laser.h>
#include <common.h>

using json = nlohmann::json;

uint32_t sonar_en = 0xffffffff;
uint32_t laser_en = 0xffffffff;
void sensor_en_cb(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("%s",__func__);
    auto j = json::parse(msg->data.c_str());
    if(j.find("params") != j.end())
    {
        if(j["params"].find("enable_supersonic") != j["params"].end())
        {
            sonar_en = j["params"]["enable_supersonic"];
            ROS_INFO("find enable_supersonic: 0x%x",sonar_en);
        }
        if(j["params"].find("enable_microlaser") != j["params"].end())
        {
            laser_en = j["params"]["enable_microlaser"];
            ROS_INFO("find enable_microlaser: 0x%x",laser_en);
        }
    }
}



