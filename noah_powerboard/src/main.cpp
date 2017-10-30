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

#include "../include/noah_powerboard/powerboard.h"

class NoahPowerboard;

void sigintHandler(int sig)
{
    ROS_INFO("killing on exit");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "noah_powerboard_node");
    bool is_log_on = false;
    if(ros::param::has("is_log_on"))
    {
        ROS_INFO("has param");
        ros::param::get("~is_log_on",is_log_on);
        ROS_INFO("param value is %d",is_log_on);
    }
    else
    {
        is_log_on = false;
        ROS_INFO("has not param");
    }
    NoahPowerboard *powerboard = new NoahPowerboard(is_log_on);
    float rate = 1000;
    ros::Rate loop_rate(rate);
    uint32_t cnt = 0;
    bool flag = 0;
    pthread_t can_protocol_proc_handle;
    pthread_create(&can_protocol_proc_handle, NULL, CanProtocolProcess,(void*)powerboard);  

    get_bat_info_t get_bat_info;
    get_bat_info.reserve = 0;
    while(ros::ok())
    {
        if(flag == 0)
        {
            flag = 1;
            //powerboard.get_bat_info_vector.push_back(get_bat_info);
            //init 
        }
        if(cnt++ % (uint32_t)(rate * 5) == (uint32_t)rate/2)
        {
            //test_fun((void*)powerboard); 
            powerboard->get_bat_info_vector.push_back(get_bat_info);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

}
