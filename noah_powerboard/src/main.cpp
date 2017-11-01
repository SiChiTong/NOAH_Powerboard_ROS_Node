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
    bool is_log_on = 0;
    ROS_INFO("creating noah powerboard node...");
    if(ros::param::has("noah_pb_can_data_log_on"))
    {
        ros::param::get("/noah_pb_can_data_log_on",is_log_on);
        ROS_INFO("can data log is %d",is_log_on);
    }
    else
    {
        is_log_on = false;
        ROS_INFO("can data log is off");
    }
    NoahPowerboard *powerboard = new NoahPowerboard(is_log_on);
    float rate = 1000;
    ros::Rate loop_rate(rate);
    uint32_t cnt = 0;
    bool flag = 0;
    pthread_t can_protocol_proc_handle;
    pthread_create(&can_protocol_proc_handle, NULL, CanProtocolProcess,(void*)powerboard);  

    get_bat_info_t get_bat_info;
    set_leds_effect_t set_led_effect;
    get_sys_status_t get_sys_status;

    get_bat_info.reserve = 0;

    set_led_effect.mode = LIGHTS_MODE_NOMAL;
    set_led_effect.reserve = 0;

    get_sys_status.reserve = 0;

    while(ros::ok())
    {
        if(flag == 0)
        {
            if(cnt % (uint32_t)(rate * 2) == (uint32_t)rate/2)
            {
                flag = 1;
                do
                {
                    boost::mutex::scoped_lock(powerboard->mtx);        
                    powerboard->set_leds_effect_vector.push_back(set_led_effect);
                }while(0);
            }
        }
        if(cnt % (uint32_t)(rate * 5) == (uint32_t)rate/2)
        {
            //test_fun((void*)powerboard); 
            do
            {
                boost::mutex::scoped_lock(powerboard->mtx);        
                powerboard->get_bat_info_vector.push_back(get_bat_info);
                powerboard->get_sys_status_vector.push_back(get_sys_status);
            }while(0);
        }
        
        if(cnt % (uint32_t)(rate * 1) == (uint32_t)rate/2)
        {
            powerboard->update_sys_status();
        }
        cnt++;
        ros::spinOnce();
        loop_rate.sleep();
    }

}
