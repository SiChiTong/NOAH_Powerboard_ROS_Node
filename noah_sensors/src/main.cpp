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

#include <ultrasonic.h>
#include <laser.h>
//class NoahPowerboard;

#if 0
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
#endif

void sigintHandler(int sig)
{
    ROS_INFO("killing on exit");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "noah_sensors");
    bool is_log_on = 0;
    ROS_INFO("creating noah sensors node...");
    if(ros::param::has("noah_sensors_can_data_log_on"))
    {
        ros::param::get("/noah_sensors_can_data_log_on",is_log_on);
        ROS_INFO("can data log is %d",is_log_on);
    }
    else
    {
        is_log_on = false;
        ROS_INFO("can data log is off");
    }
    Ultrasonic *ultrasonic = new Ultrasonic(is_log_on); 
    Laser *laser = new Laser(is_log_on); 
    float rate = 1000;
    ros::Rate loop_rate(rate);
    uint32_t cnt = 0;
    bool flag = 0;
    while(ros::ok())
    {
        if(flag == 0)
        {
            if(cnt % (uint32_t)(rate * 2) == (uint32_t)rate/2)
            {
                flag = 1;
            }
        }
#if 1//ultrasonic
        if(cnt % (uint32_t)(rate / 60) == 0)
        {   
            static uint8_t period = 0;
            period++;
            if(period >= sizeof(ultrasonic->id_group) / sizeof(ultrasonic->id_group[0]))
            {
                period = 0;
            }
            //ROS_INFO("group num: %d",period); 
            for(uint8_t i = 0; i < sizeof(ultrasonic->id_group[0]) / sizeof(ultrasonic->id_group[0][0]); i++)
            {
                if(ultrasonic->id_group[period][i] < ULTRASONIC_NUM_MAX)
                {
                    ultrasonic->start_measurement(ultrasonic->id_group[period][i]);
                }
            }
           
        }
        if(cnt % (uint32_t)(rate / 10) == 0)
        {
            ultrasonic->update_status();
            ultrasonic->pub_ultrasonic_data_to_navigation(ultrasonic->distance);
        }
#endif


#if 1 //laser
        if(cnt % (uint32_t)(rate / 60) == 0)
        {   
            static uint8_t i = 0;
            laser->start_measurement(i % LASER_NUM_MAX);
            i++;
        }
        if(cnt % (uint32_t)(rate / 10) == 0)
        {
            laser->update_status();
            laser->pub_laser_data_to_navigation(laser->distance);
        }
#endif
        cnt++;
        ros::spinOnce();
        loop_rate.sleep();
    }

}



