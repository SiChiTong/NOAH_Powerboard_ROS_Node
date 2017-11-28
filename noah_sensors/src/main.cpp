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
//class NoahPowerboard;

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
        if(cnt % (uint32_t)(rate / 5) == 0)
        {
            //ROS_INFO("request ultrasonic measurement"); 
            ultrasonic->start_measurement(1);
        }
        if(cnt % (uint32_t)(rate * 5) == (uint32_t)rate*2)
        {
        }
        
        if(cnt % (uint32_t)(rate * 1) == (uint32_t)rate/2)
        {
        }
        cnt++;
        ros::spinOnce();
        loop_rate.sleep();
    }

}
