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
    NoahPowerboard powerboard;
    float rate = 1000;
    ros::Rate loop_rate(rate);
    uint32_t cnt = 0;
    
    while(ros::ok())
    {
        if(cnt++ % (uint32_t)rate == (uint32_t)rate/2)
        {
            //heart beat 
            powerboard.sys_powerboard->module_status_set.on_off = 0;
            powerboard.sys_powerboard->module_status_set.module = POWER_5V_EN;
            powerboard.SetModulePowerOnOff(powerboard.sys_powerboard);
            
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

}
