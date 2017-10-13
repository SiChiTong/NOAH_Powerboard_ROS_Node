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
    NoahPowerboard powerboard;
    float rate = 1000;
    ros::Rate loop_rate(rate);
    uint32_t cnt = 0;
    pthread_t can_protocol_proc_handle;
    pthread_create(&can_protocol_proc_handle, NULL, CanProtocolProcess,(void*)&powerboard);  
    while(ros::ok())
    {
        if(cnt++ % (uint32_t)rate == (uint32_t)rate/2)
        {
            //test_fun(); 
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

}
