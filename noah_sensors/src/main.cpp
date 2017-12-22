/* 
 *  main.cpp 
 *  Author: Kaka Xie 
 *  Date:2017/11/30
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
//#include <vector>
#include <iostream>
//#include <pthread.h>

#include <ultrasonic.h>
#include <laser.h>
#include <hall.h>
#include <common.h>
uint16_t laser_test_data[13] = {0};
void sigintHandler(int sig)
{
    ROS_INFO("killing on exit");
    ros::shutdown();
}

#define ULTRASONIC_INIT_TIME_OUT        5.0     //unit: Second
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
    Hall *hall = new Hall(is_log_on); 
    uint32_t rate = 1000;
    ros::Rate loop_rate(rate);
    uint32_t cnt = 0;
    //bool ul_init_flag = 0;
    ros::Time ul_init_start_time = ros::Time::now();
    ultrasonic->work_mode = ULTRASONIC_MODE_TURNING;
    while(ros::ok())
    {
        if(ultrasonic->group_init_flag == 0)
        {
            for(uint8_t i = 0; i < ULTRASONIC_NUM_MAX; i++)
            {
                ultrasonic->get_version(i); 
                usleep(1000);
                //ROS_INFO("start to get ultrasonic %d version",i);
            }
            if(ros::Time::now() - ul_init_start_time <= ros::Duration(ULTRASONIC_INIT_TIME_OUT))
            {
                if(ultrasonic->work_mode == ULTRASONIC_MODE_TURNING)
                {
                    for(uint8_t i=0; i<sizeof(ultrasonic->group_mode_turning) / sizeof(ultrasonic->group_mode_turning[0]); i++)
                    {
                        for(uint8_t j=0; j<sizeof(ultrasonic->group_mode_turning[0]) / sizeof(ultrasonic->group_mode_turning[0][0]); j++)
                        {
                            ultrasonic->set_group(ultrasonic->group_mode_turning[i][j],ULTRASONIC_MODE_TURNING * ULTRASONIC_NUM_MAX + i + 1); 
                            usleep(30*1000);
                        }
                    }
                }
                else if(ultrasonic->work_mode == ULTRASONIC_MODE_FORWARD)
                {
                    for(uint8_t i=0; i<sizeof(ultrasonic->group_mode_forward) / sizeof(ultrasonic->group_mode_forward[0]); i++)
                    {
                        for(uint8_t j=0; j<sizeof(ultrasonic->group_mode_forward[0]) / sizeof(ultrasonic->group_mode_forward[0][0]); j++)
                        {
                            ultrasonic->set_group(ultrasonic->group_mode_forward[i][j], ULTRASONIC_MODE_FORWARD * ULTRASONIC_NUM_MAX + i + 1); 
                            usleep(30*1000);
                        }
                    }
                }
                else if(ultrasonic->work_mode == ULTRASONIC_MODE_BACKWARD)
                {
                    for(uint8_t i=0; i<sizeof(ultrasonic->group_mode_backward) / sizeof(ultrasonic->group_mode_backward[0]); i++)
                    {
                        for(uint8_t j=0; j<sizeof(ultrasonic->group_mode_backward[0]) / sizeof(ultrasonic->group_mode_backward[0][0]); j++)
                        {
                            ultrasonic->set_group(ultrasonic->group_mode_backward[i][j],ULTRASONIC_MODE_BACKWARD * ULTRASONIC_NUM_MAX + i + 1); 
                            usleep(30*1000);
                        }
                    }
                }
#if 0
#endif
                //ultrasonic->set_group(12,1); 
                //usleep(1000);
                //ultrasonic->set_group(6,2); 
                //usleep(1000);
                //ultrasonic->set_group(8,3); 
                //usleep(1000);
            }
            else
            {
                ultrasonic->group_init_flag = 1;
            }
        }

        else if(ultrasonic->group_init_flag == 1)
        {

#if 1//ultrasonic
            if(cnt % (uint32_t)(rate / 9 ) == 0)
            {   
                static uint8_t ul_id = 0;
#if 0
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
                        if(sonar_en & (1<<ultrasonic->id_group[period][i]))                    
                        {
                            //ultrasonic->start_measurement(ultrasonic->id_group[period][i]);
                        }
                    }
                }
#endif
                static uint8_t group = 0;
                if(ultrasonic->work_mode == ULTRASONIC_MODE_FORWARD)
                {
                    ultrasonic->broadcast_measurement(ultrasonic->work_mode * ULTRASONIC_NUM_MAX + group + 1);
                    if(group >= sizeof(ultrasonic->group_mode_forward) / sizeof(ultrasonic->group_mode_forward[0]) - 1)
                    {
                        group = 0;
                    }
                    else
                    {
                        group++;
                    }
                }
                else if(ultrasonic->work_mode == ULTRASONIC_MODE_BACKWARD)
                {
                    ultrasonic->broadcast_measurement(ultrasonic->work_mode * ULTRASONIC_NUM_MAX + group + 1);
                    if(group >= sizeof(ultrasonic->group_mode_backward) / sizeof(ultrasonic->group_mode_backward[0]) - 1)
                    {
                        group = 0;
                    }
                    else
                    {
                        group++;
                    }
                }
                else if(ultrasonic->work_mode == ULTRASONIC_MODE_TURNING)
                {
                    ultrasonic->broadcast_measurement(ultrasonic->work_mode * ULTRASONIC_NUM_MAX + group + 1);
                    if(group >= sizeof(ultrasonic->group_mode_turning) / sizeof(ultrasonic->group_mode_turning[0]) - 1)
                    {
                        group = 0;
                    }
                    else
                    {
                        group++;
                    }
                }
                if(ul_id < ULTRASONIC_NUM_MAX)
                {
                    ul_id++;
                }
                else
                {
                    ul_id = 0;
                }

            }
            if(cnt % (uint32_t)(rate / 10) == 0)
            {
                ultrasonic->update_status();
                ultrasonic->pub_ultrasonic_data_to_navigation(ultrasonic->distance);
            }

            if(cnt % (uint32_t)(rate / 20) == 0)
            {
                ultrasonic->update_measure_en(sonar_en);
            }

#endif


#if 0 //laser
            if(cnt % (uint32_t)(rate / 80) == 0)
            {   
                static uint8_t i = 0;
                if(laser_en & (1<<(i % LASER_NUM_MAX)))                    
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
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

}



