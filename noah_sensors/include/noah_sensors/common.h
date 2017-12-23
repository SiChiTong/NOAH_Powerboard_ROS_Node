#ifndef _COMMON__H
#define _COMMON__H

enum{
    INFRARED = 0,
    ULTRASOUND = 1,  
    HALL = 2,
};

void sensor_en_cb(const std_msgs::String::ConstPtr &msg);

extern uint32_t sonar_en;
extern uint32_t laser_en;

extern ros::Time  sensor_en_start_time;

#endif
