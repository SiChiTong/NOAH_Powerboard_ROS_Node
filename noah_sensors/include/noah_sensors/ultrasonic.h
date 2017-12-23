#include "ros/ros.h"
#include "std_msgs/String.h"
#include "json.hpp"
#include <mrobot_driver_msgs/vci_can.h>
#include <roscan/can_long_frame.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud2.h>
#include <common.h>

#ifndef _ULTRASONIC__H
#define _ULTRASONIC__H

#define ULTRASONIC_CAN_SRC_MAC_ID_BASE      0x60

#define CAN_SOURCE_ID_START_MEASUREMENT     0x80
#define CAN_SOURCE_ID_MEASUREMENT_EN        0x81
#define CAN_SOURCE_ID_GET_VERSION           0x82
#define CAN_SOURCE_ID_SET_GROUP             0x83

#define ULTRASONIC_NUM_MAX                  14 
#define FILTER_BUF_SIZE                     3 

#define GROUP_PERIOD                        100//ms

#define DISTANCE_MAX                        2.00
#define ERR_COMMUNICATE_TIME_OUT            1
#define DISTANCE_ERR_TIME_OUT               2.55 

enum
{
    
    ULTRASONIC_MODE_FORWARD     = 0,
    ULTRASONIC_MODE_BACKWARD    = 1,
    ULTRASONIC_MODE_TURNING     = 2,
    ULTRASONIC_MODE_MAX,
    ULTRASONIC_MODE_NONE,
};

typedef struct
{
    uint8_t id;
    uint8_t group;
}group_id_t;

class Ultrasonic
{
    public:

        Ultrasonic(bool log_on = false)
        {
            is_log_on = log_on;
            //ultrasonic_pub = n.advertise<std_msgs::String>("ultrasonic_to_can",1000);
            sensor_en = n.subscribe("/map_server_mrobot/region_params_changer/sensor_params",1000,sensor_en_cb);
            pub_to_can_node = n.advertise<mrobot_driver_msgs::vci_can>("ultrasonic_to_can", 1000);
            sub_from_can_node = n.subscribe("can_to_ultrasonic", 1000, &Ultrasonic::rcv_from_can_node_callback, this);
            ultrasonic_pub_to_navigation = n.advertise<sensor_msgs::Range>("sonar_msg",20);
            group_id_vec.clear();
        }
        
        int start_measurement(uint8_t ul_id);
        void get_version(uint8_t ul_id);
        void ultrasonic_en(uint8_t ul_id, bool en);
        int broadcast_measurement(uint8_t group);
        int set_group(uint8_t ul_id, uint8_t group);
        void rcv_from_can_node_callback(const mrobot_driver_msgs::vci_can::ConstPtr &c_msg);
        void update_status(void);
        void pub_ultrasonic_data_to_navigation(double *data);
        void update_measure_en(uint32_t ul_en);

        uint8_t group_init_flag = 0;
        bool is_log_on;
        can_long_frame  long_frame;
        ros::Time start_measure_time[ULTRASONIC_NUM_MAX];
        uint8_t err_status[ULTRASONIC_NUM_MAX];
        uint8_t online[ULTRASONIC_NUM_MAX];
        uint8_t work_mode = ULTRASONIC_MODE_FORWARD;

        uint8_t group_mode_forward[2][4] = 
                {
                    {10,11,0xff,0xff},
                    {0,1,12,13},
                };
        uint8_t group_mode_backward[1][2] = 
                {
                    {6,7},
                };
        uint8_t group_mode_turning[1][14] = 
                {
                    {0,1,2,3,4,5,6,7,8,9,10,11,12,13},
                };

        vector<group_id_t> group_id_vec;
        double max_distance = DISTANCE_MAX;
        double distance[ULTRASONIC_NUM_MAX] = {0};
        double distance_buf[ULTRASONIC_NUM_MAX][FILTER_BUF_SIZE] = {{0}};
        uint8_t distance_buf_proc[ULTRASONIC_NUM_MAX][FILTER_BUF_SIZE] = {{0}};

        uint8_t group[ULTRASONIC_NUM_MAX] = {0};

        std::string version[ULTRASONIC_NUM_MAX];
        sensor_msgs::Range ultrasonic_data;
        ros::Publisher ultrasonic_pub_to_navigation;


#if 0
        uint8_t id_group[6][3] = 
        {
            //{10,  11,   3,   7 },
            //{0,   13,   5,   8 },
            //{1,   9,  4,  0xff},
            //{2,  6,  12,  0xff}

            {0,  3,   7 },
            {13,  9,  5},
            {11, 4,  0xff},
            {10,  2,  0xff},
            //{0xff,  2,  0xff},
            {1,  8,  0xff},
            {12,  6,  0xff}
		
        };
#else
        uint8_t id_group[14][1] = 
        {

            {0 },
            {1 },
            {2},
            {3},
            //{10,  2,  0xff},
            {4},
            {5},
            {6},
            {7},
            {8},
            //{10,  2,  0xff},
            {9},
            {10},
            {11},
            {12},
            {13}
        };
#endif

    private:
        ros::NodeHandle n;
        ros::Subscriber noah_ultrasonic_sub;
        ros::Subscriber sub_from_can_node;
        ros::Subscriber sensor_en;

        ros::Publisher  ultrasonic_pub;
        ros::Publisher  pub_to_can_node;

        std::string ultrasonic_frames[ULTRASONIC_NUM_MAX] = {"sonar_frame_0","sonar_frame_1","sonar_frame_2","sonar_frame_3","sonar_frame_4","sonar_frame_5","sonar_frame_6","sonar_frame_7", "sonar_frame_8","sonar_frame_9","sonar_frame_10","sonar_frame_11","sonar_frame_12","sonar_frame_13"};

        uint8_t ultrasonic_real_num = ULTRASONIC_NUM_MAX;
        uint32_t measure_en_ack = 0xffffffff;

        bool is_ultrasonic_can_id(CAN_ID_UNION id);
        uint8_t parse_ultrasonic_id(CAN_ID_UNION id);
};


#endif
