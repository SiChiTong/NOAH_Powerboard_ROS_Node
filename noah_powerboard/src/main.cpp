#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "tf/transform_broadcaster.h"

#include <sstream>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <pthread.h>
#include "../include/noah_powerboard/config.h"
#include "../include/noah_powerboard/system.h"
#include "../include/noah_powerboard/report.h"
#include "../include/noah_powerboard/powerboard.h"
//#include "../include/noah_powerboard/navigation.h"
//#include "../include/noah_powerboard/handle_command.h"

class NoahPowerboard;
int main(int argc, char **argv)
{
    NoahPowerboard  powerboard;
    ros::init(argc, argv, "noah_powerboard_node");
    ros::NodeHandle n;
    ros::Publisher noah_power_pub = n.advertise<std_msgs::String>("noah_power_tx", 1000);
    ros::Rate loop_rate(2);
    //uint8_t test_data[] = {0x5a, 0x0a ,0x01, 0x04, 0xff, 0xff, 0xff, 0xff, 0x65, 0xa5};
    //uint8_t test_data[] = {0x5a, 0x0a, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0x60, 0xa5};
    //char dev[] = "/dev/ttyUSB0";
    //int fd;
    powerboard.PowerboardParamInit();
    //fd = open_com_device(sys_powerboard->dev);
    //fd = open(dev, O_RDWR);

    sys_powerboard->device = open_com_device(sys_powerboard->dev);
    if(sys_powerboard->device < 0 )
    {
        ROS_ERROR("Open %s Failed !",sys_powerboard->dev);
    }
    else
    {
        set_speed(sys_powerboard->device,115200);
        set_parity(sys_powerboard->device,8,1,'N');  
        ROS_INFO("Open %s OK.",sys_powerboard->dev);
    }

    while(ros::ok())
    {
        powerboard.handle_receive_data(sys_powerboard);
#if 1   // Set LED effect test function
        sys_powerboard->led_set.color.r = 0x12;
        sys_powerboard->led_set.color.g = 0x34;
        sys_powerboard->led_set.color.b = 0x56;
        sys_powerboard->led_set.period = 0xff;

        if( sys_powerboard->led_set.effect< LIGHTS_MODE_EMERGENCY_STOP)
        {
            sys_powerboard->led_set.effect++;
        }
        else
        {
            //sys_powerboard->led_set.effect = LIGHTS_MODE_SETTING;
            sys_powerboard->led_set.effect = LIGHTS_MODE_DEFAULT;
        }
        powerboard.SetLedEffect(sys_powerboard);
#endif

#if 1   //Get battery info test function
        sys_powerboard->bat_info.cmd = 1; 
        powerboard.GetBatteryInfo(sys_powerboard);
#endif
#if 1   //Get Current test function
        sys_powerboard->current_cmd_frame.cmd = SEND_RATE_SINGLE;
        powerboard.GetAdcData(sys_powerboard); 
#endif
#if 1   //Get version test function
        {
            static uint8_t i = 0;
            i++;
            //if(i%2)
            {
                sys_powerboard->get_version_type = VERSION_TYPE_FW;
            }
            //else
            {
                //sys_powerboard->get_version_type = VERSION_TYPE_PROTOCOL;
            }
            powerboard.GetVersion(sys_powerboard);
        }
#endif

#if 1  //Get system status
        powerboard.GetSysStatus(sys_powerboard);
#endif
#if 1   //Infrared LED ctrl test funcion

        //sys_powerboard->ir_cmd.cmd = IR_CMD_READ;
        sys_powerboard->ir_cmd.cmd = IR_CMD_WRITE;
        sys_powerboard->ir_cmd.set_ir_percent = 75;
        powerboard.InfraredLedCtrl(sys_powerboard);
#endif 
#if 0
        sys_powerboard->module_status_set.module = POWER_12V_EN | POWER_24V_EN; 
        sys_powerboard->module_status_set.on_off = MODULE_CTRL_OFF; 
        SetModulePowerOnOff(sys_powerboard);
        usleep(TEST_WAIT_TIME);
        handle_receive_data(sys_powerboard);
#endif
#if 1
        powerboard.GetModulePowerOnOff(sys_powerboard);
#endif
        loop_rate.sleep();
    }
    //close(fd);
    close(sys_powerboard->device);

}
