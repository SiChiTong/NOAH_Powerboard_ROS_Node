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
    NoahPowerboard  powerboard;
    ros::Rate loop_rate(10);
    uint32_t cnt = 0;
    powerboard.PowerboardParamInit();

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
    signal(SIGINT, sigintHandler);

//    powerboard.handle_receive_data(sys_powerboard);
    while(ros::ok())
    {
        cnt++;
        if(cnt % 20 == 10)
        {
            
#if 1   //Get battery info test function
            sys_powerboard->bat_info.cmd = 2; 
            powerboard.GetBatteryInfo(sys_powerboard);
            usleep(10*1000);
#endif
#if 1  //Get system status
            powerboard.GetSysStatus(sys_powerboard);
            usleep(50*1000);
            //powerboard.PubPower();
#endif
        }
        //powerboard.handle_receive_data(sys_powerboard);
#if 0   // Set LED effect test function
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

#if 0   //Get Current test function
        sys_powerboard->current_cmd_frame.cmd = SEND_RATE_SINGLE;
        powerboard.GetAdcData(sys_powerboard); 
#endif
#if 0   //Get version test function
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

#if 0   //Infrared LED ctrl test funcion

        //sys_powerboard->ir_cmd.cmd = IR_CMD_READ;
        sys_powerboard->ir_cmd.cmd = IR_CMD_WRITE;
        sys_powerboard->ir_cmd.set_ir_percent = 75;
        powerboard.InfraredLedCtrl(sys_powerboard);
#endif 
#if 0
        {
            static uint16_t cnt = 0;
            cnt++;
            if(cnt % 10 > 4)
            {
                sys_powerboard->module_status_set.on_off = MODULE_CTRL_OFF; 
            }
            else
            {
                sys_powerboard->module_status_set.on_off = MODULE_CTRL_ON;
            }
            sys_powerboard->module_status_set.module = POWER_24V_PRINTER; 
            powerboard.SetModulePowerOnOff(sys_powerboard);
        }
#endif
#if 0
        powerboard.GetModulePowerOnOff(sys_powerboard);
#endif
        ros::spinOnce();
        loop_rate.sleep();
    }
    //close(fd);
    close(sys_powerboard->device);

}
