/* 
 *  powerboard.cpp 
 *  Communicate Protocol.
 *  Author: Kaka Xie 
 *  Date:2017/10/13
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h" 
#include "pthread.h"
#include <math.h>
#include <stdio.h>     
#include <stdlib.h>     
#include <unistd.h>     
#include <sys/types.h>  
#include <sys/stat.h>   
#include <fcntl.h>      
#include <termios.h>   
#include <errno.h>     
#include <string.h>
#include <time.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include "stdlib.h"
#include "cstdlib"
#include "string"
#include "sstream"
#include "../include/noah_powerboard/powerboard.h"
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <mrobot_driver_msgs/vci_can.h>
//#include <roscan/can_long_frame.h>
#include <roscan/can_long_frame.h>

#define TEST_WAIT_TIME     90*1000

#define PowerboardInfo     ROS_INFO

static int led_over_time_flag = 0;
static int last_unread_bytes = 0;
static unsigned char recv_buf_last[BUF_LEN] = {0};


void test_fun(void * arg)
{
    NoahPowerboard *pNoahPowerboard =  (NoahPowerboard*)arg;
    module_ctrl_t param;  
    static uint8_t cnt = 2;
    param.module = POWER_5V_EN | POWER_12V_EN;
    param.group_num = 1;
    if(cnt % 2)
    {
        param.on_off = 0;
    }
    else
    {
        param.on_off = 1;
    }
#if 1
    pNoahPowerboard->module_set_vector.push_back(param);
    set_ir_duty_t set_ir_duty;
    
    set_ir_duty.duty = 90;
    set_ir_duty.reserve = 0;
    
    pNoahPowerboard->set_ir_duty_vector.push_back(set_ir_duty);

    get_version_t get_version;
    get_version.get_version_type = 1;
    pNoahPowerboard->get_version_vector.push_back(get_version);
    
    get_version.get_version_type = 2;
    pNoahPowerboard->get_version_vector.push_back(get_version);

    get_version.get_version_type = 3;
    pNoahPowerboard->get_version_vector.push_back(get_version);

    get_adc_t get_adc;
    get_adc.frq = 0;
    pNoahPowerboard->get_adc_vector.push_back(get_adc);
#endif
    set_leds_effect_t set_led_effect;

    set_led_effect.mode= cnt % (LIGHTS_MODE_EMERGENCY_STOP + 1) ; 
    pNoahPowerboard->set_leds_effect_vector.push_back(set_led_effect);

    cnt++;
}

class NoahPowerboard;
#define MODULE_SET_TIME_OUT         1000//ms
#define GET_BAT_INFO_TIME_OUT       1000//ms
#define GET_SYS_STSTUS_TIME_OUT     1000//ms
#define SET_IR_DUTY_TIME_OUT        1000//ms
#define GET_VERSION_TIME_OUT        1000//ms
#define GET_ADC_TIME_OUT            1000//ms
#define SET_LED_EFFECT_TIME_OUT     1000//ms
void *CanProtocolProcess(void* arg)
{
    module_ctrl_t module_set;
    get_bat_info_t get_bat_info;
    get_sys_status_t get_sys_status;
    set_ir_duty_t set_ir_duty;
    get_version_t get_version;
    get_adc_t   get_adc;
    set_leds_effect_t set_led_effect;

    NoahPowerboard *pNoahPowerboard =  (NoahPowerboard*)arg;

    bool module_flag = 0;
    bool get_bat_info_flag = 0;
    bool get_sys_status_flag = 0;
    bool set_ir_duty_flag = 0;
    bool get_version_flag = 0;
    bool get_adc_flag = 0;
    bool set_led_effect_flag = 0;
    while(ros::ok())
    {
        /* --------  module ctrl protocol  -------- */
        do
        {
            boost::mutex::scoped_lock(mtx);
            if(!pNoahPowerboard->module_set_vector.empty())
            {
                auto a = pNoahPowerboard->module_set_vector.begin(); 
                module_set = *a;
                if((module_set.module <= POWER_ALL ) && (module_set.on_off <= 1) && (module_set.group_num <= 2))
                {
                    module_flag = 1;            
                }

                pNoahPowerboard->module_set_vector.erase(a);

            }

        }while(0);

        if(module_flag == 1)
        {   
            uint8_t flag = 0;
            uint32_t time_out_cnt = 0;
            static uint8_t err_cnt = 0;

            module_flag = 0;

            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("get set module cmd");
                ROS_INFO("module_set_param.module = 0x%x",module_set.module);
                ROS_INFO("module_set_param.group_num = %d",module_set.group_num);
                ROS_INFO("module_set_param.on_off = %d",module_set.on_off);
            }
            do
            {
                boost::mutex::scoped_lock(mtx);
                pNoahPowerboard->module_set_ack_vector.clear();
            }while(0);

module_set_restart:
            pNoahPowerboard->sys_powerboard->module_status_set.module = module_set.module;
            pNoahPowerboard->sys_powerboard->module_status_set.group_num = module_set.group_num;
            pNoahPowerboard->sys_powerboard->module_status_set.on_off = module_set.on_off;
            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("module ctrl:send cmd to mcu");
            }
            pNoahPowerboard->SetModulePowerOnOff(pNoahPowerboard->sys_powerboard);
            bool module_ack_flag = 0;
            module_ctrl_ack_t module_set_ack;
            while(time_out_cnt < MODULE_SET_TIME_OUT/10)
            {
                time_out_cnt++;
                do
                {
                    boost::mutex::scoped_lock(mtx);
                    if(!pNoahPowerboard->module_set_ack_vector.empty())
                    {
                        auto b = pNoahPowerboard->module_set_ack_vector.begin();

                        module_set_ack = *b;

                        pNoahPowerboard->module_set_ack_vector.erase(b);

                        if(pNoahPowerboard->is_log_on == true)
                        {
                            ROS_INFO("module_set_ack_vector is not empty");
                            ROS_INFO("module_set_ack.module = 0x%x",module_set_ack.module);
                            ROS_INFO("module_set_ack.group_num = %d",module_set_ack.group_num);
                            ROS_INFO("module_set_ack.on_off = %d",module_set_ack.on_off);
                        }
                        if((module_set_ack.module <= POWER_ALL ) && (module_set_ack.on_off <= 1) && (module_set_ack.group_num <= 2))
                        {
                            module_ack_flag = 1;
                            if(pNoahPowerboard->is_log_on == true)
                            {
                                ROS_INFO("get right module ctrl ack");
                            }
                        }
                    }
                }while(0);
                if(module_ack_flag == 1)
                {
                    module_ack_flag = 0;
                    if(pNoahPowerboard->is_log_on == true)
                    {
                        ROS_INFO("get module_set_ack_vector data");
                    }
                    if(module_set_ack.module == module_set.module)
                    {
                        if(pNoahPowerboard->is_log_on == true)
                        {
                            ROS_INFO("get right module ack, module is 0x%x",module_set_ack.module);
                            ROS_INFO("now start to publish module ctrl relative topic");
                        }


                        if(module_set_ack.module & POWER_VSYS_24V_NV)
                        {
                            if(pNoahPowerboard->is_log_on == true)
                            {
                                ROS_INFO("module %d",module_set_ack.module);
                            }

                            pNoahPowerboard->j.clear();
                            pNoahPowerboard->j = 
                            {
                                {"sub_name","set_module_state"},
                                {
                                    "data",
                                    {
                                        {"door_ctrl_state",(bool)(module_set_ack.module_status_ack & POWER_VSYS_24V_NV)},
                                        {"error_code", CAN_SOURCE_ID_SET_MODULE_STATE},
                                    } 
                                }
                            };
                            pNoahPowerboard->pub_json_msg_to_app(pNoahPowerboard->j);
                        }

                        break;
                    }
                    else
                    {
                        usleep(10*1000);
                    }
                }
                else
                {
                    usleep(10*1000);
                }
            }
            if(time_out_cnt < MODULE_SET_TIME_OUT/10)
            {
                ROS_INFO("module ctrl flow OK");
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("module ctrl time out");
                time_out_cnt = 0;
                if(err_cnt++ < 3)
                {
                    ROS_ERROR("module ctrl start to resend msg....");
                    goto module_set_restart;
                }
                ROS_ERROR("CAN NOT COMMUNICATE with powerboard mcu, module ctrl failed !");
                err_cnt = 0;

                if(module_set.module & POWER_VSYS_24V_NV)
                {
                    ROS_ERROR("set door ctrl time out !");

                    pNoahPowerboard->j.clear();
                    pNoahPowerboard->j = 
                    {
                        {"sub_name","set_module_state"},
                        {
                            "data",
                            {
                                {"door_ctrl_state",(bool)(module_set_ack.module_status_ack & POWER_VSYS_24V_NV)},
                                {"error_code", -1},
                            } 
                        }
                    };
                    pNoahPowerboard->pub_json_msg_to_app(pNoahPowerboard->j);
                }

            }

        }
        /* --------  module ctrl protocol end -------- */

        
        /* --------  bat info protocol begin -------- */
        do
        {
            boost::mutex::scoped_lock(mtx);
            if(!pNoahPowerboard->get_bat_info_vector.empty())
            {
                auto a = pNoahPowerboard->get_bat_info_vector.begin(); 
                get_bat_info = *a;
                if(get_bat_info.reserve == 0 )
                {
                    get_bat_info_flag = 1;            
                }

                pNoahPowerboard->get_bat_info_vector.erase(a);

            }

        }while(0);

        if(get_bat_info_flag == 1)
        {   
            uint8_t flag = 0;
            uint32_t time_out_cnt = 0;
            static uint8_t err_cnt = 0;

            get_bat_info_flag = 0;

            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("get get_bat_info cmd");
            }
            do
            {
                boost::mutex::scoped_lock(mtx);
                pNoahPowerboard->get_bat_info_ack_vector.clear();
            }while(0);

get_bat_info_restart:
            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("get bat info:send cmd to mcu");
            }
            pNoahPowerboard->GetBatteryInfo(pNoahPowerboard->sys_powerboard);
            bool get_bat_info_ack_flag = 0;
            get_bat_info_ack_t get_bat_info_ack;
            while(time_out_cnt < GET_BAT_INFO_TIME_OUT/10)
            {
                time_out_cnt++;
                do
                {
                    boost::mutex::scoped_lock(mtx);
                    if(!pNoahPowerboard->get_bat_info_ack_vector.empty())
                    {
                        if(pNoahPowerboard->is_log_on == true)
                        {
                            ROS_INFO("get_bat_info_ack_vector is not empty");
                        }
                        auto b = pNoahPowerboard->get_bat_info_ack_vector.begin();

                        get_bat_info_ack = *b;

                        pNoahPowerboard->get_bat_info_ack_vector.erase(b);

                        if(pNoahPowerboard->is_log_on == true)
                        {
                            ROS_INFO("get_bat_info_ack.bat_vol = %d",get_bat_info_ack.bat_vol);
                            ROS_INFO("get_bat_info_ack.bat_percent = %d",get_bat_info_ack.bat_percent);
                        }
                        if( get_bat_info_ack.bat_percent <= 100)
                        {
                            get_bat_info_ack_flag = 1;
                            if(pNoahPowerboard->is_log_on == true)
                            {
                                ROS_INFO("get get_bat_info ack");
                            }
                        }
                    }
                }while(0);
                if(get_bat_info_ack_flag == 1)
                {
                    get_bat_info_ack_flag = 0;
                    if(pNoahPowerboard->is_log_on == true)
                    {
                        ROS_INFO("get get_bat_info_ack_vector data");
                        ROS_INFO("get right get_bat_info ack, bat_vol is %d",get_bat_info_ack.bat_vol);
                        ROS_INFO("get right get_bat_info ack, bat_percent is %d",get_bat_info_ack.bat_percent);
                    }

                    break;
                }
                else
                {
                    usleep(10*1000);
                }
            }
            if(time_out_cnt < GET_BAT_INFO_TIME_OUT/10)
            {
                ROS_INFO("get_bat_info flow OK");
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("get_bat_info time out");
                time_out_cnt = 0;
                if(err_cnt++ < 3)
                {
                    ROS_ERROR("get_bat_info start to resend msg....");
                    goto get_bat_info_restart;
                }
                ROS_ERROR("CAN NOT COMMUNICATE with powerboard mcu, get bat info failed !");
                err_cnt = 0;
            }

        }
        /* -------- get bat info protocol end -------- */


        /* -------- set ir duty protocol begin -------- */
        do
        {
            boost::mutex::scoped_lock(mtx);
            if(!pNoahPowerboard->set_ir_duty_vector.empty())
            {
                auto a = pNoahPowerboard->set_ir_duty_vector.begin(); 
                set_ir_duty = *a;
                if((set_ir_duty.duty <= 100 ) && (set_ir_duty.reserve == 0))
                {
                    set_ir_duty_flag = 1;            
                }

                pNoahPowerboard->set_ir_duty_vector.erase(a);

            }

        }while(0);

        if(set_ir_duty_flag == 1)
        {   
            uint8_t flag = 0;
            uint32_t time_out_cnt = 0;
            static uint8_t err_cnt = 0;

            set_ir_duty_flag = 0;

            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("get set ir duty cmd");
            }
            do
            {
                boost::mutex::scoped_lock(mtx);
                pNoahPowerboard->set_ir_duty_ack_vector.clear();
            }while(0);

set_ir_duty_restart:

            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("get sys status:send cmd to mcu");
            }
            pNoahPowerboard->InfraredLedCtrl(pNoahPowerboard->sys_powerboard);

            bool set_ir_duty_ack_flag = 0;
            set_ir_duty_ack_t set_ir_duty_ack;
            while(time_out_cnt < SET_IR_DUTY_TIME_OUT/10)
            {
                time_out_cnt++;
                do
                {
                    boost::mutex::scoped_lock(mtx);
                    if(!pNoahPowerboard->set_ir_duty_ack_vector.empty())
                    {
                        if(pNoahPowerboard->is_log_on == true)
                        {
                            ROS_INFO("set_ir_duty_ack_vector is not empty");
                        }
                        auto b = pNoahPowerboard->set_ir_duty_ack_vector.begin();
                        set_ir_duty_ack = *b;
                        pNoahPowerboard->set_ir_duty_ack_vector.erase(b);

                        if(pNoahPowerboard->is_log_on == true)
                        {
                            ROS_INFO("set_ir_duty_ack.duty = %d",set_ir_duty_ack.duty);
                        }

                        if( set_ir_duty_ack.duty <= 100 )
                        {
                            set_ir_duty_ack_flag = 1;
                            if(pNoahPowerboard->is_log_on == true)
                            {
                                ROS_INFO("get right set_ir_duty  ack");
                            }
                        }
                    }
                }while(0);
                if(set_ir_duty_ack_flag == 1)
                {
                    set_ir_duty_ack_flag = 0;
                    ROS_INFO("get set_ir_duty_ack_vector data");

                    break;
                }
                else
                {
                    usleep(10*1000);
                }
            }
            if(time_out_cnt < SET_IR_DUTY_TIME_OUT/10)
            {
                ROS_INFO("set ir duty flow OK");
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("set ir duty time out");
                time_out_cnt = 0;
                if(err_cnt++ < 3)
                {
                    ROS_ERROR("set ir duty start to resend msg....");
                    goto set_ir_duty_restart;
                }
                ROS_ERROR("CAN NOT COMMUNICATE with powerboard mcu,set ir duty failed !");
                err_cnt = 0;
            }

        }
        /* -------- set ir duty protocol end -------- */


        /* -------- get sys status protocol begin -------- */
        do
        {
            boost::mutex::scoped_lock(mtx);
            if(!pNoahPowerboard->get_sys_status_vector.empty())
            {
                auto a = pNoahPowerboard->get_sys_status_vector.begin(); 
                get_sys_status = *a;
                if(get_sys_status.reserve == 0 )
                {
                    get_sys_status_flag = 1;            
                }

                pNoahPowerboard->get_sys_status_vector.erase(a);

            }

        }while(0);

        if(get_sys_status_flag == 1)
        {   
            uint8_t flag = 0;
            uint32_t time_out_cnt = 0;
            static uint8_t err_cnt = 0;

            get_sys_status_flag = 0;

            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("get get sys status cmd");
            }
            do
            {
                boost::mutex::scoped_lock(mtx);
                pNoahPowerboard->get_sys_status_ack_vector.clear();
            }while(0);

get_sys_status_restart:

            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("get sys status:send cmd to mcu");
            }
            pNoahPowerboard->GetSysStatus(pNoahPowerboard->sys_powerboard);

            bool get_sys_status_ack_flag = 0;
            get_sys_status_ack_t get_sys_status_ack;
            while(time_out_cnt < GET_SYS_STSTUS_TIME_OUT/10)
            {
                time_out_cnt++;
                do
                {
                    boost::mutex::scoped_lock(mtx);
                    if(!pNoahPowerboard->get_sys_status_ack_vector.empty())
                    {
                        if(pNoahPowerboard->is_log_on == true)
                        {
                            ROS_INFO("get_sys_status_vector is not empty");
                        }
                        auto b = pNoahPowerboard->get_sys_status_ack_vector.begin();
                        get_sys_status_ack = *b;
                        pNoahPowerboard->get_sys_status_ack_vector.erase(b);

                        if(pNoahPowerboard->is_log_on == true)
                        {
                            ROS_INFO("get_sys_status_ack.sys_status = %d",get_sys_status_ack.sys_status);
                        }

                        if(get_sys_status_ack.sys_status <= 0x1ff )
                        {
                            get_sys_status_ack_flag = 1;
                            if(pNoahPowerboard->is_log_on == true)
                            {
                                ROS_INFO("get right get_sys_status  ack");
                            }
                        }
                    }
                }while(0);
                if(get_sys_status_ack_flag == 1)
                {
                    get_sys_status_ack_flag = 0;
                    if(pNoahPowerboard->is_log_on == true)
                    {
                        ROS_INFO("get right get_sys_status ack, status is 0x%x",get_sys_status_ack.sys_status);
                        ROS_INFO("get get_sys_status_ack_vector data");
                    }

                    break;
                }
                else
                {
                    usleep(10*1000);
                }
            }
            if(time_out_cnt < GET_SYS_STSTUS_TIME_OUT/10)
            {
                ROS_INFO("get sys status flow OK");
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("get sys status time out");
                time_out_cnt = 0;
                if(err_cnt++ < 3)
                {
                    ROS_ERROR("get sys status start to resend msg....");
                    goto get_sys_status_restart;
                }
                ROS_ERROR("CAN NOT COMMUNICATE with powerboard mcu, get_sys_status failed !");
                err_cnt = 0;
            }

        }
        /* -------- get sys status protocol end -------- */


        /* --------  get version protocol begin -------- */
        do
        {
            boost::mutex::scoped_lock(mtx);
            if(!pNoahPowerboard->get_version_vector.empty())
            {
                auto a = pNoahPowerboard->get_version_vector.begin(); 
                get_version = *a;
                if(get_version.get_version_type <= 3 )
                {
                    get_version_flag = 1;            
                }

                pNoahPowerboard->get_version_vector.erase(a);
            }
        }while(0);

        if(get_version_flag == 1)
        {   
            uint8_t flag = 0;
            uint32_t time_out_cnt = 0;
            static uint8_t err_cnt = 0;

            get_version_flag = 0;

            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("get get version cmd");
            }
            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("get_version.get_version_type = 0x%x",get_version.get_version_type);
            }
            do
            {
                boost::mutex::scoped_lock(mtx);
                pNoahPowerboard->get_version_ack_vector.clear();
            }while(0);

get_version_restart:
            pNoahPowerboard->sys_powerboard->get_version_type = get_version.get_version_type;
            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("get_verion:send cmd to mcu");
            }
            pNoahPowerboard->GetVersion(pNoahPowerboard->sys_powerboard);
            bool get_version_ack_flag = 0;
            get_version_ack_t get_version_ack;
            while(time_out_cnt < GET_VERSION_TIME_OUT/10)
            {
                time_out_cnt++;
                do
                {
                    boost::mutex::scoped_lock(mtx);
                    if(!pNoahPowerboard->get_version_ack_vector.empty())
                    {
                        if(pNoahPowerboard->is_log_on == true)
                        {
                            ROS_INFO("get_version_ack_vector is not empty"); 
                        }
                        auto b = pNoahPowerboard->get_version_ack_vector.begin();

                        get_version_ack = *b;

                        pNoahPowerboard->get_version_ack_vector.erase(b);

                        if(pNoahPowerboard->is_log_on == true)
                        {
                            ROS_INFO("version type : %d",get_version_ack.get_version_type);
                        }
                        if( get_version_ack.get_version_type <= 3) 
                        {
                            get_version_ack_flag = 1;
                            if(pNoahPowerboard->is_log_on == true)
                            {
                                ROS_INFO("get right get_version ack");
                            }
                        }
                    }
                }while(0);
                if(get_version_ack_flag == 1)
                {
                    get_version_ack_flag = 0;
                    if(pNoahPowerboard->is_log_on == true)
                    {
                        ROS_INFO("get get_version_ack_vector data");
                    }
                    if(get_version_ack.get_version_type == get_version.get_version_type)
                    {
                        if(pNoahPowerboard->is_log_on == true)
                        {
                            ROS_INFO("get right get version ack");
                        }
                        break;
                    }
                    else
                    {
                        usleep(10*1000);
                    }
                }
                else
                {
                    usleep(10*1000);
                }
            }
            if(time_out_cnt < GET_VERSION_TIME_OUT/10)
            {
                ROS_INFO("get version flow OK");
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("get version time out");
                time_out_cnt = 0;
                if(err_cnt++ < 3)
                {
                    ROS_ERROR("get version start to resend msg....");
                    goto get_version_restart;
                }
                ROS_ERROR("CAN NOT COMMUNICATE with powerboard mcu, get version failed !");
                err_cnt = 0;
            }

        }
        /* --------  get version protocol end -------- */








        /* --------  adc protocol begin -------- */
        do
        {
            boost::mutex::scoped_lock(mtx);
            if(!pNoahPowerboard->get_adc_vector.empty())
            {
                auto a = pNoahPowerboard->get_adc_vector.begin(); 
                get_adc = *a;
                if((get_adc.frq <= 10 ) ||  (get_adc.frq == 0xff))
                {
                    get_adc_flag = 1;            
                }

                pNoahPowerboard->get_adc_vector.erase(a);

            }

        }while(0);

        if(get_adc_flag == 1)
        {   
            uint8_t flag = 0;
            uint32_t time_out_cnt = 0;
            static uint8_t err_cnt = 0;

            get_adc_flag = 0;

            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("get_adc.frq = %d",get_adc.frq);
                ROS_INFO("get get adc cmd");
            }
            do
            {
                boost::mutex::scoped_lock(mtx);
                pNoahPowerboard->get_adc_ack_vector.clear();
            }while(0);

get_adc_restart:
            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("module ctrl:send cmd to mcu");
            }
            pNoahPowerboard->GetAdcData(pNoahPowerboard->sys_powerboard);
            bool get_adc_ack_flag = 0;
            get_adc_ack_t get_adc_ack;
            while(time_out_cnt < GET_ADC_TIME_OUT/10)
            {
                time_out_cnt++;
                do
                {
                    boost::mutex::scoped_lock(mtx);
                    if(!pNoahPowerboard->get_adc_ack_vector.empty())
                    {
                        if(pNoahPowerboard->is_log_on == true)
                        {
                            ROS_INFO("get_adc_ack_vector is not empty");
                        }
                        auto b = pNoahPowerboard->get_adc_ack_vector.begin();

                        get_adc_ack = *b;

                        pNoahPowerboard->get_adc_ack_vector.erase(b);

                        //if((get_adc ) && (module_set_ack.on_off <= 1) && (module_set_ack.group_num <= 2))
                        {
                            get_adc_ack_flag = 1;
                            if(pNoahPowerboard->is_log_on == true)
                            {
                                ROS_INFO("get right get adc ack");
                            }
                        }
                    }
                }while(0);
                if(get_adc_ack_flag == 1)
                {
                    get_adc_ack_flag = 0;
                    if(pNoahPowerboard->is_log_on == true)
                    {
                        ROS_INFO("get right get adc ack");
                    }
                    memcpy(&(pNoahPowerboard->sys_powerboard->voltage_info.voltage_data), &get_adc_ack, sizeof(get_adc_ack_t));
                    if(pNoahPowerboard->is_log_on == true)
                    {
                        ROS_INFO("get get_adc_ack_vector data");
                        ROS_INFO("5v dcdc:%d",pNoahPowerboard->sys_powerboard->voltage_info.voltage_data._5V_voltage);
                        ROS_INFO("12v dcdc:%d",pNoahPowerboard->sys_powerboard->voltage_info.voltage_data._12V_voltage);
                        ROS_INFO("24v dcdc:%d",pNoahPowerboard->sys_powerboard->voltage_info.voltage_data._24V_voltage);
                        ROS_INFO("bat voltage:%d",pNoahPowerboard->sys_powerboard->voltage_info.voltage_data.bat_voltage);

                        ROS_INFO("24v temperature:%d",pNoahPowerboard->sys_powerboard->voltage_info.voltage_data._24V_temp);
                        ROS_INFO("12v temperature:%d",pNoahPowerboard->sys_powerboard->voltage_info.voltage_data._12V_temp);
                        ROS_INFO("5v temperature:%d",pNoahPowerboard->sys_powerboard->voltage_info.voltage_data._5V_temp);
                        ROS_INFO("air temperature:%d",pNoahPowerboard->sys_powerboard->voltage_info.voltage_data.air_temp);
                    }
                    break;
                }
                else
                {
                    usleep(10*1000);
                }
            }
            if(time_out_cnt < GET_ADC_TIME_OUT/10)
            {
                ROS_INFO("get adc flow OK");
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("get adc time out");
                time_out_cnt = 0;
                if(err_cnt++ < 3)
                {
                    ROS_ERROR("get adc start to resend msg....");
                    goto get_adc_restart;
                }
                ROS_ERROR("CAN NOT COMMUNICATE with powerboard mcu, get adc failed !");
                err_cnt = 0;
            }

        }
        /* --------  adc protocol end -------- */






        /* --------  serial led protocol begin -------- */
        do
        {
            boost::mutex::scoped_lock(mtx);
            if(!pNoahPowerboard->set_leds_effect_vector.empty())
            {
                auto a = pNoahPowerboard->set_leds_effect_vector.begin(); 
                set_led_effect = *a;
                {
                    set_led_effect_flag = 1;            
                }

                pNoahPowerboard->set_leds_effect_vector.erase(a);

            }

        }while(0);

        if(set_led_effect_flag == 1)
        {   
            uint8_t flag = 0;
            uint32_t time_out_cnt = 0;
            static uint8_t err_cnt = 0;

            set_led_effect_flag = 0;

            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("set_led_effect.mode = %d",set_led_effect.mode);
                ROS_INFO("get set led effect cmd");
            }
            do
            {
                boost::mutex::scoped_lock(mtx);
                pNoahPowerboard->set_leds_effect_ack_vector.clear();
            }while(0);

set_leds_effect_restart:
            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("set led effect :send cmd to mcu");
            }
            pNoahPowerboard->sys_powerboard->led_set.mode = set_led_effect.mode;
            pNoahPowerboard->sys_powerboard->led_set.color = set_led_effect.color;
            pNoahPowerboard->sys_powerboard->led_set.period = set_led_effect.period;

            pNoahPowerboard->SetLedEffect(pNoahPowerboard->sys_powerboard);
            bool set_led_effect_ack_flag = 0;
            set_leds_effect_ack_t set_led_effect_ack;
            while(time_out_cnt < SET_LED_EFFECT_TIME_OUT/10)
            {
                time_out_cnt++;
                do
                {
                    boost::mutex::scoped_lock(mtx);
                    if(!pNoahPowerboard->set_leds_effect_ack_vector.empty())
                    {
                        if(pNoahPowerboard->is_log_on == true)
                        {
                            ROS_INFO("set_leds_effect_ack_vector is not empty");
                        }
                        auto b = pNoahPowerboard->set_leds_effect_ack_vector.begin();

                        set_led_effect_ack = *b;

                        pNoahPowerboard->set_leds_effect_ack_vector.erase(b);

                        if(set_led_effect_ack.mode == set_led_effect.mode)
                        {
                            set_led_effect_ack_flag = 1;
                            if(pNoahPowerboard->is_log_on == true)
                            {
                                ROS_INFO("get right set led effect ack");
                            }
                        }
                    }
                }while(0);
                if(set_led_effect_ack_flag == 1)
                {
                    set_led_effect_ack_flag = 0;
                    if(pNoahPowerboard->is_log_on == true)
                    {
                        ROS_INFO("get right set led effect ack");
                    }
                    pNoahPowerboard->sys_powerboard->led.mode = set_led_effect_ack.mode;
                    pNoahPowerboard->sys_powerboard->led.color = set_led_effect_ack.color;
                    pNoahPowerboard->sys_powerboard->led.period = set_led_effect_ack.period;
                    if(pNoahPowerboard->is_log_on == true)
                    {
                        ROS_INFO("get led mode:%d",set_led_effect_ack.mode);
                        ROS_INFO("get led color:r=%d,g=%d,b=%d",set_led_effect_ack.color.r,set_led_effect_ack.color.g,set_led_effect_ack.color.b);
                        ROS_INFO("get led period:%d",set_led_effect_ack.period);
                    }
                    break;
                }
                else
                {
                    usleep(10*1000);
                }
            }
            if(time_out_cnt < SET_LED_EFFECT_TIME_OUT/10)
            {
                ROS_INFO("set led effect flow OK");
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("set led effect time out");
                time_out_cnt = 0;
                if(err_cnt++ < 3)
                {
                    ROS_ERROR("set led effect start to resend msg....");
                    goto set_leds_effect_restart;
                }
                ROS_ERROR("CAN NOT COMMUNICATE with powerboard mcu, set led effect failed !");
                err_cnt = 0;
            }

        }
        /* -------- serial led protocol end -------- */

        usleep(10 * 1000);
    }
}


int NoahPowerboard::PowerboardParamInit(void)
{
    sys_powerboard->led_set.mode = LIGHTS_MODE_NOMAL;
    return 0;
}



int NoahPowerboard::SetLedEffect(powerboard_t *powerboard)     // done
{
    int error = 0; 
    mrobot_driver_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_SET_LED_EFFECT;
    id.CanID_Struct.SrcMACID = 0;//CAN_SUB_PB_SRC_ID;
    id.CanID_Struct.DestMACID = NOAH_POWERBOARD_CAN_SRCMAC_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 7;
    can_msg.Data.resize(7);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = 0;//reserve
    can_msg.Data[2] = powerboard->led_set.mode;
    *(color_t*)&can_msg.Data[3] = powerboard->led_set.color;
    can_msg.Data[6] = powerboard->led_set.period;
    this->pub_to_can_node.publish(can_msg);
    return error;
}
int NoahPowerboard::GetBatteryInfo(powerboard_t *sys)      // done
{
    int error = 0; 
    mrobot_driver_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_GET_BAT_STATE;
    id.CanID_Struct.SrcMACID = 0;//CAN_SUB_PB_SRC_ID;
    id.CanID_Struct.DestMACID = NOAH_POWERBOARD_CAN_SRCMAC_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = 0;//reserve
    this->pub_to_can_node.publish(can_msg);
    return error;
}
int NoahPowerboard::SetModulePowerOnOff(powerboard_t *sys)
{
    int error = 0; 

    mrobot_driver_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_SET_MODULE_STATE;
    id.CanID_Struct.SrcMACID = 0;//CAN_SUB_PB_SRC_ID;
    id.CanID_Struct.DestMACID = NOAH_POWERBOARD_CAN_SRCMAC_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    //    ROS_INFO("Set Module State id: 0x%08x", id.CANx_ID);
    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 8;
    can_msg.Data.resize(8);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = 0;//reserve
    can_msg.Data[2] = 1;///group_num;
    uint32_t state = sys->module_status_set.module;
    *(uint32_t *)&can_msg.Data[3] = state;
    can_msg.Data[7] = sys->module_status_set.on_off;   
    this->pub_to_can_node.publish(can_msg);


    return error;
}

int NoahPowerboard::GetModulePowerOnOff(powerboard_t *sys)
{
    return 1;
}


int NoahPowerboard::GetAdcData(powerboard_t *sys)      // done
{
    int error = 0; 
    mrobot_driver_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_GET_ADC_DATA;
    id.CanID_Struct.SrcMACID = 0;//CAN_SUB_PB_SRC_ID;
    id.CanID_Struct.DestMACID = NOAH_POWERBOARD_CAN_SRCMAC_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = 0;//reserve
    this->pub_to_can_node.publish(can_msg);
    return error;
}

int NoahPowerboard::GetVersion(powerboard_t *sys)      // done
{
    int error = 0; 
    mrobot_driver_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_READ_VERSION;
    id.CanID_Struct.SrcMACID = 0;//CAN_SUB_PB_SRC_ID;
    id.CanID_Struct.DestMACID = NOAH_POWERBOARD_CAN_SRCMAC_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = sys->get_version_type;//reserve
    this->pub_to_can_node.publish(can_msg);
    return error;
}

int NoahPowerboard::GetSysStatus(powerboard_t *sys)     // done
{
    int error = 0; 
    mrobot_driver_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_GET_SYS_STATE;
    id.CanID_Struct.SrcMACID = 0;//CAN_SUB_PB_SRC_ID;
    id.CanID_Struct.DestMACID = NOAH_POWERBOARD_CAN_SRCMAC_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = 0;//reserve
    this->pub_to_can_node.publish(can_msg);
    return error;
}

int NoahPowerboard::InfraredLedCtrl(powerboard_t *sys)     // done
{
    int error = 0; 
    mrobot_driver_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_SET_IR_LED_LIGHTNESS;
    id.CanID_Struct.SrcMACID = 0;//CAN_SUB_PB_SRC_ID;
    id.CanID_Struct.DestMACID = NOAH_POWERBOARD_CAN_SRCMAC_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = sys->ir_cmd.lightness_percent; 
    this->pub_to_can_node.publish(can_msg);
    return error;
}




void NoahPowerboard::pub_json_msg_to_app( const nlohmann::json j_msg)
{
    std_msgs::String pub_json_msg;
    std::stringstream ss;

    ss.clear();
    ss << j_msg;
    pub_json_msg.data = ss.str();
    this->noah_powerboard_pub.publish(pub_json_msg);
}


void NoahPowerboard::from_app_rcv_callback(const std_msgs::String::ConstPtr &msg)
{
    //ROS_INFO("Rcv test data");
    auto j = json::parse(msg->data.c_str());
    if(j.find("pub_name") != j.end())
    {

        //ROS_INFO("find pub_name");
        if(j["pub_name"] == "set_module_state")
        {
            boost::mutex::scoped_lock(mtx);
            if(j["data"]["dev_name"] == "_24v_printer")
            {
                if(j["data"]["set_state"] == true)
                {
                    ROS_INFO("set 24v printer on");
                    module_ctrl_t param;  
                    param.group_num = 1;
                    param.module = POWER_24V_PRINTER;    
                    param.on_off = MODULE_CTRL_ON;
                    this->module_set_vector.push_back(param);
                }
                else if(j["data"]["set_state"] == false)
                {
                    ROS_INFO("set 24v printer off");
                    module_ctrl_t param;  
                    param.group_num = 1;
                    param.module = POWER_24V_PRINTER;    
                    param.on_off = MODULE_CTRL_OFF;
                    this->module_set_vector.push_back(param);
                }
            }

            if(j["data"]["dev_name"] == "_24v_dcdc")
            {
                if(j["data"]["set_state"] == true)
                {
                    ROS_INFO("set 24v dcdc on");
                    module_ctrl_t param;  
                    param.group_num = 1;
                    param.module = POWER_24V_EN;    
                    param.on_off = MODULE_CTRL_ON;
                    this->module_set_vector.push_back(param);
                }
                else if(j["data"]["set_state"] == false)
                {
                    ROS_INFO("set 24v dcdc off");
                    module_ctrl_t param;  
                    param.group_num = 1;
                    param.module = POWER_24V_EN;    
                    param.on_off = MODULE_CTRL_OFF;
                    this->module_set_vector.push_back(param);
                }
            }

            if(j["data"]["dev_name"] == "_5v_dcdc")
            {
                if(j["data"]["set_state"] == true)
                {
                    ROS_INFO("set 5v dcdc on");
                    module_ctrl_t param;  
                    param.group_num = 1;
                    param.module = POWER_5V_EN;    
                    param.on_off = MODULE_CTRL_ON;
                    this->module_set_vector.push_back(param);
                }
                else if(j["data"]["set_state"] == false)
                {
                    ROS_INFO("set 5v dcdc off");
                    module_ctrl_t param;  
                    param.group_num = 1;
                    param.module = POWER_5V_EN;    
                    param.on_off = MODULE_CTRL_OFF;
                    this->module_set_vector.push_back(param);
                }

            }

            if(j["data"]["dev_name"] == "_12v_dcdc")
            {
                if(j["data"]["set_state"] == true)
                {
                    ROS_INFO("set 12v dcdc on");
                    module_ctrl_t param;  
                    param.group_num = 1;
                    param.module = POWER_12V_EN;    
                    param.on_off = MODULE_CTRL_ON;
                    this->module_set_vector.push_back(param);
                }
                else if(j["data"]["set_state"] == false)
                {
                    ROS_INFO("set 12v dcdc off");
                    module_ctrl_t param;  
                    param.group_num = 1;
                    param.module = POWER_12V_EN;    
                    param.on_off = MODULE_CTRL_OFF;
                    this->module_set_vector.push_back(param);
                }
            }

            if(j["data"]["dev_name"] == "door_ctrl_state")
            {

                if(j["data"]["set_state"] == true)
                {
                    ROS_INFO("set door ctrl  on");
                    module_ctrl_t param;  
                    param.group_num = 1;
                    param.module = POWER_VSYS_24V_NV;    
                    param.on_off = MODULE_CTRL_ON;
                    this->module_set_vector.push_back(param);
                }
                else if(j["data"]["set_state"] == false)
                {
                    ROS_INFO("set door ctrl off");
                    module_ctrl_t param;  
                    param.group_num = 1;
                    param.module = POWER_VSYS_24V_NV;    
                    param.on_off = MODULE_CTRL_OFF;
                    this->module_set_vector.push_back(param);
                }
            }

        }
    }
}



void NoahPowerboard:: from_navigation_rcv_callback(const std_msgs::String::ConstPtr &msg)
{
    int value = atoi(msg->data.c_str());

    module_ctrl_t param;  
    param.group_num = 1;

    ROS_INFO("camera led ctrl:value is %d",value);
    ROS_INFO("%s",msg->data.c_str());

    boost::mutex::scoped_lock(mtx);
    switch(value)
    {
        case 0:
            ROS_INFO("camera led ctrl:get 00"); 
            param.module = POWER_CAMERA_LED;    
            param.on_off = MODULE_CTRL_OFF;
            this->module_set_vector.push_back(param);
            break;
        case 1:
            ROS_INFO("camera led ctrl:get 01"); 
            param.module = POWER_CAMERA_LED;    
            param.on_off = MODULE_CTRL_ON;
            this->module_set_vector.push_back(param);
            break;
        case 10:
            ROS_INFO("camera led ctrl:get 10"); 
            param.module = POWER_CAMERA_LED;    
            param.on_off = MODULE_CTRL_ON;
            this->module_set_vector.push_back(param);
            break;
        case 11:
            ROS_INFO("camera led ctrl:get 11"); 
            param.module = POWER_CAMERA_LED;    
            param.on_off = MODULE_CTRL_ON;
            this->module_set_vector.push_back(param);
            break;
        default :
            break;

    }
}

void NoahPowerboard::PubPower(powerboard_t *sys)
{
    unsigned char power = 0;
    power = sys->bat_info.bat_percent;
    unsigned char status = sys->sys_status;    //std_msgs::Int8 msg;
    std_msgs::UInt8MultiArray bytes_msg;

    bytes_msg.data.push_back(power);
    bytes_msg.data.push_back(status);
    power_pub_to_app.publish(bytes_msg);
}
void NoahPowerboard::power_from_app_rcv_callback(std_msgs::UInt8MultiArray data)
{
    uint8_t value = data.data[0]; 
    if(this->is_log_on == true)
    {
        ROS_INFO("power_from_app_rcv_callback");
        ROS_INFO("data is %d",value);
    }
    if(value == 0)
    {
        this->PubPower(this->sys_powerboard);
    }
}

void NoahPowerboard::PubChargeStatus(uint8_t status)
{
    static uint8_t last_status = 0;
    std_msgs::UInt8MultiArray data;
    if(last_status != status)
    {
        data.data.push_back(status);
        pub_charge_status_to_move_base.publish(data);
        last_status = status;
    }

}
void NoahPowerboard::rcv_from_can_node_callback(const mrobot_driver_msgs::vci_can::ConstPtr &c_msg)
{
    mrobot_driver_msgs::vci_can can_msg;
    mrobot_driver_msgs::vci_can long_msg;
    CAN_ID_UNION id;

    long_msg = this->long_frame.frame_construct(c_msg);
    mrobot_driver_msgs::vci_can* msg = &long_msg;
    if( msg->ID == 0 ) 
    {
        return;
    }
    if(this->is_log_on == true)
    {
        for(uint8_t i = 0; i < msg->DataLen; i++)
        {
            ROS_INFO("msg->Data[%d] = 0x%x",i,msg->Data[i]);
        }
    }
    can_msg.ID = msg->ID;
    id.CANx_ID = can_msg.ID;
    can_msg.DataLen = msg->DataLen;
    if(id.CanID_Struct.SrcMACID != NOAH_POWERBOARD_CAN_SRCMAC_ID)
    {
        return ;
    }

    //can_msg.Data.resize(can_msg.DataLen);
    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_SET_MODULE_STATE)
    {
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_SET_MODULE_STATE");
        module_ctrl_ack_t module_ctrl_ack;
        module_ctrl_ack.module = *(uint32_t *)&msg->Data[2];
        module_ctrl_ack.group_num = msg->Data[1];
        module_ctrl_ack.on_off = msg->Data[10];

        module_ctrl_ack.module_status_ack  = *(uint32_t *)&msg->Data[6];
        do
        {
            boost::mutex::scoped_lock(this->mtx);        
            this->module_set_ack_vector.push_back(module_ctrl_ack);
        }while(0);

        if(this->is_log_on == true)
        {
            ROS_INFO("receive module : 0x%x",module_ctrl_ack.module);
            ROS_INFO("receive module state : 0x%x",module_ctrl_ack.module_status_ack);
        }

    }

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_GET_BAT_STATE)
    {
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_GET_BAT_STATE");
        get_bat_info_ack_t get_bat_info_ack;
        get_bat_info_ack.bat_vol = *(uint16_t *)&msg->Data[1];
        get_bat_info_ack.bat_percent = *(uint16_t *)&msg->Data[3];

        do
        {
            boost::mutex::scoped_lock(this->mtx);        
            this->get_bat_info_ack_vector.push_back(get_bat_info_ack);
        }while(0);

        if(this->is_log_on == true)
        {
            ROS_INFO("receive bat vol : %d",get_bat_info_ack.bat_vol);
            ROS_INFO("receive bat percent : %d",get_bat_info_ack.bat_percent);
        }
        this->sys_powerboard->bat_info.bat_vol = get_bat_info_ack.bat_vol;
        this->sys_powerboard->bat_info.bat_percent = get_bat_info_ack.bat_percent;
        //this->PubPower(this->sys_powerboard);
    }

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_GET_SYS_STATE)
    {
        if(id.CanID_Struct.ACK == 1)
        {
            ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_GET_SYS_STATE");
            get_sys_status_ack_t get_sys_status_ack;
            *(uint16_t *)&msg->Data[1];
            get_sys_status_ack.sys_status = *(uint16_t *)&msg->Data[1];
            this->sys_powerboard->sys_status = get_sys_status_ack.sys_status;

            do
            {
                boost::mutex::scoped_lock(this->mtx);        
                this->get_sys_status_ack_vector.push_back(get_sys_status_ack);
            }while(0);

            //this->PubPower(this->sys_powerboard);
        }
        if(id.CanID_Struct.ACK == 0)
        {
            ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_GET_SYS_STATE");
            ROS_INFO("sys_status :mcu upload");
            *(uint16_t *)&msg->Data[1];
            this->sys_powerboard->sys_status = *(uint16_t *)&msg->Data[1];

            if((this->sys_powerboard->sys_status & STATE_IS_RECHARGE_IN) || (this->sys_powerboard->sys_status & STATE_IS_CHARGER_IN))
            {
                if(this->sys_powerboard->sys_status & STATE_IS_RECHARGE_IN)
                {
                    ROS_INFO("recharge plug in");
                }
                if(this->sys_powerboard->sys_status & STATE_IS_CHARGER_IN)
                {
                    ROS_INFO("charge plug in");
                }
            }
            else
            {
                ROS_INFO("charge not plug in");
                ROS_INFO("recharge not plug in");
            }
        }
    }

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_SET_IR_LED_LIGHTNESS)
    {
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_SET_IR_LED_LIGHTNESS");
        set_ir_duty_ack_t set_ir_duty_ack;
        set_ir_duty_ack.duty = msg->Data[1];

        do
        {
            boost::mutex::scoped_lock(this->mtx);        
            this->set_ir_duty_ack_vector.push_back(set_ir_duty_ack);
        }while(0);

        this->sys_powerboard->ir_cmd.lightness_percent = set_ir_duty_ack.duty;
    }

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_READ_VERSION)
    {
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_READ_VERSION");
        get_version_ack_t get_version_ack;
        get_version_ack.get_version_type = msg->Data[0];
        if(get_version_ack.get_version_type == 1)//software version
        {
            memcpy(&get_version_ack.sw_version[0], &msg->Data[1], SW_VERSION_SIZE);
        }
        else if(get_version_ack.get_version_type == 2)//protocol version
        {
            memcpy(&get_version_ack.protocol_version[0], &msg->Data[1], PROTOCOL_VERSION_SIZE);
        }
        else if(get_version_ack.get_version_type == 3)//hardware version
        {
            memcpy(&get_version_ack.hw_version[0], &msg->Data[1], HW_VERSION_SIZE);
        }

        do
        {
            boost::mutex::scoped_lock(this->mtx);        
            this->get_version_ack_vector.push_back(get_version_ack);
        }while(0);
    }

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_GET_ADC_DATA)
    {
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_GET_ADC_DATA");
        get_adc_ack_t get_adc_ack;
        get_adc_ack = *(get_adc_ack_t *)&msg->Data[0];

        if(id.CanID_Struct.ACK == 1)
        {
            do
            {
                boost::mutex::scoped_lock(this->mtx);        
                this->get_adc_ack_vector.push_back(get_adc_ack);
            }while(0);
        }
        if(id.CanID_Struct.ACK == 0)
        {
            ROS_INFO("adc : mcu upload"); 
        }

    }

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_SET_LED_EFFECT)
    {
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_SET_LED_EFFECT");
        set_leds_effect_t set_led_effect_ack;

        set_led_effect_ack.mode = msg->Data[2];
        set_led_effect_ack.color = *(color_t *)&msg->Data[3];
        set_led_effect_ack.period = msg->Data[6];
        if(this->is_log_on == true)
        {
            ROS_INFO("led mode %d",set_led_effect_ack.mode);
        }
        if(id.CanID_Struct.ACK == 1)
        {
            do
            {
                boost::mutex::scoped_lock(this->mtx);        
                this->set_leds_effect_ack_vector.push_back(set_led_effect_ack);
            }while(0);
        }
        if(id.CanID_Struct.ACK == 0)
        {
            ROS_INFO("set led effect : mcu upload"); 
        }

    }
}


void NoahPowerboard::update_sys_status(void)
{
    uint16_t sys_status = this->sys_powerboard->sys_status;
    uint16_t  power_percent = this->sys_powerboard->bat_info.bat_percent;
    static bool charging_low_flag = 0;
    static bool charging_medium_flag = 0;
    static bool charging_full_flag = 0;
    static bool power_low_flag = 0;
    static bool power_medium_flag = 0;


    /* ---- set led effect ----*/
    set_leds_effect_t set_led_effect;
    set_led_effect.reserve = 0;
    ROS_INFO("sys status 0x%x",sys_status);
    if((sys_status & STATE_IS_RECHARGE_IN) || (sys_status & STATE_IS_CHARGER_IN))
    {
        if(power_percent < VBAT_POWER_CHARGING_LOW)
        {
            ROS_INFO("sys status is charging, power low, %d",power_percent);
            if(charging_low_flag == 0)
            {
                ROS_INFO("set led effect charging low power");
                set_led_effect.mode = LIGHTS_MODE_CHARGING_POWER_LOW;
                do
                {
                    boost::mutex::scoped_lock(this->mtx);        
                    this->set_leds_effect_vector.push_back(set_led_effect);
                }while(0);
                charging_low_flag = 1;
                charging_medium_flag = 0;
                charging_full_flag = 0;
            }
        }
        else if(power_percent < VBAT_POWER_CHARGING_MEDIUM)
        {
            ROS_INFO("sys status is charging, power medium, %d",power_percent);
            if(charging_medium_flag == 0)
            {
                ROS_INFO("set led effect charging medium power");
                set_led_effect.mode = LIGHTS_MODE_CHARGING_POWER_MEDIUM;
                do
                {
                    boost::mutex::scoped_lock(this->mtx);        
                    this->set_leds_effect_vector.push_back(set_led_effect);
                }while(0);
                charging_low_flag = 0;
                charging_medium_flag = 1;
                charging_full_flag = 0;
            }
        }
        else if(power_percent == VBAT_POWER_CHARGING_FULL)
        {
            ROS_INFO("sys status is charging, power full, %d",power_percent);
            if(charging_full_flag == 0)
            {
                ROS_INFO("set led effect charging full power");
                set_led_effect.mode = LIGHTS_MODE_CHARGING_FULL;
                do
                {
                    boost::mutex::scoped_lock(this->mtx);        
                    this->set_leds_effect_vector.push_back(set_led_effect);
                }while(0);
                charging_low_flag = 0;
                charging_medium_flag = 0;
                charging_full_flag = 1;
            }
        }
        
    }
    else
    {
        if(power_percent < VBAT_POWER_LOW_WARNING_PERCENTAGE)
        {
            ROS_INFO("sys status is not charging, power low warning, %d",power_percent);
            if(power_low_flag == 0)
            {
                ROS_INFO("set led effect not charging low warning power");
                set_led_effect.mode = LIGHTS_MODE_LOW_POWER;
                do
                {
                    boost::mutex::scoped_lock(this->mtx);        
                    this->set_leds_effect_vector.push_back(set_led_effect);
                }while(0);
                power_low_flag = 1;
                power_medium_flag = 0;
            }
        }
        else 
        {
            ROS_INFO("sys status is not charging, power normal, %d",power_percent);
            if(power_medium_flag == 0)
            {
                ROS_INFO("set led effect not charging normal power");
                set_led_effect.mode = LIGHTS_MODE_NOMAL;
                do
                {
                    boost::mutex::scoped_lock(this->mtx);        
                    this->set_leds_effect_vector.push_back(set_led_effect);
                }while(0);
                power_low_flag = 0;
                power_medium_flag = 1;
            }
        }
    }
}

