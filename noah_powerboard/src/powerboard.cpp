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
#include <can_long_frame.h>

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

            ROS_INFO("get set module cmd");
            ROS_INFO("module_set_param.module = 0x%x",module_set.module);
            ROS_INFO("module_set_param.group_num = %d",module_set.group_num);
            ROS_INFO("module_set_param.on_off = %d",module_set.on_off);
            do
            {
                boost::mutex::scoped_lock(mtx);
                pNoahPowerboard->module_set_ack_vector.clear();
            }while(0);

module_set_restart:
            pNoahPowerboard->sys_powerboard->module_status_set.module = module_set.module;
            pNoahPowerboard->sys_powerboard->module_status_set.group_num = module_set.group_num;
            pNoahPowerboard->sys_powerboard->module_status_set.on_off = module_set.on_off;
            ROS_INFO("module ctrl:send cmd to mcu");
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
                        ROS_INFO("module_set_ack_vector is not empty");
                        auto b = pNoahPowerboard->module_set_ack_vector.begin();

                        module_set_ack = *b;

                        pNoahPowerboard->module_set_ack_vector.erase(b);

                        ROS_INFO("module_set_ack.module = 0x%x",module_set_ack.module);
                        ROS_INFO("module_set_ack.group_num = %d",module_set_ack.group_num);
                        ROS_INFO("module_set_ack.on_off = %d",module_set_ack.on_off);
                        if((module_set_ack.module <= POWER_ALL ) && (module_set_ack.on_off <= 1) && (module_set_ack.group_num <= 2))
                        {
                            module_ack_flag = 1;
                            ROS_INFO("get right module ctrl ack");
                        }
                    }
                }while(0);
                if(module_ack_flag == 1)
                {
                    module_ack_flag = 0;
                    ROS_INFO("get module_set_ack_vector data");
                    if(module_set_ack.module == module_set.module)
                    {
                        ROS_INFO("get right module ack, module is 0x%x",module_set_ack.module);
                        ROS_INFO("now start to publish module ctrl relative topic");


                        if(module_set_ack.module & POWER_VSYS_24V_NV)
                        {
                            ROS_INFO("module %d",module_set_ack.module);

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

            ROS_INFO("get get_bat_info cmd");
            do
            {
                boost::mutex::scoped_lock(mtx);
                pNoahPowerboard->get_bat_info_ack_vector.clear();
            }while(0);

get_bat_info_restart:
            ROS_INFO("get bat info:send cmd to mcu");
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
                        ROS_INFO("get_bat_info_ack_vector is not empty");
                        auto b = pNoahPowerboard->get_bat_info_ack_vector.begin();

                        get_bat_info_ack = *b;

                        pNoahPowerboard->get_bat_info_ack_vector.erase(b);

                        ROS_INFO("get_bat_info_ack.bat_vol = 0x%x",get_bat_info_ack.bat_vol);
                        ROS_INFO("get_bat_info_ack.bat_percent = %d",get_bat_info_ack.bat_percent);
                        if( get_bat_info_ack.bat_percent <= 100)
                        {
                            get_bat_info_ack_flag = 1;
                            ROS_INFO("get get_bat_info ack");
                        }
                    }
                }while(0);
                if(get_bat_info_ack_flag == 1)
                {
                    get_bat_info_ack_flag = 0;
                    ROS_INFO("get get_bat_info_ack_vector data");
                    ROS_INFO("get right get_bat_info ack, bat_vol is %d",get_bat_info_ack.bat_vol);
                    ROS_INFO("get right get_bat_info ack, bat_percent is %d",get_bat_info_ack.bat_percent);

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

            ROS_INFO("get set ir duty cmd");
            do
            {
                boost::mutex::scoped_lock(mtx);
                pNoahPowerboard->set_ir_duty_ack_vector.clear();
            }while(0);

set_ir_duty_restart:

            ROS_INFO("get sys status:send cmd to mcu");
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
                        ROS_INFO("set_ir_duty_ack_vector is not empty");
                        auto b = pNoahPowerboard->set_ir_duty_ack_vector.begin();
                        set_ir_duty_ack = *b;
                        pNoahPowerboard->set_ir_duty_ack_vector.erase(b);

                        ROS_INFO("set_ir_duty_ack.duty = %d",set_ir_duty_ack.duty);

                        if( set_ir_duty_ack.duty <= 100 )
                        {
                            set_ir_duty_ack_flag = 1;
                            ROS_INFO("get right set_ir_duty  ack");
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

            ROS_INFO("get get sys status cmd");
            do
            {
                boost::mutex::scoped_lock(mtx);
                pNoahPowerboard->get_sys_status_ack_vector.clear();
            }while(0);

get_sys_status_restart:

            ROS_INFO("get sys status:send cmd to mcu");
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
                        ROS_INFO("get_sys_status_vector is not empty");
                        auto b = pNoahPowerboard->get_sys_status_ack_vector.begin();
                        get_sys_status_ack = *b;
                        pNoahPowerboard->get_sys_status_ack_vector.erase(b);

                        ROS_INFO("get_sys_status_ack.sys_status = %d",get_sys_status_ack.sys_status);

                        if(get_sys_status_ack.sys_status <= 0x1ff )
                        {
                            get_sys_status_ack_flag = 1;
                            ROS_INFO("get right get_sys_status  ack");
                        }
                    }
                }while(0);
                if(get_sys_status_ack_flag == 1)
                {
                    get_sys_status_ack_flag = 0;
                    ROS_INFO("get get_sys_status_ack_vector data");
                    ROS_INFO("get right get_sys_status ack, status is 0x%x",get_sys_status_ack.sys_status);

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

            ROS_INFO("get get version cmd");
            ROS_INFO("get_version.get_version_type = 0x%x",get_version.get_version_type);
            do
            {
                boost::mutex::scoped_lock(mtx);
                pNoahPowerboard->get_version_ack_vector.clear();
            }while(0);

get_version_restart:
            pNoahPowerboard->sys_powerboard->get_version_type = get_version.get_version_type;
            ROS_INFO("get_verion:send cmd to mcu");
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
                        ROS_INFO("get_version_ack_vector is not empty"); 
                        auto b = pNoahPowerboard->get_version_ack_vector.begin();

                        get_version_ack = *b;

                        pNoahPowerboard->get_version_ack_vector.erase(b);

                        ROS_INFO("version type : %d",get_version_ack.get_version_type);
                        if( get_version_ack.get_version_type <= 3) 
                        {
                            get_version_ack_flag = 1;
                            ROS_INFO("get right get_version ack");
                        }
                    }
                }while(0);
                if(get_version_ack_flag == 1)
                {
                    get_version_ack_flag = 0;
                    ROS_INFO("get get_version_ack_vector data");
                    if(get_version_ack.get_version_type == get_version.get_version_type)
                    {
                        ROS_INFO("get right get version ack");
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

            ROS_INFO("get get adc cmd");
            ROS_INFO("get_adc.frq = %d",get_adc.frq);
            do
            {
                boost::mutex::scoped_lock(mtx);
                pNoahPowerboard->get_adc_ack_vector.clear();
            }while(0);

get_adc_restart:
            ROS_INFO("module ctrl:send cmd to mcu");
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
                        ROS_INFO("get_adc_ack_vector is not empty");
                        auto b = pNoahPowerboard->get_adc_ack_vector.begin();

                        get_adc_ack = *b;

                        pNoahPowerboard->get_adc_ack_vector.erase(b);

                        //if((get_adc ) && (module_set_ack.on_off <= 1) && (module_set_ack.group_num <= 2))
                        {
                            get_adc_ack_flag = 1;
                            ROS_INFO("get right get adc ack");
                        }
                    }
                }while(0);
                if(get_adc_ack_flag == 1)
                {
                    get_adc_ack_flag = 0;
                    ROS_INFO("get get_adc_ack_vector data");
                    ROS_INFO("get right get adc ack");
                    memcpy(&(pNoahPowerboard->sys_powerboard->voltage_info.voltage_data), &get_adc_ack, sizeof(get_adc_ack_t));
                    ROS_INFO("5v dcdc:%d",pNoahPowerboard->sys_powerboard->voltage_info.voltage_data._5V_voltage);
                    ROS_INFO("12v dcdc:%d",pNoahPowerboard->sys_powerboard->voltage_info.voltage_data._12V_voltage);
                    ROS_INFO("24v dcdc:%d",pNoahPowerboard->sys_powerboard->voltage_info.voltage_data._24V_voltage);
                    ROS_INFO("bat voltage:%d",pNoahPowerboard->sys_powerboard->voltage_info.voltage_data.bat_voltage);

                    ROS_INFO("24v temperature:%d",pNoahPowerboard->sys_powerboard->voltage_info.voltage_data._24V_temp);
                    ROS_INFO("12v temperature:%d",pNoahPowerboard->sys_powerboard->voltage_info.voltage_data._12V_temp);
                    ROS_INFO("5v temperature:%d",pNoahPowerboard->sys_powerboard->voltage_info.voltage_data._5V_temp);
                    ROS_INFO("air temperature:%d",pNoahPowerboard->sys_powerboard->voltage_info.voltage_data.air_temp);
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

            ROS_INFO("get set led effect cmd");
            ROS_INFO("set_led_effect.mode = %d",set_led_effect.mode);
            do
            {
                boost::mutex::scoped_lock(mtx);
                pNoahPowerboard->set_leds_effect_ack_vector.clear();
            }while(0);

set_leds_effect_restart:
            ROS_INFO("set led effect :send cmd to mcu");
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
                        ROS_INFO("set_leds_effect_ack_vector is not empty");
                        auto b = pNoahPowerboard->set_leds_effect_ack_vector.begin();

                        set_led_effect_ack = *b;

                        pNoahPowerboard->set_leds_effect_ack_vector.erase(b);

                        //if((get_adc ) && (module_set_ack.on_off <= 1) && (module_set_ack.group_num <= 2))
                        {
                            set_led_effect_ack_flag = 1;
                            ROS_INFO("get right set led effect ack");
                        }
                    }
                }while(0);
                if(set_led_effect_ack_flag == 1)
                {
                    set_led_effect_ack_flag = 0;
                    ROS_INFO("get set_leds_effect_ack_vector data");
                    ROS_INFO("get right set led effect ack");
                    pNoahPowerboard->sys_powerboard->led.mode = set_led_effect_ack.mode;
                    pNoahPowerboard->sys_powerboard->led.color = set_led_effect_ack.color;
                    pNoahPowerboard->sys_powerboard->led.period = set_led_effect_ack.period;
                    ROS_INFO("get led mode:%d",set_led_effect_ack.mode);
                    ROS_INFO("get led color:r=%d,g=%d,b=%d",set_led_effect_ack.color.r,set_led_effect_ack.color.g,set_led_effect_ack.color.b);
                    ROS_INFO("get led period:%d",set_led_effect_ack.period);
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
    sys_powerboard->led_set.mode = LIGHTS_MODE_DEFAULT;
    return 0;
}


int NoahPowerboard::send_serial_data(powerboard_t *sys)
{
    boost::mutex io_mutex;
    boost::mutex::scoped_lock lock(io_mutex);
}

uint8_t NoahPowerboard::CalCheckSum(uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for(uint8_t i = 0; i < len; i++)
    {
        sum += data[i];
    }
    return sum;  
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


#if 0

    error = this->handle_receive_data(sys);
    if(error < 0)
    {
        if(err_cnt++ < COM_ERR_REPEAT_TIME)
        {
            usleep(50*1000); 
            goto begin; 
        }
        ROS_ERROR("com error");
        if(sys->module_status_set.module & POWER_VSYS_24V_NV)
        {

            this->j.clear();
            this->j = 
            {
                {"sub_name","set_module_state"},
                {
                    "data",
                    {
                        //{"_xx_xxx_state",!(bool)(sys->module_status.module & POWER_5V_EN)},
                        {"door_ctrl_state",(bool)(sys->module_status.module & POWER_VSYS_24V_NV)},
                        {"error_code", error},
                    } 
                }
            };
            this->pub_json_msg_to_app(this->j);
        }
    }
    else 
    {
        err_cnt = 0;


        if(sys->module_status_set.module & POWER_24V_EXTEND)
        {

        }
    }
#endif
    return error;
}

int NoahPowerboard::GetModulePowerOnOff(powerboard_t *sys)
{
    int error = -1;
    sys->send_data_buf[0] = PROTOCOL_HEAD;
    sys->send_data_buf[1] = 6;
    sys->send_data_buf[2] = FRAME_TYPE_GET_MODULE_STATE;
    sys->send_data_buf[3] = 1;
    sys->send_data_buf[4] = CalCheckSum(sys->send_data_buf, 4);
    sys->send_data_buf[5] = PROTOCOL_TAIL;
    this->send_serial_data(sys);
    usleep(TEST_WAIT_TIME);
    error = this->handle_receive_data(sys);
    if(error < 0)
    {

    }
    return error;
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


int NoahPowerboard::handle_receive_data(powerboard_t *sys)
{
    int nread = 0;
    int i = 0;
    int j = 0;
    int data_Len = 0;
    int frame_len = 0;
    unsigned char recv_buf[BUF_LEN] = {0};
    unsigned char recv_buf_complete[BUF_LEN] = {0};
    unsigned char recv_buf_temp[BUF_LEN] = {0};

    struct stat file_info;
    int error = -1;
    if(0 != last_unread_bytes)
    {
        for(j=0;j<last_unread_bytes;j++)
        {
            recv_buf_complete [j] = recv_buf_last[j];
        }
    }
    //PowerboardInfo("start read ...");
    //PowerboardInfo("rcv device is %d",sys->device);
    if((nread = read(sys->device, recv_buf, BUF_LEN))>0)
    { 
        //PowerboardInfo("read complete ... ");
        memcpy(recv_buf_complete+last_unread_bytes,recv_buf,nread);
        data_Len = last_unread_bytes + nread;
        last_unread_bytes = 0;
        //        for(i = 0; i < data_Len; i++)
        //        {
        //            PowerboardInfo("rcv buf %2x ", recv_buf_complete[last_unread_bytes + i]);
        //        }

        while(i<data_Len)
        {
            if(0x5A == recv_buf_complete [i])
            {

                frame_len = recv_buf_complete[i+1]; 
                if(i+frame_len <= data_Len)
                {
                    if(0xA5 == recv_buf_complete[i+frame_len-1])
                    {
                        for(j=0;j<frame_len;j++)
                        {
                            recv_buf_temp[j] = recv_buf_complete[i+j];
                            //PowerboardInfo("rcv buf %d is %2x",j, recv_buf_temp[j]);
                        }

                        error = this->handle_rev_frame(sys,recv_buf_temp);
                        i = i+ frame_len;
                    }
                    else
                    {
                        i++;
                    }
                }
                else
                {
                    last_unread_bytes = data_Len - i;
                    for(j=0;j<last_unread_bytes;j++)
                    {
                        recv_buf_last[j] = recv_buf_complete[i+j];
                    }
                    break;
                }
            }
            else 
            {
                i++;
            }
        }
    }
    else 
    {
        i = stat(sys->dev,&file_info);
        if(-1 == i)
        {
            //sys->com_state = COM_CLOSING;
        }
    }
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

int NoahPowerboard::handle_rev_frame(powerboard_t *sys,unsigned char * frame_buf)
{
    int frame_len = 0;
    int i = 0;
    int j = 0;
    int command = 0; 
    unsigned char check_data = 0;
    uint8_t cmd_type = 0;
    int error = -1;
    frame_len = frame_buf[1];

    for(i=0;i<frame_len-2;i++)
    {
        check_data += frame_buf[i];
    }

    if(check_data != frame_buf[frame_len-2] || PROTOCOL_TAIL != frame_buf[frame_len -1])
    {
        PowerboardInfo("led receive frame check error");
        return -1;
    }
    //    PowerboardInfo("Powrboard recieve data check OK.");
    cmd_type = frame_buf[2];
    switch(cmd_type)
    {
        case FRAME_TYPE_LEDS_CONTROL:
            //rcv_serial_leds_frame_t rcv_serial_led_frame;
            memcpy((uint8_t *)&sys->rcv_serial_leds_frame, &frame_buf[3], sizeof(rcv_serial_leds_frame_t) );
            PowerboardInfo("Get leds mode is %d", sys->rcv_serial_leds_frame.cur_light_mode );
            PowerboardInfo("color.r is %2x",       sys->rcv_serial_leds_frame.color.r);
            PowerboardInfo("color.g is %2x",        sys->rcv_serial_leds_frame.color.g);
            PowerboardInfo("color.b is %2x",        sys->rcv_serial_leds_frame.color.b);
            PowerboardInfo("period is %d",          sys->rcv_serial_leds_frame.period);
            error = FRAME_TYPE_LEDS_CONTROL;
            break;

        case FRAME_TYPE_BAT_STATUS:
            sys->bat_info.cmd = frame_buf[3];
            error = FRAME_TYPE_BAT_STATUS;
            break;

        case FRAME_TYPE_GET_CURRENT:
            memcpy((uint8_t *)&sys->voltage_info.voltage_data, &frame_buf[4], sizeof(voltage_data_t));
            PowerboardInfo("_12v_voltage     is %5d mv",sys->voltage_info.voltage_data._12V_voltage);
            PowerboardInfo("_24v_voltage     is %5d mv",sys->voltage_info.voltage_data._24V_voltage);
            PowerboardInfo("_5v_voltage      is %5d mv",sys->voltage_info.voltage_data._5V_voltage);
            PowerboardInfo("bat_voltage     is %5d mv",sys->voltage_info.voltage_data.bat_voltage);
            PowerboardInfo("_24V_temp       is %d",sys->voltage_info.voltage_data._24V_temp);
            PowerboardInfo("_12V_temp       is %d",sys->voltage_info.voltage_data._12V_temp);
            PowerboardInfo("_5V_temp        is %d",sys->voltage_info.voltage_data._5V_temp);
            PowerboardInfo("air_temp        is %d",sys->voltage_info.voltage_data.air_temp);
            PowerboardInfo("send_rate       is %d",sys->voltage_info.send_rate);
            error = FRAME_TYPE_GET_CURRENT;
            break;

        case FRAME_TYPE_GET_VERSION:
            sys->get_version_type = frame_buf[3];
            if(sys->get_version_type == VERSION_TYPE_FW)
            {
                memcpy((uint8_t *)&sys->hw_version,&frame_buf[4], HW_VERSION_SIZE);
                memcpy((uint8_t *)&sys->sw_version,&frame_buf[7], SW_VERSION_SIZE);
                PowerboardInfo("hw version: %s",sys->hw_version);
                PowerboardInfo("sw version: %s",sys->sw_version);
            }

            if(sys->get_version_type == VERSION_TYPE_PROTOCOL)
            {
                memcpy((uint8_t *)&sys->protocol_version,&frame_buf[4], PROTOCOL_VERSION_SIZE);
                PowerboardInfo("protocol version: %s",sys->protocol_version);
            }

            error = FRAME_TYPE_GET_VERSION;
            break;

        case FRAME_TYPE_SYS_STATUS:
            sys->sys_status = (frame_buf[4]) | (frame_buf[5] << 8);
            ROS_INFO("sys_status is :%04x",sys->sys_status);
            switch(sys->sys_status & 0x0f)
            {
                case SYS_STATUS_OFF:
                    //PowerboardInfo("system status: off");
                    break;
                case SYS_STATUS_TURNING_ON:
                    //PowerboardInfo("system status: turning on...");
                    break;
                case SYS_STATUS_ON:
                    //PowerboardInfo("system status: on");
                    break;
                case SYS_STATUS_TURNING_OFF:
                    //PowerboardInfo("system status:turning off...");
                    break;
                case SYS_STATUS_ERR:
                    //PowerboardInfo("system status:error");
                    break;
                default:
                    break;
            }
            if(sys->sys_status & STATE_IS_CHARGER_IN )
            {
                ROS_INFO("charger plug in");
            }
            else
            {
                ROS_INFO("charger not plug in");
            }

            if(sys->sys_status & STATE_IS_RECHARGE_IN )
            {
                ROS_INFO("recharger plug in");
                this->PubChargeStatus(1);
            }
            else
            {
                ROS_INFO("recharger not plug in");
                this->PubChargeStatus(0);
            }

            error = FRAME_TYPE_SYS_STATUS;
            break;

        case FRAME_TYPE_IRLED_CONTROL:
            sys->ir_cmd.lightness_percent = frame_buf[3];
            PowerboardInfo("ir lightness is %d",sys->ir_cmd.lightness_percent);
            error = FRAME_TYPE_IRLED_CONTROL;
            break;
        case FRAME_TYPE_MODULE_CONTROL:
            {
                uint32_t tmp = 0;
                memcpy((uint8_t *)&tmp, &frame_buf[3], 4);
                if(tmp != HW_NO_SUPPORT)
                {
                    sys->module_status.module = tmp;
                    PowerboardInfo("sys->module_status.module is %08x",sys->module_status.module);
                }
                else
                {
                    PowerboardInfo("hard ware not support !");
                }
                error = FRAME_TYPE_MODULE_CONTROL;
                break; 
            }
        case FRAME_TYPE_GET_MODULE_STATE:
            {
                uint32_t tmp = 0;
                memcpy((uint8_t *)&tmp, &frame_buf[4], 4);
                if(tmp != HW_NO_SUPPORT)
                {
                    sys->module_status.module = tmp;
                    PowerboardInfo("sys->module_status.module is %08x",sys->module_status.module);
                }
                else
                {
                    PowerboardInfo("hardware not support !");
                }
                error = FRAME_TYPE_GET_MODULE_STATE;
                break; 
            }

        default :
            break;
    }
    return error;
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
    ROS_INFO("power_from_app_rcv_callback");
    ROS_INFO("data is %d",value);
    if(value == 0)
    {
        //this->PubPower();
        get_bat_info_t  get_bat_info;
        get_bat_info.reserve = 0;
        this->get_bat_info_vector.push_back(get_bat_info);
        usleep(10*1000);
        get_sys_status_t get_sys_status;
        get_sys_status.reserve = 0;
        this->get_sys_status_vector.push_back(get_sys_status);
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
    for(uint8_t i = 0; i < msg->DataLen; i++)
    {
        ROS_INFO("msg->Data[%d] = 0x%x",i,msg->Data[i]);
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

        ROS_INFO("receive module : 0x%x",module_ctrl_ack.module);
        ROS_INFO("receive module state : 0x%x",module_ctrl_ack.module_status_ack);

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

        ROS_INFO("receive bat vol : %d",get_bat_info_ack.bat_vol);
        ROS_INFO("receive bat percent : %d",get_bat_info_ack.bat_percent);
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
            get_sys_status_ack.sys_status = *(uint16_t *)&msg->Data[1];

            do
            {
                boost::mutex::scoped_lock(this->mtx);        
                this->get_sys_status_ack_vector.push_back(get_sys_status_ack);
            }while(0);

            this->sys_powerboard->sys_status = get_sys_status_ack.sys_status;
            this->PubPower(this->sys_powerboard);
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
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_GET_ADC_DATA");
        set_leds_effect_t set_led_effect_ack;

        set_led_effect_ack.mode = msg->Data[2];
        set_led_effect_ack.color = *(color_t *)&msg->Data[3];
        set_led_effect_ack.period = msg->Data[6];
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



