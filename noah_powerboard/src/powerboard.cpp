/*
 *  powerboard.cpp
 *  Communicate Protocol.
 *  Author: Kaka Xie
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <powerboard.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <mrobot_msgs/vci_can.h>
#include <roscan/can_long_frame.h>

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

//class NoahPowerboard;
#define MODULE_SET_TIME_OUT                     500//ms
#define GET_BAT_INFO_TIME_OUT                   500//ms
#define GET_SYS_STSTUS_TIME_OUT                 500//ms
#define SET_IR_DUTY_TIME_OUT                    500//ms
#define GET_VERSION_TIME_OUT                    500//ms
#define GET_ADC_TIME_OUT                        500//ms
#define SET_LED_EFFECT_TIME_OUT                 500//ms
#define SET_REMOTE_POWER_CTRL_TIME_OUT          500//ms
#define GET_SERIALS_LEDS_VERSION_TIME_OUT       500//ms
#define SET_CONVEYOR_BELT_WORK_MODE_TIME_OUT    500//ms
#define SET_STATUS_LED_TIME_OUT                 500//ms
#define GET_DEV_ID_TIME_OUT                     500//ms

#define MODULE_SET_RETRY_CNT                    6
#define SET_IR_DUTY_RETRY_CNT                   6
#define GET_VERSION_RETRY_CNT                   5
#define GET_ADC_RETRY_CNT                       3
#define SET_LED_EFFECT_RETRY_CNT                6
#define SET_REMOTE_POWER_CTRL_RETRY_CNT         6
#define GET_SERIALS_LEDS_VERSION_RETRY_CNT      5
#define SET_CONVEYOR_BELT_WORK_MODE_RETRY_CNT   5
#define SET_LED_STATUS_RETRY_CNT                6
#define GET_DEV_ID_RETRY_CNT                    8
void *CanProtocolProcess(void* arg)
{
    module_ctrl_t module_set;
    get_bat_info_t get_bat_info;
    get_sys_status_t get_sys_status;
    set_ir_duty_t set_ir_duty;
    get_version_t get_version;
    get_adc_t   get_adc;
    set_leds_effect_t set_led_effect;
    remote_power_ctrl_t remote_power_ctrl;
    get_serials_leds_version_t get_serials_leds_version;
    conveyor_belt_t set_conveyor_belt_mode;
    status_led_t status_led;
    dev_id_t get_dev_id_tmp;

    NoahPowerboard *pNoahPowerboard =  (NoahPowerboard*)arg;

    bool module_flag = 0;
    bool get_bat_info_flag = 0;
    bool get_sys_status_flag = 0;
    bool set_ir_duty_flag = 0;
    bool get_version_flag = 0;
    bool get_adc_flag = 0;
    bool set_led_effect_flag = 0;
    bool remote_power_ctrl_flag = 0;
    bool get_serials_leds_version_flag = 0;
    bool set_conveyor_belt_flag = 0;
    bool set_led_status_flag = 0;
    bool get_dev_id_flag = 0;
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


                        if(module_set_ack.module & POWER_DOOR_CTRL)
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
                                        {"door_ctrl_state",(bool)(module_set_ack.module_status_ack & POWER_DOOR_CTRL)},
                                        {"error_code", CAN_SOURCE_ID_SET_MODULE_STATE},
                                    }
                                }
                            };
                            pNoahPowerboard->pub_json_msg_to_app(pNoahPowerboard->j);
                        }


                        if((module_set_ack.module & POWER_3V3_CARD_EN_1) || (module_set_ack.module & POWER_3V3_CARD_EN_2) || \
                                (module_set_ack.module & POWER_3V3_CARD_EN_3) || (module_set_ack.module & POWER_3V3_CARD_EN_4) )
                        {
                            bool on_off_status = false;
                            int door_id;
                            if(module_set_ack.module & POWER_3V3_CARD_EN_1)
                            {
                                door_id = 1;
                                on_off_status = (bool)(module_set_ack.module_status_ack & POWER_3V3_CARD_EN_1);

                            }
                            if(module_set_ack.module & POWER_3V3_CARD_EN_2)
                            {
                                door_id = 2;
                                on_off_status = (bool)(module_set_ack.module_status_ack & POWER_3V3_CARD_EN_2);
                            }
                            if(module_set_ack.module & POWER_3V3_CARD_EN_3)
                            {
                                door_id = 3;
                                on_off_status = (bool)(module_set_ack.module_status_ack & POWER_3V3_CARD_EN_3);
                            }
                            if(module_set_ack.module & POWER_3V3_CARD_EN_4)
                            {
                                door_id = 4;
                                on_off_status = (bool)(module_set_ack.module_status_ack & POWER_3V3_CARD_EN_4);
                            }

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
                                        {"door_ctrl_state",on_off_status},
                                        {"error_code", CAN_SOURCE_ID_SET_MODULE_STATE},
                                        {"door_id", door_id},
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
                //ROS_INFO("module ctrl flow OK");
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("module ctrl time out");
                time_out_cnt = 0;
                if(err_cnt++ < MODULE_SET_RETRY_CNT)
                {
                    ROS_ERROR("module ctrl start to resend msg....");
                    goto module_set_restart;
                }
                ROS_ERROR("CAN NOT COMMUNICATE with powerboard mcu, module ctrl failed !");
                err_cnt = 0;

                if(module_set.module & POWER_DOOR_CTRL)
                {
                    ROS_ERROR("set door ctrl time out !");

                    pNoahPowerboard->j.clear();
                    pNoahPowerboard->j =
                    {
                        {"sub_name","set_module_state"},
                        {
                            "data",
                            {
                                {"door_ctrl_state",(bool)(module_set_ack.module_status_ack & POWER_DOOR_CTRL)},
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

#if 0
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
                //ROS_INFO("get_bat_info flow OK");
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
#endif
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

            pNoahPowerboard->sys_powerboard->ir_cmd.set_ir_percent = set_ir_duty.duty;
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
                //ROS_INFO("set ir duty flow OK");
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("set ir duty time out");
                time_out_cnt = 0;
                if(err_cnt++ < SET_IR_DUTY_RETRY_CNT)
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
#if 0
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
                //ROS_INFO("get sys status flow OK");
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
#endif
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
                //ROS_INFO("get version flow OK");
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("get version time out");
                time_out_cnt = 0;
                if(err_cnt++ < GET_VERSION_RETRY_CNT)
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
                //ROS_INFO("get adc flow OK");
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("get adc time out");
                time_out_cnt = 0;
                if(err_cnt++ < GET_ADC_RETRY_CNT)
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
                //ROS_INFO("set led effect flow OK");
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("set led effect time out");
                time_out_cnt = 0;
                if(err_cnt++ < SET_LED_EFFECT_RETRY_CNT)
                {
                    ROS_ERROR("set led effect start to resend msg....");
                    goto set_leds_effect_restart;
                }
                ROS_ERROR("CAN NOT COMMUNICATE with powerboard mcu, set led effect failed !");
                err_cnt = 0;
            }

        }
        /* -------- serial led protocol end -------- */


        /* --------  remote device power ctrl  protocol begin -------- */
        do
        {
            boost::mutex::scoped_lock(mtx);
            if(!pNoahPowerboard->remote_power_ctrl_vector.empty())
            {
                auto a = pNoahPowerboard->remote_power_ctrl_vector.begin();
                remote_power_ctrl = *a;
                {
                    remote_power_ctrl_flag = 1;
                }

                pNoahPowerboard->remote_power_ctrl_vector.erase(a);

            }

        }while(0);

        if(remote_power_ctrl_flag == 1)
        {
            uint8_t flag = 0;
            uint32_t time_out_cnt = 0;
            static uint8_t err_cnt = 0;

            remote_power_ctrl_flag = 0;

            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("remote_power_ctrl.remote_power_ctrl = %d",remote_power_ctrl.remote_power_ctrl);
                ROS_INFO("get set remote_power_ctrl cmd");
            }
            do
            {
                boost::mutex::scoped_lock(mtx);
                pNoahPowerboard->remote_power_ctrl_ack_vector.clear();
            }while(0);

set_remote_power_ctrl_restart:
            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("remote power control :send cmd to mcu");
            }
            pNoahPowerboard->sys_powerboard->remote_power_ctrl_set = remote_power_ctrl;

            pNoahPowerboard->RemotePowerCtrl(pNoahPowerboard->sys_powerboard);
            bool remote_power_ctrl_ack_flag = 0;
            remote_power_ctrl_t remote_power_ctrl_ack;
            while(time_out_cnt < SET_REMOTE_POWER_CTRL_TIME_OUT/10)
            {
                time_out_cnt++;
                do
                {
                    boost::mutex::scoped_lock(mtx);
                    if(!pNoahPowerboard->remote_power_ctrl_ack_vector.empty())
                    {
                        if(pNoahPowerboard->is_log_on == true)
                        {
                            ROS_INFO("remote_power_ctrl_ack_vector is not empty");
                        }
                        auto b = pNoahPowerboard->remote_power_ctrl_ack_vector.begin();

                        remote_power_ctrl_ack = *b;

                        pNoahPowerboard->remote_power_ctrl_ack_vector.erase(b);

                        if(remote_power_ctrl_ack.remote_power_ctrl == remote_power_ctrl.remote_power_ctrl)
                        {
                            remote_power_ctrl_ack_flag = 1;
                            if(pNoahPowerboard->is_log_on == true)
                            {
                                ROS_INFO("get right set remote power ctrl ack");
                            }
                        }
                        else
                        {
                            ROS_ERROR("error: get ack remote power ctrl is %d", remote_power_ctrl_ack.remote_power_ctrl);
                        }
                    }
                }while(0);
                if(remote_power_ctrl_ack_flag == 1)
                {
                    remote_power_ctrl_ack_flag = 0;
                    if(pNoahPowerboard->is_log_on == true)
                    {
                        //ROS_INFO("get right set led effect ack");
                    }
                    pNoahPowerboard->sys_powerboard->remote_power_ctrl_set = remote_power_ctrl_ack;
                    if(pNoahPowerboard->is_log_on == true)
                    {
                        ROS_INFO("get set remote power ctrl:%d",remote_power_ctrl_ack.remote_power_ctrl);
                    }
                    if(remote_power_ctrl_ack.status == 0)
                    {
                        ROS_WARN("remote power ctrl is in operation ...");
                    }
                    else if(remote_power_ctrl_ack.status == 1)
                    {
                        ROS_WARN("remote power ctrl parameter error !");
                    }
                    else if(remote_power_ctrl_ack.status == 2)
                    {
                        ROS_WARN("remote power ctrl error: device is not power on yet !");
                    }
                    else
                    {
                        ROS_INFO("mcu upload parameter parse error");
                    }
                    break;
                }
                else
                {
                    usleep(10*1000);
                }
            }
            if(time_out_cnt < SET_REMOTE_POWER_CTRL_TIME_OUT/10)
            {
                //ROS_INFO("set led effect flow OK");
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("set remote power ctrl time out");
                time_out_cnt = 0;
                if(err_cnt++ < SET_REMOTE_POWER_CTRL_RETRY_CNT)
                {
                    ROS_ERROR("set remote power ctrl start to resend msg....");
                    goto set_remote_power_ctrl_restart;
                }
                ROS_ERROR("CAN NOT COMMUNICATE with powerboard mcu, set remote power ctrl failed !");
                err_cnt = 0;
            }

        }
        /* -------- remote device power ctrl  protocol end -------- */


#if 1
        /* --------  get serials leds version  protocol begin -------- */
        do
        {
            boost::mutex::scoped_lock(mtx);
            if(!pNoahPowerboard->get_serials_leds_version_vector.empty())
            {
                auto a = pNoahPowerboard->get_serials_leds_version_vector.begin();
                get_serials_leds_version = *a;
                {
                    get_serials_leds_version_flag = 1;
                }

                pNoahPowerboard->get_serials_leds_version_vector.erase(a);

            }

        }while(0);

        if(get_serials_leds_version_flag == 1)
        {
            uint8_t flag = 0;
            uint32_t time_out_cnt = 0;
            static uint8_t err_cnt = 0;

            get_serials_leds_version_flag = 0;

            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("get_serials_leds_version.reserve = %d",get_serials_leds_version.reserve);
                ROS_INFO("get get serials leds version cmd");
            }
            do
            {
                boost::mutex::scoped_lock(mtx);
                pNoahPowerboard->get_serials_leds_version_ack_vector.clear();
            }while(0);

get_serials_leds_version_restart:
            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("get serials leds version :send cmd to mcu");
            }

            pNoahPowerboard->get_serials_leds_version(pNoahPowerboard->sys_powerboard);
            bool get_serails_leds_version_ack_flag = 0;

            get_serials_leds_version_t get_serials_leds_version_ack;

            while(time_out_cnt < GET_SERIALS_LEDS_VERSION_TIME_OUT/10)
            {
                time_out_cnt++;
                do
                {
                    boost::mutex::scoped_lock(mtx);
                    if(!pNoahPowerboard->get_serials_leds_version_ack_vector.empty())
                    {
                        if(pNoahPowerboard->is_log_on == true)
                        {
                            ROS_INFO("get_serials_leds_version_ack_vector is not empty");
                        }
                        auto b = pNoahPowerboard->get_serials_leds_version_ack_vector.begin();

                        get_serials_leds_version_ack = *b;

                        pNoahPowerboard->get_serials_leds_version_ack_vector.erase(b);

                        //if(get_serials_leds_version_ack.reserve == 0)
                        {
                            get_serails_leds_version_ack_flag = 1;
                            if(pNoahPowerboard->is_log_on == true)
                            {
                                ROS_INFO("get right get serials leds version ack");
                            }
                        }
                    }
                }while(0);

                if(get_serails_leds_version_ack_flag == 1)
                {
                    get_serails_leds_version_ack_flag = 0;
                    if(pNoahPowerboard->is_log_on == true)
                    {
                        //ROS_INFO("get right set led effect ack");
                    }
                    break;
                }
                else
                {
                    usleep(10*1000);
                }
            }
            if(time_out_cnt < GET_SERIALS_LEDS_VERSION_TIME_OUT/10)
            {
                //ROS_INFO("set led effect flow OK");
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("get serials leds version time out");
                time_out_cnt = 0;
                if(err_cnt++ < GET_SERIALS_LEDS_VERSION_RETRY_CNT)
                {
                    ROS_ERROR("get serials leds version start to resend msg....");
                    goto get_serials_leds_version_restart;
                }
                ROS_ERROR("CAN NOT COMMUNICATE with powerboard mcu, get serials leds version failed !");
                err_cnt = 0;
            }

        }
        /* -------- get serials leds version end -------- */
#endif



        /* --------  status leds control start  -------- */

        do
        {
            boost::mutex::scoped_lock(mtx);
            if(!pNoahPowerboard->set_led_status_vector.empty())
            {
                auto a = pNoahPowerboard->set_led_status_vector.begin();
                status_led = *a;
                {
                    set_led_status_flag = 1;
                }

                pNoahPowerboard->set_led_status_vector.erase(a);

            }

        }while(0);

        if(set_led_status_flag == 1)
        {
            uint8_t flag = 0;
            uint32_t time_out_cnt = 0;
            static uint8_t err_cnt = 0;

            set_led_status_flag = 0;

            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("set status led: led: %d, status: %d",status_led.led, status_led.status);
            }
            do
            {
                boost::mutex::scoped_lock(mtx);
                pNoahPowerboard->set_led_status_ack_vector.clear();
            }while(0);

set_led_status_restart:
            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("set led status :send cmd to mcu");
            }
            pNoahPowerboard->sys_powerboard->status_led_set = status_led;

            pNoahPowerboard->set_status_led(pNoahPowerboard->sys_powerboard);
            bool set_led_status_ack_flag = 0;
            status_led_t status_led_ack;
            while(time_out_cnt < SET_STATUS_LED_TIME_OUT / 10)
            {
                time_out_cnt++;
                do
                {
                    boost::mutex::scoped_lock(mtx);
                    if(!pNoahPowerboard->set_led_status_ack_vector.empty())
                    {
                        if(pNoahPowerboard->is_log_on == true)
                        {
                            ROS_INFO("set_led_status_ack_vector is not empty");
                        }
                        auto b = pNoahPowerboard->set_led_status_ack_vector.begin();

                        status_led_ack = *b;

                        pNoahPowerboard->set_led_status_ack_vector.erase(b);

                        if((status_led_ack.led == status_led.led) && (status_led_ack.status == status_led.status))
                        {
                            set_led_status_ack_flag = 1;
                            if(pNoahPowerboard->is_log_on == true)
                            {
                                ROS_INFO("get right set led status ack");
                            }
                        }
                        else
                        {
                            ROS_ERROR("error: get ack of set status led is: led:%d, status:%d", status_led_ack.led, status_led_ack.status);
                        }
                    }
                }while(0);
                if(set_led_status_ack_flag == 1)
                {
                    set_led_status_ack_flag = 0;
                    if(pNoahPowerboard->is_log_on == true)
                    {
                        ROS_INFO("get right set led status ack");
                    }
                    if(pNoahPowerboard->is_log_on == true)
                    {
                            ROS_INFO("get ack of set status led is: led:%d, status:%d", status_led_ack.led, status_led_ack.status);
                    }
                    pNoahPowerboard->ack_status_led_ctrl(status_led_ack, LED_STATUS_ACK_OK);
                    break;
                }
                else
                {
                    usleep(10*1000);
                }
            }
            if(time_out_cnt < SET_STATUS_LED_TIME_OUT / 10)
            {
                //ROS_INFO("set led effect flow OK");
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("set led status time out");
                time_out_cnt = 0;
                if(err_cnt++ < SET_LED_STATUS_RETRY_CNT)
                {
                    ROS_ERROR("set remote power ctrl start to resend msg....");
                    goto set_led_status_restart;
                }
                ROS_ERROR("CAN NOT COMMUNICATE with powerboard mcu, set led status failed !");
                pNoahPowerboard->ack_status_led_ctrl(status_led_ack, LED_STATUS_ACK_TIMEOUT);
                err_cnt = 0;
            }

        }
        /* --------  status leds control end  -------- */



        /* --------  get device id  protocol begin -------- */
        do
        {
            boost::mutex::scoped_lock(mtx);
            if(!pNoahPowerboard->get_dev_id_vector.empty())
            {
                auto a = pNoahPowerboard->get_dev_id_vector.begin();
                get_dev_id_tmp = *a;
                {
                    get_dev_id_flag = 1;
                }

                pNoahPowerboard->get_dev_id_vector.erase(a);

            }

        }while(0);

        if(get_dev_id_flag == 1)
        {
            uint8_t flag = 0;
            uint32_t time_out_cnt = 0;
            static uint8_t err_cnt = 0;

            get_dev_id_flag = 0;

            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("get dev id");
            }
            do
            {
                boost::mutex::scoped_lock(mtx);
                pNoahPowerboard->get_dev_id_ack_vector.clear();
            }while(0);

get_dev_id_restart:
            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("get dev id :send cmd to mcu");
            }
            pNoahPowerboard->sys_powerboard->dev_id = get_dev_id_tmp;

            pNoahPowerboard->get_dev_id(pNoahPowerboard->sys_powerboard);
            bool get_dev_id_ack_flag = 0;
            dev_id_t get_dev_id_ack;
            while(time_out_cnt < GET_DEV_ID_TIME_OUT / 10)
            {
                time_out_cnt++;
                do
                {
                    boost::mutex::scoped_lock(mtx);
                    if(!pNoahPowerboard->get_dev_id_ack_vector.empty())
                    {
                        if(pNoahPowerboard->is_log_on == true)
                        {
                            ROS_INFO("get_dev_id_ack_vector is not empty");
                        }
                        auto b = pNoahPowerboard->get_dev_id_ack_vector.begin();

                        get_dev_id_ack = *b;

                        pNoahPowerboard->get_dev_id_ack_vector.erase(b);

                        get_dev_id_ack_flag = 1;
                    }
                }while(0);

                if(get_dev_id_ack_flag == 1)
                {
                    get_dev_id_ack_flag = 0;
                    break;
                }
                else
                {
                    usleep(10*1000);
                }
            }
            if(time_out_cnt < GET_DEV_ID_TIME_OUT / 10)
            {
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("get dev id time out");
                time_out_cnt = 0;
                if(err_cnt++ < GET_DEV_ID_RETRY_CNT)
                {
                    ROS_ERROR("get dev id start to resend msg....");
                    goto get_dev_id_restart;
                }
                ROS_ERROR("CAN NOT COMMUNICATE with powerboard mcu, get dev id failed !");
                err_cnt = 0;
            }

        }
        /* -------- get device id protocol end -------- */



#if 0
        /* --------  ser conveyor belt work mode protocol begin -------- */
        do
        {
            boost::mutex::scoped_lock(mtx);
            if(!pNoahPowerboard->set_conveyor_belt_work_mode_vector.empty())
            {
                auto a = pNoahPowerboard->set_conveyor_belt_work_mode_vector.begin();
                set_conveyor_belt_mode = *a;
                {
                    set_conveyor_belt_flag = 1;
                }

                pNoahPowerboard->set_conveyor_belt_work_mode_vector.erase(a);

            }

        }while(0);

        if(set_conveyor_belt_flag == 1)
        {
            uint8_t flag = 0;
            uint32_t time_out_cnt = 0;
            static uint8_t err_cnt = 0;

            set_conveyor_belt_flag = 0;

            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("set_conveyor_belt_mode.set_work_mode = %d",set_conveyor_belt_mode.set_work_mode);
                ROS_INFO("get set set_conveyor_belt_mode cmd");
            }
            do
            {
                boost::mutex::scoped_lock(mtx);
                pNoahPowerboard->set_conveyor_belt_work_mode_vector.clear();
            }while(0);

set_conveyor_belt_work_mode_restart:
            if(pNoahPowerboard->is_log_on == true)
            {
                ROS_INFO("set conveyor belt work mode :send cmd to mcu");
            }
            pNoahPowerboard->sys_powerboard->conveyor_belt = set_conveyor_belt_mode;

            pNoahPowerboard->set_conveyor_belt_work_mode(pNoahPowerboard->sys_powerboard);
            bool set_conveyor_belt_work_mode_ack_flag = 0;
            conveyor_belt_t set_conveyor_belt_work_mode_ack;
            while(time_out_cnt < SET_CONVEYOR_BELT_WORK_MODE_TIME_OUT / 10)
            {
                time_out_cnt++;
                do
                {
                    boost::mutex::scoped_lock(mtx);
                    if(!pNoahPowerboard->set_conveyor_belt_work_mode_ack_vector.empty())
                    {
                        if(pNoahPowerboard->is_log_on == true)
                        {
                            ROS_INFO("set_conveyor_belt_work_mode_ack_vector is not empty");
                        }
                        auto b = pNoahPowerboard->set_conveyor_belt_work_mode_ack_vector.begin();

                        set_conveyor_belt_work_mode_ack = *b;

                        pNoahPowerboard->set_conveyor_belt_work_mode_ack_vector.erase(b);

                        if(set_conveyor_belt_work_mode_ack.set_work_mode == set_conveyor_belt_mode.set_work_mode)
                        {
                            set_conveyor_belt_work_mode_ack_flag = 1;
                            if(pNoahPowerboard->is_log_on == true)
                            {
                                ROS_INFO("get right set conveyor belt work mode ack");
                            }
                        }
                        else
                        {
                            ROS_ERROR("error: get ack set conveyor belt work mode is %d", set_conveyor_belt_work_mode_ack.set_work_mode);
                        }
                    }
                }while(0);
                if(set_conveyor_belt_work_mode_ack_flag == 1)
                {
                    set_conveyor_belt_work_mode_ack_flag = 0;
                    if(pNoahPowerboard->is_log_on == true)
                    {
                        //ROS_INFO("get right set led effect ack");
                    }
                    pNoahPowerboard->sys_powerboard->conveyor_belt = set_conveyor_belt_work_mode_ack;
                    if(pNoahPowerboard->is_log_on == true)
                    {
                        ROS_INFO("get set conveyor belt work mode:%d",set_conveyor_belt_work_mode_ack.set_work_mode);
                    }
                    if(set_conveyor_belt_work_mode_ack.err_status == 0)
                    {
                        ROS_INFO("conveyor belt status ok");
                    }
                    else if(set_conveyor_belt_work_mode_ack.err_status == CONVEYOR_BELT_LOAD_ERROR)
                    {
                        ROS_ERROR("conveyor belt load error !");
                    }
                    else if(set_conveyor_belt_work_mode_ack.err_status == CONVEYOR_BELT_UNLOAD_ERROR)
                    {
                        ROS_ERROR("conveyor belt unload error !");
                    }
                    else if(set_conveyor_belt_work_mode_ack.err_status == CONVEYOR_BELT_STATUS_ERROR)
                    {
                        ROS_ERROR("conveyor belt status error !");
                    }
                    else
                    {
                        ROS_ERROR("conveyor belt unknow error !");
                    }
                    break;
                }
                else
                {
                    usleep(10*1000);
                }
            }
            if(time_out_cnt < SET_CONVEYOR_BELT_WORK_MODE_TIME_OUT / 10)
            {
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("set conveyor belt work mode time out");
                time_out_cnt = 0;
                if(err_cnt++ < SET_CONVEYOR_BELT_WORK_MODE_RETRY_CNT)
                {
                    ROS_ERROR("set remote power ctrl start to resend msg....");
                    goto set_conveyor_belt_work_mode_restart;
                }
                ROS_ERROR("CAN NOT COMMUNICATE with powerboard mcu, set conveyor belt work mode failed !");
                err_cnt = 0;
            }

        }
        /* -------- set conveyor belt work mode protocol end -------- */
#endif


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
    mrobot_msgs::vci_can can_msg;
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
    mrobot_msgs::vci_can can_msg;
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

    mrobot_msgs::vci_can can_msg;
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
    can_msg.Data[2] = sys->module_status_set.group_num;///group_num;
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
    mrobot_msgs::vci_can can_msg;
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
    mrobot_msgs::vci_can can_msg;
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
    mrobot_msgs::vci_can can_msg;
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
    mrobot_msgs::vci_can can_msg;
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
    can_msg.Data[1] = sys->ir_cmd.set_ir_percent;
    ROS_WARN("start to set ir duty to %d",sys->ir_cmd.set_ir_percent);
    this->pub_to_can_node.publish(can_msg);
    return error;
}



int NoahPowerboard::RemotePowerCtrl(powerboard_t *sys)     // done
{
    int error = 0;
    mrobot_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_REMOTE_POWRER_CTRL;
    id.CanID_Struct.SrcMACID = 0;//CAN_SUB_PB_SRC_ID;
    id.CanID_Struct.DestMACID = NOAH_POWERBOARD_CAN_SRCMAC_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = sys->remote_power_ctrl_set.remote_power_ctrl;
    ROS_WARN("start to set remote power ctrl to %d",sys->remote_power_ctrl_set.remote_power_ctrl);
    this->pub_to_can_node.publish(can_msg);
    return error;
}

int NoahPowerboard::get_serials_leds_version(powerboard_t *sys)     // done
{
    ROS_INFO("start to get serials leds version . . . ");
    int error = 0;
    mrobot_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_GET_SERIALS_LEDS_VERSION;
    id.CanID_Struct.SrcMACID = 0;
    id.CanID_Struct.DestMACID = NOAH_POWERBOARD_CAN_SRCMAC_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = 0;
    ROS_WARN("start to get serials leds version ...");
    this->pub_to_can_node.publish(can_msg);
    return error;
}


int NoahPowerboard::ack_mcu_upload(CAN_ID_UNION id, uint8_t serial_num)
{
    ROS_INFO("start to ack mcu upload info. . . ");
    int error = 0;
    mrobot_msgs::vci_can can_msg;

    id.CanID_Struct.SrcMACID = 0;
    id.CanID_Struct.DestMACID = NOAH_POWERBOARD_CAN_SRCMAC_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 1;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = serial_num;
    this->pub_to_can_node.publish(can_msg);
    return error;
}


int NoahPowerboard::set_conveyor_belt_work_mode(powerboard_t *sys)
{
    ROS_INFO("start to set conveyor belt work mode . . . ");
    int error = 0;
    mrobot_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_SET_CONVEYOR_BELT_WORK_MODE;
    id.CanID_Struct.SrcMACID = 0;
    id.CanID_Struct.DestMACID = NOAH_POWERBOARD_CAN_SRCMAC_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    if((sys->conveyor_belt.set_work_mode >= 0) && (sys->conveyor_belt.set_work_mode <= 2))
    {
        can_msg.Data[1] = sys->conveyor_belt.set_work_mode;
    }
    else
    {
        ROS_ERROR("set conveyor belt work mode: parameter error !  set work mode %d", sys->conveyor_belt.set_work_mode);
        return -1;
    }

    this->pub_to_can_node.publish(can_msg);
    return error;
}



int NoahPowerboard::set_status_led(powerboard_t *sys)     // done
{
    int error = 0;
    mrobot_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_SET_LED_STATUS;
    id.CanID_Struct.SrcMACID = 0;//CAN_SUB_PB_SRC_ID;
    id.CanID_Struct.DestMACID = NOAH_POWERBOARD_CAN_SRCMAC_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 3;
    can_msg.Data.resize(3);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = sys->status_led_set.led;
    can_msg.Data[2] = sys->status_led_set.status;
    ROS_WARN("start to set led status: led %d, status:%d",sys->status_led_set.led, sys->status_led_set.status);
    this->pub_to_can_node.publish(can_msg);
    return error;
}



int NoahPowerboard::get_dev_id(powerboard_t *sys)     // done
{
    int error = 0;
    mrobot_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_GET_DEV_ID;
    id.CanID_Struct.SrcMACID = 0;//CAN_SUB_PB_SRC_ID;
    id.CanID_Struct.DestMACID = NOAH_POWERBOARD_CAN_SRCMAC_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = 0;
    ROS_WARN("start to get dev id . . .");
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

bool NoahPowerboard::service_remote_power_ctrl(noah_powerboard::remote_power_ctrl_srv::Request  &ctrl,  noah_powerboard::remote_power_ctrl_srv::Response &status)
{
    ROS_INFO("%s: srv call test",__func__);
    if((0 == ctrl.power_ctrl) || (1 == ctrl.power_ctrl))
    {
        remote_power_ctrl_t remote_power_ctrl;
        remote_power_ctrl.remote_power_ctrl = ctrl.power_ctrl + 1;
        this->remote_power_ctrl_vector.push_back(remote_power_ctrl);
        status.result = 0;
        return true;
    }

    ROS_ERROR("%s: parameter error!  Request.power_ctrl: %d",__func__, ctrl.power_ctrl);
    status.result = -1;
    return true;
}

void NoahPowerboard::from_app_rcv_callback(const std_msgs::String::ConstPtr &msg)
{
    //ROS_INFO("Rcv test data");
    auto j = json::parse(msg->data.c_str());
    std::string j_str = j.dump();
    ROS_WARN("%s",j_str.data());

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

            if(j["data"]["dev_name"] == "door_ctrl_state")
            {
                if(j["data"].find("door_id") != j["data"].end())
                {
                    ROS_INFO("start to check door id ...");
                    if(j["data"]["door_id"].empty())
                    {
                        ROS_WARN("door id value is NULL !");//
                        if(j["data"]["set_state"] == true)
                        {
                            ROS_INFO("set door ctrl  on");
                            module_ctrl_t param;
                            param.group_num = 1;
                            param.module = POWER_DOOR_CTRL | POWER_VSYS_24V_NV;
                            param.on_off = MODULE_CTRL_ON;
                            this->module_set_vector.push_back(param);
                        }
                        else if(j["data"]["set_state"] == false)
                        {
                            ROS_INFO("set door ctrl off");
                            module_ctrl_t param;
                            param.group_num = 1;
                            param.module = POWER_DOOR_CTRL | POWER_VSYS_24V_NV;
                            param.on_off = MODULE_CTRL_OFF;
                            this->module_set_vector.push_back(param);
                        }
                    }
                    else
                    {
                        ROS_INFO("door id is not NULL");
                        uint8_t door_id = j["data"]["door_id"];
                        //bool on_off = j["data"]["set_state"];
                        bool on_off;
                        if(j["data"].find("set_state") != j["data"].end())
                        {
                            on_off = j["data"]["set_state"];
                        }
                        else
                        {
                            return;
                        }
                        module_ctrl_t param;

                        param.group_num = 1;
                        ROS_WARN("get door id");//

                        if(on_off == true)
                        {
                            param.on_off = MODULE_CTRL_ON;
                        }
                        else
                        {
                            param.on_off = MODULE_CTRL_OFF;
                        }
                        switch(door_id)
                        {
                            case 1:
                                param.module = POWER_3V3_CARD_EN_1;
                                break;
                            case 2:
                                param.module = POWER_3V3_CARD_EN_2;
                                break;
                            case 3:
                                param.module = POWER_3V3_CARD_EN_3;
                                break;
                            case 4:
                                param.module = POWER_3V3_CARD_EN_4;
                                break;
                            default :
                                ROS_ERROR("door ctrl id  error !");
                                break;
                        }
                        ROS_INFO("set door %d ctrl %d",door_id, param.on_off);
                        this->module_set_vector.push_back(param);
                    }
                }
            }
        }
    }
}


void NoahPowerboard::basestate_callback(std_msgs::UInt8MultiArray data)
{
    //if(data.data.size() == 7)
    {
        if((data.data[2] & (1<<4)) || (data.data[3]))
        {
            emg_stop = true;
        }
        else
        {
            emg_stop = false;
        }
    }
}

void NoahPowerboard::serials_leds_turning_effect_callback(std_msgs::UInt8MultiArray effects)
{
    ROS_INFO("%s",__func__);
    if(effects.data.size() == 1)
    {
        if(effects.data[0] < 3)
        {
            turnning_direction = effects.data[0];
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
            param.module = POWER_CAMERA_FRONT_LED |POWER_CAMERA_BACK_LED  ;
            param.on_off = MODULE_CTRL_OFF;
            this->module_set_vector.push_back(param);
            break;
        case 1:
            ROS_INFO("camera led ctrl:get 01");
            param.module = POWER_CAMERA_BACK_LED | POWER_CAMERA_FRONT_LED;
            param.on_off = MODULE_CTRL_ON;
            this->module_set_vector.push_back(param);

            //param.module = POWER_CAMERA_FRONT_LED;
            //param.on_off = MODULE_CTRL_OFF;
            //this->module_set_vector.push_back(param);
            break;
        case 10:
            ROS_INFO("camera led ctrl:get 10");
            param.module = POWER_CAMERA_FRONT_LED | POWER_CAMERA_BACK_LED;
            param.on_off = MODULE_CTRL_ON;
            this->module_set_vector.push_back(param);

            //param.module = POWER_CAMERA_BACK_LED;
            //param.on_off = MODULE_CTRL_OFF;
            //this->module_set_vector.push_back(param);
            break;
        case 11:
            ROS_INFO("camera led ctrl:get 11");
            param.module = POWER_CAMERA_FRONT_LED |POWER_CAMERA_BACK_LED  ;
            param.on_off = MODULE_CTRL_ON;
            this->module_set_vector.push_back(param);
            break;

        case 100:
            ROS_INFO("camera led ctrl:get 100");
            param.module = POWER_HEAD_CAMERA_LED;
            param.on_off = MODULE_CTRL_OFF;
            param.group_num = 2;
            this->module_set_vector.push_back(param);

            //param.module = POWER_CAMERA_BACK_LED;
            //param.on_off = MODULE_CTRL_OFF;
            //this->module_set_vector.push_back(param);
            break;
        case 101:
            ROS_INFO("camera led ctrl:get 101");
            param.module = POWER_HEAD_CAMERA_LED;
            param.on_off = MODULE_CTRL_ON;
            param.group_num = 2;
            this->module_set_vector.push_back(param);
            break;
        default :
            break;

    }
}

void NoahPowerboard::leds_ctrl_callback(const std_msgs::String::ConstPtr &msg)
{
    auto j = json::parse(msg->data.c_str());
    std::string j_str = j.dump();
    ROS_WARN("topic: leds_ctrl, json data: %s",j_str.data());

    if(j.find("pub_name") != j.end())
    {
        if(j["pub_name"] == "status_led_ctrl")
        {
            status_led_t set_led_status = {0};
            if(j["data"].find(STR_LED_WIFI) != j["data"].end())
            {
                std::string wifi_status = j["data"][STR_LED_WIFI];
                ROS_INFO("get wifi status: %s", wifi_status.c_str());
                if(j["data"][STR_LED_WIFI] == STR_LED_STATUS_OK)
                {
                    ROS_INFO("set wifi status: ok");
                    set_led_status.led = LED_WIFI;
                    set_led_status.status = LED_STATUS_OK;
                    do
                    {
                        boost::mutex::scoped_lock(this->mtx);
                        this->set_led_status_vector.push_back(set_led_status);
                    }while(0);
                }
                else if(j["data"][STR_LED_WIFI] == STR_LED_STATUS_ERR)
                {
                    ROS_INFO("set wifi status: err");
                    set_led_status.led = LED_WIFI;
                    set_led_status.status = LED_STATUS_ERR;
                    do
                    {
                        boost::mutex::scoped_lock(this->mtx);
                        this->set_led_status_vector.push_back(set_led_status);
                    }while(0);
                }
                else if(j["data"][STR_LED_WIFI] == STR_LED_STATUS_OFF)
                {
                    ROS_INFO("set wifi status: off");
                    set_led_status.led = LED_WIFI;
                    set_led_status.status = LED_STATUS_OFF;
                    do
                    {
                        boost::mutex::scoped_lock(this->mtx);
                        this->set_led_status_vector.push_back(set_led_status);
                    }while(0);
                }
                else if(j["data"][STR_LED_WIFI] == STR_LED_STATUS_WARN)
                {
                    ROS_INFO("set wifi status: warn");
                    set_led_status.led = LED_WIFI;
                    set_led_status.status = LED_STATUS_WARN;
                    do
                    {
                        boost::mutex::scoped_lock(this->mtx);
                        this->set_led_status_vector.push_back(set_led_status);
                    }while(0);
                }
                else
                {
                    ROS_ERROR("wifi status parameter error: %s", wifi_status.c_str());
                }
            }

            if(j["data"].find(STR_LED_TRANS) != j["data"].end())
            {
                std::string trans_status = j["data"][STR_LED_TRANS];
                if(j["data"][STR_LED_TRANS] == STR_LED_STATUS_OK)
                {
                    ROS_INFO("set trans status: ok");
                    set_led_status.led = LED_TRANS;
                    set_led_status.status = LED_STATUS_OK;
                    do
                    {
                        boost::mutex::scoped_lock(this->mtx);
                        this->set_led_status_vector.push_back(set_led_status);
                    }while(0);
                }
                else if(j["data"][STR_LED_TRANS] == STR_LED_STATUS_ERR)
                {
                    ROS_INFO("set trans status: err");
                    set_led_status.led = LED_TRANS;
                    set_led_status.status = LED_STATUS_ERR;
                    do
                    {
                        boost::mutex::scoped_lock(this->mtx);
                        this->set_led_status_vector.push_back(set_led_status);
                    }while(0);
                }
                else if(j["data"][STR_LED_TRANS] == STR_LED_STATUS_OFF)
                {
                    ROS_INFO("set trans status: off");
                    set_led_status.led = LED_TRANS;
                    set_led_status.status = LED_STATUS_OFF;
                    do
                    {
                        boost::mutex::scoped_lock(this->mtx);
                        this->set_led_status_vector.push_back(set_led_status);
                    }while(0);
                }
                else if(j["data"][STR_LED_TRANS] == STR_LED_STATUS_WARN)
                {
                    ROS_INFO("set trans status: warn");
                    set_led_status.led = LED_TRANS;
                    set_led_status.status = LED_STATUS_WARN;
                    do
                    {
                        boost::mutex::scoped_lock(this->mtx);
                        this->set_led_status_vector.push_back(set_led_status);
                    }while(0);
                }
                else
                {
                    /*
                    TODO: parameter error
                    */
                    ROS_ERROR("trans status parameter error: %s", trans_status.c_str());
                }
            }


            if(j["data"].find(STR_LED_BATTERY) != j["data"].end())
            {
                std::string battery_status = j["data"][STR_LED_BATTERY];
                if(j["data"][STR_LED_BATTERY] == STR_LED_STATUS_OK)
                {
                    ROS_INFO("set battery status: ok");
                    set_led_status.led = LED_BATTERY;
                    set_led_status.status = LED_STATUS_OK;
                    do
                    {
                        boost::mutex::scoped_lock(this->mtx);
                        this->set_led_status_vector.push_back(set_led_status);
                    }while(0);
                }
                else if(j["data"][STR_LED_BATTERY] == STR_LED_STATUS_ERR)
                {
                    ROS_INFO("set battery status: err");
                    set_led_status.led = LED_BATTERY;
                    set_led_status.status = LED_STATUS_ERR;
                    do
                    {
                        boost::mutex::scoped_lock(this->mtx);
                        this->set_led_status_vector.push_back(set_led_status);
                    }while(0);
                }
                else if(j["data"][STR_LED_BATTERY] == STR_LED_STATUS_OFF)
                {
                    ROS_INFO("set battery status: off");
                    set_led_status.led = LED_BATTERY;
                    set_led_status.status = LED_STATUS_OFF;
                    do
                    {
                        boost::mutex::scoped_lock(this->mtx);
                        this->set_led_status_vector.push_back(set_led_status);
                    }while(0);
                }
                else if(j["data"][STR_LED_BATTERY] == STR_LED_STATUS_WARN)
                {
                    ROS_INFO("set battery status: warn");
                    set_led_status.led = LED_BATTERY;
                    set_led_status.status = LED_STATUS_WARN;
                    do
                    {
                        boost::mutex::scoped_lock(this->mtx);
                        this->set_led_status_vector.push_back(set_led_status);
                    }while(0);
                }
                else
                {
                    /*
                    TODO: parameter error
                    */
                    ROS_ERROR("battery status parameter error: %s", battery_status.c_str());
                }
            }
        }

        if(j["pub_name"] == "serial_leds_ctrl")
        {
            int period = 0;
            color_t color = {0};
            if(j["data"].find("period") != j["data"].end())
            {
                period = j["data"]["period"];
                ROS_INFO("get period: %d", period);
            }

            if(j["data"].find("color") != j["data"].end())
            {
                if(j["data"]["color"].find("r") != j["data"]["color"].end())
                {
                    color.r = j["data"]["color"]["r"];
                    ROS_INFO("get r: %d", color.r);
                }
                else
                {
                    /*
                    TODO: parameter error
                    */
                }
                if(j["data"]["color"].find("g") != j["data"]["color"].end())
                {
                    color.g = j["data"]["color"]["g"];
                    ROS_INFO("get g: %d", color.g);
                }
                else
                {
                    /*
                    TODO: parameter error
                    */
                }
                if(j["data"]["color"].find("b") != j["data"]["color"].end())
                {
                    color.b = j["data"]["color"]["b"];
                    ROS_INFO("get b: %d", color.b);
                }
                else
                {
                    /*
                    TODO: parameter error
                    */
                }

                //set_leds_effect_vector
                set_leds_effect_t set_led_effect;
                set_led_effect.reserve = 0;
                set_led_effect.mode = LIGHTS_MODE_SETTING;
                set_led_effect.period = period;
                set_led_effect.color = color;
                this->set_leds_effect_vector.push_back(set_led_effect);
            }
        }
    }
}

void NoahPowerboard::PubPower(powerboard_t *sys)
{
    unsigned char power = 0;
    power = sys->bat_info.bat_percent;
    uint16_t status = sys->sys_status;    //std_msgs::Int8 msg;
    std_msgs::UInt8MultiArray bytes_msg;

    bytes_msg.data.push_back(power);
    bytes_msg.data.push_back(status & 0xff);
    bytes_msg.data.push_back(status >> 8);
    power_pub_to_app.publish(bytes_msg);
}

void NoahPowerboard::pub_event_key_value(uint8_t value)
{
    std_msgs::String pub_json_msg;
    std::stringstream ss;

    this->j_event_key.clear();
    this->j_event_key =
        {
            {"event_key_num", 1},
            {"key_value", value},
        };

    ss.clear();
    ss << j_event_key;
    pub_json_msg.data = ss.str();
    this->pub_event_key.publish(pub_json_msg);
}


void NoahPowerboard::ack_status_led_ctrl(status_led_t status_led, uint8_t err_code)
{
    json j;
    std_msgs::String pub_json_msg;
    std::stringstream ss;
    std::string led;
    std::string status;
    std::string str_err_code;

    switch(status_led.led)
    {
        case LED_WIFI:
            led = STR_LED_WIFI;
            break;
        case LED_TRANS:
            led = STR_LED_TRANS;
            break;
        case LED_BATTERY:
            led = STR_LED_BATTERY;
            break;
        default: break;
    }

    switch(status_led.status)
    {
        case LED_STATUS_OFF:
            status = STR_LED_STATUS_OFF;
            break;
        case LED_STATUS_WARN:
            status = STR_LED_STATUS_WARN;
            break;
        case LED_STATUS_ERR:
            status = STR_LED_STATUS_ERR;
            break;
        case LED_STATUS_OK:
            status = STR_LED_STATUS_OK;
            break;
        default: break;
    }

    switch(err_code)
    {
        case LED_STATUS_ACK_OK:
            str_err_code = STR_LED_STATUS_ACK_OK;
            break;
        case LED_STATUS_ACK_TIMEOUT:
            str_err_code = STR_LED_STATUS_ACK_TIMEOUT;
            break;
        case LED_STATUS_ACK_PARAM_ERR:
            str_err_code = STR_LED_STATUS_ACK_PARAM_ERR;
            break;
        default: break;
    }

    j.clear();
    j =
        {
            {"pub_name", "status_led_ctrl"},
            {
                "data",
                {
                    {led.c_str(), status.c_str()},
                    {"result",str_err_code.c_str()},
                }
            }
        };

    ss.clear();
    ss << j;
    pub_json_msg.data = ss.str();
    this->status_led_ctrl_ack_pub.publish(pub_json_msg);
}


#if 0
void NoahPowerboard::power_from_app_rcv_callback(std_msgs::UInt8MultiArray data)
{
    if(data.data.size() == 1)
    {
        uint8_t value = data.data[0];
        if(this->is_log_on == true)
        {
            ROS_INFO("power_from_app_rcv_callback");
            ROS_INFO("data is %d",value);
        }
        if(value == 0)
        {
            //this->PubPower(this->sys_powerboard);
        }
    }
}
#endif
void NoahPowerboard::remote_power_ctrl_callback(std_msgs::UInt8MultiArray data)
{
    ROS_INFO("%s",__func__);
    if(data.data.size() == 1)
    {
        ROS_INFO("%s: data length is right",__func__);
        ROS_INFO("%s:  data[0]: %d",__func__,data.data[0]);
        if( (data.data[0] == 1) || (data.data[0] == 2) )
        {
            remote_power_ctrl_t remote_power_ctrl;
            remote_power_ctrl.remote_power_ctrl = data.data[0];
            this->remote_power_ctrl_vector.push_back(remote_power_ctrl);
            ROS_INFO("callback: get remote_power_ctrl:%d",remote_power_ctrl.remote_power_ctrl);
        }
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



void NoahPowerboard::pub_battery_info(get_bat_info_ack_t *bat_info)
{
    json j;
    j.clear();
    j =
    {
        {"pub_name","test_battery_info"},
        {
            "data",
            {
                {"percent", bat_info->bat_percent},
                {"voltage", bat_info->bat_vol},
                {"current", bat_info->pack_current},
                {"current_soc", bat_info->pack_current_soc},
                {"totoal_soc", bat_info->pack_totoal_soc},
                {"recharge_cycle", bat_info->pack_recharge_cycle},
                {"com_status", bat_info->com_status},
            }
        },
    };
    std_msgs::String pub_json_msg;
    std::stringstream ss;
    ss.clear();
    ss << j;
    pub_json_msg.data.clear();
    pub_json_msg.data = ss.str();
    battery_test_pub.publish(pub_json_msg);
}


void NoahPowerboard::rcv_from_can_node_callback(const mrobot_msgs::vci_can::ConstPtr &c_msg)
{
    mrobot_msgs::vci_can can_msg;
    mrobot_msgs::vci_can long_msg;
    CAN_ID_UNION id;

    long_msg = this->long_frame.frame_construct(c_msg);
    mrobot_msgs::vci_can* msg = &long_msg;
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
            ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_SET_MODULE_STATE");
            ROS_INFO("receive module : 0x%x",module_ctrl_ack.module);
            ROS_INFO("receive module state : 0x%x",module_ctrl_ack.module_status_ack);
        }

    }

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_GET_BAT_STATE)
    {
        get_bat_info_ack_t get_bat_info_ack;
        get_bat_info_ack.bat_vol = *(uint16_t *)&msg->Data[1];
        get_bat_info_ack.bat_percent = msg->Data[3];
        if(msg->DataLen > 3)
        {
            get_bat_info_ack.pack_current = *(uint16_t *)&msg->Data[4];
            get_bat_info_ack.pack_current_soc = *(uint16_t *)&msg->Data[6];
            get_bat_info_ack.pack_totoal_soc = *(uint16_t *)&msg->Data[8];
            get_bat_info_ack.pack_recharge_cycle = *(uint16_t *)&msg->Data[10];
            get_bat_info_ack.com_status = msg->Data[12];
            ROS_INFO("battery voltage: %d,  percentage: %d", get_bat_info_ack.bat_vol, get_bat_info_ack.bat_percent);
            ROS_INFO("battery current: %d", get_bat_info_ack.pack_current);
            ROS_INFO("battery recharge cycle: %d", get_bat_info_ack.pack_recharge_cycle);
            ROS_INFO("battery current soc: %d, totoal soc: %d", get_bat_info_ack.pack_current_soc, get_bat_info_ack.pack_totoal_soc);
            ROS_INFO("battery com status: %d", get_bat_info_ack.com_status);
            pub_battery_info(&get_bat_info_ack);
        }

        do
        {
            boost::mutex::scoped_lock(this->mtx);
            this->get_bat_info_ack_vector.push_back(get_bat_info_ack);
        }while(0);

        if(this->is_log_on == true)
        {
            ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_GET_BAT_STATE");
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
            //ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_GET_SYS_STATE");
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
            can_upload_ack_t can_upload_ack = {0};
            if(msg->DataLen >= 1)
            {
                can_upload_ack.serial_num = msg->Data[msg->DataLen - 1];
                can_upload_ack.id.CanID_Struct.ACK = 1;
                can_upload_ack.id.CanID_Struct.SourceID = CAN_SOURCE_ID_GET_SYS_STATE;
                this->ack_mcu_upload(can_upload_ack.id, can_upload_ack.serial_num);
                ROS_INFO("serial num: %d", can_upload_ack.serial_num);
            }
            //ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_GET_SYS_STATE");
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

        this->sys_powerboard->ir_cmd.set_ir_percent = set_ir_duty_ack.duty;
    }

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_READ_VERSION)
    {
        uint8_t len;
        len = msg->Data[1];
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_READ_VERSION");
        get_version_ack_t get_version_ack;
        get_version_ack.get_version_type = msg->Data[0];
        if(get_version_ack.get_version_type == 1)//software version
        {
            sys_powerboard->sw_version.resize(len);
            sys_powerboard->sw_version.clear();
            for(uint8_t i = 0; i < len; i++)
            {
                sys_powerboard->sw_version.push_back(*(char *)&(msg->Data[i+2]));
                get_version_ack.sw_version.push_back(*(char *)&(msg->Data[i+2]));
            }

            n.setParam(software_version_param,sys_powerboard->sw_version.data());
        }
        else if(get_version_ack.get_version_type == 2)//protocol version
        {
            sys_powerboard->protocol_version.resize(len);
            sys_powerboard->protocol_version.clear();
            for(uint8_t i = 0; i < len; i++)
            {
                sys_powerboard->protocol_version.push_back(*(char *)&(msg->Data[i+2]));
                get_version_ack.protocol_version.push_back(*(char *)&(msg->Data[i+2]));
            }
            n.setParam(protocol_version_param,sys_powerboard->protocol_version.data());
        }
        else if(get_version_ack.get_version_type == 3)//hardware version
        {
            sys_powerboard->hw_version.resize(len);
            sys_powerboard->hw_version.clear();
            //ROS_ERROR("hardware version length: %d",len);
            for(uint8_t i = 0; i < len; i++)
            {
                sys_powerboard->hw_version.push_back(*(char *)&(msg->Data[i+2]));
                get_version_ack.hw_version.push_back(*(char *)&(msg->Data[i+2]));
            }
            n.setParam(hardware_version_param,sys_powerboard->hw_version.data());
        }

        do
        {
            boost::mutex::scoped_lock(this->mtx);
            this->get_version_ack_vector.push_back(get_version_ack);
        }while(0);
    }

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_GET_ADC_DATA)
    {
        //ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_GET_ADC_DATA");
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
        set_leds_effect_t set_led_effect_ack;

        set_led_effect_ack.mode = msg->Data[2];
        set_led_effect_ack.color = *(color_t *)&msg->Data[3];
        set_led_effect_ack.period = msg->Data[6];
        if(this->is_log_on == true)
        {
            ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_SET_LED_EFFECT");
            ROS_INFO("led mode %d",set_led_effect_ack.mode);
        }
        //if(id.CanID_Struct.ACK == 1)
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
    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_POWER_OFF_SIGNAL)
    {

        if(id.CanID_Struct.ACK == 0)
        {
            ROS_INFO("shutdown signal : mcu upload");
            std_msgs::UInt8MultiArray shutdown_signal;
            uint8_t time_second = msg->Data[0];
            shutdown_signal.data.push_back(time_second);

            device_shutdown_signal_pub.publish(shutdown_signal);
        }

    }


    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_REMOTE_POWRER_CTRL)
    {
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_REMOTE_POWRER_CTRL");
        remote_power_ctrl_t remote_power_ctrl_ack;
        remote_power_ctrl_ack.remote_power_ctrl = msg->Data[0];
        remote_power_ctrl_ack.status = msg->Data[1];

        do
        {
            boost::mutex::scoped_lock(this->mtx);
            this->remote_power_ctrl_ack_vector.push_back(remote_power_ctrl_ack);
        }while(0);

        this->sys_powerboard->remote_power_ctrl_set.remote_power_ctrl = remote_power_ctrl_ack.remote_power_ctrl;
    }


    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_GET_SERIALS_LEDS_VERSION)
    {
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_GET_SERIALS_LEDS_VERSION");

        get_serials_leds_version_t get_serials_leds_version_ack;
        get_serials_leds_version_ack.reserve = 0;
        for(uint8_t i = 0; i < msg->DataLen; i++)
        {
            get_serials_leds_version_ack.version.push_back(msg->Data[i]);
        }

        do
        {
            boost::mutex::scoped_lock(this->mtx);
            this->get_serials_leds_version_ack_vector.push_back(get_serials_leds_version_ack);
        }while(0);

        sys_powerboard->serials_leds_mcu_version = get_serials_leds_version_ack.version;
        ROS_INFO("set param of serials leds verion ");
        ROS_INFO("serials leds verison is : %s",sys_powerboard->serials_leds_mcu_version.c_str());
        n.setParam(serials_leds_mcu_version_param, sys_powerboard->serials_leds_mcu_version.c_str());
    }

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_SET_CONVEYOR_BELT_WORK_MODE)
    {
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_SET_CONVEYOR_BELT_WORK_MODE");
        conveyor_belt_t conveyor_belt_ack;
        if(id.CanID_Struct.ACK == 0)
        {
            if(msg->DataLen == 1)
            {
                ROS_INFO("MCU upload: CAN_SOURCE_ID_SET_CONVEYOR_BELT_WORK_MODE");
                conveyor_belt_ack.set_result = msg->Data[0];
                ROS_INFO("conveyor belt result :%d", conveyor_belt_ack.set_result);
            }
            else
            {
                ROS_ERROR("can data len error ! data len: %d", msg->DataLen);
            }
        }
        else
        {
            conveyor_belt_ack.set_result = msg->Data[0];
            conveyor_belt_ack.set_work_mode = msg->Data[1];
            conveyor_belt_ack.err_status = msg->Data[2];
            do
            {
                boost::mutex::scoped_lock(this->mtx);
                this->set_conveyor_belt_work_mode_ack_vector.push_back(conveyor_belt_ack);
            } while (0);

            this->sys_powerboard->conveyor_belt.ack_work_mode = conveyor_belt_ack.ack_work_mode;
            this->sys_powerboard->conveyor_belt.err_status = conveyor_belt_ack.err_status;
        }
    }

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_EVENT_BUTTON)
    {
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_EVENT_BUTTON");
        uint8_t event_button_state = 0;
        uint8_t serial_num = 0;
        if(id.CanID_Struct.ACK == 0)
        {
            if(msg->DataLen == 2)
            {
                CAN_ID_UNION ack_id = {0};
                ROS_INFO("MCU upload: CAN_SOURCE_ID_EVENT_BUTTON");
                event_button_state = msg->Data[0];
                serial_num = msg->Data[1];
                ROS_INFO("get button state :%d", event_button_state);
                ROS_INFO("get serial num :%d", serial_num);

                ack_id.CanID_Struct.SourceID = CAN_SOURCE_ID_EVENT_BUTTON;
                this->ack_mcu_upload(ack_id, serial_num);
                this->pub_event_key_value(event_button_state);
            }
            else
            {
                ROS_ERROR("can data len error ! data len: %d", msg->DataLen);
            }
        }
    }


    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_SET_LED_STATUS)
    {
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_SET_LED_STATUS");
        status_led_t status_led_ack;
        status_led_ack.led = msg->Data[0];
        status_led_ack.status = msg->Data[1];

        do
        {
            boost::mutex::scoped_lock(this->mtx);
            this->set_led_status_ack_vector.push_back(status_led_ack);
        }while(0);
    }


    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_GET_DEV_ID)
    {
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_GET_DEV_ID");
        dev_id_t dev_id = {0};
        dev_id.dev_id = msg->Data[0] | (msg->Data[1]) << 8;
        ROS_WARN("get dev id: 0x%x", dev_id.dev_id);
        do
        {
            boost::mutex::scoped_lock(this->mtx);
            this->get_dev_id_ack_vector.push_back(dev_id);
        }while(0);
    }

}


void NoahPowerboard::update_sys_status(void)
{
    uint16_t sys_status = this->sys_powerboard->sys_status;
    uint8_t  power_percent = this->sys_powerboard->bat_info.bat_percent;
    static uint8_t prv_led_effect = LIGHTS_MODE_NONE;

    /* ---- set led effect ----*/
    set_leds_effect_t set_led_effect;
    set_led_effect.reserve = 0;


    if((sys_status & STATE_IS_RECHARGE_IN) || (sys_status & STATE_IS_CHARGER_IN))
    {
        if(power_percent < VBAT_POWER_CHARGING_LOW)
        {
            if(prv_led_effect != LIGHTS_MODE_CHARGING_POWER_LOW)
            {
                ROS_INFO("set led effect charging low power");
                prv_led_effect = LIGHTS_MODE_CHARGING_POWER_LOW;
                set_led_effect.mode = LIGHTS_MODE_CHARGING_POWER_LOW;
                do
                {
                    boost::mutex::scoped_lock(this->mtx);
                    this->set_leds_effect_vector.push_back(set_led_effect);
                }while(0);
            }
        }
        else if(power_percent < VBAT_POWER_CHARGING_FULL)
        {
            if(prv_led_effect != LIGHTS_MODE_CHARGING_POWER_MEDIUM)
            {
                ROS_INFO("set led effect charging medium power");
                prv_led_effect = LIGHTS_MODE_CHARGING_POWER_MEDIUM;
                set_led_effect.mode = LIGHTS_MODE_CHARGING_POWER_MEDIUM;
                do
                {
                    boost::mutex::scoped_lock(this->mtx);
                    this->set_leds_effect_vector.push_back(set_led_effect);
                }while(0);
            }
        }
        else if(power_percent >= VBAT_POWER_CHARGING_FULL)
        {
            if(prv_led_effect != LIGHTS_MODE_CHARGING_FULL)
            {
                ROS_INFO("set led effect charging full power");
                prv_led_effect = LIGHTS_MODE_CHARGING_FULL;
                set_led_effect.mode = LIGHTS_MODE_CHARGING_FULL;
                do
                {
                    boost::mutex::scoped_lock(this->mtx);
                    this->set_leds_effect_vector.push_back(set_led_effect);
                }while(0);
            }
        }

    }
    else if(emg_stop == true)
    {
        if(prv_led_effect != LIGHTS_MODE_EMERGENCY_STOP)
        {

            ROS_INFO("set led effect emgency stop ");
            prv_led_effect = LIGHTS_MODE_EMERGENCY_STOP;
            set_led_effect.mode = LIGHTS_MODE_EMERGENCY_STOP;
            do
            {
                boost::mutex::scoped_lock(this->mtx);
                this->set_leds_effect_vector.push_back(set_led_effect);
            }while(0);
        }
    }
    else if(turnning_direction > 0)
    {

        if(turnning_direction == 1)//left
        {
            if(prv_led_effect != LIGHTS_MODE_TURN_LEFT)
            {
                ROS_INFO("set led effect turnning left");
                prv_led_effect = LIGHTS_MODE_TURN_LEFT;
                set_led_effect.mode = LIGHTS_MODE_TURN_LEFT;
                do
                {
                    boost::mutex::scoped_lock(this->mtx);
                    this->set_leds_effect_vector.push_back(set_led_effect);
                }while(0);
            }
        }
        else if(turnning_direction == 2)//right
        {
            if(prv_led_effect != LIGHTS_MODE_TURN_RIGHT)
            {
                ROS_INFO("set led effect turnning right");
                prv_led_effect = LIGHTS_MODE_TURN_RIGHT;
                set_led_effect.mode = LIGHTS_MODE_TURN_RIGHT;
                do
                {
                    boost::mutex::scoped_lock(this->mtx);
                    this->set_leds_effect_vector.push_back(set_led_effect);
                }while(0);
            }
        }
    }
    else
    {
        if(power_percent < VBAT_POWER_LOW_WARNING_PERCENTAGE)
        {
            if(prv_led_effect != LIGHTS_MODE_LOW_POWER)
            {
                ROS_INFO("set led effect not charging low warning power");
                prv_led_effect = LIGHTS_MODE_LOW_POWER;
                set_led_effect.mode = LIGHTS_MODE_LOW_POWER;
                do
                {
                    boost::mutex::scoped_lock(this->mtx);
                    this->set_leds_effect_vector.push_back(set_led_effect);
                }while(0);
            }
        }
        else
        {
            if(prv_led_effect != LIGHTS_MODE_NOMAL)
            {
                ROS_INFO("set led effect not charging normal power");
                prv_led_effect = LIGHTS_MODE_NOMAL;
                set_led_effect.mode = LIGHTS_MODE_NOMAL;
                do
                {
                    boost::mutex::scoped_lock(this->mtx);
                    this->set_leds_effect_vector.push_back(set_led_effect);
                }while(0);
            }
        }
    }
}


void NoahPowerboard::get_ir_duty_param(void)
{
    int param_duty = 0;
    static int current_duty = 0;
    n.getParam("/noah_powerboard/ir_duty",param_duty);
    if(param_duty >= 0)
    {
        if(param_duty >= 100)
        {
            param_duty = 100;
        }
        if(param_duty < 0)
        {
            param_duty = 0;
        }

        if(current_duty != param_duty)
        {
            set_ir_duty_t duty_tmp;
            duty_tmp.reserve = 0;
            duty_tmp.duty = param_duty;
            this->set_ir_duty_vector.push_back(duty_tmp);
            current_duty = duty_tmp.duty;
            ROS_WARN("set ir duty to %d",duty_tmp.duty);
        }
    }
}
