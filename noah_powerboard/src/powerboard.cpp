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


void test_fun(void)
{
    module_ctrl_t param;  
    static uint8_t cnt = 0;
    param.module = POWER_5V_EN | POWER_12V_EN | POWER_LED_MCU;
    param.group_num = 1;
    if(cnt++ % 2)
    {
        param.on_off = 0;
    }
    else
    {
        param.on_off = 1;
    }

    //    module_set_vector.push_back(param);
}

class NoahPowerboard;
#define MODULE_SET_TIME_OUT         1000//ms
#define GET_BAT_INFO_TIME_OUT       1000//ms
#define GET_SYS_STSTUS_TIME_OUT     1000//ms
void *CanProtocolProcess(void* arg)
{
    module_ctrl_t module_set;


    NoahPowerboard *pNoahPowerboard =  (NoahPowerboard*)arg;

    bool module_flag = 0;
    while(ros::ok())
    {
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
                        //if((module_set_ack.module <= POWER_ALL ) && (module_set_ack.on_off <= 1) && (module_set_ack.group_num <= 2))
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
            }

        }

        if(!pNoahPowerboard->get_bat_info_vector.empty()) 
        {   
            auto a = pNoahPowerboard->get_bat_info_vector.begin(); 

            uint8_t flag = 0;
            uint32_t time_out_cnt = 0;

            pNoahPowerboard->get_bat_info_vector.erase(a);
            ROS_INFO("get get bat info cmd");

            pNoahPowerboard->get_bat_info_ack_vector.clear();

            pNoahPowerboard->GetBatteryInfo(pNoahPowerboard->sys_powerboard);

            while(time_out_cnt < GET_BAT_INFO_TIME_OUT/5)
            {
                time_out_cnt++;
                if(!pNoahPowerboard->get_bat_info_ack_vector.empty())
                {
                    auto b = pNoahPowerboard->get_bat_info_ack_vector.begin();
                    get_bat_info_ack_t get_bat_info_ack = *b;
                    ROS_INFO("get get_bat_info_ack_vector data");
                    ROS_INFO("get bat voltage %d",get_bat_info_ack.bat_vol);
                    ROS_INFO("get bat percent %d",get_bat_info_ack.bat_percent);
                    break;
                }
                else
                {
                    usleep(5*1000);
                }
            }
            if(time_out_cnt < GET_BAT_INFO_TIME_OUT/5)
            {
                ROS_INFO("get bat info flow OK");
            }
            else
            {
                ROS_ERROR("get bat info time out");
            }
        }



        if(!pNoahPowerboard->get_sys_status_vector.empty()) 
        {   
            auto a = pNoahPowerboard->get_sys_status_vector.begin(); 

            uint8_t flag = 0;
            uint32_t time_out_cnt = 0;

            pNoahPowerboard->get_sys_status_vector.erase(a);
            ROS_INFO("get get bat info cmd");

            pNoahPowerboard->get_sys_status_ack_vector.clear();

            pNoahPowerboard->GetSysStatus(pNoahPowerboard->sys_powerboard);

            while(time_out_cnt < GET_SYS_STSTUS_TIME_OUT/5)
            {
                time_out_cnt++;
                if(!pNoahPowerboard->get_sys_status_ack_vector.empty())
                {
                    auto b = pNoahPowerboard->get_sys_status_ack_vector.begin();
                    get_sys_status_ack_t get_sys_status_ack = *b;
                    ROS_INFO("get get_sys_status_ack_vector data");
                    ROS_INFO("sys status is  %d",get_sys_status_ack.sys_status);
                    break;
                }
                else
                {
                    usleep(5*1000);
                }
            }
            if(time_out_cnt < GET_SYS_STSTUS_TIME_OUT/5)
            {
                ROS_INFO("get bat info flow OK");
            }
            else
            {
                ROS_ERROR("get bat info time out");
            }
        }

        usleep(5000);
    }
}


int NoahPowerboard::PowerboardParamInit(void)
{
    sys_powerboard->led_set.effect = LIGHTS_MODE_DEFAULT;
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
begin:
    static uint8_t err_cnt = 0;

    int error = -1;
    powerboard->send_data_buf[0] = PROTOCOL_HEAD;
    powerboard->send_data_buf[1] = 0x0a;
    powerboard->send_data_buf[2] = FRAME_TYPE_LEDS_CONTROL;
    powerboard->send_data_buf[3] = powerboard->led_set.effect;
    memcpy(&powerboard->send_data_buf[4], (uint8_t *)&(powerboard->led_set.color), sizeof(color_t));
    powerboard->send_data_buf[7] = powerboard->led_set.period;
    powerboard->send_data_buf[8] = this->CalCheckSum(powerboard->send_data_buf, 8);
    powerboard->send_data_buf[9] = PROTOCOL_TAIL;
    this->send_serial_data(powerboard);
    usleep(TEST_WAIT_TIME);
    if((error = this->handle_receive_data(powerboard)) < 0)
    {

        if(err_cnt++ < COM_ERR_REPEAT_TIME)
        {
            usleep(500*1000); 
            ROS_ERROR("Set leds effect start to resend");
            goto begin; 
        }
        ROS_ERROR("Set Leds Effecct : com error !");
    }
    else
    {
        err_cnt = 0;
    }
    return error;
}
int NoahPowerboard::GetBatteryInfo(powerboard_t *sys)      // done
{
    int error = -1; 
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
    static uint8_t err_cnt = 0;
    int error = -1; 



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
    int error = -1;
    sys->send_data_buf[0] = PROTOCOL_HEAD;
    sys->send_data_buf[1] = 8;
    sys->send_data_buf[2] = FRAME_TYPE_GET_CURRENT;
    sys->send_data_buf[3] = 0x01;
    sys->send_data_buf[4] = 0x01;
    sys->send_data_buf[5] = sys->current_cmd_frame.cmd;
    sys->send_data_buf[6] = this->CalCheckSum(sys->send_data_buf, 5);
    sys->send_data_buf[7] = PROTOCOL_TAIL;
    this->send_serial_data(sys);
    usleep(TEST_WAIT_TIME);
    error = this->handle_receive_data(sys);
    if(error < 0)
    {

    }
    return error;
}

int NoahPowerboard::GetVersion(powerboard_t *sys)      // done
{
    int error = -1;
    sys->send_data_buf[0] = PROTOCOL_HEAD;
    sys->send_data_buf[1] = 6;
    sys->send_data_buf[2] = FRAME_TYPE_GET_VERSION;
    sys->send_data_buf[3] = sys->get_version_type;
    sys->send_data_buf[4] = this->CalCheckSum(sys->send_data_buf, 4);
    sys->send_data_buf[5] = PROTOCOL_TAIL;
    this->send_serial_data(sys);
    usleep(TEST_WAIT_TIME);
    error = this->handle_receive_data(sys);
    if(error < 0)
    {

    }
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
    int error = -1;
    sys->send_data_buf[0] = PROTOCOL_HEAD;
    sys->send_data_buf[1] = 7;
    sys->send_data_buf[2] = FRAME_TYPE_IRLED_CONTROL;
    sys->send_data_buf[3] = sys->ir_cmd.cmd;
    if(sys->send_data_buf[3] == IR_CMD_READ)
    {
        sys->send_data_buf[4] = 0;
    }
    else if(sys->send_data_buf[3] == IR_CMD_WRITE)
    {
        sys->send_data_buf[4] = sys->ir_cmd.set_ir_percent;
    }
    sys->send_data_buf[5] = this->CalCheckSum(sys->send_data_buf, 5);
    sys->send_data_buf[6] = PROTOCOL_TAIL;
    this->send_serial_data(sys);
    usleep(TEST_WAIT_TIME);
    error = this->handle_receive_data(sys);
    if(error < 0)
    {

    }
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
            //PowerboardInfo("sys->bat_info.cmd is %d",sys->bat_info.cmd);
#if 0
            if(sys->bat_info.cmd == CMD_BAT_VOLTAGE)
            {
                sys->bat_info.bat_info = frame_buf[5]<< 8  | frame_buf[4]; 
                PowerboardInfo("battery voltage is %d",sys->bat_info.bat_info);
            }
            if(sys->bat_info.cmd == CMD_BAT_PERCENT)
            {
                sys->bat_info.bat_info = frame_buf[5] << 8  | frame_buf[4]; 
                if(sys->bat_info.bat_info > 100)
                {
                    sys->bat_info.bat_info = 100;
                }
                PowerboardInfo("battery voltage is %d",sys->bat_info.bat_info);
            }
#endif
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
        module_ctrl_ack.module_status_ack  = *(uint32_t *)&msg->Data[6];
        do
        {
            boost::mutex::scoped_lock(this->mtx);        
            this->module_set_ack_vector.push_back(module_ctrl_ack);
        }while(0);

        ROS_INFO("receive module : 0x%x",module_ctrl_ack.module);
        ROS_INFO("receive module state : 0x%x",module_ctrl_ack.module_status_ack);

#if 0
        if(module_ctrl_ack.module & POWER_VSYS_24V_NV)
        {

            ROS_INFO("module %d",module_ctrl_ack.module);

            this->j.clear();
            this->j = 
            {
                {"sub_name","set_module_state"},
                {
                    "data",
                    {
                        {"door_ctrl_state",(bool)(module_ctrl_ack.module_status_ack & POWER_VSYS_24V_NV)},
                        {"error_code", CAN_SOURCE_ID_SET_MODULE_STATE},
                    } 
                }
            };
            this->pub_json_msg_to_app(this->j);
        }
#endif 

    }

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_GET_BAT_STATE)
    {
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_GET_BAT_STATE");
        get_bat_info_ack_t get_bat_info_ack;
        get_bat_info_ack.bat_vol = *(uint16_t *)&msg->Data[1];
        get_bat_info_ack.bat_percent = *(uint16_t *)&msg->Data[3];

        this->get_bat_info_ack_vector.push_back(get_bat_info_ack);

        ROS_INFO("receive bat vol : %d",get_bat_info_ack.bat_vol);
        ROS_INFO("receive bat percent : %d",get_bat_info_ack.bat_percent);
        this->sys_powerboard->bat_info.bat_vol = get_bat_info_ack.bat_vol;
        this->sys_powerboard->bat_info.bat_percent = get_bat_info_ack.bat_percent;
        //this->PubPower(this->sys_powerboard);
    }

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_GET_SYS_STATE)
    {
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_GET_SYS_STATE");
        get_sys_status_ack_t get_sys_status_ack;
        get_sys_status_ack.sys_status = *(uint16_t *)&msg->Data[1];

        this->get_sys_status_ack_vector.push_back(get_sys_status_ack);

        this->sys_powerboard->sys_status = get_sys_status_ack.sys_status;
        this->PubPower(this->sys_powerboard);
    }
}



