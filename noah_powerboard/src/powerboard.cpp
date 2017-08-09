#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
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

#include "../include/noah_powerboard/powerboard.h"

#define TEST_WAIT_TIME      30*1000

#define PowerboardInfo     ROS_INFO

static int led_over_time_flag = 0;
static int last_unread_bytes = 0;
static unsigned char recv_buf_last[BUF_LEN] = {0};

powerboard_t    sys_powerboard_ram; 
powerboard_t    *sys_powerboard = &sys_powerboard_ram;


void NoahPowerboard::PowerboardParamInit(void)
{
    //char dev_path[] = "/dev/ttyUSB0";
    char dev_path[] = "/dev/noah_powerboard";
    memcpy(sys_powerboard->dev,dev_path, sizeof(dev_path));
    sys_powerboard->led_set.effect = LIGHTS_MODE_DEFAULT;
}


int NoahPowerboard::send_serial_data(powerboard_t *sys)
{
    int len = 0;

    int send_buf_len = 0;

    if((sys->device < 0) || (NULL == sys->send_data_buf))
    {
        ROS_INFO("dev or send_buf NULL!");
        return -1;
    }

    send_buf_len = sys->send_data_buf[1];

    if(send_buf_len <= 0 )
    {
        PowerboardInfo("noah_power send_buf len: %d small 0!",send_buf_len);
        return -1;
    }
    for(int i =0;i<send_buf_len;i++)
    {
        //PowerboardInfo("noah_power send_buf :%02x",sys->send_data_buf[i]);
    }
    len = write(sys->device,sys->send_data_buf,send_buf_len);
    if (len == send_buf_len)
    {
        //PowerboardInfo("noah_powerboard send ok");
        return 0;
    }     
    else   
    {               
        tcflush(sys->device,TCOFLUSH);
        if(-1 == len)
        {
            //sys->com_state = COM_CLOSING;
        }
        return -1;
    }
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

void NoahPowerboard::SetLedEffect(powerboard_t *powerboard)     // done
{
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
    this->handle_receive_data(powerboard);
}
void NoahPowerboard::GetBatteryInfo(powerboard_t *sys)      // done
{
    sys->send_data_buf[0] = PROTOCOL_HEAD;
    sys->send_data_buf[1] = 6;
    sys->send_data_buf[2] = FRAME_TYPE_BAT_STATUS;
    sys->send_data_buf[3] = sys->bat_info.cmd;
    sys->send_data_buf[4] = this->CalCheckSum(sys->send_data_buf, 4);
    sys->send_data_buf[5] = PROTOCOL_TAIL;
    this->send_serial_data(sys);
    usleep(TEST_WAIT_TIME);
    this->handle_receive_data(sys);

}
void NoahPowerboard::SetModulePowerOnOff(powerboard_t *sys)
{
    sys->send_data_buf[0] = PROTOCOL_HEAD;
    sys->send_data_buf[1] = 10;
    sys->send_data_buf[2] = FRAME_TYPE_MODULE_CONTROL;
    memcpy(&sys->send_data_buf[3],(uint8_t *)&sys->module_status_set.module, 4 );
    sys->send_data_buf[7] = sys->module_status_set.on_off;
    sys->send_data_buf[8] = this->CalCheckSum(sys->send_data_buf, 8);
    sys->send_data_buf[9] = PROTOCOL_TAIL;
    this->send_serial_data(sys);
    usleep(TEST_WAIT_TIME);
    this->handle_receive_data(sys);
}

void NoahPowerboard::GetModulePowerOnOff(powerboard_t *sys)
{
    sys->send_data_buf[0] = PROTOCOL_HEAD;
    sys->send_data_buf[1] = 6;
    sys->send_data_buf[2] = FRAME_TYPE_GET_MODULE_STATE;
    sys->send_data_buf[3] = 1;
    sys->send_data_buf[4] = CalCheckSum(sys->send_data_buf, 4);
    sys->send_data_buf[5] = PROTOCOL_TAIL;
    this->send_serial_data(sys);
    usleep(TEST_WAIT_TIME);
    this->handle_receive_data(sys);
}
void NoahPowerboard::GetAdcData(powerboard_t *sys)      // done
{
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
    this->handle_receive_data(sys);
}

void NoahPowerboard::GetVersion(powerboard_t *sys)      // done
{
    sys->send_data_buf[0] = PROTOCOL_HEAD;
    sys->send_data_buf[1] = 6;
    sys->send_data_buf[2] = FRAME_TYPE_GET_VERSION;
    sys->send_data_buf[3] = sys->get_version_type;
    sys->send_data_buf[4] = this->CalCheckSum(sys->send_data_buf, 4);
    sys->send_data_buf[5] = PROTOCOL_TAIL;
    this->send_serial_data(sys);
    usleep(TEST_WAIT_TIME);
    this->handle_receive_data(sys);
}

void NoahPowerboard::GetSysStatus(powerboard_t *sys)     // done
{
    sys->send_data_buf[0] = PROTOCOL_HEAD;
    sys->send_data_buf[1] = 6;
    sys->send_data_buf[2] = FRAME_TYPE_SYS_STATUS;
    sys->send_data_buf[3] = 0x00;
    sys->send_data_buf[4] = this->CalCheckSum(sys->send_data_buf, 4);
    sys->send_data_buf[5] = PROTOCOL_TAIL;
    this->send_serial_data(sys);
    usleep(TEST_WAIT_TIME);
    this->handle_receive_data(sys);
}

void NoahPowerboard::InfraredLedCtrl(powerboard_t *sys)     // done
{
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
    this->handle_receive_data(sys);
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

                        this->handle_rev_frame(sys,recv_buf_temp);
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
    return 0;
}


void NoahPowerboard::FromAppRcvCallback(const std_msgs::String::ConstPtr msg)
{
    ROS_INFO("Rcv test data");
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
         
void NoahPowerboard::handle_rev_frame(powerboard_t *sys,unsigned char * frame_buf)
{
    int frame_len = 0;
    int i = 0;
    int j = 0;
    int command = 0; 
    unsigned char check_data = 0;
    uint8_t cmd_type = 0;
    frame_len = frame_buf[1];

    for(i=0;i<frame_len-2;i++)
    {
        check_data += frame_buf[i];
    }

    if(check_data != frame_buf[frame_len-2] || PROTOCOL_TAIL != frame_buf[frame_len -1])
    {
        PowerboardInfo("led receive frame check error");
        return;
    }
    //PowerboardInfo("Powrboard recieve data check OK.");
#if 0
    for(i =0;i<frame_len;i++)
    {
        //PowerboardInfo("led receive:%02x",frame_buf[i]);
    }
    if(2 != sys->work_normal)
    {
        sys->work_normal = 2;
    }
#endif
    cmd_type = frame_buf[2];
    switch(cmd_type)
    {
        case FRAME_TYPE_LEDS_CONTROL:
            //rcv_serial_leds_frame_t rcv_serial_led_frame;
            memcpy((uint8_t *)&sys->rcv_serial_leds_frame, &frame_buf[3], sizeof(rcv_serial_leds_frame_t) );
            PowerboardInfo("cur_light_mode is %d", sys->rcv_serial_leds_frame.cur_light_mode );
            PowerboardInfo("color.r is %2x",       sys->rcv_serial_leds_frame.color.r);
            PowerboardInfo("color.g is %2x",        sys->rcv_serial_leds_frame.color.g);
            PowerboardInfo("color.b is %2x",        sys->rcv_serial_leds_frame.color.b);
            PowerboardInfo("period is %d",          sys->rcv_serial_leds_frame.period);

            this->j.clear();
            this->j = 
            {
                {"sub_name","led_ctrl"},
                {
                    "data",
                    {
                        {"cur_light_mode",sys->rcv_serial_leds_frame.cur_light_mode},
                        {"color_r",sys->rcv_serial_leds_frame.color.r},
                        {"color_g",sys->rcv_serial_leds_frame.color.g},
                        {"color_b",sys->rcv_serial_leds_frame.color.b},
                        {"period",sys->rcv_serial_leds_frame.period},
                    }
                }
            };
            this->pub_json_msg_to_app(this->j);
            break;

        case FRAME_TYPE_BAT_STATUS:
            sys->bat_info.cmd = frame_buf[3];
            PowerboardInfo("sys->bat_info.cmd is %d",sys->bat_info.cmd);
            if(sys->bat_info.cmd == CMD_BAT_VOLTAGE)
            {
                sys->bat_info.bat_info = frame_buf[5]<< 8  | frame_buf[4]; 
                PowerboardInfo("battery voltage is %d",sys->bat_info.bat_info);

                this->j.clear();
                this->j = 
                {
                    {"sub_name","battery_info"},
                    {
                        "data",
                        {
                            {"battery_voltage",sys->bat_info.bat_info},
                        }
                    }
                };
                this->pub_json_msg_to_app(this->j);
            }
            if(sys->bat_info.cmd == CMD_BAT_PERCENT)
            {
                sys->bat_info.bat_info = frame_buf[5] << 8  | frame_buf[4]; 
                if(sys->bat_info.bat_info > 100)
                {
                    sys->bat_info.bat_info = 100;
                }
                PowerboardInfo("battery voltage is %d",sys->bat_info.bat_info);
                this->j.clear();
                this->j = 
                {
                    {"sub_name","battery_info"},
                    {
                        "data",
                        {
                            {"battery_percent",sys->bat_info.bat_info},
                        }
                    }
                };
                this->pub_json_msg_to_app(this->j);
            }
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

            this->j.clear();
            this->j = 
            {
                {"sub_name","get_adc_data"},
                {
                    "data",
                    {

                        {"_12v_voltage", sys->voltage_info.voltage_data._12V_voltage},
                        {"_24v_voltage", sys->voltage_info.voltage_data._24V_voltage},
                        {"_5v_voltage", sys->voltage_info.voltage_data._5V_voltage},
                        {"bat_voltage", sys->voltage_info.voltage_data.bat_voltage},
                        {"_24V_temp", sys->voltage_info.voltage_data._24V_temp},
                        {"_12V_temp", sys->voltage_info.voltage_data._12V_temp},
                        {"_5V_temp", sys->voltage_info.voltage_data._5V_temp},
                        {"air_temp", sys->voltage_info.voltage_data.air_temp},
                        {"send_rate", sys->voltage_info.send_rate},
                    }
                }
            };
            this->pub_json_msg_to_app(this->j);
            break;

        case FRAME_TYPE_GET_VERSION:
            sys->get_version_type = frame_buf[3];
            if(sys->get_version_type == VERSION_TYPE_FW)
            {
                memcpy((uint8_t *)&sys->hw_version,&frame_buf[4], HW_VERSION_SIZE);
                memcpy((uint8_t *)&sys->sw_version,&frame_buf[7], SW_VERSION_SIZE);
                PowerboardInfo("hw version: %s",sys->hw_version);
                PowerboardInfo("sw version: %s",sys->sw_version);
                this->j.clear();
                this->j = 
                {
                    {"sub_name","get_version"},
                    {
                        "data",
                        {
                            {"hw_version",sys->hw_version},     
                            {"sw_version",sys->sw_version},     
                        }
                    }
                };
                this->pub_json_msg_to_app(this->j);
            }

            if(sys->get_version_type == VERSION_TYPE_PROTOCOL)
            {
                memcpy((uint8_t *)&sys->protocol_version,&frame_buf[4], PROTOCOL_VERSION_SIZE);
                PowerboardInfo("protocol version: %s",sys->protocol_version);
                this->j.clear();
                this->j = 
                {
                    {"sub_name","get_version"},
                    {
                        "data",
                        {
                            {"protocol_version",sys->protocol_version},     
                        }
                    }
                };
                this->pub_json_msg_to_app(this->j);
            }


            break;

        case FRAME_TYPE_SYS_STATUS:
            sys->sys_status = frame_buf[4];
            switch(sys->sys_status)
            {
                case SYS_STATUS_OFF:
                    PowerboardInfo("system status: off");
                    break;
                case SYS_STATUS_TURNING_ON:
                    PowerboardInfo("system status: turning on...");
                    break;
                case SYS_STATUS_ON:
                    PowerboardInfo("system status: on");
                    break;
                case SYS_STATUS_TURNING_OFF:
                    PowerboardInfo("system status:turning off...");
                    break;
                case SYS_STATUS_ERR:
                    PowerboardInfo("system status:error");
                    break;
                default:
                    break;
            }

            this->j.clear();
            this->j = 
            {
                {"pub_name","get_sys_status"},
                {
                    "data",
                    {
                        {"sys_status",sys->sys_status},
                    }
                }
            };
            this->pub_json_msg_to_app(this->j);

            break;

        case FRAME_TYPE_IRLED_CONTROL:
            sys->ir_cmd.lightness_percent = frame_buf[3];
            PowerboardInfo("ir lightness is %d",sys->ir_cmd.lightness_percent);

            this->j.clear();
            this->j = 
            {
                {"sub_name","ir_lightness"},
                {
                    "data",
                    {
                        {"ir_lightness",sys->ir_cmd.lightness_percent},
                    }
                }
            };
            this->pub_json_msg_to_app(this->j);

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
                    PowerboardInfo("hard ware not support !");
                }

                this->j.clear();
                this->j = 
                {
                    {"sub_name","get_module_state"},
                    {
                       "data",
                       {
                           {"module_status",sys->module_status.module},
                       } 
                    }
                };
                this->pub_json_msg_to_app(this->j);

                break; 
            }

        default :
            break;
    }
}

