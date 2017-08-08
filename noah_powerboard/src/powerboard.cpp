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

#include "../include/noah_powerboard/config.h"
#include "../include/noah_powerboard/powerboard.h"

#if 0
static led_info_t led_info;
static led_power_sys_t led_sys;
#endif
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




static int send_serial_data(powerboard_t *sys)
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

void CheckAndSend(uint8_t *data, uint8_t len)
{

}
uint8_t CalCheckSum(uint8_t *data, uint8_t len)
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
    powerboard->send_data_buf[8] = CalCheckSum(powerboard->send_data_buf, 8);
    powerboard->send_data_buf[9] = PROTOCOL_TAIL;
    send_serial_data(powerboard);

}
void NoahPowerboard::GetBatteryInfo(powerboard_t *sys)      // done
{
    sys->send_data_buf[0] = PROTOCOL_HEAD;
    sys->send_data_buf[1] = 6;
    sys->send_data_buf[2] = FRAME_TYPE_BAT_STATUS;
    sys->send_data_buf[3] = sys->bat_info.cmd;
    sys->send_data_buf[4] = CalCheckSum(sys->send_data_buf, 4);
    sys->send_data_buf[5] = PROTOCOL_TAIL;
    send_serial_data(sys);

}
void NoahPowerboard::SetModulePowerOnOff(powerboard_t *sys)
{
    sys->send_data_buf[0] = PROTOCOL_HEAD;
    sys->send_data_buf[1] = 10;
    sys->send_data_buf[2] = FRAME_TYPE_MODULE_CONTROL;
    memcpy(&sys->send_data_buf[3],(uint8_t *)&sys->module_status_set.module, 4 );
    sys->send_data_buf[7] = sys->module_status_set.on_off;
    sys->send_data_buf[8] = CalCheckSum(sys->send_data_buf, 8);
    sys->send_data_buf[9] = PROTOCOL_TAIL;
    send_serial_data(sys);
}

void NoahPowerboard::GetModulePowerOnOff(powerboard_t *sys)
{
    sys->send_data_buf[0] = PROTOCOL_HEAD;
    sys->send_data_buf[1] = 6;
    sys->send_data_buf[2] = FRAME_TYPE_GET_MODULE_STATE;
    sys->send_data_buf[3] = 1;
    sys->send_data_buf[4] = CalCheckSum(sys->send_data_buf, 4);
    sys->send_data_buf[5] = PROTOCOL_TAIL;
    send_serial_data(sys);
}
void NoahPowerboard::GetAdcData(powerboard_t *sys)      // done
{
    sys->send_data_buf[0] = PROTOCOL_HEAD;
    sys->send_data_buf[1] = 8;
    sys->send_data_buf[2] = FRAME_TYPE_GET_CURRENT;
    sys->send_data_buf[3] = 0x01;
    sys->send_data_buf[4] = 0x01;
    sys->send_data_buf[5] = sys->current_cmd_frame.cmd;
    sys->send_data_buf[6] = CalCheckSum(sys->send_data_buf, 5);
    sys->send_data_buf[7] = PROTOCOL_TAIL;
    send_serial_data(sys);
}

void NoahPowerboard::GetVersion(powerboard_t *sys)      // done
{
    sys->send_data_buf[0] = PROTOCOL_HEAD;
    sys->send_data_buf[1] = 6;
    sys->send_data_buf[2] = FRAME_TYPE_GET_VERSION;
    sys->send_data_buf[3] = sys->get_version_type;
    sys->send_data_buf[4] = CalCheckSum(sys->send_data_buf, 4);
    sys->send_data_buf[5] = PROTOCOL_TAIL;
    send_serial_data(sys);
}

void NoahPowerboard::GetSysStatus(powerboard_t *sys)     // done
{
    sys->send_data_buf[0] = PROTOCOL_HEAD;
    sys->send_data_buf[1] = 6;
    sys->send_data_buf[2] = FRAME_TYPE_SYS_STATUS;
    sys->send_data_buf[3] = 0x00;
    sys->send_data_buf[4] = CalCheckSum(sys->send_data_buf, 4);
    sys->send_data_buf[5] = PROTOCOL_TAIL;
    send_serial_data(sys);
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
    sys->send_data_buf[5] = CalCheckSum(sys->send_data_buf, 5);
    sys->send_data_buf[6] = PROTOCOL_TAIL;
    send_serial_data(sys);
}


static void handle_rev_frame(powerboard_t *sys,unsigned char * frame_buf);
int handle_receive_data(powerboard_t *sys)
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
#if 0
    if(NULL == sys)
    {
        PowerboardInfo("sys NULL!");
        return -1;
    }
    if(COM_RUN_OK != sys->com_state && COM_CHECK_VERSION != sys->com_state)
    {
        PowerboardInfo("led_handle_receive_data: com_state != COM_RUN_OK && COM_CHECK_VERSION");
        return -1;
    }
#endif
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

                        handle_rev_frame(sys,recv_buf_temp);
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



static void handle_rev_frame(powerboard_t *sys,unsigned char * frame_buf)
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
            break;

        case FRAME_TYPE_BAT_STATUS:
            sys->bat_info.cmd = frame_buf[3];
            PowerboardInfo("sys->bat_info.cmd is %d",sys->bat_info.cmd);
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
                ROS_INFO("battery voltage is %d",sys->bat_info.bat_info);
            }
            break;

        case FRAME_TYPE_GET_CURRENT:
            memcpy((uint8_t *)&sys->voltage_info.voltage_data, &frame_buf[4], sizeof(voltage_data_t));
            PowerboardInfo("12v voltage     is %5d mv",sys->voltage_info.voltage_data._12V_voltage);
            PowerboardInfo("24v voltage     is %5d mv",sys->voltage_info.voltage_data._24V_voltage);
            PowerboardInfo("5v voltage      is %5d mv",sys->voltage_info.voltage_data._5V_voltage);
            PowerboardInfo("bat voltage     is %5d mv",sys->voltage_info.voltage_data.bat_voltage);
            PowerboardInfo("_24V_temp       is %d",sys->voltage_info.voltage_data._24V_temp);
            PowerboardInfo("_12V_temp       is %d",sys->voltage_info.voltage_data._12V_temp);
            PowerboardInfo("_5V_temp        is %d",sys->voltage_info.voltage_data._5V_temp);
            PowerboardInfo("air_temp        is %d",sys->voltage_info.voltage_data.air_temp);
            PowerboardInfo("send rate       is %d",sys->voltage_info.send_rate);
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
            break;

        case FRAME_TYPE_IRLED_CONTROL:
            sys->ir_cmd.lightness_percent = frame_buf[3];
            PowerboardInfo("ir lightness is %d",sys->ir_cmd.lightness_percent);
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
                break; 
            }

        default :
            break;
    }

#if 0
    if(0 == sys->handle_data_flag)
    {
        sys->handle_data_flag = 1;
        led_info.recv_type = frame_buf[2];

        switch (led_info.recv_type)
        {		 		 
            case 0x01:
                sys->fb_mode = (LED_POWER_TYPE)frame_buf[3];
                sys->fb_effect = (LED_EFFECT_TYPE)(((short int)frame_buf[4]<<8)|((short int)frame_buf[5]));
                break;

            case 0x02:
                command = (int)frame_buf[3];
                sys->power_status1 = frame_buf[4];
                sys->power_status2 = frame_buf[5];
                switch(command)
                {
                    case 0x00:
                        sys->power_v1 = frame_buf[6];
                        sys->power_v2 = frame_buf[7];
                        break;

                    case 0x01:
                        sys->charger_power_v1 = frame_buf[6];
                        sys->charger_power_v2 = frame_buf[7];
                        break;
                    case 0x02:
                        sys->power_p1 = frame_buf[6];
                        sys->power_p2 = frame_buf[7];
                        break;
                    default:
                        break;

                }               
                break;

            case 0x03:
                command = (int)frame_buf[3];
                switch(command)
                {	
                    case 0x01:
                        sys->power_switch_status1 = frame_buf[4];
                        sys->power_switch_status2 = frame_buf[5];
                        sys->power_switch_status3 = frame_buf[6];
                        sys->power_switch_status4 = frame_buf[7];
                        break;

                    default:
                        break;
                }

            case 0x04:
                command = (int)frame_buf[3];
                switch(command)
                {	
                    case 0x01:
                        sys->err1 = frame_buf[4];
                        sys->err2 = frame_buf[5];
                        sys->err3 = frame_buf[6];
                        sys->err4 = frame_buf[7];
                        if(sys->err1 != 0 || sys->err2 != 0 || sys->err3 != 0 || sys->err4 != 0)
                        {
                            sys->power_current_temp_err = 1;
                        }
                        break;

                    default:
                        break;
                }

            case 0x05:
                sys->ctrl_power_ack = frame_buf[3];
                break;

            case 0x06:
                sys->infrared_light = frame_buf[3];
                break;

            case 0x07:			 	
                break;

            case 0x08:
                led_info.fan_switch_ack = frame_buf[3];
                break;

            case 0x0A:
                led_info.current_ctrl_type = frame_buf[3];
                switch(led_info.current_ctrl_type)
                {
                    case 0x00:		  
                        for(j=0;j<POWER_CURRENT_LEN*2;j++)								
                        {								    
                            sys->power_i[j] = frame_buf[4+j];								
                        }

                        sys->get_power_status = 1;
                        sys->get_power_flag = 0;
                        led_info.err1 = frame_buf[70];
                        led_info.err2 = frame_buf[71];
                        led_info.err3 = frame_buf[72];
                        led_info.err4 = frame_buf[73];
                        led_info.power_i_freq = frame_buf[74]; 
                        break;

                    case 0x01:
                        led_info.get_power_current_ack = frame_buf[5];
                        break;

                    default:
                        break;
                }
                break;

            case 0x0B:
                sys->error_passageway = frame_buf[3];
                for(j=0; j<POWER_ERROR_DATA_LEN; j++)
                {
                    sys->error_data[j] = frame_buf[4+j];
                }
                sys->error_power_status = 1;
                sys->power_current_temp_err = 0;
                break;

            case 0x0E:
                for(j=0; j<LED_HARDWARE_VER_LEN; j++)
                {
                    sys->hardware_version[j] = frame_buf[4+j]; 
                }
                for(j=0; j<LED_SOFTWARE_VER_LEN; j++)
                {
                    sys->software_version[j] = frame_buf[6+j]; 
                }
                break;

            case 0x0F:
                led_info.upgrade_type = (int)frame_buf[3];
                switch(led_info.upgrade_type)
                {
                    case 0x00:
                        led_info.upgrade_ready_rlt = (int)frame_buf[4];
                        break;

                    case 0x01:                    
                        led_info.upgrade_recv_rlt = (int)frame_buf[4];
                        break;

                    case 0x02:
                        led_info.upgrade_end_rlt = (int)frame_buf[4];
                        break;

                    default:
                        break;
                }
                break;

            default:
                break;
        }

        sys->handle_data_flag = 0;
        sys->rec_num++;
    }
#endif
}

#if 0
static int send_serial(unsigned char *send_buf,led_power_sys_t *sys)
{
    int len = 0;

    int send_buf_len = 0;

    if((NULL == sys) || (NULL == send_buf))
    {
        PowerboardInfo("sys or send_buf NULL!");
        return -1;
    }

    if(COM_RUN_OK != sys->com_state && COM_CHECK_VERSION != sys->com_state)
    {
        PowerboardInfo("led_send_serial: com_state != COM_RUN_OK && COM_CHECK_VERSION");
        return 0;
    }

    send_buf_len = send_buf[1];
    if(send_buf_len <= 0 )
    {
        PowerboardInfo("led send_buf len: %d small 0!",send_buf_len);
        return -1;
    }
    for(int i =0;i<send_buf_len;i++){
        //PowerboardInfo("led send_buf :%02x",send_buf[i]);
    }
    len = write(sys->com_device,send_buf,send_buf_len);
    if (len == send_buf_len)
    {
        //PowerboardInfo("led send ok");
        return 0;
    }     
    else   
    {               
        tcflush(sys->com_device,TCOFLUSH);
        if(-1 == len)
        {
            sys->com_state = COM_CLOSING;
        }
        return -1;
    }
}
void get_led_version(void)
{
    unsigned char data[6]={0};

    data[0] = 0x5A;
    data[1] = 0x06;
    data[2] = 0x0E;
    data[3] = 0;
    data[4] = data[0]+data[1]+data[2]+data[3];
    data[5] = 0xA5;
    send_serial(data,&led_sys);
    usleep(LED_SLEEP_TIME);
    handle_receive_data(&led_sys);
}

static bool check_version(void)
{
    int i = 0;
    for(i=0; i<LED_HARDWARE_VER_LEN; i++)
    {
        if('\0' == led_sys.hardware_version[i])
        {
            return false;
        }            
    }
    for(i=0; i<LED_SOFTWARE_VER_LEN; i++)
    {
        if('\0' == led_sys.software_version[i]) 
        {
            return false;
        }
    } 
    return true;		
}

static void led_init (void)
{
    char com_device_path[]="/dev/ttyUSB0";//"/dev/ros/led";//"/dev/ttyUSB0";
    led_sys.com_rssi = 0;
    led_sys.startup_flag = 0;
    led_sys.work_normal = 0;
    led_sys.pkg_type = 1;
    led_sys.work_flag = 0;
    led_sys.get_power_flag = 0;
    led_sys.error_power_status = 0;
    led_sys.get_power_status = 0;
    led_sys.power_status1 = 0;
    led_sys.power_status2 = 0;
    led_sys.power_v1 = 0;
    led_sys.power_v2 = 0;
    led_sys.power_p1 = 0;
    led_sys.power_p2 = 0;
    led_sys.charger_power_v1 = 0;
    led_sys.charger_power_v2 = 0;
    led_sys.power_switch_status1 = 0;
    led_sys.power_switch_status2 = 0;
    led_sys.power_switch_status3 = 0;
    led_sys.power_switch_status4 = 0;
    led_sys.err1 = 0;
    led_sys.err2 = 0;
    led_sys.err3 = 0;
    led_sys.err4 = 0;
    memcpy(led_sys.dev,com_device_path,sizeof(com_device_path));

    led_info.ctrl_power_ack = -1;
    led_info.get_power_current_ack = 1;
    led_info.current_ctrl_rlt = -1;
    led_info.upgrade_ready_rlt = -1;
    led_info.upgrade_recv_rlt = -1;
    led_info.upgrade_end_rlt = -1;
    led_info.power_current_temp_err = 0;
}

static void update_led_power_state(led_power_sys_t *sys)
{
    static int last_file_flag = 0;
    struct stat file_info;
    int i = 0;

    if(NULL == sys)
    {
        PowerboardInfo("sys NULL!");
        return;
    }

    switch(sys->com_state)
    {
        case COM_OPENING:
            sys->work_normal = 0;
            led_init();
            i = stat(sys->dev,&file_info);
            if(-1 == i)
            {
                //PowerboardInfo("led com device does not exist\n");
                if(-1 != last_file_flag)
                {
                    PowerboardInfo("led com device does not exist\n");
                }
                last_file_flag = i;
                return;
            }
            last_file_flag = i;
            sys->com_device = open_com_device(sys->dev);
            if(-1 != sys->com_device)
            {
                sys->com_state = COM_CHECK_VERSION;
                sys->com_rssi = 4;
                sys->work_normal = 1;
                PowerboardInfo("open led com success!");
            }
            else
            {
                PowerboardInfo("open led com device failed");
                return;
            }

            set_speed(sys->com_device,115200);
            set_parity(sys->com_device,8,1,'N');  
            break;

        case COM_CHECK_VERSION:
            get_led_version();
            if(check_version())
            {
                sys->com_state = COM_RUN_OK;
            }
            break;

        case COM_RUN_OK:
            break;

        case COM_CLOSING:
            close(sys->com_device);
            sys->com_state = COM_OPENING;
            sys->com_rssi = 0;
            PowerboardInfo("close com device:%d\n",sys->com_device);
            break;    
        default:
            break;
    }
    return;
}

static long get_upgrade_size(char* path)
{
    FILE *file;

    long len = 0;

    file = fopen(path,"rb");
    if(NULL == file)
    {
        PowerboardInfo("get upgrade size failed");
        return 0;
    }

    fseek(file,0,SEEK_END);
    len = ftell(file);
    fclose(file);	
    return len;
}

static int send_ready_upgrade(char * path, char * md5char)
{
    unsigned char data[26] = {0};
    union {
        unsigned long lnum;
        unsigned char cnum[4];
    }filesize;
    int i = 0;

    data[0] = 0x5A;
    data[1] = 0x1A;
    data[2] = 0x0F;
    data[3] = 0x00;

    for(i=0; i<16; i++)
    {
        data[i+4] = md5char[i];
    }

    //big end or small end convert socket frame
    filesize.lnum = htonl(get_upgrade_size(path));
    if(0 == filesize.lnum)
    {
        PowerboardInfo("led upgrade file is 0\n");   
        return -1;
    }

    for(i=0; i<4; i++)
    {
        data[i+20] = filesize.cnum[i];
    }

    for(i=0; i<24; i++)
    {
        data[24] += data[i];

    }
    data[25] = 0xA5;  
    send_serial(data,&led_sys);
    usleep(LED_READY_UPGRADE_SLEEP_TIME);
    handle_receive_data(&led_sys);
    PowerboardInfo("send_ready_upgrade over");
    return 0;
}

static int send_upgrade_file_frame(char* buffer,int len)
{
    unsigned char data[LED_UPGRADE_FILE_FRAME_LEN+6]={0};
    int i = 0;
    data[0] = 0x5A; 
    data[1] = len+6;
    data[2] = 0x0F;
    data[3] = 0x01;
    for(i=0; i<len; i++)
    {
        data[i+4] = buffer[i];
    }
    for(i=0; i<len+4; i++)
    {
        data[len+4] += data[i];
    }
    data[len+5] = 0xA5;     
    send_serial(data,&led_sys); 
    return 0;
}

static void send_file_over_time(int sig)
{    
    led_over_time_flag =1;
}

static int send_upgrade_file(char* path)
{	
    char buffer[LED_UPGRADE_FILE_FRAME_LEN]={0};
    FILE *file;
    int len = 0;
    int i =0;
    file = fopen(path,"rb");
    if(NULL == file)
    {
        PowerboardInfo("open led update file failed");
        return -1;
    }

    signal(SIGALRM, send_file_over_time);
    alarm(LED_UPGRADE_OVER_TIME);

    while(!feof(file))
    { 
        PowerboardInfo("send led upgrade file count:%d",i++);
        len = fread(buffer,1,LED_UPGRADE_FILE_FRAME_LEN,file);
        PowerboardInfo("led upgrade_file len:%d",len);
        send_upgrade_file_frame(buffer,len);
        bzero(buffer,LED_UPGRADE_FILE_FRAME_LEN);  
        usleep(LED_SLEEP_TIME);

        handle_receive_data(&led_sys); 

        PowerboardInfo("led upgrade_recv_rlt:%d",led_info.upgrade_recv_rlt);
        if(led_info.upgrade_recv_rlt != 0x00)
        {
            //if send fail then resend pre frame
            PowerboardInfo("led upgrade_recv_rlt fail");
            fseek(file,-LED_UPGRADE_FILE_FRAME_LEN,SEEK_CUR);
        }

        if(1 == led_over_time_flag)
        {
            PowerboardInfo("send led upgrade file over time!!");
            fclose(file);
            led_over_time_flag = 0;
            return -4;
        }
        led_info.upgrade_recv_rlt =-1;
    }
    alarm(0);
    fclose(file);
    PowerboardInfo("send led upgrade file over");
    return 0;
}

static int send_end_upgrade(void)
{
    unsigned char data[7] = {0};
    data[0] = 0x5A;
    data[1] = 0x07;
    data[2] = 0x0F;
    data[3] = 0x02;
    data[4] = 0;
    data[5] = data[0]+data[1]+data[2]+data[3]+data[4];
    data[6] = 0xA5;
    send_serial(data,&led_sys);
    usleep(LED_END_UPGRADE_SLEEP_TIME);
    handle_receive_data(&led_sys);
    return 0;
}

int led_upgrade_one_count(char * path,char * md5char)
{
    int rlt = 0;
    rlt = send_ready_upgrade(path, md5char);
    PowerboardInfo("led upgrade_ready_rlt:%d",rlt);
    if((rlt >= 0) && (0x00 == led_info.upgrade_ready_rlt))
    {
        rlt = send_upgrade_file(path);
        if(rlt >= 0)
        {
            send_end_upgrade();
            if(0x00 == led_info.upgrade_end_rlt)
            {
                rlt = 0;
            }
            else if(0x01 == led_info.upgrade_end_rlt)
            {
                rlt = -5;
            }
            else
            {
                rlt = -7;
            }
        }
    }
    else if(led_info.upgrade_ready_rlt >0)
    {
        if(0x01 == led_info.upgrade_ready_rlt)
        {
            rlt = -2;
        }
        else if(0x02 == led_info.upgrade_ready_rlt)
        {
            rlt = -3;
        }
        else
        {
            rlt = -8;
        }
    }
    else if(-1 == led_info.upgrade_ready_rlt)
    {
        rlt = -6;
    }
    PowerboardInfo("led_upgrade_rlt:%d",rlt); 
    return rlt;
}

/*
 *return -1:file open failed;
 *       -2:led mcu memory is not enough
 *       -3:led mcu memory execute other job
 *       -4:send file over time 
 *       -5:led mcu firmware check end frame error
 *       -6:nv have not ready frame
 *       -7:nv have not end frame
 */
int led_upgrade(char * path,char * md5char)
{
    int upgrade_count = 0;
    int rlt = -1;
    while(upgrade_count < 3)
    {
        rlt = led_upgrade_one_count(path,md5char);
        if(rlt < 0)
        {
            upgrade_count++;
        }
        else
        {
            PowerboardInfo("led upgrade success");
            break;
        }
    }
    PowerboardInfo("upgrade_count:%d",upgrade_count);
    return rlt;    
}

int set_led_power_effect(LED_POWER_TYPE mode,LED_EFFECT_TYPE effect)
{
    led_sys.work_flag = 1;
    led_sys.cmd_mode = mode;
    led_sys.cmd_effect = effect;
    return 0;
}

int set_led_power_function(int module,int command)
{
    int rlt = -1;
    unsigned char data[7]={0};

    data[0] = 0x5A;
    data[1] = 0x07;
    data[2] = 0x05;
    data[3] = module;
    data[4] = command;
    data[5] = data[0]+data[1]+data[2]+data[3]+data[4];
    data[6] = 0xA5;
    send_serial(data,&led_sys);
    usleep(LED_SLEEP_TIME);
    handle_receive_data(&led_sys);
    if(1 == led_info.ctrl_power_ack)
    {
        rlt = 0;
        led_info.ctrl_power_ack = -1;
    }
    return rlt;
}

static void get_power_current(led_power_sys_t *sys)
{    
    unsigned char data[8]={0};
    unsigned char check_data = 0;

    data[0] = 0x5A;
    data[1] = 0x08;
    data[2] = 0x0A;
    data[3] = 0x01;
    data[4] = 0x01;
    data[5] = 0x00;
    for(int i=0; i<6; i++)
    {
        check_data += data[i];
    }
    data[6] = check_data;
    data[7] = 0xA5;
    send_serial(data,&led_sys);
    usleep(LED_SLEEP_TIME);
    handle_receive_data(&led_sys);
    return;
}

static void get_power_error_data(led_power_sys_t *sys)
{
    unsigned char data[6]={0};
    unsigned char check_data = 0;

    data[0] = 0x5A;
    data[1] = 0x06;
    data[2] = 0x0B;
    data[3] = 0x00;

    for(int i=0; i<4; i++)
    {
        check_data += data[i];
    }
    data[4] = check_data;
    data[5] = 0xA5;
    send_serial(data,&led_sys);
    usleep(LED_SLEEP_TIME);
    handle_receive_data(&led_sys);
    return;
}

static void get_sysstatus_voltage(led_power_sys_t *sys,int voltage_type)
{    
    unsigned char data[6]={0};
    unsigned char check_data = 0;

    data[0] = 0x5A;
    data[1] = 0x06;
    data[2] = 0x02;
    data[3] = voltage_type;
    for(int i=0; i<4; i++)
    {
        check_data += data[i];
    }
    data[4] = check_data;
    data[5] = 0xA5;
    send_serial(data,&led_sys);
    usleep(LED_SLEEP_TIME);
    handle_receive_data(&led_sys);
    return;
}

static void get_sysstatus_power_percent(led_power_sys_t *sys,int percent_type)
{
    unsigned char data[6]={0};
    unsigned char check_data = 0;

    data[0] = 0x5A;
    data[1] = 0x06;
    data[2] = 0x02;
    data[3] = percent_type;
    for(int i=0; i<4; i++)
    {
        check_data += data[i];
    }
    data[4] = check_data;
    data[5] = 0xA5;
    send_serial(data,&led_sys);
    usleep(LED_SLEEP_TIME);
    handle_receive_data(&led_sys);
    return;
}

static void get_module_switch(led_power_sys_t *sys,int switch_type)
{    
    unsigned char data[6]={0};
    unsigned char check_data = 0;

    data[0] = 0x5A;
    data[1] = 0x06;
    data[2] = 0x03;
    data[3] = switch_type;
    for(int i=0; i<4; i++)
    {
        check_data += data[i];
    }
    data[4] = check_data;
    data[5] = 0xA5;
    send_serial(data,&led_sys);
    usleep(LED_SLEEP_TIME);
    handle_receive_data(&led_sys);
    return;
}

static void get_current_fault(led_power_sys_t *sys,int fault_type)
{    
    unsigned char data[6]={0};
    unsigned char check_data = 0;

    data[0] = 0x5A;
    data[1] = 0x06;
    data[2] = 0x04;
    data[3] = fault_type;
    for(int i=0; i<4; i++)
    {
        check_data += data[i];
    }
    data[4] = check_data;
    data[5] = 0xA5;
    send_serial(data,&led_sys);
    usleep(LED_SLEEP_TIME);
    handle_receive_data(&led_sys);
    return;
}

void send_led_power_pkg(led_power_sys_t *sys)
{
    static int init_flag = 0;
    int i = 0;
    unsigned char data[8]={0};
    unsigned char check_data = 0;

    if(0 == init_flag)
    {
        if(LED_NORMAL != sys->fb_effect)
        {
            sys->cmd_mode = LED_POWER_FREEDOM;
            sys->cmd_effect = LED_NORMAL;
        }
        else
        {
            init_flag = 1;
        }
    }

    data[0] = 0x5A;
    data[1] = 0x08;
    data[2] = 0x01;
    if(1 == sys->work_flag)
    {
        data[3] = (unsigned char)sys->cmd_mode;
        data[4] = (sys->cmd_effect & 0xff00)>>8;
        data[5] = sys->cmd_effect & 0x0ff;
    }
    else
    {
        data[3] = 0;
        data[4] = 0;
        data[5] = 0;
    }
    for(i=0; i<6; i++)
    {
        check_data += data[i];
    }
    data[6] = check_data;
    data[7] = 0xA5;
    if(0 == send_serial(data,sys))
    {
        sys->work_flag = 0;
    }
    usleep(LED_SLEEP_TIME);
    handle_receive_data(&led_sys);
    return;
}

void infrared_light_ctrl(int type,int light)
{    
    unsigned char data[7]={0};
    unsigned char check_data = 0;

    data[0] = 0x5A;
    data[1] = 0x07;
    data[2] = 0x06;
    data[3] = type;
    data[4] = light;
    for(int i=0; i<5; i++)
    {
        check_data += data[i];
    }
    data[5] = check_data;
    data[6] = 0xA5;
    send_serial(data,&led_sys);
    usleep(LED_SLEEP_TIME);
    handle_receive_data(&led_sys);
    return;
}

void fan_switch_ctrl(int fan_switch)
{    
    unsigned char data[6]={0};
    unsigned char check_data = 0;

    data[0] = 0x5A;
    data[1] = 0x06;
    data[2] = 0x08;
    data[3] = fan_switch;
    for(int i=0; i<4; i++)
    {
        check_data += data[i];
    }
    data[4] = check_data;
    data[5] = 0xA5;
    send_serial(data,&led_sys);
    usleep(LED_SLEEP_TIME);
    handle_receive_data(&led_sys);
    return;
}

static int check_com_rssi(int send_num,led_power_sys_t *sys)
{
    if(0 == send_num)
    {
        if(sys->rec_num >= 8)
        {
            sys->com_rssi = 4;
        }
        else if(sys->rec_num >= 6)
        {
            sys->com_rssi = 3;
        }
        else if(sys->rec_num >= 4)
        {
            sys->com_rssi = 2;
        }
        else if(sys->rec_num >= 1)
        {
            sys->com_rssi = 1;
        }
        else
        {
            sys->com_rssi = 0;
        }

        sys->rec_num = 0;
    }
    return 0;
}

void clear_error_data(void)
{
    led_sys.error_passageway = 0;
    for(int j=0; j<POWER_ERROR_DATA_LEN; j++)
    {
        led_sys.error_data[j] = 0;
    }
}

void *led_thread_start(void *)
{
    int send_num = 0;
    static int flag = 0;
    led_sys.com_state = COM_OPENING;
    update_led_power_state(&led_sys);
    if((led_sys.led_freq <= 0) || (led_sys.led_freq >2))
    {
        led_sys.led_freq = 2.0;
    }
    ros::Rate loop_rate(led_sys.led_freq);

    while(ros::ok()) 
    {  
        if(0 == led_sys.upgrade_status)
        {
            update_led_power_state(&led_sys);
            handle_receive_data(&led_sys);

            if(COM_RUN_OK == led_sys.com_state)
            {
                check_com_rssi(send_num,&led_sys);
                send_led_power_pkg(&led_sys);
                get_sysstatus_voltage(&led_sys,0);	
                get_sysstatus_power_percent(&led_sys,2);
                get_module_switch(&led_sys,1);
                get_current_fault(&led_sys,1);
                //set_led_power_function(3,0);
                //infrared_light_ctrl(0,10);
                //fan_switch_ctrl(1);
                //set_led_power_effect(LED_POWER_FREEDOM,LED_GREEN_LONG);
                if(led_sys.power_current_temp_err == 1)
                {
                    clear_error_data();
                    get_power_error_data(&led_sys);
                }
                if(led_sys.get_power_flag == 1)
                {
                    get_power_current(&led_sys);
                }
                send_num=(send_num + 1)%10;
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
        else if(1 == led_sys.upgrade_status)
        {
            if(flag == 0){
                flag = 1;
                led_sys.upgrade_result = led_upgrade(led_sys.name,led_sys.md5);
                led_sys.upgrade_status = 0;
                flag = 0;
            }
        }
    }

    set_led_power_effect(LED_POWER_FREEDOM,LED_DEFAULT);
    send_led_power_pkg(&led_sys);
    sleep(2);
    close(led_sys.com_device);
    led_sys.com_state = COM_OPENING;
    led_sys.com_rssi = 0;
    led_sys.work_normal = 0;

    return 0;
}

led_power_sys_t *get_led_power_info(void)
{
    if(0 == led_sys.handle_data_flag)
    {
        return &led_sys;
    }
    else
    {
        return NULL;
    }
}

void set_led_prior(int type,int value)
{
    switch(type)
    {
        case 0:
            if(0 == value)
            {
                led_sys.prior_led &= (~0x01);
            }
            else
            {
                led_sys.prior_led |= 0x01;
            }
            break;
        case 1:
            if(0 == value)
            {
                led_sys.prior_led &= (~0x02);
            }
            else
            {
                led_sys.prior_led |= 0x02;
            }
            break;
        case 2:
            if(0 == value)
            {
                led_sys.prior_led &= (~0x04);
            }
            else
            {
                led_sys.prior_led |= 0x04;
            }
            break;
        default:
            break;
    }
    return;
}

int get_led_prior(void)
{
    return led_sys.prior_led;
}

int set_power_upgrade(char *str,char *md5)
{
    if((NULL == str) || (NULL == md5))
    {
        return -1;
    }
    if(0 == led_sys.upgrade_status)
    {
        memcpy(led_sys.name,str,strlen(str));
        memcpy(led_sys.md5,md5,MD5_SIZE);
        led_sys.upgrade_status = 1;
        return 0;
    }
    return 1;
}

void set_get_power_flag(void)
{
    led_sys.get_power_flag = 1;
}

void clear_get_power_status(void)
{
    led_sys.get_power_status = 0;
}

void clear_error_power_status(void)
{
    led_sys.error_power_status = 0;
}

int get_power_upgrade_result(void)
{
    int i = led_sys.upgrade_result;
    return i;
}

int get_power_upgrade_status(void)
{
    int i = led_sys.upgrade_status;
    return i;
}
#endif
