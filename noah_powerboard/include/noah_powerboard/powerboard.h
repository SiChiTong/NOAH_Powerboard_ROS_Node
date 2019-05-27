#include "ros/ros.h"
#include "std_msgs/String.h"
#include "json.hpp"
#include <mrobot_msgs/vci_can.h>
#include <roscan/can_long_frame.h>
#include <boost/thread/mutex.hpp>
#include <noah_powerboard/remote_power_ctrl_srv.h>
using json = nlohmann::json;
#ifndef LED_H
#define LED_H


#define PROTOCOL_HEAD               0x5a
#define PROTOCOL_TAIL               0xa5
#define POWER_CURRENT_LEN           33

#define BUF_LEN                    256

#define FRAME_TYPE_LEDS_CONTROL         0x01
#define FRAME_TYPE_SYS_STATUS           0x02
#define FRAME_TYPE_BAT_STATUS           0x03
#define FRAME_TYPE_GET_MODULE_STATE     0x04
//#define FRAME_TYPE_READ_ERR_CURRENT     0x05//
#define FRAME_TYPE_MODULE_CONTROL       0x06
#define FRAME_TYPE_IRLED_CONTROL        0x07
#define FRAME_TYPE_GET_CURRENT          0x0a
#define FRAME_TYPE_GET_VERSION          0x0e

#define COM_ERR_REPEAT_TIME             3



#define NOAH_POWERBOARD_CAN_SRCMAC_ID   0x52

//////  function id define  //////
#define CAN_FUN_ID_RESET        0x06
#define CAN_FUN_ID_WRITE        0x01
#define CAN_FUN_ID_READ         0x02
#define CAN_FUN_ID_TRIGGER      0x03


//////  source id define  //////
#define CAN_SOURCE_ID_READ_VERSION      0x01

#define CAN_SOURCE_ID_SET_MODULE_STATE              0x81
#define CAN_SOURCE_ID_GET_MODULE_STATE              0x82
#define CAN_SOURCE_ID_GET_SYS_STATE                 0x83
#define CAN_SOURCE_ID_GET_ERR_STATE                 0x84
#define CAN_SOURCE_ID_GET_BAT_STATE                 0x85
#define CAN_SOURCE_ID_GET_ADC_DATA                  0x86
#define CAN_SOURCE_ID_SET_IR_LED_LIGHTNESS          0x87
#define CAN_SOURCE_ID_GET_IR_LED_LIGHTNESS          0x88
#define CAN_SOURCE_ID_SET_LED_EFFECT                0x89
#define CAN_SOURCE_ID_POWER_OFF_SIGNAL              0x8a
#define CAN_SOURCE_ID_REMOTE_POWRER_CTRL            0x8b
#define CAN_SOURCE_ID_GET_SERIALS_LEDS_VERSION      0x8c
#define CAN_SOURCE_ID_EVENT_BUTTON                  0x8d
#define CAN_SOURCE_ID_SET_LED_STATUS                0x8e
#define CAN_SOURCE_ID_GET_DEV_ID                    0x8f

#define CAN_SOURCE_ID_SET_CONVEYOR_BELT_WORK_MODE   0xa0

#define HW_VERSION_SIZE             3
#define SW_VERSION_SIZE             16
#define PROTOCOL_VERSION_SIZE       14



typedef enum
{
    POWER_5V_MOTOR            = 0x00000001,
    POWER_5V_RECHARGE         = 0x00000002,
    POWER_5V_SENSOR_BOARD     = 0x00000004,
    //POWER_5V_SWITCH           = 0x00000008,
    POWER_5V_ROUTER           = 0x00000010,
    POWER_5V_EN               = 0x00000020,

    POWER_12V_PAD             = 0x00000040,
    POWER_12V_2_1_PA          = 0x00000080,
    POWER_12V_EXTEND          = 0x00000100,
    POWER_12V_X86             = 0x00000200,
    POWER_12V_NV              = 0x00000400,
    POWER_12V_EN              = 0x00000800,

    POWER_24V_EN              = 0x00001000,
    POWER_24V_PRINTER         = 0x00002000,
    POWER_24V_EXTEND          = 0x00004000,
    POWER_VSYS_24V_NV         = 0x00008000,

    POWER_485                 = 0x00010000,
    POWER_SYS_LED             = 0x00020000,
    POWER_RECHARGE_LED        = 0x00040000,
    POWER_SLAM                = 0x00080000,

    POWER_LED_MCU             = 0x00100000,
    POWER_CHARGE_FAN          = 0x00200000,
    //POWER_POLE_MOTOR          = 0x00400000,
    //POWER_5V_KEYPAD           = 0x00800000,

    POWER_CAMERA_FRONT_LED    = 0x01000000,
    POWER_CAMERA_BACK_LED     = 0x02000000,
    POWER_CTRL_OUT            = 0x04000000,
    POWER_DOOR_CTRL           = 0x08000000,

    POWER_3V3_CARD_EN_1       = 0x10000000,
    POWER_3V3_CARD_EN_2       = 0x20000000,
    POWER_3V3_CARD_EN_3       = 0x40000000,
    POWER_3V3_CARD_EN_4       = 0x80000000,


    POWER_ALL                 = 0xFFFFFFFF,
} module_ctrl_e;



/*********** power control group 2 *************/
#define POWER_LED_1                 0x00000001
#define POWER_LED_2                 0x00000002
#define POWER_LED_3                 0x00000004
#define POWER_LED_4                 0x00000008
#define POWER_LED_5                 0x00000010
#define POWER_LED_6                 0x00000020
#define POWER_LED_7                 0x00000040
#define POWER_LED_8                 0x00000080

#define POWER_MOTOR_MCU             0x00000100
#define POWER_HEAD_CAMERA_LED       0x00000200



typedef struct
{
    uint8_t serial_num;
    CAN_ID_UNION id;
}can_upload_ack_t;

#pragma pack(1)
typedef struct _VoltageData_t
{
    uint16_t              _5V_reserve1_currents;
    //uint16_t              _24V_nv_currents;
    //uint16_t              _12V_nv_currents;
    uint16_t              _48V_extend_currents;

    uint16_t              _12V_extend_currents;
    uint16_t              motor_currents;
    uint16_t              slam_currents;
    uint16_t              _2_1_pa_currents;

    uint16_t              pad_currents;
    uint16_t              printer_currents;
    uint16_t              x86_currents;
    uint16_t              ir_led_currents;

    uint16_t              _5V_leds_currents;

    uint16_t              recharge_currents;
    uint16_t              _24V_extend_currents;
    uint16_t              charge_currents;
    uint16_t              batin_currents;

    uint16_t              vbus_currents;
    uint16_t              bat_motor_currents;
    //  uint16_t              multi_channel_adc;

    uint16_t              _24V_temp;
    uint16_t              _12V_temp;
    uint16_t              _5V_temp;
    uint16_t              air_temp;

    uint16_t              _24V_all_currents;
    uint16_t              _12V_all_currents;
    uint16_t              _5V_all_currents;
    uint16_t              _24V_voltage;

    uint16_t              _12V_voltage;
    uint16_t               _5V_voltage;
    uint16_t               bat_voltage;
    uint16_t               sensor_board_currents;

    int16_t               _12V_router_currents;

    uint16_t              _24V_nv_currents;
    uint16_t              _12V_nv_currents;
    uint16_t              keypad_currents;
} voltage_data_t;
#pragma pack()


typedef struct
{
#define MODULE_CTRL_ON      1
#define MODULE_CTRL_OFF     0
    uint8_t     on_off;
    uint8_t     group_num;
#define HW_NO_SUPPORT         0xFFFFFFFF
    volatile uint32_t    module;
} module_ctrl_t;

typedef struct
{
#define MODULE_CTRL_ON      1
#define MODULE_CTRL_OFF     0
    uint8_t     on_off;
    uint8_t     group_num;
#define HW_NO_SUPPORT         0xFFFFFFFF
    volatile uint32_t    module;
    volatile uint32_t    module_status_ack;
} module_ctrl_ack_t;



typedef struct
{
    uint8_t reserve;
}get_bat_info_t;

typedef struct
{
    uint8_t reserve;
    uint8_t bat_percent;
    uint16_t bat_vol;
    uint16_t pack_current;
    uint32_t pack_current_soc;
    uint32_t pack_totoal_soc;
    uint16_t pack_recharge_cycle;
    uint8_t com_status;
}get_bat_info_ack_t;


typedef struct
{
    uint8_t reserve;
}get_sys_status_t;

typedef struct
{
    uint8_t reserve;
    uint16_t sys_status;
}get_sys_status_ack_t;

typedef struct
{
    uint8_t reserve;
    uint8_t duty;
}set_ir_duty_t;

typedef struct
{
    uint8_t reserve;
    uint8_t duty;
}set_ir_duty_ack_t;

typedef struct
{
    uint8_t get_version_type;
}get_version_t;

typedef struct
{
    uint8_t frq;
}get_adc_t;

typedef voltage_data_t get_adc_ack_t;

typedef struct
{
    uint8_t get_version_type;
    std::string    hw_version;
    std::string    sw_version;
    std::string    protocol_version;
}get_version_ack_t;


#pragma pack(1)
typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
}color_t;

typedef struct
{
    uint8_t               reserve;
    uint8_t               mode;
    color_t               color;
    uint8_t               period;
}set_leds_effect_t;

typedef set_leds_effect_t set_leds_effect_ack_t;
#pragma pack()



//
#pragma pack(1)
typedef struct
{
    uint8_t remote_power_ctrl;
    uint8_t status;
}remote_power_ctrl_t;

typedef struct
{
    uint8_t reserve;
    std::string version;
}get_serials_leds_version_t;

#pragma pack()
//
#pragma pack(1)
typedef struct
{
    //  uint8_t               ctype;
    uint8_t               cur_light_mode;
    color_t               color;
    uint8_t               period;
} rcv_serial_leds_frame_t;

typedef struct
{
    uint8_t               ctype;
    uint8_t               cur_light_mode;
    color_t               color;
    uint8_t               period;
} ack_serial_leds_frame_t;
#pragma pack()


#pragma pack(1)

typedef struct
{
    //    uint8_t               ctype;
    //    uint8_t               cmdType;
    voltage_data_t         voltage_data;
    uint8_t               fault_bit[4];
    uint8_t               send_rate;
    uint8_t               reserve[7];
} voltage_info_t;
#pragma pack()


typedef struct _recModuleControlFrame_t
{
    uint8_t               module;
#define             SYSTEM_MODULE         0x00
#define             MOTOR_MODULE          0x01
#define             SENSOR_MODULE         0x02
#define             LEDS_MODULE           0x03
#define             _5VRESERVE_MODULE     0x04
#define             PAD_MODULE            0x05
#define             _12V_ROUTER_MODULE    0x06
#define             _2_1_PA_MODULE        0x07
#define             DYP_MODULE            0x08
#define             X86_MODULE            0x09
#define             NV_MODULE             0x0A
#define             DLP_MODULE            0x0B
#define             _12V_RESERVE_MODULE   0x0C
#define             PRINTER_MODULE        0x0D
#define             _24V_RESERVE_MODULE   0x0E
#define             BAT_NV_MODULE         0x0F
#define             _5V_ALL_MODULE        0x10
#define             _12V_ALL_MODULE       0x11
#define             _24V_ALL_MODULE       0x12
#define             AIUI_MODULE           0x13
#define             _5V_ROUTER_MODULE     0x14
    uint8_t               control;
} rcv_module_control_frame_t;

typedef struct
{
    uint8_t      mode;
    color_t      color;
    uint8_t      period;
}led_t;

typedef struct
{
    uint8_t cmd;
    uint16_t bat_info;
    uint16_t bat_vol;
    uint16_t bat_percent;
}bat_info_t;

typedef struct _recTestCurrentCmdFrame_t
{
    uint8_t               cmd;
    uint8_t               sendRate;
#define                         SEND_RATE_SINGLE        ((uint8_t)0x00)
#define                         SEND_RATE_1HZ           ((uint8_t)0x01)
#define                         SEND_RATE_2HZ           ((uint8_t)0x02)
#define                         SEND_RATE_5HZ           ((uint8_t)0x03)
#define                         SEND_RATE_10HZ          ((uint8_t)0x04)
#define                         SEND_RATE_50HZ          ((uint8_t)0x05)
#define                         SEND_RATE_100HZ         ((uint8_t)0x06)
#define                         SEND_RATE_0_5HZ         ((uint8_t)0x07)
#define                         SEND_RATE_0_2HZ         ((uint8_t)0x08)
#define                         SEND_RATE_0_1HZ         ((uint8_t)0x09)
} current_cmd_frame_t;

typedef struct
{
#define IR_CMD_READ     0
#define IR_CMD_WRITE    1
    uint8_t cmd;
    int set_ir_percent;
    //uint8_t lightness_percent;
}ir_cmd_t;

typedef struct
{
#define CONVEYOR_BELT_STATUS_STOP       0
#define CONVEYOR_BELT_STATUS_LOAD       1
#define CONVEYOR_BELT_STATUS_UNLOAD     2
    uint8_t set_work_mode;
    uint8_t set_result;
    uint8_t ack_work_mode;
#define CONVEYOR_BELT_LOAD_ERROR        0xF0
#define CONVEYOR_BELT_UNLOAD_ERROR      0xF1
#define CONVEYOR_BELT_STATUS_ERROR      0xFF
    uint8_t err_status;
}conveyor_belt_t;





#define STR_LED_WIFI        "wifi"
#define STR_LED_TRANS       "trans"
#define STR_LED_BATTERY     "battery"

typedef enum
{
    LED_MIN = 0,
    LED_WIFI = 1,
    LED_TRANS,
    LED_BATTERY,
    LED_MAX,
}status_led_e;


#define STR_LED_STATUS_OFF      "off"
#define STR_LED_STATUS_OK       "ok"
#define STR_LED_STATUS_ERR      "err"
#define STR_LED_STATUS_WARN     "warn"

typedef enum
{
    LED_STATUS_MIN = 0,
    LED_STATUS_OFF = 1,
    LED_STATUS_OK ,
    LED_STATUS_ERR,
    LED_STATUS_WARN,
    LED_STATUS_MAX,
}led_ctrl_status_e;


#define STR_LED_STATUS_ACK_OK           "ok"
#define STR_LED_STATUS_ACK_TIMEOUT      "timeout"
#define STR_LED_STATUS_ACK_PARAM_ERR    "param_err"

typedef enum
{
    LED_STATUS_ACK_OK = 1,
    LED_STATUS_ACK_TIMEOUT,
    LED_STATUS_ACK_PARAM_ERR,
}led_status_ack_e;

typedef struct
{
    uint8_t led;
    uint8_t status;
}status_led_t;


typedef struct
{
    uint8_t reserve;
    uint16_t dev_id;
}dev_id_t;

#define VBAT_POWER_OFF_PERCENTAGE           10  // %
#define VBAT_POWER_LOW_WARNING_PERCENTAGE   20  // %

#define VBAT_POWER_CHARGING_LOW             30
#define VBAT_POWER_CHARGING_MEDIUM          40
#define VBAT_POWER_CHARGING_FULL            100
typedef struct
{
    led_t                       led;
    set_leds_effect_t           led_set;
    rcv_serial_leds_frame_t     rcv_serial_leds_frame;
    bat_info_t                  bat_info;

    current_cmd_frame_t         current_cmd_frame;
    voltage_info_t               voltage_info;

#define VERSION_TYPE_FW             0
#define VERSION_TYPE_PROTOCOL       1
    uint8_t                     get_version_type;

    std::string                 hw_version;
    std::string                 sw_version;
    std::string                 protocol_version;
    std::string                 serials_leds_mcu_version;

#define SYS_STATUS_OFF              0
#define SYS_STATUS_TURNING_ON       1
#define SYS_STATUS_ON               2
#define SYS_STATUS_TURNING_OFF      3
#define SYS_STATUS_ERR              4
#define                 STATE_IS_CHARGING       0x10
#define                 STATE_IS_LOW_POWER      0x20
#define                 STATE_IS_AUTO_UPLOAD    0x40
#define                 STATE_IS_CHARGER_IN     0x80
#define                 SYSTEM_IS_SLEEP         0x00 //set 0x00 to no use

#define                 STATE_IS_RECHARGE_IN    0x0100

    uint16_t                     sys_status;

    ir_cmd_t                    ir_cmd;
    module_ctrl_t               module_status_set;
    module_ctrl_t               module_status;
    remote_power_ctrl_t         remote_power_ctrl_set;
    conveyor_belt_t             conveyor_belt;
    status_led_t                status_led_set;
    dev_id_t                    dev_id;
    dev_id_t                    dev_id_ack;

#define SEND_DATA_BUF_LEN           255
    uint8_t                     send_data_buf[SEND_DATA_BUF_LEN];
}powerboard_t;

typedef enum
{
    LIGHTS_MODE_NONE                    = 0,
    LIGHTS_MODE_NOMAL                   = 1,
    LIGHTS_MODE_ERROR                   = 2,
    LIGHTS_MODE_LOW_POWER,
    LIGHTS_MODE_CHARGING_POWER_MEDIUM,
    LIGHTS_MODE_CHARGING_POWER_LOW,
    LIGHTS_MODE_CHARGING_FULL,
    LIGHTS_MODE_TURN_LEFT,
    LIGHTS_MODE_TURN_RIGHT,
    LIGHTS_MODE_COM_ERROR,
    LIGHTS_MODE_EMERGENCY_STOP,


    LIGHTS_MODE_SETTING                 = 0xff,
}light_mode_t;

extern powerboard_t    *sys_powerboard;

class NoahPowerboard
{
    public:
        NoahPowerboard(bool log_on = false)
        {
            is_log_on = log_on;
            noah_powerboard_pub = n.advertise<std_msgs::String>("tx_noah_powerboard_node",1000);
            pub_charge_status_to_move_base = n.advertise<std_msgs::UInt8MultiArray>("charge_status_to_move_base",1000);
            resp_navigation_camera_leds = n.advertise<std_msgs::String>("resp_lane_follower_node/camera_using_n",1000);
            power_pub_to_app = n.advertise<std_msgs::UInt8MultiArray>("app_sub_power",1);
            noah_powerboard_sub = n.subscribe("rx_noah_powerboard_node",1000,&NoahPowerboard::from_app_rcv_callback,this);
            sub_navigation_camera_leds = n.subscribe("lane_follower_node/camera_using_n",1000,&NoahPowerboard::from_navigation_rcv_callback,this);

            pub_to_can_node = n.advertise<mrobot_msgs::vci_can>("noah_powerboard_to_can", 1000);
            pub_event_key = n.advertise<std_msgs::String>("post_event_key", 10);


            device_shutdown_signal_pub = n.advertise<std_msgs::UInt8MultiArray>("device_shutdown_signal", 1000);
            sub_from_can_node = n.subscribe("can_to_noah_powerboard", 1000, &NoahPowerboard::rcv_from_can_node_callback, this);
            sub_from_basestate = n.subscribe("basestate", 10, &NoahPowerboard::basestate_callback, this);
            sub_from_serials_leds_turnning_ctrl = n.subscribe("serials_leds_turnning_ctrl", 10, &NoahPowerboard::serials_leds_turning_effect_callback, this);
            sub_from_remote_power_ctrl = n.subscribe("remote_power_ctrl", 100, &NoahPowerboard::remote_power_ctrl_callback, this);
            remote_power_ctrl_service = n.advertiseService("remote_power_ctrl",&NoahPowerboard::service_remote_power_ctrl,this);
            led_ctrl_sub = n.subscribe("led_ctrl", 10, &NoahPowerboard::leds_ctrl_callback,this);

            status_led_ctrl_ack_pub = n.advertise<std_msgs::String>("led_ctrl_ack", 10);
            battery_test_pub = n.advertise<std_msgs::String>("test_battery_info", 10);

            sys_powerboard = &sys_powerboard_ram;
            sys_powerboard->sys_status = 0;
            sys_powerboard->bat_info.bat_percent = 0;
            sys_powerboard->bat_info.bat_vol = 0;

            emg_stop = false;
            turnning_direction = 0;

            remote_power_ctrl_vector.clear();
            module_set_vector.clear();
            get_bat_info_vector.clear();
            get_sys_status_vector.clear();
            set_ir_duty_vector.clear();
            get_version_vector.clear();
            get_adc_vector.clear();
            set_leds_effect_vector.clear();
            get_serials_leds_version_vector.clear();
        }
        int PowerboardParamInit(void);
        int SetLedEffect(powerboard_t *powerboard);
        int GetBatteryInfo(powerboard_t *sys);
        int GetAdcData(powerboard_t *sys);
        int GetVersion(powerboard_t *sys);
        int GetSysStatus(powerboard_t *sys);
        int InfraredLedCtrl(powerboard_t *sys);
        int SetModulePowerOnOff(powerboard_t *sys);
        int GetModulePowerOnOff(powerboard_t *sys);
        int RemotePowerCtrl(powerboard_t *sys);
        int get_serials_leds_version(powerboard_t *sys);
        int set_conveyor_belt_work_mode(powerboard_t *sys);
        int set_status_led(powerboard_t *sys);
        int get_dev_id(powerboard_t *sys);
        int ack_mcu_upload(CAN_ID_UNION id, uint8_t serial_num);

        int send_serial_data(powerboard_t *sys);
        int handle_receive_data(powerboard_t *sys);
        void from_app_rcv_callback(const std_msgs::String::ConstPtr &msg);
        void from_navigation_rcv_callback(const std_msgs::String::ConstPtr &msg);
        void power_from_app_rcv_callback(std_msgs::UInt8MultiArray data);
        void PubPower(powerboard_t *sys);
        void PubChargeStatus(uint8_t status);
        void pub_event_key_value(uint8_t value);

        void rcv_from_can_node_callback(const mrobot_msgs::vci_can::ConstPtr &c_msg);
        void basestate_callback(std_msgs::UInt8MultiArray data);
        void serials_leds_turning_effect_callback(std_msgs::UInt8MultiArray data);
        void remote_power_ctrl_callback(std_msgs::UInt8MultiArray data);
        void leds_ctrl_callback(const std_msgs::String::ConstPtr &msg);
        void ack_status_led_ctrl(status_led_t led, uint8_t err_code);

        void get_ir_duty_param(void);


        json j;
        json j_event_key;
        void pub_json_msg_to_app(const nlohmann::json j_msg);
        powerboard_t    *sys_powerboard;
        can_long_frame  long_frame;

        void update_sys_status(void);


        vector<module_ctrl_t>           module_set_vector;
        vector<module_ctrl_ack_t>       module_set_ack_vector;

        vector<get_bat_info_t>          get_bat_info_vector;
        vector<get_bat_info_ack_t>      get_bat_info_ack_vector;

        vector<get_sys_status_t>        get_sys_status_vector;
        vector<get_sys_status_ack_t>    get_sys_status_ack_vector;

        vector<set_ir_duty_t>           set_ir_duty_vector;
        vector<set_ir_duty_ack_t>       set_ir_duty_ack_vector;

        vector<get_version_t>           get_version_vector;
        vector<get_version_ack_t>       get_version_ack_vector;

        vector<get_adc_t>               get_adc_vector;
        vector<get_adc_ack_t>           get_adc_ack_vector;

        vector<set_leds_effect_t>       set_leds_effect_vector;
        vector<set_leds_effect_t>       set_leds_effect_ack_vector;

        vector<remote_power_ctrl_t>     remote_power_ctrl_vector;
        vector<remote_power_ctrl_t>     remote_power_ctrl_ack_vector;

        vector<get_serials_leds_version_t> get_serials_leds_version_vector;
        vector<get_serials_leds_version_t> get_serials_leds_version_ack_vector;

        vector<conveyor_belt_t>         set_conveyor_belt_work_mode_vector;
        vector<conveyor_belt_t>         set_conveyor_belt_work_mode_ack_vector;

        vector<status_led_t>            set_led_status_vector;
        vector<status_led_t>            set_led_status_ack_vector;

        vector<dev_id_t>                get_dev_id_vector;
        vector<dev_id_t>                get_dev_id_ack_vector;

        boost::mutex mtx;
        bool is_log_on;

    private:
        uint8_t CalCheckSum(uint8_t *data, uint8_t len);
        int handle_rev_frame(powerboard_t *sys,unsigned char * frame_buf);
        bool service_remote_power_ctrl(noah_powerboard::remote_power_ctrl_srv::Request  &ctrl, noah_powerboard::remote_power_ctrl_srv::Response &status);
        void pub_battery_info(get_bat_info_ack_t *bat_info);

        ros::NodeHandle n;
        ros::Publisher noah_powerboard_pub;
        ros::Subscriber noah_powerboard_sub;
        ros::Subscriber sub_navigation_camera_leds;
        ros::Publisher resp_navigation_camera_leds;
        ros::Publisher power_pub_to_app;
        ros::Publisher pub_charge_status_to_move_base;
        ros::Publisher pub_to_can_node;//publish to roscan node
        ros::Subscriber sub_from_can_node;
        ros::Subscriber sub_from_basestate;
        ros::Subscriber sub_from_serials_leds_turnning_ctrl;
        ros::Publisher device_shutdown_signal_pub;
        ros::Subscriber sub_from_remote_power_ctrl;
        ros::Subscriber led_ctrl_sub;
        ros::Publisher status_led_ctrl_ack_pub;
        ros::Publisher battery_test_pub;


        ros::Publisher pub_event_key;

        ros::ServiceServer remote_power_ctrl_service;

        powerboard_t    sys_powerboard_ram;
        uint8_t emg_stop;
        uint8_t turnning_direction;

        std::string software_version_param = "mcu_noah_powerboard_version";
        std::string hardware_version_param = "noah_powerboard_hardware_version";
        std::string protocol_version_param = "noah_powerboard_protocol_version";
        std::string serials_leds_mcu_version_param = "mcu_serials_leds_version";

};
int handle_receive_data(powerboard_t *sys);

void *CanProtocolProcess(void* arg);


void test_fun(void * arg);

typedef struct
{
    uint8_t reserve;
    uint8_t group_num;
    uint8_t module;
    uint8_t on_off;
}module_set_t;
#endif



