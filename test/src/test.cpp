#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include "json.hpp"
using json = nlohmann::json;
void callback(const std_msgs::String::ConstPtr msg)
{
    //ROS_INFO("test_node_callback");
    auto j = json::parse(msg->data.c_str());
    if(j.find("sub_name") != j.end())
    {
        //ROS_INFO("find sub_name");
        if(j["sub_name"] == "get_version")
        {
            ROS_INFO("find get_version");
            //if(j["data"]["dev_name"] == "_24v_printer")
            if(j["data"].find("hw_version") != j["data"].end())
            {
                ROS_INFO("find hw_version");
            }
            {
                if(j["data"]["set_dev"] == true)
                {

                }
                else if(j["data"]["set_dev"] == false)
                {

                }

            }
        }
        if(j["sub_name"] == "get_adc_data")
        {
            if(j["data"].find("_12v_voltage") != j["data"].end()) 
            {
                //ROS_INFO("_12v_voltage:%d",j["data"]["_12v_voltage"]);
            }
        }
    }
    if(j.find("sub_name") != j.end())
    {
        if(j["sub_name"] == "set_module_state")
        {
            ROS_INFO("find get_module_state");

            if(j["data"]["_xx_xxx_state"] == true)
            {
                ROS_INFO("_xx_xxx_state is on");
            }
            if(j["data"]["_xx_xxx_state"] == false)
            {
                ROS_INFO("_xx_xxx_state is off");
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"test_node");
    ros::NodeHandle n;
    ros::Publisher test_pub = n.advertise<std_msgs::String>("rx_noah_powerboard_node",1000);
    ros::Subscriber test_sub = n.subscribe("tx_noah_powerboard_node", 1000, &callback);
    ros::Rate loop_rate(0.3);
    json j;
    static uint32_t cnt = 0;
    while(ros::ok())
    {
        {
            bool state;

            std_msgs::String pub_json_msg;
            std::stringstream ss;
            state = cnt % 2 == 1 ? true:false;
            cnt++;
#if 0
            j.clear();
            j = 
            {
                {"pub_name","set_module_state"},
                {
                    "data",
                    {
                        {"dev_name","_24v_printer"},
                        {"set_state",(bool)state},
                    }
                },
            };
            ss.clear();
            ss << j;
            pub_json_msg.data.clear();  
            pub_json_msg.data = ss.str();
            test_pub.publish(pub_json_msg);
#endif


#if 0
            j.clear();
            j = 
            {
                {"pub_name","set_module_state"},
                {
                    "data",
                    {
                        {"dev_name","_24v_dcdc"},
                        {"set_state",(bool)state},
                    }
                },
            };
            std_msgs::String pub_json_msg_2;
            std::stringstream ss_2;
            ss_2.clear();
            ss_2 << j;
            pub_json_msg_2.data.clear();  
            pub_json_msg_2.data = ss_2.str();
            test_pub.publish(pub_json_msg_2);
            usleep(500*1000);
#endif


#if 1
            j.clear();
            j = 
            {
                {"pub_name","set_module_state"},
                {
                    "data",
                    {
                        {"dev_name","_5v_dcdc"},
                        {"set_state",(bool)state},
                    }
                },
            };
            std_msgs::String pub_json_msg_4;
            std::stringstream ss_4;
            ss_4.clear();
            ss_4 << j;
            pub_json_msg_4.data.clear();  
            pub_json_msg_4.data = ss_4.str();
            test_pub.publish(pub_json_msg_4);
            usleep(500*1000);
#endif

#if 0
            j.clear();
            j = 
            {
                {"pub_name","set_module_state"},
                {
                    "data",
                    {
                        {"dev_name","_12v_dcdc"},
                        {"set_state",(bool)state},
                    }
                },
            };
            std_msgs::String pub_json_msg_3;
            std::stringstream ss_3;
            ss_3.clear();
            ss_3 << j;
            pub_json_msg_3.data.clear();  
            pub_json_msg_3.data = ss_3.str();
            test_pub.publish(pub_json_msg_3);
            usleep(500*1000);
#endif


        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
