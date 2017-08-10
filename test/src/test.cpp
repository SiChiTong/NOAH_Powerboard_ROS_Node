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
        ROS_INFO("find sub_name");
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

}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"test_node");
    ros::NodeHandle n;
    ros::Publisher test_pub = n.advertise<std_msgs::String>("rx_noah_powerboard_node",1000);
    ros::Subscriber test_sub = n.subscribe("tx_noah_powerboard_node", 1000, &callback);
    ros::Rate loop_rate(1);
    json j;
    while(ros::ok())
    {
        {
            j.clear();
            j = 
            {
                {"pub_name","set_module_state"},
                {
                    "data",
                    {
                        {"dev_name","_24v_printer"},
                        {"set_state",true},
                    }
                },
            };
            std_msgs::String pub_json_msg;
            std::stringstream ss;

            ss.clear();
            ss << j;
            pub_json_msg.data = ss.str();
            test_pub.publish(pub_json_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
