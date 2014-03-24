// std lib includes
#include <stdlib.h>
#include <stdio.h>

// ROS includes
#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main(int argc, char **argv)
{
    // Basic ROS environment setup
    ros::init(argc, argv, "publish_wifi_strength");
    ros::NodeHandle nh;
    ros::Publisher wifi_reporter = nh.advertise<std_msgs::Int32>(
            "wifi_ss", 100);
    std_msgs::Int32 msg;
    ros::Rate rate(2);

    FILE *in;
    char buff[256];
    while(ros::ok())
    {
        // Fetch wireless signal strength from system
        if (!(in = popen("iwconfig wlan0|grep Quality|cut -d'=' -f2|cut -d'/' -f1", "r")))
            return 1;
        std::fgets(buff, sizeof(buff), in);
        pclose(in);

        // Publish wifi signal strength to /wifi_ss topic
        msg.data = std::atoi(buff);
//        ROS_INFO_STREAM("Sending value " << msg.data << " to wifi_ss");
        wifi_reporter.publish(msg);

        rate.sleep();
    }
}
