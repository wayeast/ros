/**************************************************
 * Name:         Huston Bokinsky
 *               Karina Liles
 *               Xian Wu
 * File:         wifi_mon.cpp
 * Assignment:   Project 4 -- create autodrive
 * Date:         10 April, 2014
 * Course:       CSCE 574 - Robotics
 * Instructor:   Dr. O'Kane
 * Description:  Have iRobot interact with its environment 
 *               autonomously by driving safely down middle of a 
 *               passage. 
 * 
 **************************************************/

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
    ros::Publisher wifi_mon = nh.advertise<std_msgs::Int32>(
            "wifi_ss", 1);
    std_msgs::Int32 msg;
    //ros::Rate rate(2);

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
        wifi_mon.publish(msg);

        //rate.sleep();
    }
}