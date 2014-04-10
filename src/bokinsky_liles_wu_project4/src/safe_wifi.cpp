/**************************************************
 * Name:         Huston Bokinsky
 *               Karina Liles
 *               Xian Wu
 * File:         report_batt_wifi.cpp
 * Assignment:   Project 3 - Turtlebot Setup
 * Date:         25 March, 2014
 * Course:       CSCE 574 - Robotics
 * Instructor:   Dr. O'Kane
 * Description:  Set up basic connections and functionality for 
 *               our turtlebot robots. Create basic launch file; 
 *               get wifi signal strength from robot's netbook; 
 *               monitor and print robot's diagnostic messages, 
 *               battery charge, and wifi strength; control robot 
 *               via network connection from lab workstation. 
 * 
 **************************************************/

// ROS includes

#include "safe_wifi.h"


void callback(const std_msgs::Int32& msg)
{
    if ((float)msg.data / safe_wifi::max_wifi_signal * 100 < 23.0)
    {
        safe_wifi::safety_mon.publish(safe_wifi::safety_brake);
        ROS_INFO_STREAM("Wifi signal strength weak -- stopping robot");
    }
}


int main(int argc, char **argv)
{
    // Basic ROS environment + subscribers setup
    ros::init(argc, argv, "report_battery_and_wifi");
    ros::NodeHandle nh;
    ros::Subscriber wifi_rpt = nh.subscribe("/wifi_ss",
        1, &callback);

    safe_wifi::safety_mon = nh.advertise<geometry_msgs::Twist>(
            "cmd_vel_mux/input/safety_controller", 100);
    safe_wifi::safety_brake = geometry_msgs::Twist();
    safe_wifi::safety_brake.linear.x = 0.0;
    safe_wifi::safety_brake.angular.z = 0.0;

    ros::spin();
}
