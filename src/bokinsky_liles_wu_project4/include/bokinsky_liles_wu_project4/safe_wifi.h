/**************************************************
 * Name:         Huston Bokinsky
 *               Karina Liles
 *               Xian Wu
 * File:         safe_wifi.h
 * Assignment:   Project 4 -- create autodrive
 * Date:         10 April, 2014
 * Course:       CSCE 574 - Robotics
 * Instructor:   Dr. O'Kane
 * Description:  Have iRobot interact with its environment 
 *               autonomously by driving safely down middle of a 
 *               passage. 
 * 
 **************************************************/

#ifndef SAFE_WIFI_H
#define SAFE_WIFI_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <create_node/TurtlebotSensorState.h>

namespace safe_wifi
{
    const float max_wifi_signal = 70.0;

    ros::Publisher safety_mon;
    geometry_msgs::Twist safety_brake;

}

#endif