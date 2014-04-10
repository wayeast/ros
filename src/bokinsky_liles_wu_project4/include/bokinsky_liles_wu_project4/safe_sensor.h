/**************************************************
 * Name:         Huston Bokinsky
 *               Karina Liles
 *               Xian Wu
 * File:         safe_sensor.h
 * Assignment:   Project 4 -- create autodrive
 * Date:         10 April, 2014
 * Course:       CSCE 574 - Robotics
 * Instructor:   Dr. O'Kane
 * Description:  Have iRobot interact with its environment 
 *               autonomously by driving safely down middle of a 
 *               passage. 
 * 
 **************************************************/

#ifndef SAFE_SENSOR_H
#define SAFE_SENSOR_H

#include <stdint.h>
#include <iomanip>

#include <ros/ros.h>
#include <create_node/TurtlebotSensorState.h>
#include <geometry_msgs/Twist.h>
#include <diagnostic_msgs/DiagnosticArray.h>

namespace safe_sensor
{
    ros::Publisher safety_mon;
    geometry_msgs::Twist safety_brake;
}

#endif