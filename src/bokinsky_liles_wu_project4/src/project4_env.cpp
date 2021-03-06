/**************************************************
 * Name:         Huston Bokinsky
 *               Karina Liles
 *               Xian Wu
 * File:         project4_env.cpp
 * Assignment:   Project 4 -- create autodrive
 * Date:         10 April, 2014
 * Course:       CSCE 574 - Robotics
 * Instructor:   Dr. O'Kane
 * Description:  Have iRobot interact with its environment 
 *               autonomously by driving safely down middle of a 
 *               passage. 
 * 
 **************************************************/

#include "project4.h"
Project4Env::Project4Env(ros::NodeHandle* nh)
{
    sub = nh->subscribe("scan", 100, &Project4Env::sub_callback, this);
    pub = nh->advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 100);
}

/*
 * Publish next Twist messge to /cmd_vel_mux based on largest,
 * open-est space found in laserscan
 */
void
Project4Env::sub_callback(const sensor_msgs::LaserScan& msg)
{
    //ROS_INFO_STREAM("in sub_callback, received laserscan " << msg.header.seq);
    struct create_autodrive::create_autodrive_space s = 
        create_autodrive::find_drive_space(msg);
    geometry_msgs::Twist pub_msg;
    pub_msg.linear.x  = create_autodrive::select_linear_vel(s);
    pub_msg.angular.z = create_autodrive::select_angular_vel(s);
    pub.publish(pub_msg);
}