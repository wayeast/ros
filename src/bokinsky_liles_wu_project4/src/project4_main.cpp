/**************************************************
 * Name:         Huston Bokinsky
 *               Karina Liles
 *               Xian Wu
 * File:         project4_main.cpp
 * Assignment:   Project 4 -- create autodrive
 * Date:         10 April, 2014
 * Course:       CSCE 574 - Robotics
 * Instructor:   Dr. O'Kane
 * Description:  Have iRobot interact with its environment 
 *               autonomously by driving safely down middle of a 
 *               passage. 
 * 
 **************************************************/

#include <ros/ros.h>
#include "project4.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sunday_drive");
    ros::NodeHandle nh;

    Project4Env env(&nh);

    ros::spin();

}