/*
 * Name:         Huston Bokinsky
 * Date:         1 February, 2014
 * Course:       CSCE 574 - Robotics
 * Instructor:   Dr. O'Kane
 * Assignment:   Project 1 - Block "C"
 * Description:  Use ros::Publisher and ros::Subscriber nodes to make
 *               a turtlesim turtle describe a block C route.
 *
 * Disclaimer:   This code is for a class project and should not be
 *               used in production environments where people's lives
 *               are dependent upon its proper functioning.
 */

#ifndef _TURTLEHERDER_H_
#define _TURTLEHERDER_H_

#include <list>
#include <fstream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>


/*
 * struct instruction represents target coordinates (targX, 
 * targY, targZ) and the cmd_vel message values to publish
 * when those target coordinates are reached.
 */
struct instruction
{
    double targX;
    double targY;
    double targZ;
    double velX;
    double velZ;
};


/*
 * Class TurtleHerder contains data and methods to control movements
 * of a turtlesim turtle.
 *  @ targX, targY, targZ : anticipated coordinates of turtle
 *  @ pub_msg: cmd_vel values to publish as long as targ* have
 *             not been reached
 *  @ instructions: instructions read in from file specified at
 *                  program invocation
 *  @ publisher: ros::Publisher instance to publish to cmd_vel topic
 *  @ __init_instructions: private method to build instruction list
 *                         from data file
 *  @ advanceInstruction: private method to advance instructions and
 *                        update member data
 *  @ checkPosition: public callback method to be called on messages
 *                   published to /turtle1/pose topic
 */
class TurtleHerder
{
    double targX, targY, targZ;
    std::list<struct instruction> instructions;
    ros::Publisher publisher;
    geometry_msgs::Twist pub_msg;

    std::list<struct instruction> __init_instructions(std::string);
    void advanceInstruction();

    public:
    TurtleHerder(std::string, ros::NodeHandle&);
    void checkPosition(const turtlesim::Pose&);
};


#endif

