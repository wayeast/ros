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

#include "turtleherder.h"

#define XY_TOL   0.005
#define Z_TOL    0.001

/*
 * Constructor initializes member data and instantiates ros::Publisher
 * from a ros::NodeHandle.
 */
TurtleHerder::TurtleHerder(std::string filename, ros::NodeHandle& nh) 
{
    publisher = nh.advertise<geometry_msgs::Twist>(
            "turtle1/cmd_vel", 1000);

    instructions = __init_instructions(filename);
    targX = instructions.front().targX;
    targY = instructions.front().targY;
    targZ = instructions.front().targZ;
    pub_msg.linear.x  = instructions.front().velX;
    pub_msg.angular.z = instructions.front().velZ;
}


std::list<struct instruction>
TurtleHerder::__init_instructions(std::string filename)
{
    std::list<struct instruction> L;

    std::ifstream infile(filename.c_str());
    std::string line;
    while (std::getline(infile, line)) {
        if (line.empty()) continue;
        if (line[0] == '#') continue;

        std::stringstream pieces(line);
        struct instruction next;
        pieces >> next.targX
            >> next.targY
            >> next.targZ
            >> next.velX
            >> next.velZ ;
        L.push_back(next);
    }
    return L;
}

/*
 * Callback function called by ros::Subscriber.
 */
void
TurtleHerder::checkPosition(const turtlesim::Pose& msg)
{
    // 'Twould be wonderful if I could get exact coordinates
    // of turtle's start position, rather than fudge it...
    if ( (msg.x >= targX - XY_TOL && msg.x <= targX + XY_TOL) &&
         (msg.y >= targY - XY_TOL && msg.y <= targY + XY_TOL) &&
         (msg.theta >= targZ - Z_TOL && msg.theta <= targZ + Z_TOL)
       ) 
    {
        ROS_INFO_STREAM("Arrived at coordinates: "
                << msg.x << " : " << msg.y << " : " << msg.theta);
        advanceInstruction();
        publisher.publish(pub_msg);
        ROS_INFO_STREAM("Publishing cmd_vel message: "
                << pub_msg.linear.x << " : "
                << pub_msg.angular.z);
    }
    else
        publisher.publish(pub_msg);
}

void
TurtleHerder::advanceInstruction(void)
{
    instructions.pop_front();
    targX = instructions.front().targX;
    targY = instructions.front().targY;
    targZ = instructions.front().targZ;
    pub_msg.linear.x  = instructions.front().velX;
    pub_msg.angular.z = instructions.front().velZ;

    ROS_INFO_STREAM("Advancing targets to: " << targX
            << " : " << targY << " : " << targZ);
}

