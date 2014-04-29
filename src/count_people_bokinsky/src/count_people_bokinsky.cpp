/**************************************************
 * Name:         Huston Bokinsky
 * File:         count_people_bokinsky.cpp
 * Assignment:   count_people_bokinsky
 * Date:         23 February, 2014
 * Course:       CSCE 574 - Robotics
 * Instructor:   Dr. O'Kane
 * Description:  
 * 
 **************************************************/

#include <ros/ros.h>
#include "project2.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "count_people");
    ros::NodeHandle nh;
 
    Project2Env p2(&nh);

    ros::spin();

}  // end main