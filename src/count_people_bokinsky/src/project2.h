/**************************************************
 * Name:         Huston Bokinsky
 * File:         project2.h
 * Assignment:   count_people_bokinsky
 * Date:         23 February, 2014
 * Course:       CSCE 574 - Robotics
 * Instructor:   Dr. O'Kane
 * Description:  
 * 
 **************************************************/

#ifndef PROJECT_2_H
#define PROJECT_2_H


#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include "trackers.h"



class Project2Env
{
    PersonTracker person_tracker;

    // Publishers
    ros::Publisher entity_pub;
    ros::Publisher person_pub;
    ros::Publisher newcomer_pub;

    // Subscribers
    ros::Subscriber scan_sub;
    ros::Subscriber cloud_sub;

    public:
      // Environment constructor and callback functions
      Project2Env(ros::NodeHandle *);
      void findDistinctEntities(const sensor_msgs::LaserScan &);
      void findPersons(const sensor_msgs::PointCloud &);

};

#endif