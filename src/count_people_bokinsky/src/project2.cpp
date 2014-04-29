/**************************************************
 * Name:         Huston Bokinsky
 * File:         project2.cpp
 * Assignment:   count_people_bokinsky
 * Date:         23 February, 2014
 * Course:       CSCE 574 - Robotics
 * Instructor:   Dr. O'Kane
 * Description:  
 * 
 **************************************************/

#include "project2.h"
Project2Env::Project2Env(ros::NodeHandle *nh)
{
    /* Publisher nodes */
    entity_pub = nh->advertise<sensor_msgs::PointCloud>(
                "entity_tracker", 1000);
    person_pub = nh->advertise<sensor_msgs::PointCloud>(
                "person_locations", 1000);
    newcomer_pub = nh->advertise<geometry_msgs::Point32>(
                "person_appearances", 1000);

    /* Initialize project2::person_tracker */
    person_tracker = PersonTracker(&person_pub, &newcomer_pub);

    /* Subscriber nodes */
    scan_sub = nh->subscribe("scan", 1000,
            &Project2Env::findDistinctEntities, this);
    cloud_sub = nh->subscribe("entity_tracker", 1000,
            &Project2Env::findPersons, this);
}

/*
 * Callback function called by scan_sub.
 */
void
Project2Env::findDistinctEntities(const sensor_msgs::LaserScan& scan)
{
    float theta;
    int i;
    EntityTracker et (scan);

    for (theta=scan.angle_min, i=0; 
            theta<=scan.angle_max;
            theta+=scan.angle_increment, i++)
    {
        et.update(theta, scan.ranges[i]);
    }
    et.close(&entity_pub);
    /* For testing -- change before final submission.
       if (et.close(entity_pub))
       ROS_INFO_STREAM(scan);
       */
}

/*
 * Callback function called by cloud_sub.
 */
void
Project2Env::findPersons(const sensor_msgs::PointCloud& pcloud)
{
    person_tracker.update(sensor_msgs::PointCloud(pcloud));
}