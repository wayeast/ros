/**************************************************
 * Name:         Huston Bokinsky
 * File:         trackers.h
 * Assignment:   count_people_bokinsky
 * Date:         23 February, 2014
 * Course:       CSCE 574 - Robotics
 * Instructor:   Dr. O'Kane
 * Description:  
 * 
 **************************************************/

#ifndef ENTITY_TRACKER_H
#define ENTITY_TRACKER_H
#include <cmath>   // for sin, cos
#include <map>
#include <set>
#include <vector>

/* ROS includes */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>

/* Entity identification preprocessor directives */
#define DIFF_ENT_LIMIT 0.25
#define REAL_ENTITY_CONTIGUOUS_HIT_MIN 3
#define MAX_NOISY_NANS 3

/* Person identification preprocessor directives */
#define IDENTITY_THRESHOLD 0.1
#define MOVEMENT_THRESHOLD 0.02
#define TICK_COUNT 15


class EntityTracker
{
    // Auxiliary variables
    bool entity_present;
    int  theta_count, range_count;
    float theta, theta_accum, range_accum;
    float last_range;

    // PointCloud message for publishing to /person_locations
    sensor_msgs::PointCloud pc;

    // Auxiliary functions
    void resetVars(float, float);
    void pushPoint();
    void incrementVars(float, float);
    bool likelyDifferentEntity(float);

    public:
        EntityTracker(const sensor_msgs::LaserScan &);
        void update(float, float);
        void close(ros::Publisher *);
};


/* Typedefs for navigating maps and point vectors in PersonTracker */
typedef std::map<char, geometry_msgs::Point32>::iterator map_it;
typedef std::vector<geometry_msgs::Point32>::iterator pt_it;
struct pt_cmp
{
    bool operator() (const geometry_msgs::Point32 &lhp,
                     const geometry_msgs::Point32 &rhp)
    {
        return (  (pow(lhp.x, 2.0) + pow(lhp.y, 2.0))
                < (pow(rhp.x, 2.0) + pow(rhp.y, 2.0))
               );
    }
};

class PersonTracker
{
    // Variables and data bins
    char next_key;
    int count;
    std::map<char, geometry_msgs::Point32> monitored_points;
    std::map<char, geometry_msgs::Point32> displacements;
    std::set<char> persons;
    ros::Publisher *loc_pub;
    ros::Publisher *new_pub;

    // Auxiliary functions
    bool isPerson(char);
    void addPersonLoc(char, geometry_msgs::Point32,
            sensor_msgs::PointCloud *);
    bool likelyIdentical(geometry_msgs::Point32, const geometry_msgs::Point32);
    void registerNewPoint(geometry_msgs::Point32);
    void scramble(char, const geometry_msgs::Point32,
            std::set<geometry_msgs::Point32, pt_cmp> *,
            std::set<char> *);
    void publish(const sensor_msgs::PointCloud &);

    public:
        PersonTracker();
        PersonTracker(ros::Publisher *, ros::Publisher *);
        void update(sensor_msgs::PointCloud);
};

#endif