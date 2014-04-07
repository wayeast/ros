#ifndef PROJECT_4_H
#define PROJECT_4_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "create_autodrive.h"


class Project4Env
{
    private:
        ros::Subscriber sub;
        ros::Publisher  pub;

    public:
        Project4Env(ros::NodeHandle *);
        void sub_callback(const sensor_msgs::LaserScan &);
};


#endif
