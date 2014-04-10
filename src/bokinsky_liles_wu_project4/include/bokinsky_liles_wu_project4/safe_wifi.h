#ifndef SAFE_WIFI_H
#define SAFE_WIFI_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <create_node/TurtlebotSensorState.h>

namespace safe_wifi
{
    const float max_wifi_signal = 70.0;

    ros::Publisher safety_mon;
    geometry_msgs::Twist safety_brake;

}

#endif
