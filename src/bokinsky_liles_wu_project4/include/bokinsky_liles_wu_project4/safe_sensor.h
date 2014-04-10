#ifndef SAFE_SENSOR_H
#define SAFE_SENSOR_H

#include <stdint.h>
#include <iomanip>

#include <ros/ros.h>
#include <create_node/TurtlebotSensorState.h>
#include <geometry_msgs/Twist.h>
#include <diagnostic_msgs/DiagnosticArray.h>

namespace safe_sensor
{
    ros::Publisher safety_mon;
    geometry_msgs::Twist safety_brake;
}

#endif
