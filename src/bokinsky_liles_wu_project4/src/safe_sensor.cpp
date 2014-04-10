/**************************************************
 * Name:         Huston Bokinsky
 *               Karina Liles
 *               Xian Wu
 * File:         sensor_info.cpp
 * Assignment:   Project 3 - Turtlebot Setup
 * Date:         25 March, 2014
 * Course:       CSCE 574 - Robotics
 * Instructor:   Dr. O'Kane
 * Description:  Set up basic connections and functionality for 
 *               our turtlebot robots. Create basic launch file; 
 *               get wifi signal strength from robot's netbook; 
 *               monitor and print robot's diagnostic messages, 
 *               battery charge, and wifi strength; control robot 
 *               via network connection from lab workstation. 
 * 
 **************************************************/

#include "safe_sensor.h"


void callback1(const create_node::TurtlebotSensorState& msg)
{
    uint8_t mask = 1;
    if (mask & msg.bumps_wheeldrops)
    {
        safe_sensor::safety_mon.publish(safe_sensor::safety_brake);
        ROS_INFO_STREAM("Bump right -- stopping robot.");
    }

    mask <<= 1;
    if (mask & msg.bumps_wheeldrops)
    {
        safe_sensor::safety_mon.publish(safe_sensor::safety_brake);
        ROS_INFO_STREAM("Bump left -- stopping robot.");
    }

    mask <<= 1;
    if (mask & msg.bumps_wheeldrops)
    {
        safe_sensor::safety_mon.publish(safe_sensor::safety_brake);
        ROS_INFO_STREAM("Wheel drop right -- stopping robot.");
    }

    mask <<= 1;
    if (mask & msg.bumps_wheeldrops)
    {
        safe_sensor::safety_mon.publish(safe_sensor::safety_brake);
        ROS_INFO_STREAM("Wheel drop left -- stopping robot.");
    }

    mask <<= 1;
    if (mask & msg.bumps_wheeldrops)
    {
        safe_sensor::safety_mon.publish(safe_sensor::safety_brake);
        ROS_INFO_STREAM("Wheel drop caster -- stopping robot.");
    }
}

void callback2(const diagnostic_msgs::DiagnosticArray& msg){

    for(int i =0; i < msg.status.size(); i++){
        if(msg.status[i].level==1)
        {
            safe_sensor::safety_mon.publish(safe_sensor::safety_brake);
            ROS_INFO_STREAM("WARNING from " <<
                    msg.status[i].name << ": " <<
                    msg.status[i].message <<
                    " -- stopping robot");
        }
        if(msg.status[i].level==2)      
        {
            safe_sensor::safety_mon.publish(safe_sensor::safety_brake);
            ROS_INFO_STREAM("ERROR from " << 
                    msg.status[i].name << ": " <<
                    msg.status[i].message <<
                    " -- stopping robot"); 
        }

    }
}



int main(int argc, char **argv){

    ros::init(argc, argv, "subscribe_to_sensor_info");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/turtlebot_node/sensor_state",
            1000, &callback1);
    ros::Subscriber sub2 = nh.subscribe("/diagnostics",
            1000, &callback2);
    safe_sensor::safety_mon = nh.advertise<geometry_msgs::Twist>(
            "cmd_vel_mux/input/safety_controller", 100);
    safe_sensor::safety_brake = geometry_msgs::Twist();
    safe_sensor::safety_brake.linear.x = 0.0;
    safe_sensor::safety_brake.angular.z = 0.0;

    ros::spin();
}
