#include <stdint.h>
#include <iomanip>

#include <ros/ros.h>
#include <create_node/TurtlebotSensorState.h>
#include <diagnostic_msgs/DiagnosticArray.h>



void getSensorInfo(const create_node::TurtlebotSensorState& msg)
{
    uint8_t mask = 1;
    if (mask & msg.bumps_wheeldrops == 1)
        ROS_INFO_STREAM("Bump right.");
    mask << 1;
    if (mask & msg.bumps_wheeldrops == 2)
        ROS_INFO_STREAM("Bump left.");
    mask << 1;
    if (mask & msg.bumps_wheeldrops == 4)
        ROS_INFO_STREAM("Wheel drop right.");
    mask << 1;
    if (mask & msg.bumps_wheeldrops == 8)
        ROS_INFO_STREAM("Wheel drop left.");
    mask << 1;
    if (mask & msg.bumps_wheeldrops == 16)
        ROS_INFO_STREAM("Wheel drop caster.");
}

void getMsg(const diagnostic_msgs::DiagnosticArray& msg){

    for(int i =0; i < msg.status.size(); i++){
        if(msg.status[i].level==1)
            ROS_INFO_STREAM("WARNING from " <<
                    msg.status[i].name << ": " <<
                    msg.status[i].message);
        if(msg.status[i].level==2)      
            ROS_INFO_STREAM("ERROR from " << 
                    msg.status[i].name << ": " <<
                    msg.status[i].message); 

    }
}



int main(int argc, char **argv){

    ros::init(argc, argv, "subscribe_to_sensor_info");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/turtlebot_node/sensor_state",
            1000, &getSensorInfo);
    ros::Subscriber sub2 = nh.subscribe("/diagnostics",
            1000, &getMsg);

    ros::spin();
}
