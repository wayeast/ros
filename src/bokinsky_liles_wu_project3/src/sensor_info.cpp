#include <ros/ros.h>
#include <create_node/TurtlebotSensorState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <iomanip>

void getInfo(const create_node::TurtlebotSensorState& msg){
    if(msg.bumps_wheeldrops==15)
        ROS_INFO_STREAM("Wheel drop caster.");
    if(msg.bumps_wheeldrops==20)
        ROS_INFO_STREAM("Wheel drop left.");
    if(msg.bumps_wheeldrops==24)
        ROS_INFO_STREAM("Wheel drop right.");
    if(msg.bumps_wheeldrops==28)
        ROS_INFO_STREAM("Robot lifted.");
    if(msg.bumps_wheeldrops==31)
        ROS_INFO_STREAM("Robot bump sensor pressed");
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
            1000, &getInfo);
    ros::Subscriber sub2 = nh.subscribe("/diagnostics",
            1000, &getMsg);

    ros::spin();
}
