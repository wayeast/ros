// ROS includes
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <create_node/TurtlebotSensorState.h>


#define WIFI_MAX_CAPACITY  70


void pubBattery(const create_node::TurtlebotSensorState& msg){
    ROS_INFO_STREAM("Create battery status: " << 
            //std::setprecision(0) << std::fixed <<
            (int)((float)msg.charge/(float)msg.capacity*100) << 
            "%");
}

void pubWifi(const std_msgs::Int32& msg)
{
    float max = WIFI_MAX_CAPACITY;
    ROS_INFO_STREAM("Wifi signal strength: " <<
        (int)((float)msg.data / max * 100) <<
        "%");
}


int main(int argc, char **argv)
{
    // Basic ROS environment + subscribers setup
    ros::init(argc, argv, "report_battery_and_wifi");
    ros::NodeHandle nh;
    ros::Subscriber wifi_rpt = nh.subscribe("/wifi_ss",
        1, &pubWifi);
    ros::Subscriber batt_rpt = nh.subscribe("/turtlebot_node/sensor_state",
        1, &pubBattery);
    ros::Rate rate(0.2);

    while(ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
    }
}
