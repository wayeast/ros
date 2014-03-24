#include <ros/ros.h>
#include <turtlebot_node/TurtlebotSensorState.h>

void getInfo(const turtlebot_node::TurtlebotSensorState& msg){
	ROS_INFO_STREAM("Message: " << msg.bump_wheeldrops);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "subscribe to sensor_info");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("turtlebot_node/sensor_state", 1000, &getInfo);

	ros::spin();
}
