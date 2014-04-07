#include <ros/ros.h>
#include "project4.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sunday_drive");
    ros::NodeHandle nh;

    Project4Env env(&nh);

    ros::spin();

}
