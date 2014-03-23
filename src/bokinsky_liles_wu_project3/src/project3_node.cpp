#include <stdlib.h>
#include <stdio.h>
//#include <iostream.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main(int argc, char **argv)
{
    FILE *in;
    char buff[256];
   ros::init(argc, argv, "publish_wifi_strength");
   ros::NodeHandle nh;

   ros::Publisher wifi_reporter = nh.advertise<std_msgs::Int32>(
          "wifi_ss", 100);
   std_msgs::Int32 msg;
   ros::Rate rate(2);

   while(ros::ok())
   {
       if (!(in = popen("iwconfig wlan0|grep Quality|cut -d'=' -f2|cut -d'/' -f1", "r")))
       {
           return 1;
       }
      
       std::fgets(buff, sizeof(buff), in);
       msg.data = std::atoi(buff);
      ROS_INFO_STREAM("Sending value " << msg.data << " to wifi_ss");
      wifi_reporter.publish(msg);
      pclose(in);
      rate.sleep();
   }
}
