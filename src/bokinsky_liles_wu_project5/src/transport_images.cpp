#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


const static std::string CV_VIEWER_WINDOW = "opencv_video";

namespace xport
{
    image_transport::Subscriber sub;
    ros::Publisher              pub;
    void image_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::imshow(CV_VIEWER_WINDOW, cv_ptr->image);
        cv::waitKey(3);

        // Output modified video stream
        pub.publish(*msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_processing");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    cv::namedWindow(CV_VIEWER_WINDOW, CV_WINDOW_AUTOSIZE);
    cv::startWindowThread();

    xport::pub = nh.advertise<sensor_msgs::Image>("camera/transported_image", 1);
    xport::sub = it.subscribe("camera/rgb/image_color", 1,
            &xport::image_callback);

    ros::spin();
    cv::destroyWindow(CV_VIEWER_WINDOW);
}
