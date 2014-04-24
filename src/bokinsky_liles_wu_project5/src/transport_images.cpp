#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


const static std::string CV_VIEWER_WINDOW = "opencv_video";
const static std::string HSV_VIEWER_WINDOW = "hsv_video";
const static std::string THRESH_VIEWER_WINDOW  = "color_rangefinder";

/************* Some forward declarations ***********
void xport::image_callback(const sensor_msgs::ImageConstPtr&);
void track_image::setTrackWindow();
cv::Mat track_image::getHSVImage(cv::Mat);
cv::Mat track_image::getThresholdedImage(cv::Mat);
*/

namespace track_image
{
    int lowerH=0;
    int lowerS=0;
    int lowerV=0;

    int upperH=180;
    int upperS=256;
    int upperV=256;

    void 
    setTrackWindow()
    {
        cv::namedWindow(HSV_VIEWER_WINDOW, CV_WINDOW_AUTOSIZE);
        cv::namedWindow(THRESH_VIEWER_WINDOW, CV_WINDOW_AUTOSIZE);

        cv::createTrackbar("LowerH", THRESH_VIEWER_WINDOW,
                &lowerH, 320, NULL);
        cv::createTrackbar("UpperH", THRESH_VIEWER_WINDOW,
                &upperH, 320, NULL);

        cv::createTrackbar("LowerS", THRESH_VIEWER_WINDOW,
                &lowerS, 256, NULL);
        cv::createTrackbar("UpperS", THRESH_VIEWER_WINDOW,
                &upperS, 256, NULL);

        cv::createTrackbar("LowerV", THRESH_VIEWER_WINDOW, 
                &lowerV, 256, NULL);
        cv::createTrackbar("UpperV", THRESH_VIEWER_WINDOW,
                &upperV, 256, NULL); 
    }  
    
    cv::Mat
    getHSVImage(cv::Mat imgOrig)
    {
        cv::Mat imgHSV = cv::Mat(imgOrig.size(), CV_8UC3);
        cv::cvtColor(imgOrig, imgHSV, CV_BGR2HSV);
        return imgHSV;
    }

    cv::Mat
    getThresholdedImage(cv::Mat imgHSV)
    {
        cv::Mat imgThresh =cv::Mat(imgHSV.size(), CV_8UC1);
        cv::inRange(imgHSV,
                cv::Scalar(lowerH,lowerS,lowerV),
                cv::Scalar(upperH,upperS,upperV),
                //cv::Scalar(110,50,100),
                //cv::Scalar(120,140,255),
                imgThresh);
        return imgThresh;
    }
}



namespace xport
{
    image_transport::Subscriber sub;
    ros::Publisher              pub;
    void image_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg,
                    sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::imshow(CV_VIEWER_WINDOW, cv_ptr->image);

        cv::Mat hsv_image = track_image::getHSVImage(cv_ptr->image);
        cv::imshow(HSV_VIEWER_WINDOW, hsv_image);

        cv::Mat t_image   = track_image::getThresholdedImage(hsv_image);
        cv::imshow(THRESH_VIEWER_WINDOW, t_image);

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
    track_image::setTrackWindow();
    cv::startWindowThread();

    xport::pub = nh.advertise<sensor_msgs::Image>(
            "camera/transported_image", 1);
    xport::sub = it.subscribe("camera/rgb/image_color", 1,
            &xport::image_callback);

    ros::spin();
    cv::destroyAllWindows();
}
