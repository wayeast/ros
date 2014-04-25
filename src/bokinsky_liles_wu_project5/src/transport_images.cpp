#include <cmath>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


const static std::string CV_VIEWER_WINDOW = "opencv_video";
const static std::string HSV_VIEWER_WINDOW = "hsv_video";
const static std::string THRESH_VIEWER_WINDOW  = "color_rangefinder";
const static double IMAGE_PROPORTION = 0.1;
static ros::Publisher drive_pub;
//static geometry_msgs::Twist drive_msg;


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
        ROS_INFO_STREAM("testing orig image channels = " << imgOrig.channels());
        cv::Mat imgHSV = cv::Mat(imgOrig.size(), CV_8UC3);
        cv::cvtColor(imgOrig, imgHSV, CV_BGR2HSV);
        return imgHSV;
    }

    cv::Mat
    getThresholdedImage(cv::Mat imgHSV)
    {
        cv::Mat imgThresh =cv::Mat(imgHSV.size(), CV_8UC1);
        cv::inRange(imgHSV,
                cv::Scalar(110, 90, 100),
                cv::Scalar(120, 160, 255),
                //cv::Scalar(lowerH,lowerS,lowerV),
                //cv::Scalar(upperH,upperS,upperV),
                imgThresh);
        return imgThresh;
    }
}


namespace xport
{
    image_transport::Subscriber sub;
    //ros::Publisher              pub;

    // Output modified video stream
    //pub.publish(*msg);
}


namespace follow_blue_tape_road
{

    double
    getLinearVel(int targ_col, int tot_cols)
    {
        double midway = (double) tot_cols / 2.0;
        double numerator = std::abs(midway - (double) targ_col);
        double x = midway / 2.0 - numerator;
        numerator = (x < 0.0) ? 0.0 : x;
        return numerator / (2 * midway);
    }

    double
    getAngularVel(int targ_col, int tot_cols, double lin_vel)
    {
        if (targ_col == 0)
            return 0.5;
        //double mass = 1.0 - lin_vel;
        //return (targ_col < tot_cols / 2) ? mass : (-1) * mass;
        return (targ_col < tot_cols / 2) ? 0.2 : -0.2;
    }

    geometry_msgs::Twist
    getDrivingDirections(cv::Mat img)
    {
        int target_col;
        int row_weight = (int) img.rows * IMAGE_PROPORTION;
        int row;
        int col;
        int current_row_count;
        long current_row_avg;
        unsigned long total_row_avg = 0;
        long total_row_denom = 0;
        //ROS_INFO_STREAM("entered getDrivingDirections");
        ROS_INFO_STREAM("b/f loop no rows: " << img.rows <<
                        " no cols: " << img.cols <<
                        " row_weight: " << row_weight <<
                        " channels: " << img.channels());
        for (row=img.rows - 1; row_weight > 0; row--, row_weight--)
        {
            //ROS_INFO_STREAM("made it to outer loop");
            current_row_count = current_row_avg = 0;
            for (col=0; col < img.cols; col++)
            {
                //ROS_INFO_STREAM("made it to inner loop");
                //int pixel = 0;
                unsigned char pixel = img.at<unsigned char>(row, col);
                if (pixel)
                {
                    //ROS_INFO_STREAM("found non-black pixel at " << row <<
                    //        ", " << col);
                    current_row_avg += col;
                    current_row_count++;
                }
            }
            ROS_INFO_STREAM(current_row_count << 
                    " white pixels on row " << row);
            current_row_avg = (current_row_count == 0) ?
                0 : (long) current_row_avg / current_row_count;
            //ROS_INFO_STREAM("current_row_avg " << current_row_avg);
            total_row_avg += (unsigned long) current_row_avg * row_weight;
            //ROS_INFO_STREAM("total_row_avg " << total_row_avg);
            total_row_denom += row_weight;
            //ROS_INFO_STREAM("total_row_denom " << total_row_avg);
        }  // outer (row) loop

        target_col = (int) total_row_avg / total_row_denom;
        ROS_INFO_STREAM("target col = " << target_col);
        geometry_msgs::Twist msg;
        msg.linear.x = getLinearVel(target_col, img.cols);
        msg.angular.z = getAngularVel(target_col, img.cols, msg.linear.x);
        ROS_INFO_STREAM("forward vel: " << msg.linear.x <<
                        "angle vel: " << msg.angular.z);
        //msg.linear.x = 0.0;
        //msg.angular.z = 0.0;
        return msg;
    }  // getDrivingDirections
}  // namespace follow_blue_tape_road


void
image_callback(const sensor_msgs::ImageConstPtr& msg)
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

    ROS_INFO_STREAM("in callback, sending straight drive message");
    //drive_msg.linear.x = 1;
    //drive_msg.angular.z = 0;
    geometry_msgs::Twist drive_msg = 
        follow_blue_tape_road::getDrivingDirections(t_image);
    drive_pub.publish(drive_msg);

    cv::waitKey(3);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_processing");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    cv::namedWindow(CV_VIEWER_WINDOW, CV_WINDOW_AUTOSIZE);
    track_image::setTrackWindow();
    cv::startWindowThread();

    //xport::pub = nh.advertise<sensor_msgs::Image>(
    //        "camera/transported_image", 1);
    xport::sub = it.subscribe("camera/rgb/image_color", 1,
            &image_callback);
    drive_pub = nh.advertise<geometry_msgs::Twist>(
            "/cmd_vel_mux/input/navi", 100);

    //ROS_INFO_STREAM("set up; starting ros::spin");
    ros::spin();
    cv::destroyAllWindows();
}
