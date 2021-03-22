#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera");

    ros::NodeHandle n;

    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("/camera/color/image_raw", 10);
    sensor_msgs::ImagePtr msg;

    VideoCapture cap(0);
    if (cap.isOpened())
    {
        ROS_INFO("Open camera successful.");
    }
    else
    {
        ROS_INFO("Open camera failure!");
    }
    
    n.setParam("frame_width", cap.get(cv::CAP_PROP_FRAME_WIDTH));
    n.setParam("frame_height", cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    Mat frame;

    ros::Rate loop_rate(60);

    while (ros::ok())
    {
        cap >> frame;
        if (frame.empty())
        {
            ROS_INFO("Frame is empty!");
            continue;
        }
        
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    
    cap.release();
}