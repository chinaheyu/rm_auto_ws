#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <string>
#include <common/target_center.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <mutex>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>

std::mutex cent_mux;
std::mutex info_mux;
bool is_found;
float centerpix[2];
rs2_intrinsics inrist;

bool is_align;
bool with_depth;
image_transport::Publisher pub;
cv::Scalar thresh1(8, 110, 0);
cv::Scalar thresh2(30, 255, 255);
double min_area_size = 200;
std::vector<std::vector<cv::Point> > contours;
int idx;
ros::Publisher pubcenter;

bool findBiggestContour(const cv::Mat& frame)
{
    cv::Mat color_image, img_hsv, img_morph;
    frame.copyTo(color_image);
    cv::GaussianBlur(color_image, color_image, cv::Size(5, 5), 3);
    cv::cvtColor(color_image, img_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(img_hsv, thresh1, thresh2, img_morph);

    cv::Mat struct1 = getStructuringElement(0, cv::Size(3, 3));
    cv::erode(img_morph, img_morph, struct1, cv::Point(-1, -1), 3);
    cv::dilate(img_morph, img_morph, struct1, cv::Point(-1, -1), 3);

    
    cv::findContours(img_morph, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    if (!contours.empty())
    {
        double area, max_area = 0;
        for (int i = 0; i < contours.size(); ++i)
        {
            area = cv::contourArea(contours[i]);
            if (area > max_area)
            {
                max_area = area;
                idx = i;
            }
        }
        if (max_area < min_area_size)
            return false;
        return true;
    }
    else
    {
        return false;
    }
}

void frameCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (is_align)
    {
        common::target_center centmsg;
        std::stringstream ss;
        cv::Mat raw_frame(cv_bridge::toCvShare(msg, "bgr8")->image);
        int frame_width = raw_frame.rows;
        int frame_height = raw_frame.cols;
        if (findBiggestContour(raw_frame))
        {
            cv::RotatedRect box = cv::minAreaRect(contours[idx]);
            ss << "center(" << box.center.x << ", " << box.center.y << ")";
            cv::drawContours(raw_frame, contours, idx, cv::Scalar(128, 0, 64), 2);
            cv::rectangle(raw_frame, box.boundingRect(), cv::Scalar(73, 245, 189), 2);
            cv::circle(raw_frame, box.center, 3, cv::Scalar(73, 245, 189), cv::FILLED);
            cv::putText(raw_frame, "Target detected.", cv::Point(0, 24), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 121, 242), 2);
            cv::putText(raw_frame, ss.str(), cv::Point(0, 60), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 121, 242), 2);

            if (with_depth)
            {
                cent_mux.lock();
                is_found = true;
                centerpix[0] = box.center.x;
                centerpix[1] = box.center.y;
                cent_mux.unlock();
            }
            else
            {
                centmsg.is_found = true;
                centmsg.dx = (box.center.x - frame_width / 2) / frame_width;
                centmsg.dy = (box.center.y - frame_height / 2) / frame_height;
                centmsg.dz = 0;
                pubcenter.publish(centmsg);
            }
        }
        else
        {

            if (with_depth)
            {
                cent_mux.lock();
                is_found = false;
                cent_mux.unlock();
            }
            else
            {
                centmsg.is_found = false;
                pubcenter.publish(centmsg);
            }

            ss << "Target is missing.";
            cv::putText(raw_frame, ss.str(), cv::Point(0, 24), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 185), 2);
        }

        sensor_msgs::ImagePtr smsg;
        smsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", raw_frame).toImageMsg();
        pub.publish(smsg);
    }
}

void depthCallback(const sensor_msgs::ImageConstPtr& data)
{
    if (is_align)
    {
        cv::Mat depth_frame(cv_bridge::toCvShare(data)->image);
        float depth = 0.001 * depth_frame.at<uint16_t>(int(centerpix[1]), int(centerpix[0]));
        float coord[3];
        common::target_center centmsg;
        cent_mux.lock();
        info_mux.lock();
        if (is_found)
        {
            rs2_deproject_pixel_to_point(coord, &inrist, centerpix, depth);
            centmsg.is_found = is_found;
            centmsg.dx = coord[0];
            centmsg.dy = coord[1];
            centmsg.dz = coord[2];
        }
        else
        {
            centmsg.is_found = false;
        }
        info_mux.unlock();
        cent_mux.unlock();
        pubcenter.publish(centmsg);
    }
}

void cameraInfoCallback(const sensor_msgs::CameraInfo& data)
{
    if (is_align)
    {
        info_mux.lock();
        inrist.width = data.width;
        inrist.height = data.height;
        inrist.ppx = data.K[2];
        inrist.ppy = data.K[5];
        inrist.fx = data.K[0];
        inrist.fy = data.K[4];
        if (data.distortion_model == "plumb_bob")
        {
            inrist.model = rs2_distortion::RS2_DISTORTION_BROWN_CONRADY;
        }
        else if (data.distortion_model == "equidistant")
        {
            inrist.model = rs2_distortion::RS2_DISTORTION_KANNALA_BRANDT4;
        }
        for (size_t i = 0; i < 5; i++)
        {
            inrist.coeffs[i] = data.D[i];
        }
        info_mux.unlock();
    }
}

void alignStateCallback(const std_msgs::Bool::ConstPtr& msg)
{
    is_align = msg->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detector");

    ros::NodeHandle n;
    n.getParam("with_depth", with_depth);
    pubcenter = n.advertise<common::target_center>("/target_center", 1);
    ros::Subscriber cmd_align_sub = n.subscribe("/align_state", 10, alignStateCallback);
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, frameCallback);
    image_transport::Subscriber depth_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depthCallback);
    ros::Subscriber camera_info_sub = n.subscribe("/camera/aligned_depth_to_color/camera_info", 1, cameraInfoCallback);
    pub = it.advertise("/test_frame", 1);

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}