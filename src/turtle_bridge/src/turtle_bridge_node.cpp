#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <common/serial_send_msg.h>
#include <common/serial_received_msg.h>
#include <turtlesim/Pose.h>
#include <turtlesim/SetPen.h>

ros::Publisher turtle_pub;
ros::Publisher fake_pub;
ros::ServiceClient pen_client;

int state = 0;
uint8_t b = 255;
uint8_t g = 0;
uint8_t r = 0;

void simSendMsgCallback(const common::serial_send_msg::ConstPtr& msg)
{
    geometry_msgs::Twist tmp;
    tmp.linear.x = msg->vx;
    tmp.linear.y = msg->vy;
    tmp.angular.z = msg->vw;
    turtle_pub.publish(tmp);
}

void poseCallback(const turtlesim::Pose::ConstPtr &data)
{
    // Rainbow trail ^_^
    if (data->linear_velocity > 0)
    {
        if (state == 0)
        {
            b--;
            g++;
            if (b == 0)
            {
                state = 1;
            }
        }
        else if (state == 1)
        {
            g--;
            r++;
            if (g == 0)
            {
                state = 2;
            }
        }
        else
        {
            r--;
            b++;
            if (r == 0)
            {
                state = 0;
            }
        }
        turtlesim::SetPen srv;
        srv.request.b = b;
        srv.request.g = g;
        srv.request.r = r;
        srv.request.width = 5;
        pen_client.call(srv);
    }

    common::serial_received_msg msg;
    msg.sign = 1;
    // TODO: Publish pose message from turtle1
    fake_pub.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_bridge");
    ros::NodeHandle n;
    pen_client = n.serviceClient<turtlesim::SetPen>("turtle1/set_pen");
    turtlesim::SetPen pen_srv;
    ros::Subscriber serial_sub = n.subscribe("/serial_send_msg", 10, simSendMsgCallback);
    turtle_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    fake_pub = n.advertise<common::serial_received_msg>("/serial_received_msg", 10);
    ros::Subscriber pose_sub = n.subscribe("/turtle1/pose", 1, poseCallback);
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
}