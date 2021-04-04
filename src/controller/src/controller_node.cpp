#include <ros/ros.h>
#include <common/cmd_action.h>
#include <common/cmd_belt.h>
#include <common/cmd_vel.h>
#include <common/serial_received_msg.h>
#include <common/serial_send_msg.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Bool.h>


std::mutex mux;
common::serial_send_msg send_buf;
ros::Publisher imu_pub;
uint8_t sign;
ros::Publisher odom_pub;
ros::Publisher sign_pub;

void cmdVelCallback(const common::cmd_vel::ConstPtr& msg)
{
    mux.lock();
    send_buf.vx = msg->vx;
    send_buf.vy = msg->vy;
    send_buf.vw = msg->vw;
    mux.unlock();
}

void cmdActionCallback(const common::cmd_action::ConstPtr& msg)
{
    uint8_t act_tmp = 0;
    if (msg->body_lift)
    {
        act_tmp += 1;
    }
    act_tmp <<= 1;
    if (msg->grasp)
    {
        act_tmp += 1;
    }
    act_tmp <<= 1;
    if (msg->push)
    {
        act_tmp += 1;
    }
    act_tmp <<= 1;
    if (msg->camera_lift)
    {
        act_tmp += 1;
    }
    act_tmp <<= 1;
    if (msg->flip)
    {
        act_tmp += 1;
    }
    mux.lock();
    send_buf.action = act_tmp;
    mux.unlock();
}

void cmdBeltCallback(const common::cmd_belt::ConstPtr& msg)
{
    mux.lock();
    send_buf.belt = msg->belt;
    mux.unlock();
}

void serialReceivedCallback(const common::serial_received_msg::ConstPtr& msg)
{
    static uint32_t seq = 0;
    sign = msg->sign;
    nav_msgs::Odometry odom_msg;
    odom_msg.child_frame_id = "odom";
    odom_msg.header.frame_id = "world";
    odom_msg.header.seq = seq++;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.pose.pose.position.x = msg->position_x_mm / 1000.0;
    odom_msg.pose.pose.position.y = msg->position_y_mm / 1000.0;
    odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(msg->gyro_angle / 1800.0 * M_PI);
    odom_msg.twist.twist.linear.x = msg->v_x_mm / 1000.0;
    odom_msg.twist.twist.linear.y = msg->v_y_mm / 1000.0;
    odom_msg.twist.twist.angular.z = msg->gyro_rate / 1800.0 * M_PI;
    odom_pub.publish(odom_msg);
    
    std_msgs::Bool bool_msg;
    bool_msg.data = msg->sign == 1;
    sign_pub.publish(bool_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;
    ros::Publisher serial_pub = n.advertise<common::serial_send_msg>("/serial_send_msg", 10);
    ros::Subscriber serial_sub = n.subscribe("/serial_received_msg", 10, serialReceivedCallback);
    ros::Subscriber vel_sub = n.subscribe("/cmd_vel", 10, cmdVelCallback);
    ros::Subscriber act_sub = n.subscribe("/cmd_action", 10, cmdActionCallback);
    ros::Subscriber belt_sub = n.subscribe("/cmd_belt", 10, cmdBeltCallback);
    sign_pub = n.advertise<std_msgs::Bool>("/sign", 10);
    odom_pub = n.advertise<nav_msgs::Odometry>("/odometry", 10);
    ros::Rate loop_rate(50);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    while (ros::ok())
    {
        if (sign == 1)
        {
            mux.lock();
            send_buf.sign = 1;
            serial_pub.publish(send_buf);
            mux.unlock();
        }
        
        loop_rate.sleep();
    }
    spinner.stop();

    return 0;
}
