#include <ros/ros.h>
#include <common/target_center.h>
#include <common/cmd_align.h>
#include <common/cmd_vel.h>
#include <std_msgs/Bool.h>
#include <mutex>

bool with_depth;
bool is_align;
double align_kp;
ros::Publisher vel_pub;
ros::Publisher state_pub;

void centerMsgCallback(const common::target_center::ConstPtr& msg)
{
    if (is_align)
    {
        if (msg->is_found)
        {
            common::cmd_vel vel_msg;
            if (with_depth)
            {
                /* code */
            }
            else
            {
                if (std::abs(msg->dx) > 0.01)
                {
                    vel_msg.vx = align_kp * msg->dx;
                    vel_msg.vy = 0;
                    vel_msg.vw = 0;
                    vel_pub.publish(vel_msg);
                }
                else
                {
                    is_align = false;
                    std_msgs::Bool bool_msg;
                    bool_msg.data = is_align;
                    state_pub.publish(bool_msg);
                }
            }
        }
    }
    
}

void cmdAlignCallback(const common::cmd_align::ConstPtr& msg)
{
    is_align = msg->do_align;
    std_msgs::Bool bool_msg;
    bool_msg.data = is_align;
    state_pub.publish(bool_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auto_align");
    ros::NodeHandle n;
    n.getParam("with_depth", with_depth);
    n.getParam("align_kp", align_kp);
    ros::Subscriber center_msg_sub = n.subscribe("/target_center", 10, centerMsgCallback);
    ros::Subscriber cmd_align_sub = n.subscribe("/cmd_align", 10, cmdAlignCallback);
    state_pub = n.advertise<std_msgs::Bool>("/align_state", 1);
    vel_pub = n.advertise<common::cmd_vel>("/cmd_vel", 1);

    ros::spin();

    return 0;
}