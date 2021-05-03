#include <ros/ros.h>
#include "referee_system/referee_system.h"
#include <std_srvs/Empty.h>
#include <memory>
#include <std_msgs/Bool.h>
#include "common/bounding_box.h"
#include <thread>


std::unique_ptr<RefereeSystem> rs;
bool pre_align_state = false;


bool drawUICallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    rs->drawGuideLine(RED_ENGINEER_ROBOT, RED_ENGINEER_CLIENT, 0);
    return true;
}

bool clearUICallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    rs->deleteAll(2, 0x0102);
    return true;
}

void alignStateCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data != pre_align_state) {
        pre_align_state = msg->data;
        rs->deleteLayer(RED_ENGINEER_ROBOT, RED_ENGINEER_CLIENT, 4);
        if (msg->data) {
            rs->drawString(RED_ENGINEER_ROBOT, RED_ENGINEER_CLIENT, (uint8_t*)"ali", 4, 12, STUDENT_CLIENT_WIDTH / 2 - 55, 350, (uint8_t*)"Aligning...", 11, 1, 2);
        }
    }
}

void drawUiThread() {
    while (ros::ok()) {
        rs->drawGuideLine(RED_ENGINEER_ROBOT, RED_ENGINEER_CLIENT, 0);
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
}

void drawBoxCallback(const common::bounding_box::ConstPtr& msg) {
    rs->deleteLayer(RED_ENGINEER_ROBOT, RED_ENGINEER_CLIENT, 6);
    rs->drawRectangle(RED_ENGINEER_ROBOT, RED_ENGINEER_CLIENT, (uint8_t*)"bdb", 6, msg->x - msg->w / 2, msg->y - msg->h / 2, msg->x + msg->w / 2, msg->y + msg->h / 2, 2, 2);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "referee_system");
    ros::NodeHandle nh;
    ros::ServiceServer draw_ui_srv = nh.advertiseService("referee/draw_ui", drawUICallback);
    ros::ServiceServer clear_ui_srv = nh.advertiseService("referee/clear_ui", clearUICallback);
    ros::Subscriber align_state_sub = nh.subscribe("/align_state", 1, alignStateCallback);
    ros::Subscriber box_sub = nh.subscribe("/referee/draw_box", 1, drawBoxCallback);
    
    std::string serial_port_name;
    if (nh.getParam("/referee_serial_port_name", serial_port_name)) {
        ROS_INFO("Get referee system serial port name successful.");
    }
    else {
        ROS_INFO("Get referee system serial port name failure.\nPlease check if the ros param \"/referee_serial_port_name\" is set.");
        return -1;
    }
    
    rs = std::make_unique<RefereeSystem>(serial_port_name, nh);
    std::thread draw_ui_thread(drawUiThread);
    draw_ui_thread.detach();

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(60);

    while (nh.ok()) {
        // Publish referee system message.
        rs->publishFrame();
        loop_rate.sleep();
    }

    spinner.stop();
    return 0;
}
