#ifndef REFEREE_SYSTEM_H
#define REFEREE_SYSTEM_H

#include "referee_system/referee_option.h"
#include "referee_system/serial_device.h"
#include "referee_system/crc.h"
#include "referee_system/referee_system_def.h"
#include "common/game_state.h"


#include <ros/ros.h>


class RefereeSystem {
private:
    ros::NodeHandle nh;
    SerialDevice serial;
    uint8_t seq;

    // ROS Publisher
    ros::Publisher game_state_pub;

    void sendData(uint16_t cmd_id, void *p_buf, uint16_t len);
    void sendInteractiveData(uint16_t data_cmd_id, uint16_t sender_ID, uint16_t receiver_ID, void *p_buf, uint16_t len);
    void publishROSMessage(uint8_t* p_frame);

#ifdef REFEREE_SCREEN_COORD_TRANS
    void msgCoordTrans(graphic_data_struct_t* p_graphic_data_struct);
#endif

public:
    explicit RefereeSystem(std::string serial_name, ros::NodeHandle& node_handle) : serial(serial_name), seq(0), nh(node_handle) {
        game_state_pub = nh.advertise<common::game_state>("/referee/game_state", 1);
    }


    /* Referee system draw ui function */
    void drawLine(uint16_t sender_ID, uint16_t receiver_ID, const uint8_t* graphic_name, uint32_t layer, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y, uint32_t width, uint32_t color);

    void drawRectangle(uint16_t sender_ID, uint16_t receiver_ID, const uint8_t* graphic_name, uint32_t layer, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y, uint32_t width, uint32_t color);

    void drawCircle(uint16_t sender_ID, uint16_t receiver_ID, const uint8_t* graphic_name, uint32_t layer, uint32_t start_x, uint32_t start_y, uint32_t radius, uint32_t width, uint32_t color);
    
    void drawEllipse(uint16_t sender_ID, uint16_t receiver_ID, const uint8_t* graphic_name, uint32_t layer, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y, uint32_t width, uint32_t color);
    
    void drawArc(uint16_t sender_ID, uint16_t receiver_ID, const uint8_t* graphic_name, uint32_t layer, uint32_t start_angle, uint32_t end_angle, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y, uint32_t width, uint32_t color);
    
    void drawFloat(uint16_t sender_ID, uint16_t receiver_ID, const uint8_t* graphic_name, uint32_t layer, uint32_t font_size, uint32_t decimal_digits, uint32_t start_x, uint32_t start_y, float value, uint32_t width, uint32_t color);
    
    void drawInteger(uint16_t sender_ID, uint16_t receiver_ID, const uint8_t* graphic_name, uint32_t layer, uint32_t font_size, uint32_t start_x, uint32_t start_y, int value, uint32_t width, uint32_t color);

    void drawString(uint16_t sender_ID, uint16_t receiver_ID, const uint8_t* graphic_name, uint32_t layer, uint32_t font_size, uint32_t start_x, uint32_t start_y, const uint8_t* value, uint32_t text_lenght, uint32_t width, uint32_t color);

    void deleteLayer(uint16_t sender_ID, uint16_t receiver_ID, uint8_t layer);

    void deleteAll(uint16_t sender_ID, uint16_t receiver_ID);

    void drawGuideLine(uint16_t sender_ID, uint16_t receiver_ID, uint8_t layer);

    void publishFrame();
};


#endif //REFEREE_SYSTEM_H