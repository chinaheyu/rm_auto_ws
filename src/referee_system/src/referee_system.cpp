#include "referee_system/referee_system.h"

void RefereeSystem::sendData(uint16_t cmd_id, void *p_buf, uint16_t len) {

    uint8_t txBuf[REF_PROTOCOL_FRAME_MAX_SIZE] = {0};

    /* Set frame header */
    auto *pHeader = (frame_header_t *)txBuf;
    pHeader->sof = 0xA5;
    pHeader->data_length = len;
    pHeader->seq = seq++;

    /* Calculate data size */
    uint16_t headSize = REF_PROTOCOL_HEADER_SIZE;
    uint16_t frameSize = len + REF_HEADER_CRC_CMDID_LEN;

    /* Apend CRC */
    memcpy(txBuf + headSize, &cmd_id, sizeof(cmd_id));
    ref_append_crc8(txBuf, headSize);
    memcpy(txBuf + headSize + sizeof(cmd_id), p_buf, len);
    ref_append_crc16(txBuf, frameSize);
#ifdef REFEREE_DEBUG_MODE
    ROS_INFO("Send %d bytes to referee system, cmd_id: 0x%x", frameSize, cmd_id);
    ROS_INFO("Send data header CRC8 verify result: %d", ref_verify_crc8(txBuf, headSize));
    ROS_INFO("Send data total frame CRC16 verify result: %d", ref_verify_crc16(txBuf, frameSize));
#endif
    serial.write(txBuf, frameSize);
}

void RefereeSystem::sendInteractiveData(uint16_t data_cmd_id, uint16_t sender_ID, uint16_t receiver_ID, void *p_buf, uint16_t len) {
    
    uint8_t buf[REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN];

    auto* pHeader = (ext_student_interactive_header_data_t*)buf;
    pHeader->data_cmd_id = data_cmd_id;
    pHeader->sender_ID = sender_ID;
    pHeader->receiver_ID = receiver_ID;

    memcpy(buf + sizeof(ext_student_interactive_header_data_t), p_buf, len);
#ifdef REFEREE_DEBUG_MODE
    ROS_INFO("Send %lu bytes interactive data from %d to 0x%x, data_cmd_id: 0x%x", sizeof(ext_student_interactive_header_data_t) + len, pHeader->sender_ID, pHeader->receiver_ID, pHeader->data_cmd_id);
#endif
    sendData(0x0301, buf, sizeof(ext_student_interactive_header_data_t) + len);
}

void RefereeSystem::drawLine(uint16_t sender_ID, uint16_t receiver_ID, const uint8_t* graphic_name, uint32_t layer, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y, uint32_t width, uint32_t color) {
    ext_client_custom_graphic_single_t ext_client_custom_graphic_single;
    memset(&ext_client_custom_graphic_single, 0, sizeof(ext_client_custom_graphic_single_t));
    memcpy(ext_client_custom_graphic_single.graphic_data_struct.graphic_name, graphic_name, 3);
    ext_client_custom_graphic_single.graphic_data_struct.operate_type = 1;
    ext_client_custom_graphic_single.graphic_data_struct.graphic_type = 0;
    ext_client_custom_graphic_single.graphic_data_struct.layer = layer;
    ext_client_custom_graphic_single.graphic_data_struct.color = color;
    ext_client_custom_graphic_single.graphic_data_struct.width = width;
    ext_client_custom_graphic_single.graphic_data_struct.start_x = start_x;
    ext_client_custom_graphic_single.graphic_data_struct.start_y = start_y;
    ext_client_custom_graphic_single.graphic_data_struct.end_x = end_x;
    ext_client_custom_graphic_single.graphic_data_struct.end_y = end_y;
#ifdef REFEREE_SCREEN_COORD_TRANS
    msgCoordTrans(&ext_client_custom_graphic_single.graphic_data_struct);
#endif
#ifdef REFEREE_DEBUG_MODE
    ROS_INFO("Draw a line named %c%c%c at %d layer.", graphic_name[0], graphic_name[1], graphic_name[2], layer);
#endif
    sendInteractiveData(0x0101, sender_ID, receiver_ID, &ext_client_custom_graphic_single, sizeof(ext_client_custom_graphic_single_t));
}

void RefereeSystem::drawRectangle(uint16_t sender_ID, uint16_t receiver_ID, const uint8_t* graphic_name, uint32_t layer, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y, uint32_t width, uint32_t color) {
    ext_client_custom_graphic_single_t ext_client_custom_graphic_single;
    memset(&ext_client_custom_graphic_single, 0, sizeof(ext_client_custom_graphic_single_t));
    memcpy(ext_client_custom_graphic_single.graphic_data_struct.graphic_name, graphic_name, 3);
    ext_client_custom_graphic_single.graphic_data_struct.operate_type = 1;
    ext_client_custom_graphic_single.graphic_data_struct.graphic_type = 1;
    ext_client_custom_graphic_single.graphic_data_struct.layer = layer;
    ext_client_custom_graphic_single.graphic_data_struct.color = color;
    ext_client_custom_graphic_single.graphic_data_struct.width = width;
    ext_client_custom_graphic_single.graphic_data_struct.start_x = start_x;
    ext_client_custom_graphic_single.graphic_data_struct.start_y = start_y;
    ext_client_custom_graphic_single.graphic_data_struct.end_x = end_x;
    ext_client_custom_graphic_single.graphic_data_struct.end_y = end_y;
#ifdef REFEREE_SCREEN_COORD_TRANS
    msgCoordTrans(&ext_client_custom_graphic_single.graphic_data_struct);
#endif
#ifdef REFEREE_DEBUG_MODE
    ROS_INFO("Draw a rectangle named %c%c%c at %d layer.", graphic_name[0], graphic_name[1], graphic_name[2], layer);
#endif
    sendInteractiveData(0x0101, sender_ID, receiver_ID, &ext_client_custom_graphic_single, sizeof(ext_client_custom_graphic_single_t));
}

void RefereeSystem::drawCircle(uint16_t sender_ID, uint16_t receiver_ID, const uint8_t* graphic_name, uint32_t layer, uint32_t start_x, uint32_t start_y, uint32_t radius, uint32_t width, uint32_t color) {
    ext_client_custom_graphic_single_t ext_client_custom_graphic_single;
    memset(&ext_client_custom_graphic_single, 0, sizeof(ext_client_custom_graphic_single_t));
    memcpy(ext_client_custom_graphic_single.graphic_data_struct.graphic_name, graphic_name, 3);
    ext_client_custom_graphic_single.graphic_data_struct.operate_type = 1;
    ext_client_custom_graphic_single.graphic_data_struct.graphic_type = 2;
    ext_client_custom_graphic_single.graphic_data_struct.layer = layer;
    ext_client_custom_graphic_single.graphic_data_struct.color = color;
    ext_client_custom_graphic_single.graphic_data_struct.width = width;
    ext_client_custom_graphic_single.graphic_data_struct.start_x = start_x;
    ext_client_custom_graphic_single.graphic_data_struct.start_y = start_y;
    ext_client_custom_graphic_single.graphic_data_struct.radius = radius;
#ifdef REFEREE_SCREEN_COORD_TRANS
    msgCoordTrans(&ext_client_custom_graphic_single.graphic_data_struct);
#endif
#ifdef REFEREE_DEBUG_MODE
    ROS_INFO("Draw a circle named %c%c%c at %d layer.", graphic_name[0], graphic_name[1], graphic_name[2], layer);
#endif
    sendInteractiveData(0x0101, sender_ID, receiver_ID, &ext_client_custom_graphic_single, sizeof(ext_client_custom_graphic_single_t));
}

void RefereeSystem::drawEllipse(uint16_t sender_ID, uint16_t receiver_ID, const uint8_t* graphic_name, uint32_t layer, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y, uint32_t width, uint32_t color) {
    ext_client_custom_graphic_single_t ext_client_custom_graphic_single;
    memset(&ext_client_custom_graphic_single, 0, sizeof(ext_client_custom_graphic_single_t));
    memcpy(ext_client_custom_graphic_single.graphic_data_struct.graphic_name, graphic_name, 3);
    ext_client_custom_graphic_single.graphic_data_struct.operate_type = 1;
    ext_client_custom_graphic_single.graphic_data_struct.graphic_type = 3;
    ext_client_custom_graphic_single.graphic_data_struct.layer = layer;
    ext_client_custom_graphic_single.graphic_data_struct.color = color;
    ext_client_custom_graphic_single.graphic_data_struct.width = width;
    ext_client_custom_graphic_single.graphic_data_struct.start_x = start_x;
    ext_client_custom_graphic_single.graphic_data_struct.start_y = start_y;
    ext_client_custom_graphic_single.graphic_data_struct.end_x = end_x;
    ext_client_custom_graphic_single.graphic_data_struct.end_y = end_y;
#ifdef REFEREE_SCREEN_COORD_TRANS
    msgCoordTrans(&ext_client_custom_graphic_single.graphic_data_struct);
#endif
#ifdef REFEREE_DEBUG_MODE
    ROS_INFO("Draw a ellipse named %c%c%c at %d layer.", graphic_name[0], graphic_name[1], graphic_name[2], layer);
#endif
    sendInteractiveData(0x0101, sender_ID, receiver_ID, &ext_client_custom_graphic_single, sizeof(ext_client_custom_graphic_single_t));
}

void RefereeSystem::drawArc(uint16_t sender_ID, uint16_t receiver_ID, const uint8_t* graphic_name, uint32_t layer, uint32_t start_angle, uint32_t end_angle, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y, uint32_t width, uint32_t color) {
    ext_client_custom_graphic_single_t ext_client_custom_graphic_single;
    memset(&ext_client_custom_graphic_single, 0, sizeof(ext_client_custom_graphic_single_t));
    memcpy(ext_client_custom_graphic_single.graphic_data_struct.graphic_name, graphic_name, 3);
    ext_client_custom_graphic_single.graphic_data_struct.operate_type = 1;
    ext_client_custom_graphic_single.graphic_data_struct.graphic_type = 4;
    ext_client_custom_graphic_single.graphic_data_struct.layer = layer;
    ext_client_custom_graphic_single.graphic_data_struct.color = color;
    ext_client_custom_graphic_single.graphic_data_struct.start_angle = start_angle;
    ext_client_custom_graphic_single.graphic_data_struct.end_angle = end_angle;
    ext_client_custom_graphic_single.graphic_data_struct.width = width;
    ext_client_custom_graphic_single.graphic_data_struct.start_x = start_x;
    ext_client_custom_graphic_single.graphic_data_struct.start_y = start_y;
    ext_client_custom_graphic_single.graphic_data_struct.end_x = end_x;
    ext_client_custom_graphic_single.graphic_data_struct.end_y = end_y;
#ifdef REFEREE_SCREEN_COORD_TRANS
    msgCoordTrans(&ext_client_custom_graphic_single.graphic_data_struct);
#endif
#ifdef REFEREE_DEBUG_MODE
    ROS_INFO("Draw an arc named %c%c%c at %d layer.", graphic_name[0], graphic_name[1], graphic_name[2], layer);
#endif
    sendInteractiveData(0x0101, sender_ID, receiver_ID, &ext_client_custom_graphic_single, sizeof(ext_client_custom_graphic_single_t));
}

void RefereeSystem::drawFloat(uint16_t sender_ID, uint16_t receiver_ID, const uint8_t* graphic_name, uint32_t layer, uint32_t font_size, uint32_t decimal_digits, uint32_t start_x, uint32_t start_y, float value, uint32_t width, uint32_t color) {
    ext_client_custom_graphic_single_t ext_client_custom_graphic_single;
    memset(&ext_client_custom_graphic_single, 0, sizeof(ext_client_custom_graphic_single_t));
    memcpy(ext_client_custom_graphic_single.graphic_data_struct.graphic_name, graphic_name, 3);
    ext_client_custom_graphic_single.graphic_data_struct.operate_type = 1;
    ext_client_custom_graphic_single.graphic_data_struct.graphic_type = 5;
    ext_client_custom_graphic_single.graphic_data_struct.layer = layer;
    ext_client_custom_graphic_single.graphic_data_struct.color = color;
    ext_client_custom_graphic_single.graphic_data_struct.start_angle = font_size;
    ext_client_custom_graphic_single.graphic_data_struct.end_angle = decimal_digits;
    ext_client_custom_graphic_single.graphic_data_struct.width = width;
    ext_client_custom_graphic_single.graphic_data_struct.start_x = start_x;
    ext_client_custom_graphic_single.graphic_data_struct.start_y = start_y;
    ext_client_custom_graphic_single.graphic_data_struct.value = int32_t(1000 * value);
#ifdef REFEREE_SCREEN_COORD_TRANS
    msgCoordTrans(&ext_client_custom_graphic_single.graphic_data_struct);
#endif
#ifdef REFEREE_DEBUG_MODE
    ROS_INFO("Draw float named %c%c%c at %d layer.", graphic_name[0], graphic_name[1], graphic_name[2], layer);
#endif
    sendInteractiveData(0x0101, sender_ID, receiver_ID, &ext_client_custom_graphic_single, sizeof(ext_client_custom_graphic_single_t));
}

void RefereeSystem::drawInteger(uint16_t sender_ID, uint16_t receiver_ID, const uint8_t* graphic_name, uint32_t layer, uint32_t font_size, uint32_t start_x, uint32_t start_y, int value, uint32_t width, uint32_t color) {
    ext_client_custom_graphic_single_t ext_client_custom_graphic_single;
    memset(&ext_client_custom_graphic_single, 0, sizeof(ext_client_custom_graphic_single_t));
    memcpy(ext_client_custom_graphic_single.graphic_data_struct.graphic_name, graphic_name, 3);
    ext_client_custom_graphic_single.graphic_data_struct.operate_type = 1;
    ext_client_custom_graphic_single.graphic_data_struct.graphic_type = 6;
    ext_client_custom_graphic_single.graphic_data_struct.layer = layer;
    ext_client_custom_graphic_single.graphic_data_struct.color = color;
    ext_client_custom_graphic_single.graphic_data_struct.start_angle = font_size;
    ext_client_custom_graphic_single.graphic_data_struct.width = width;
    ext_client_custom_graphic_single.graphic_data_struct.start_x = start_x;
    ext_client_custom_graphic_single.graphic_data_struct.start_y = start_y;
    ext_client_custom_graphic_single.graphic_data_struct.value = int32_t(value);
#ifdef REFEREE_SCREEN_COORD_TRANS
    msgCoordTrans(&ext_client_custom_graphic_single.graphic_data_struct);
#endif
#ifdef REFEREE_DEBUG_MODE
    ROS_INFO("Draw integer named %c%c%c at %d layer.", graphic_name[0], graphic_name[1], graphic_name[2], layer);
#endif
    sendInteractiveData(0x0101, sender_ID, receiver_ID, &ext_client_custom_graphic_single, sizeof(ext_client_custom_graphic_single_t));
}

void RefereeSystem::drawString(uint16_t sender_ID, uint16_t receiver_ID, const uint8_t* graphic_name, uint32_t layer, uint32_t font_size, uint32_t start_x, uint32_t start_y, const uint8_t* value, uint32_t text_lenght, uint32_t width, uint32_t color) {
    if (text_lenght > 30)
        text_lenght = 30;
    ext_client_custom_character_t ext_client_custom_character;
    memset(&ext_client_custom_character, 0, sizeof(ext_client_custom_character_t));
    memcpy(ext_client_custom_character.graphic_data_struct.graphic_name, graphic_name, 3);
    ext_client_custom_character.graphic_data_struct.operate_type = 1;
    ext_client_custom_character.graphic_data_struct.graphic_type = 7;
    ext_client_custom_character.graphic_data_struct.start_angle = font_size;
    ext_client_custom_character.graphic_data_struct.end_angle = text_lenght;
    ext_client_custom_character.graphic_data_struct.width = width;
    ext_client_custom_character.graphic_data_struct.start_x = start_x;
    ext_client_custom_character.graphic_data_struct.start_y = start_y;
    memcpy(ext_client_custom_character.data, value, text_lenght);
#ifdef REFEREE_SCREEN_COORD_TRANS
    msgCoordTrans(&ext_client_custom_character.graphic_data_struct);
#endif
#ifdef REFEREE_DEBUG_MODE
    ROS_INFO("Draw string named %c%c%c at %d layer.", graphic_name[0], graphic_name[1], graphic_name[2], layer);
#endif
    sendInteractiveData(0x0110, sender_ID, receiver_ID, &ext_client_custom_character, sizeof(ext_client_custom_character_t));
}

void RefereeSystem::deleteLayer(uint16_t sender_ID, uint16_t receiver_ID, uint8_t layer) {
    ext_client_custom_graphic_delete_t ext_client_custom_graphic_delete;
    ext_client_custom_graphic_delete.operate_type = 1;
    ext_client_custom_graphic_delete.layer = layer;
#ifdef REFEREE_DEBUG_MODE
    ROS_INFO("Delete %d layer.", layer);
#endif
    sendInteractiveData(0x0100, sender_ID, receiver_ID, &ext_client_custom_graphic_delete, sizeof(ext_client_custom_graphic_delete_t));
}

void RefereeSystem::deleteAll(uint16_t sender_ID, uint16_t receiver_ID) {
    ext_client_custom_graphic_delete_t ext_client_custom_graphic_delete;
    ext_client_custom_graphic_delete.operate_type = 2;
#ifdef REFEREE_DEBUG_MODE
    ROS_INFO("Delete all layer.");
#endif
    sendInteractiveData(0x0100, sender_ID, receiver_ID, &ext_client_custom_graphic_delete, sizeof(ext_client_custom_graphic_delete_t));
}

void RefereeSystem::drawGuideLine(uint16_t sender_ID, uint16_t receiver_ID, uint8_t layer) {
    drawLine(RED_ENGINEER_ROBOT, RED_ENGINEER_CLIENT, (uint8_t*)"bd1", layer, 1047, 603, 1391, 1080, 3, 4);
    drawLine(RED_ENGINEER_ROBOT, RED_ENGINEER_CLIENT, (uint8_t*)"bd2", layer, 873, 603, 529, 1080, 3, 4);
    drawLine(RED_ENGINEER_ROBOT, RED_ENGINEER_CLIENT, (uint8_t*)"bd3", layer, 1127, 603, 1471, 1080, 3, 6);
    drawLine(RED_ENGINEER_ROBOT, RED_ENGINEER_CLIENT, (uint8_t*)"bd4", layer, 793, 603, 449, 1080, 3, 6);
    drawLine(RED_ENGINEER_ROBOT, RED_ENGINEER_CLIENT, (uint8_t*)"or1", layer, 1091, 659, 1091, 875, 3, 2);
    drawLine(RED_ENGINEER_ROBOT, RED_ENGINEER_CLIENT, (uint8_t*)"or2", layer, 829, 659, 829, 875, 3, 2);
    drawString(RED_ENGINEER_ROBOT, RED_ENGINEER_CLIENT, (uint8_t*)"sch", layer, 40, 830, 240, (uint8_t*)"WUST NB", 7, 4, 2);
    drawString(RED_ENGINEER_ROBOT, RED_ENGINEER_CLIENT, (uint8_t*)"key", layer, 16, 1540, 380, (uint8_t*)"QF\nZZ\nXXXX\nCCC\nVVVVV", 7, 2, 2);
}

#ifdef REFEREE_SCREEN_COORD_TRANS
void RefereeSystem::msgCoordTrans(graphic_data_struct_t* p_graphic_data_struct) {
    p_graphic_data_struct->start_y = STUDENT_CLIENT_HEIGHT - p_graphic_data_struct->start_y;
    // line and rectangle.
    if (p_graphic_data_struct->graphic_type == 0 || 1)
    {
        p_graphic_data_struct->end_y = STUDENT_CLIENT_HEIGHT - p_graphic_data_struct->end_y;
    }
}
#endif

void RefereeSystem::publishFrame() {
    uint8_t byte = 0;
    uint8_t sof = REF_PROTOCOL_HEADER;
    unpack_data_t p_obj{};


    while (true)
    {
        if (0 == serial.read(&byte, 1)) {
            continue;
        }
        switch (p_obj.unpack_step)
        {
        case STEP_HEADER_SOF:
        {
            if (byte == sof)
            {
                p_obj.unpack_step = STEP_LENGTH_LOW;
                p_obj.protocol_packet[p_obj.index++] = byte;
            }
            else
            {
                p_obj.index = 0;
            }
        }
        break;

        case STEP_LENGTH_LOW:
        {
#ifdef REFEREE_DEBUG_MODE
            ROS_INFO("Unpack frame, step 1.");
#endif
            p_obj.data_len = byte;
            p_obj.protocol_packet[p_obj.index++] = byte;
            p_obj.unpack_step = STEP_LENGTH_HIGH;
        }
        break;

        case STEP_LENGTH_HIGH:
        {
#ifdef REFEREE_DEBUG_MODE
            ROS_INFO("Unpack frame, step 2.");
#endif
            p_obj.data_len |= (byte << 8);
            p_obj.protocol_packet[p_obj.index++] = byte;

            if (p_obj.data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
            {
                p_obj.unpack_step = STEP_FRAME_SEQ;
            }
            else
            {
                p_obj.unpack_step = STEP_HEADER_SOF;
                p_obj.index = 0;
            }
        }
        break;

        case STEP_FRAME_SEQ:
        {
#ifdef REFEREE_DEBUG_MODE
            ROS_INFO("Unpack frame, step 3.");
#endif
            p_obj.protocol_packet[p_obj.index++] = byte;
            p_obj.unpack_step = STEP_HEADER_CRC8;
        }
        break;

        case STEP_HEADER_CRC8:
        {
#ifdef REFEREE_DEBUG_MODE
            ROS_INFO("Unpack frame, step 4.");
#endif
            p_obj.protocol_packet[p_obj.index++] = byte;

            if (p_obj.index == REF_PROTOCOL_HEADER_SIZE)
            {
                if (ref_verify_crc8(p_obj.protocol_packet, REF_PROTOCOL_HEADER_SIZE))
                {
                    p_obj.unpack_step = STEP_DATA_CRC16;
                }
                else
                {
                    p_obj.unpack_step = STEP_HEADER_SOF;
                    p_obj.index = 0;
                }
            }
        }
        break;

        case STEP_DATA_CRC16:
        {
#ifdef REFEREE_DEBUG_MODE
            ROS_INFO("Unpack frame, step 5.");
#endif
            if (p_obj.index < (REF_HEADER_CRC_CMDID_LEN + p_obj.data_len))
            {
                p_obj.protocol_packet[p_obj.index++] = byte;
            }
            if (p_obj.index >= (REF_HEADER_CRC_CMDID_LEN + p_obj.data_len))
            {
                p_obj.unpack_step = STEP_HEADER_SOF;
                p_obj.index = 0;

                if (ref_verify_crc16(p_obj.protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj.data_len))
                {
                    publishROSMessage(p_obj.protocol_packet);
                    return;
                }
            }
        }
        break;

        default:
        {
            p_obj.unpack_step = STEP_HEADER_SOF;
            p_obj.index = 0;
        }
        break;
        }
    }
}

void RefereeSystem::publishROSMessage(uint8_t* p_frame) {
    uint16_t cmd_id = *(uint16_t *)(p_frame + REF_PROTOCOL_HEADER_SIZE);
    uint8_t *data_addr = p_frame + REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CMD_SIZE;
#ifdef REFEREE_DEBUG_MODE
    ROS_INFO("Received frame, cmd_id: 0x%x.", cmd_id);
#endif
    switch (cmd_id) {
    case 0x0001:
        // 比赛状态数据
        ext_game_status_t* ext_game_status = (ext_game_status_t*)data_addr;
        common::game_state gs;
        if (ext_game_status->game_type == 1) {
            gs.game_type = "RoboMaster Competitions";
        } else if(ext_game_status->game_type == 2) {
            gs.game_type = "RoboMaster University Technical Challenge";
        } else if(ext_game_status->game_type == 3) {
            gs.game_type = "ICRA RoboMaster University AI Challenge";
        } else if(ext_game_status->game_type == 4) {
            gs.game_type = "RoboMaster University League 3V3 Confrontation";
        } else if(ext_game_status->game_type == 5) {
            gs.game_type = "RoboMaster University League Standard Confrontation";
        } else {
            gs.game_type = "Unknown";
        }
        if (ext_game_status->game_progress == 0) {
            gs.game_progress = "Pre-competition stage";
        } else if (ext_game_status->game_progress == 1) {
            gs.game_progress = "Setup period";
        } else if (ext_game_status->game_progress == 2) {
            gs.game_progress = "Initialization stage";
        } else if (ext_game_status->game_progress == 3) {
            gs.game_progress = "5-second countdown";
        } else if (ext_game_status->game_progress == 4) {
            gs.game_progress = "In combat";
        } else if (ext_game_status->game_progress == 5) {
            gs.game_progress = "calculating competition results";
        } else {
            gs.game_progress = "Unknown";
        }
        gs.stage_remain_time = ext_game_status->stage_remain_time;
        gs.SyncTimeStamp = ext_game_status->SyncTimeStamp;
        game_state_pub.publish(gs);
        break;

    }
}
