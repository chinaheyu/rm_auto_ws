#ifndef REFEREE_SYSTEM_DEF_H
#define REFEREE_SYSTEM_DEF_H

#include <stdint.h>

#define REF_PROTOCOL_HEADER                 0xA5
#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#define REF_PROTOCOL_FRAME_MAX_SIZE         128
#define REF_PROTOCOL_CMD_MAX_NUM            20

#define REF_USER_TO_SERVER_MAX_DATA_LEN     64
#define REF_SERVER_TO_USER_MAX_DATA_LEN     32

typedef struct
{
    uint8_t  sof;
    uint16_t data_length;
    uint8_t  seq;
    uint8_t  crc8;
}__attribute__((packed)) frame_header_t;

typedef enum
{
    STEP_HEADER_SOF  = 0,
    STEP_LENGTH_LOW  = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ   = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct
{
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
}__attribute__((packed)) ext_student_interactive_header_data_t;

typedef struct
{
    uint8_t operate_type;
    uint8_t layer;
}__attribute__((packed)) ext_client_custom_graphic_delete_t;

typedef struct
{
    uint8_t graphic_name[3];
    uint32_t operate_type:3;
    uint32_t graphic_type:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;
    uint32_t end_angle:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    union {
        struct {
            uint32_t radius:10;
            uint32_t end_x:11;
            uint32_t end_y:11;
        };
        int32_t value:32;
    };
}__attribute__((packed)) graphic_data_struct_t;

typedef struct
{
    graphic_data_struct_t graphic_data_struct;
}__attribute__((packed)) ext_client_custom_graphic_single_t;

typedef struct
{
    graphic_data_struct_t graphic_data_struct[2];
}__attribute__((packed)) ext_client_custom_graphic_double_t;

typedef struct
{
    graphic_data_struct_t graphic_data_struct[5];
}__attribute__((packed)) ext_client_custom_graphic_five_t;

typedef struct
{
    graphic_data_struct_t graphic_data_struct[7];
}__attribute__((packed)) ext_client_custom_graphic_seven_t;

typedef struct
{
    graphic_data_struct_t graphic_data_struct;
    uint8_t data[30];
}__attribute__((packed)) ext_client_custom_character_t;

typedef struct
{
    frame_header_t *p_header;
    uint16_t       data_len;
    uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
    unpack_step_e  unpack_step;
    uint16_t       index;
} unpack_data_t;

typedef struct
{
    uint8_t game_type:4;
    uint8_t game_progress:4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
}__attribute__((packed)) ext_game_status_t;


// Robot type define.
#define RED_HERO_ROBOT 1
#define RED_ENGINEER_ROBOT 2
#define RED_STANDARD_ROBOT_1 3
#define RED_STANDARD_ROBOT_2 4
#define RED_STANDARD_ROBOT_3 5
#define RED_AERIAL_ROBOT 6

#define BLUE_HERO_ROBOT 101
#define BLUE_ENGINEER_ROBOT 102
#define BLUE_STANDARD_ROBOT_1 103
#define BLUE_STANDARD_ROBOT_2 104
#define BLUE_STANDARD_ROBOT_3 105
#define BLUE_AERIAL_ROBOT 106


// Student client define.
#define RED_HERO_CLIENT 0x0101
#define RED_ENGINEER_CLIENT 0x0102
#define RED_STANDARD_CLIENT_1 0x0103
#define RED_STANDARD_CLIENT_2 0x0104
#define RED_STANDARD_CLIENT_3 0x0105
#define RED_AERIAL_CLIENT 0x0106

#define BLUE_HERO_CLIENT 0x0165
#define BLUE_ENGINEER_CLIENT 0x0166
#define BLUE_STANDARD_CLIENT_1 0x0167
#define BLUE_STANDARD_CLIENT_2 0x0168
#define BLUE_STANDARD_CLIENT_3 0x0169
#define BLUE_AERIAL_CLIENT 0x016A

#endif