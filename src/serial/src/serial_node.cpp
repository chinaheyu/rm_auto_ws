#include <ros/ros.h>
#include <boost/asio.hpp>
#include <string>
#include <common/serial_received_msg.h>
#include <common/serial_send_msg.h>
#include <chrono>

typedef struct
{
    uint32_t seq;
    union
    {
        struct
        {
            uint32_t sign;
            float x_gyro;
            float y_gyro;
        } frame0;

        struct
        {
            float z_gyro;
            float x_accel;
            float y_accel;
        } frame1;

        struct
        {
            float z_accel;
            float x_quat;
            float y_quat;
        } frame2;

        struct
        {
            float z_quat;
            float w_quat;
            uint32_t unused;
        } frame3;
    };
} DataFrame;

// typedef struct
// {
//     float x_gyro;
//     float y_gyro;
//     float z_gyro;
//     float x_accel;
//     float y_accel;
//     float z_accel;
//     float x_quat;
//     float y_quat;
//     float z_quat;
//     float w_quat;
//     uint8_t sign;
// }__attribute__((packed)) ReceivedMsgStruct;

typedef struct
{
    uint8_t sign;
    float vx;
    float vy;
    float vw;
    uint8_t action;
    int32_t belt;
}__attribute__((packed)) SendMsgStruct;

common::serial_received_msg rxmsg;
SendMsgStruct txbuf;

boost::asio::io_service io_s;
boost::asio::serial_port sp(io_s);

bool openSerialPort(const char* address)
{
    sp.open(address);
    if (sp.is_open())
    {
        sp.set_option(boost::asio::serial_port::baud_rate(115200));
        sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
        sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        sp.set_option(boost::asio::serial_port::character_size(8));
    }
    else
    {
        return false;
    }
    return true;
}

void sendMsgCallback(const common::serial_send_msg::ConstPtr& msg)
{
    txbuf.sign = msg->sign;
    txbuf.vx = msg->vx;
    txbuf.vy = msg->vy;
    txbuf.vw = msg->vw;
    txbuf.action = msg->action;
    txbuf.belt = msg->belt;
    boost::asio::write(sp, boost::asio::buffer(&txbuf, sizeof(SendMsgStruct)));
}

int main(int argc, char** argv) try
{
    ros::init(argc, argv, "serial");
    ros::NodeHandle n;
    ros::Publisher serial_msg_pub = n.advertise<common::serial_received_msg>("/serial_received_msg", 10);
    ros::Subscriber serial_msg_sub = n.subscribe("/serial_send_msg", 10, sendMsgCallback);
    std::string serial_port_name;
    if (n.getParam("/serial_port_name", serial_port_name))
    {
        ROS_INFO("Get serial port name successful.");
        if (openSerialPort(serial_port_name.c_str()))
        {
            ROS_INFO("Open serial port successful.");
        }
        else
        {
            ROS_INFO("Open serial port failure.");
            return 1;
        }
    }
    else
    {
        ROS_INFO("Get serial port name failure.\nPlease check if the ros param \"/serial_port_name\" is set.");
        return 1;
    }
    ros::Rate loop_rate(50);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int next_id = 0;
    int error_cnt = 0;
    while(ros::ok())
    {
        DataFrame frame_tmp;
        if (boost::asio::read(sp, boost::asio::buffer(&frame_tmp, sizeof(DataFrame))) == sizeof(DataFrame))
        {
            if (frame_tmp.seq == next_id)
            {
                switch (next_id)
                {
                case 0:
                    rxmsg.sign = (uint8_t)frame_tmp.frame0.sign;
                    rxmsg.x_gyro = frame_tmp.frame0.x_gyro;
                    rxmsg.y_gyro = frame_tmp.frame0.y_gyro;
                    break;
                case 1:
                    rxmsg.z_gyro = frame_tmp.frame1.z_gyro;
                    rxmsg.x_accel = frame_tmp.frame1.x_accel;
                    rxmsg.y_accel = frame_tmp.frame1.y_accel;
                    break;
                case 2:
                    rxmsg.z_accel = frame_tmp.frame2.z_accel;
                    rxmsg.x_quat = frame_tmp.frame2.x_quat;
                    rxmsg.y_quat = frame_tmp.frame2.y_quat;
                    break;
                case 3:
                    rxmsg.z_quat = frame_tmp.frame3.z_quat;
                    rxmsg.w_quat = frame_tmp.frame3.w_quat;
                    if (frame_tmp.frame3.unused == 0xAABB)
                    {
                        serial_msg_pub.publish(rxmsg);
                        error_cnt = 0;
                    }
                    
                    break;
                }

                next_id++;
                if (next_id > 3)
                {
                    next_id = 0;
                }
            }
            else
            {
                next_id = 0;
                error_cnt++;

                // Auto reconnect
                if (error_cnt > 10)
                {
                    ROS_INFO("Serial is connecting...");
                    sp.close();
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    if (openSerialPort(serial_port_name.c_str()))
                    {
                        ROS_INFO("Open serial port successful.");
                    }
                    else
                    {
                        ROS_INFO("Open serial port failure.");
                        return 1;
                    }
                }
            }
            
        }

        loop_rate.sleep();
    }
    spinner.stop();
    sp.close();
    return 0;

}
catch(const std::exception& e)
{
    std::cerr << e.what() << '\n';
}
