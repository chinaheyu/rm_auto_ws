#include <ros/ros.h>
#include <boost/asio.hpp>
#include <string>
#include <common/serial_received_msg.h>
#include <common/serial_send_msg.h>

typedef struct
{
    uint8_t sign;
    float x_gyro;
    float y_gyro;
    float z_gyro;
    float x_accel;
    float y_accel;
    float z_accel;
}__attribute__((packed)) ReceivedMsgStruct;

typedef struct
{
    uint8_t sign;
    float vx;
    float vy;
    float vw;
    uint8_t action;
    int32_t belt;
}__attribute__((packed)) SendMsgStruct;

ReceivedMsgStruct rxbuf;
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
    common::serial_received_msg rxmsg;
    spinner.start();
    while(ros::ok())
    {
        if (boost::asio::read(sp, boost::asio::buffer(&rxbuf, sizeof(ReceivedMsgStruct))) == sizeof(ReceivedMsgStruct))
        {
            rxmsg.sign = rxbuf.sign;
            rxmsg.x_gyro = rxbuf.x_gyro;
            rxmsg.y_gyro = rxbuf.y_gyro;
            rxmsg.z_gyro = rxbuf.z_gyro;
            rxmsg.x_accel = rxbuf.x_accel;
            rxmsg.y_accel = rxbuf.y_accel;
            rxmsg.z_accel = rxbuf.z_accel;
            serial_msg_pub.publish(rxmsg);
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
