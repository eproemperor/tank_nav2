/*
帧头部分依次为帧头SOF（（0xAA）
，length（（数据部分总字长度）），令字字（判题机发
送为1，接收为0）
，都为uint64_t类型；
数据部分，发送给判题机时为两个uint64_t类型的密码片段（顺序任意），接收判题机消
息时为int64_t类型的完整密码；
帧尾部分只有一个uint64_t类型的校验0xBB。
*/
#ifndef ROBOT_MSGS.H
#define ROBOT_MSGS .H

#include <iostream>
#include <iomanip>
#include <cmath>
#include "msg_serialize.h"
#include <string>
#include <chrono> // 时间相关
#include <memory>
#include <mutex>
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp" // ROS2核心库
#include "std_msgs/msg/string.hpp"

namespace messageprocess
{
#define SUCCESS 1
#define FAIL 0

    message_data head
    {
        uint8_t header = 0xAA;
        uint8_t length = 0;
        uint8_t id = 0;
    };

    message_data password_send // 发送的密码片段
    {
        uint64_t Password1;
        uint64_t Password2;
    };

    message_data password_receive // 接收到的密码
    {
        uint64_t Password_rec;
    };

    message_data tail
    {
        uint64_t tail_msg = 0xBB;
    };

    class RobotMsgProcess : public ms::Writer<head, tail>
    {
    public:

    private:
        int receive_account = 0;
        uint64_t Password1;
        uint64_t Password1;
        uint64_t Password_rec;
    };

}

namespace Serialprocess
{
    class SerialComm
    {
    private:
        int fd;                // 串口文件描述符
        std::string port_name; // 串口设备路径
        bool is_open;

    public:
        SerialComm(const std::string &port = "/dev/pts/2");

        ~SerialComm();

        bool open();

        void close();

        bool isOpen() const { return is_open; }

        bool receive_password(); // 接收密码片段
        
        bool send_password();    // 发送密码

        bool sendFireCommand();

        std::string getLastError() const { return last_error; }

    private:
        std::string last_error;

        bool configurePort();
    };

    class SerialComman : public rclcpp::Node
    {
    public:
        SerialComman();

        ~SerialComman();

        void run();

    private:
        float angle;
        bool isshout;

        std::shared_ptr<SerialComm> serial_comm_;

        bool Serial_ok_{false};

        std::mutex mutex_;

        rclcpp::Subscription<vision_kalman_filter::msg::Serialcommand>::SharedPtr command_sub;

        void SerialCallback(const vision_kalman_filter::msg::Serialcommand::SharedPtr msg);
    };
}

#endif