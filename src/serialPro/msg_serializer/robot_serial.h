/*
1.哨兵每击毁一辆敌方步兵会收到一个密码片段（立即发送且只发送一次）话题/password_segment  example_interfaces::msg::Int64
2.需要将两个密码片段按一定格式通过虚拟串口发送至判题机，
3.若消息正确则判题机会发送正确密码回来，串口
4.前往中央蓝色区域发送密码到指定话题 /password  example_interfaces::msg::Int64
5.规定计数判断密码模式，接收话题/sendmode,int类型数据

帧头部分依次为帧头SOF（（0xAA）
，length（（数据部分总字长度）），令字字（判题机发
送为1，接收为0）
，都为uint64_t类型；
数据部分，发送给判题机时为两个uint64_t类型的密码片段（顺序任意），接收判题机消
息时为int64_t类型的完整密码；
帧尾部分只有一个uint64_t类型的校验0xBB。
*/
#ifndef ROBOT_SERIAL_H
#define ROBOT_SERIAL_H

#include <iostream>
#include <iomanip>
#include <cmath>
#include "msg_serialize.h"
#include <string>
#include <chrono>
#include <memory>
#include <mutex>
#include <functional>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

namespace messageprocess
{
#define FAIL 0
#define SHOOT 1
#define UNSHOOT 0
#define SEND 0          // 本机发送给判题机
#define RECEIVED 1      // 本机从判题机接收

    message_data head
    {
        uint64_t header = 0xAA;
        uint64_t length = 0;
        uint64_t id = 0;
    };

    message_data password_send // 发送的密码片段（两个uint64_t）
    {
        uint64_t Password1;
        uint64_t Password2;
    };

    message_data password_receive // 接收到的完整密码
    {
        int64_t Password_rec;
    };

    message_data tail
    {
        uint64_t tail_msg = 0xBB;
    };

    class RobotMsgProcess
    {
    private:
        int64_t Password_rec;   // 接收到的完整密码
        int fd;                 // 串口文件描述符
        std::string port_name;
        std::string last_error;
        bool is_open;

        bool configurePort();
        bool initwriter();

    public:
        RobotMsgProcess(const std::string &port = "/dev/pts/3");
        ~RobotMsgProcess();

        bool open();
        void close();
        bool isOpen() const { return is_open; }
        int getFd() const { return fd; }

        int64_t getPassword_rec() const { return Password_rec; }

        /*
         * @brief 接收判题机返回的完整密码（阻塞读取）
         * @return true 接收成功，false 失败
         */
        bool receive_password();

        /*
         * @brief 向判题机发送两个密码片段
         * @param Password1 片段1
         * @param Password2 片段2
         * @return true 发送成功，false 失败
         */
        bool send_password(uint64_t Password1, uint64_t Password2);

    private:
        ms::Writer<head, tail> robot_writer;
        ms::Listener<head, tail> robot_listener;
    };
}

#endif