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
#include <fcntl.h>   // 文件控制定义
#include <unistd.h>  // UNIX标准函数定义
#include <termios.h> // 终端I/O控制定义
#include <cstring>   // 字符串操作
#include <iostream>  // 错误输出

namespace messageprocess
{
#define SUCCESS 1
#define FAIL 0
#define SHOOT 1
#define UNSHOOT 0
#define SEND 0
#define RECEIVED #define
#define UP 90.0;
#define LEFT 180.0;
#define DOWN 270.0;
#define RIGHT 0.0;

    message_data head
    {
        uint64_t header = 0xAA;
        uint64_t length = 0;
        uint64_t id = 0;
    };

    message_data password_send // 发送的密码片段
    {
        uint64_t Password1;
        uint64_t Password2;
    };

    message_data password_receive // 接收到的密码
    {
        int64_t Password_rec;
    };

    message_data pose_move_send // 位置姿态移动
    {
        double pose_x;
        double pose_y;
        double theta;
    };

    message_data shoot_msg_send
    {
        bool isshout;
    };

    message_data tail
    {
        uint64_t tail_msg = 0xBB;
    };


    class RobotMsgProcess : public ms::Writer<head, tail>
    {

    private:
        int receive_account = 0;
        uint64_t Password1;
        uint64_t Password2;
        int64_t Password_rec;
        double pose_x;
        double pose_y;
        double theta;

        int fd;                // 串口文件描述符
        std::string port_name; // 串口设备路径
        std::string last_error;
        bool is_open;
        bool configurePort();

    public:
        RobotMsgProcess(const std::string &port = "/dev/pts/2");

        ~RobotMsgProcess();

        bool open();

        void close();

        bool isOpen() const { return is_open; }

        /*
         *@brief 接收密码片段
         *
         */
        int64_t receive_password();

        /*
         *@brief 发送密码
         *
         *  @param Password1 密码片段1
         *  @param Password2 密码片段2;
         */
        bool send_password(uint64_t Password1, uint64_t Password2);

        /*
         *@brief 发送移动指令
         *
         * @param pose_x;
         * @param pose_y;
         */
        bool SendMoveMsgs(double x, double y);

        /*
         *@brief 发送开火指令
         *
         *@param theta
         */
        bool SendFireCommand(double theta);
    };

    // 序列化数据-创建对象
    ms::Writer<head, tail> robot_writer;
    

}

#endif