#include "robot_serial.h"

namespace messageprocess
{
#define BAUDRATE B115200

    RobotMsgProcess::RobotMsgProcess(const std::string &port)
        : fd(-1), port_name(port), is_open(false),
          last_error(""), robot_listener([](const head &h)
                                         { return h.length; }, [](const head &h)
                                         { return h.id; })
    {
        RobotMsgProcess::initwriter();
    }

    RobotMsgProcess::~RobotMsgProcess()
    {
        if (is_open)
        {
            close();
        }
    }

    bool RobotMsgProcess::open()
    {
        // 打开串口设备
        fd = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == -1)
        {
            last_error = "无法打开串口设备: " + port_name;
            return false;
        }

        // 配置串口参数
        if (!configurePort())
        {
            ::close(fd);
            fd = -1;
            return false;
        }

        is_open = true;
        last_error = "";
        return true;
    }

    void RobotMsgProcess::close()
    {
        if (fd != -1)
        {
            ::close(fd);
            fd = -1;
        }
        is_open = false;
    }

    bool RobotMsgProcess::configurePort()
    {
        struct termios options;

        // 获取当前串口配置
        if (tcgetattr(fd, &options) != 0)
        {
            return false;
        }

        cfsetispeed(&options, BAUDRATE);
        cfsetospeed(&options, BAUDRATE);

        // 控制模式标志
        options.c_cflag |= (CLOCAL | CREAD); // 启用接收器，忽略调制解调器控制线
        options.c_cflag &= ~CSIZE;           // 清除数据位设置
        options.c_cflag |= CS8;              // 8位数据
        options.c_cflag &= ~PARENB;          // 无校验位
        options.c_cflag &= ~CSTOPB;          // 1位停止位
        options.c_cflag &= ~CRTSCTS;         // 禁用硬件流控制

        // 本地模式标志
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 原始输入模式

        // 输入模式标志
        options.c_iflag &= ~(IXON | IXOFF | IXANY);  // 禁用软件流控制
        options.c_iflag &= ~(INLCR | ICRNL | IGNCR); // 禁用换行符转换

        // 输出模式标志
        options.c_oflag &= ~OPOST;

        // 设置读取超时
        options.c_cc[VMIN] = 40;
        options.c_cc[VTIME] = 5;

        tcflush(fd, TCIOFLUSH);

        if (tcsetattr(fd, TCSANOW, &options) != 0)
        {
            return false;
        }

        return true;
    }

    uint64_t RobotMsgProcess::getPassword1() { return RobotMsgProcess::Password1; };

    uint64_t RobotMsgProcess::getPassword2() { return RobotMsgProcess::Password2; };

    int64_t RobotMsgProcess::getPassword_rec() { return RobotMsgProcess::Password_rec; };

    void RobotMsgProcess::ls_f(const std::string &raw_data)
    {
    }

    bool RobotMsgProcess::initwriter()
    {
        robot_writer.registerSetter([](head &h, int s)
                                    { h.length = s; });
        robot_writer.registerSetter([](tail &t, const uint8_t *data, int s)
                                    { t.tail_msg = 0xBB; });
        robot_listener.registerCallback(RECEIVED, [this](const std::string &raw_data)
                                        {
                    switch(receive_account){
                        case 0:
                    Password1 = *(uint64_t*)raw_data.data();receive_account++;break;
                        case 1:
                    Password2 = *(uint64_t*)raw_data.data();receive_account++;break;
                        case 2:
                    Password_rec = *(int64_t*)raw_data.data();receive_account++;break;
                } });
        robot_listener.registerChecker([](const head &h)
                                       {if (h.header == 0xAA) return 0; else return 1; });
        robot_listener.registerErrorHandle([](int id, const std::string &_)
                                           {if (id == 1) return; });
        return true;
    }

    bool RobotMsgProcess::SendFireCommand(double theta)
    {
        auto s = robot_writer.serialize(head{.id = SEND}, pose_move_send{0, 0, theta});
        ssize_t bytes_written = ::write(fd, s.data(), s.size());
        auto ss = robot_writer.serialize(head{.id = SEND}, shoot_msg_send{true});
        bytes_written = ::write(fd, ss.data(), ss.size());
        auto sss = robot_writer.serialize(head{.id = SEND}, shoot_msg_send{false});
        bytes_written = ::write(fd, sss.data(), sss.size());
        return true;
    }

    bool RobotMsgProcess::SendMoveMsgs(double x, double y)
    {
        auto s = robot_writer.serialize(head{.id = SEND}, pose_move_send{x, y});
        ssize_t bytes_written = ::write(fd, s.data(), s.size());
        return true;
    }

    bool RobotMsgProcess::receive_password()
    {
        uint8_t buffer[40];
        int bytes_read = 0;

        while (bytes_read < 40)
        {
            int n = ::read(fd, buffer + bytes_read, 40 - bytes_read);
            if (n > 0)
            {
                bytes_read += n;
            }
            else if (n == -1 && errno != EAGAIN)
            {
                return FAIL;
            }
        }

        robot_listener.push(reinterpret_cast<const char *>(buffer), 40);

        return true;
    };

    bool RobotMsgProcess::send_password(uint64_t Password1, uint64_t Password2)
    {
        auto s = robot_writer.serialize(head{.header = 0xAA, .id = SEND}, password_send{Password1, Password2});
        ssize_t bytes_written = ::write(fd, s.data(), s.size());
        return true;
    };

    bool RobotMsgProcess::send_password(int64_t Password_rec)
    {
        auto s = robot_writer.serialize(head{.header = 0xAA, .id = SEND}, password_send{Password_rec});
        ssize_t bytes_written = ::write(fd, s.data(), s.size());
        return true;
    };

}