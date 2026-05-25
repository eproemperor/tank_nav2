#include "robot_serial.h"

namespace messageprocess
{
#define BAUDRATE B115200

    RobotMsgProcess::RobotMsgProcess(const std::string &port)
        : fd(-1), port_name(port), is_open(false), last_error("")
    {
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
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 10;

        tcflush(fd, TCIOFLUSH);

        if (tcsetattr(fd, TCSANOW, &options) != 0)
        {
            return false;
        }

        return true;
    }

    bool RobotMsgProcess::SendFireCommand(double theta)
    {
    }

    bool RobotMsgProcess::SendMoveMsgs(double x, double y) {}

    int64_t RobotMsgProcess::receive_password()
    {

        return SUCCESS;
    };

    bool RobotMsgProcess::send_password(uint64_t Password1, uint64_t Password2) {
    };

}