#include "robot_serial.h"

namespace messageprocess
{
#define BAUDRATE B115200

    RobotMsgProcess::RobotMsgProcess(const std::string &port)
        : fd(-1), port_name(port), is_open(false),
          last_error(""),
          robot_listener([](const head &h) { return h.length; },
                         [](const head &h) { return h.id; })
    {
        initwriter();
    }

    RobotMsgProcess::~RobotMsgProcess()
    {
        if (is_open) close();
    }

    bool RobotMsgProcess::open()
    {
        fd = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == -1)
        {
            last_error = "无法打开串口设备: " + port_name;
            return false;
        }
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
        if (tcgetattr(fd, &options) != 0) return false;

        cfsetispeed(&options, BAUDRATE);
        cfsetospeed(&options, BAUDRATE);

        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CRTSCTS;

        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
        options.c_oflag &= ~OPOST;

        options.c_cc[VMIN] = 40;
        options.c_cc[VTIME] = 5;

        tcflush(fd, TCIOFLUSH);
        if (tcsetattr(fd, TCSANOW, &options) != 0) return false;
        return true;
    }

    bool RobotMsgProcess::initwriter()
    {
        robot_writer.registerSetter([](head &h, int s) { h.length = s; });
        robot_writer.registerSetter([](tail &t, const uint8_t *data, int s)
                                     { t.tail_msg = 0xBB; });

        // 接收回调：判题机返回完整密码（id == RECEIVED）
        robot_listener.registerCallback(RECEIVED, [this](const std::string &raw_data)
        {
            if (raw_data.size() >= sizeof(int64_t))
            {
                Password_rec = *reinterpret_cast<const int64_t*>(raw_data.data());
            }
        });

        robot_listener.registerChecker([](const head &h)
        {
            return (h.header == 0xAA) ? 0 : 1;
        });

        robot_listener.registerErrorHandle([](int id, const std::string &_)
        {
           //错误日志
        });

        return true;
    }

    bool RobotMsgProcess::receive_password()
{
    uint8_t buffer[40];
    int bytes_read = 0;
    while (bytes_read < 40)
    {
        int n = ::read(fd, buffer + bytes_read, 40 - bytes_read);
        if (n > 0) bytes_read += n;
        else if (n == -1 && errno != EAGAIN) return false;
    }

    // 解析帧
    uint64_t sof = *reinterpret_cast<uint64_t*>(buffer + 0);
    uint64_t id = *reinterpret_cast<uint64_t*>(buffer + 16);
    int64_t password = *reinterpret_cast<int64_t*>(buffer + 24);  
    uint64_t tail = *reinterpret_cast<uint64_t*>(buffer + 32);

    if (sof == 0xAA && id == RECEIVED && tail == 0xBB)
    {
        Password_rec = password;  // 存储 -2320256767085552308
        return true;
    }
    return false;
}

    bool RobotMsgProcess::send_password(uint64_t Password1, uint64_t Password2)
    {
        auto s = robot_writer.serialize(
            head{.header = 0xAA, .id = SEND},
            password_send{Password1, Password2}
        );
        ssize_t written = ::write(fd, s.data(), s.size());
        return (written == static_cast<ssize_t>(s.size()));
    }
}