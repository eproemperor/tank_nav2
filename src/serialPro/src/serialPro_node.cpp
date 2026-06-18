#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "robot_serial.h"

#include <memory>
#include <vector>
#include <thread>
#include <atomic>
#include <queue>
#include <sys/select.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

using namespace messageprocess;

class SerialProNode : public rclcpp::Node
{
public:
    SerialProNode() : Node("serialPro_node")
    {
        this->declare_parameter<std::string>("port", "/dev/pts/3");
        std::string port = this->get_parameter("port").as_string();

        serial_.reset(new RobotMsgProcess(port));
        if (!serial_->open())
        {
            RCLCPP_ERROR(this->get_logger(), "打开串口失败 %s", port.c_str());
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "串口 %s 已打开", port.c_str());

        segment_sub_ = this->create_subscription<example_interfaces::msg::Int64>(
            "/password_segment", 10,
            std::bind(&SerialProNode::segmentCallback, this, std::placeholders::_1));

        send_cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/send_password_cmd", 10,
            std::bind(&SerialProNode::sendCmdCallback, this, std::placeholders::_1));

        password_pub_ = this->create_publisher<example_interfaces::msg::Int64>(
            "/password", 10);

        running_ = true;
        read_thread_ = std::thread(&SerialProNode::readLoop, this);

        RCLCPP_INFO(this->get_logger(), "密码发送节点启动");
    }

    ~SerialProNode()
    {
        running_ = false;
        if (read_thread_.joinable())
            read_thread_.join();
        if (serial_)
            serial_->close();
    }

private:
    void segmentCallback(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        segment_queue_.push(msg->data);
        RCLCPP_INFO(this->get_logger(), "接收片段: %ld (队列: %zu)", msg->data, segment_queue_.size());

        while (segment_queue_.size() >= 2)
        {
            uint64_t p1 = segment_queue_.front(); segment_queue_.pop();
            uint64_t p2 = segment_queue_.front(); segment_queue_.pop();

            if (serial_->send_password(p1, p2))
            {
                RCLCPP_INFO(this->get_logger(), "✅ 自动发送 [%ld, %ld] 至串口", p1, p2);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "❌ 发送失败");
                // 可选：重新放回队列
                // segment_queue_.push(p1);
                // segment_queue_.push(p2);
            }
        }
    }

    void sendCmdCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (msg->data != 0 && stored_password_ != 0)
        {
            RCLCPP_INFO(this->get_logger(), "📤 接收发送指令, 发送密码: %ld", stored_password_);
            auto pub_msg = example_interfaces::msg::Int64();
            pub_msg.data = stored_password_;
            password_pub_->publish(pub_msg);
            // 如果需要清空，取消注释下一行
            // stored_password_ = 0;
        }
        else if (msg->data != 0 && stored_password_ == 0)
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ 没有存储的密码");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "无效命令 (data=0)");
        }
    }

    void readLoop()
    {
        fd_set read_fds;
        struct timeval timeout;

        while (running_ && rclcpp::ok())
        {
            int fd = serial_->getFd();   // 需要添加 getFd()
            if (fd < 0) break;

            FD_ZERO(&read_fds);
            FD_SET(fd, &read_fds);
            timeout.tv_sec = 0;
            timeout.tv_usec = 100000;  // 100ms

            int ret = select(fd + 1, &read_fds, nullptr, nullptr, &timeout);
            if (ret < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "select error: %s", strerror(errno));
                break;
            }
            else if (ret == 0)
            {
                continue;  // 无数据，继续循环
            }

            // 有数据可读，调用 receive_password（内部会阻塞直到读满40字节，但数据已就绪）
            if (serial_->receive_password())
            {
                int64_t pwd = serial_->getPassword_rec();
                stored_password_ = pwd;
                RCLCPP_INFO(this->get_logger(), "接收并存储密码: %ld", pwd);
            }
        }
    }

    std::unique_ptr<RobotMsgProcess> serial_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr segment_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr send_cmd_sub_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr password_pub_;

    std::queue<uint64_t> segment_queue_;
    int64_t stored_password_ = 0;
    std::atomic<bool> running_;
    std::thread read_thread_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialProNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}