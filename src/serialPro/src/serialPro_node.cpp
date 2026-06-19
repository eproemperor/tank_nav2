#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "robot_serial.h"

#include <memory>
#include <queue>
#include <thread>
#include <atomic>

using namespace messageprocess;

class SerialProNode : public rclcpp::Node
{
public:
    SerialProNode() : Node("serialPro_node")
    {
        this->declare_parameter<std::string>("port", "/dev/pts/2");
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

        RCLCPP_INFO(this->get_logger(), "密码发送节点启动");
    }

    ~SerialProNode()
    {
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

            // ⭐ 发送密码片段
            if (!serial_->send_password(p1, p2))
            {
                RCLCPP_WARN(this->get_logger(), "❌ 发送失败");
                continue;
            }
            
            RCLCPP_INFO(this->get_logger(), "✅ 已发送 [%ld, %ld], 等待判题机回复...", p1, p2);
            
            // ⭐ 立即接收判题机回复（阻塞等待，带超时）
            if (serial_->receive_password())
            {
                int64_t pwd = serial_->getPassword_rec();
                stored_password_ = pwd;
                RCLCPP_INFO(this->get_logger(), "✅ 收到判题机密码: %ld", pwd);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "⚠️ 判题机无回复或超时");
            }
        }
    }

    void sendCmdCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (msg->data != 0 && stored_password_ != 0)
        {
            RCLCPP_INFO(this->get_logger(), "📤 接收发送指令, 发布密码: %ld", stored_password_);
            auto pub_msg = example_interfaces::msg::Int64();
            pub_msg.data = stored_password_;
            password_pub_->publish(pub_msg);
            // stored_password_ = 0;  // 可选：发布后清空
        }
        else if (msg->data != 0 && stored_password_ == 0)
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ 没有存储的密码");
        }
    }

    std::unique_ptr<RobotMsgProcess> serial_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr segment_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr send_cmd_sub_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr password_pub_;

    std::queue<uint64_t> segment_queue_;
    int64_t stored_password_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialProNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}