#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "robot_serial.h"

#include <memory>
#include <vector>
#include <thread>
#include <atomic>
#include <queue>

using namespace messageprocess;

class SerialProNode : public rclcpp::Node
{
public:
    SerialProNode() : Node("serialPro_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("port", "/dev/pts/2");
        std::string port = this->get_parameter("port").as_string();

        // 初始化串口
        serial_.reset(new RobotMsgProcess(port));
        if (!serial_->open())
        {
            RCLCPP_ERROR(this->get_logger(), "打开串口失败 %s", port.c_str());
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "串口 %s 已打开", port.c_str());

        // 订阅密码片段（自动接收存储，满2个自动发送）
        segment_sub_ = this->create_subscription<example_interfaces::msg::Int64>(
            "/password_segment", 10,
            std::bind(&SerialProNode::segmentCallback, this, std::placeholders::_1));

        // 订阅发送指令（收到指令才发布密码）
        send_cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/send_password_cmd", 10,
            std::bind(&SerialProNode::sendCmdCallback, this, std::placeholders::_1));

        // 发布完整密码
        password_pub_ = this->create_publisher<example_interfaces::msg::Int64>(
            "/password", 10);

        // 启动串口读取线程（持续接收判题机返回的密码）
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
    // 1. 密码片段回调：自动存储，满2个自动发送到串口
    void segmentCallback(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        segment_queue_.push(msg->data);
        RCLCPP_INFO(this->get_logger(), "接收片段: %ld (队列长度: %zu)", 
                     msg->data, segment_queue_.size());

        // 自动发送：只要队列中有2个片段，立即发送到串口
        while (segment_queue_.size() >= 2)
        {
            uint64_t p1 = segment_queue_.front();
            segment_queue_.pop();
            uint64_t p2 = segment_queue_.front();
            segment_queue_.pop();

            if (serial_->send_password(p1, p2))
            {
                RCLCPP_INFO(this->get_logger(), "✅ 自动发送 [%ld, %ld] 至串口", p1, p2);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "❌ 发送失败");
                // 发送失败，将片段放回队列头部
                // segment_queue_.push(p1);
                // segment_queue_.push(p2);
            }
        }
    }

    // 2. 发送指令回调：收到指令才发布存储的密码
    void sendCmdCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (msg->data != 0 && stored_password_ != 0)
        {
            RCLCPP_INFO(this->get_logger(), "📤 接收发送指令, 发送密码: %ld", 
                         stored_password_);
            
            auto pub_msg = example_interfaces::msg::Int64();
            pub_msg.data = stored_password_;
            password_pub_->publish(pub_msg);
            
            // 可以选择是否清空存储的密码（根据需求）
            // stored_password_ = 0;
        }
        else if (msg->data != 0 && stored_password_ == 0)
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ 没有存储的密码");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "无效命令");
        }
    }

    // 3. 串口读取线程：持续接收判题机返回的完整密码
    void readLoop()
    {
        while (running_ && rclcpp::ok())
        {
            if (serial_->receive_password())
            {
                int64_t pwd = serial_->getPassword_rec();
                
                stored_password_ = pwd;
                RCLCPP_INFO(this->get_logger(), "接收并存储密码: %ld 等待发送指令)", pwd);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // 成员变量
    std::unique_ptr<RobotMsgProcess> serial_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr segment_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr send_cmd_sub_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr password_pub_;

    std::queue<uint64_t> segment_queue_;  // 片段队列（支持多个片段）
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