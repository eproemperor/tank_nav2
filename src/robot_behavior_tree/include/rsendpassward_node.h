#ifndef RSENDPASSWORD_NODE_H_
#define RSENDPASSWORD_NODE_H_

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "robot_serial.h"
#include "math.h"
#include "example_interfaces/msg/int64.hpp"
#include "std_msgs/msg/int32.hpp"

namespace nav2_behavior_tree
{
    class RSendPassward : public BT::SyncActionNode
    {
    public:
        RSendPassward(const std::string &action_name,
                      const BT::NodeConfiguration &conf);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int>("sendpassmode", "发送密码模式 (0-3)"),
                BT::OutputPort<uint64_t>("Password1", "密码片段1"),
                BT::OutputPort<uint64_t>("Password2", "密码片段2"),
                BT::OutputPort<int64_t>("Password_rec", "最终密码"),
            };
        }

        BT::NodeStatus tick() override;

    private:
        // 密码数据
        uint64_t Password1;
        uint64_t Password2;
        int64_t Password_rec;

        // 模式控制
        int sendpasmode = 0;
        bool password1_received_ = false;   // 是否已接收片段1
        bool password2_sent_ = false;       // 是否已发送片段并收到完整密码

        // 黑板数据
        int enemy_num;
        int last_enemy_num = 2;
        double star_x;
        double star_y;
        bool sentry_is_out_of_center;
        double sentry_x;
        double sentry_y;

        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr Pos_mode_;

        messageprocess::RobotMsgProcess RobotMsgProcess_;
        std_msgs::msg::Int32 controlmode;

        void updatemsg();
    };
}

#endif