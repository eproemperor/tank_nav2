#ifndef RSENDPASSWORD_NODE_H_
#define RSENDPASSWORD_NODE_H_

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "robot_serial.h"

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
        uint64_t Password1;
        uint64_t Password2;
        int64_t Password_rec;
        int sendpasmode;
        rclcpp::Node::SharedPtr node_;
        messageprocess::RobotMsgProcess RobotMsgProcess_;
    };
}

#endif  // RSENDPASSWORD_NODE_H_