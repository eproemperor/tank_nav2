#ifndef GOBASE_NODE.H
#define GOBASE_NODE .H

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace gobase
{
    class Gobase : public BT::SyncActionNode
    {
    public:
        Gobase(const std::string &action_name,
               const BT::NodeConfiguration &conf);
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
    };
}

#endif