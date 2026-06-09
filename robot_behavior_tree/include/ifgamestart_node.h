#ifndef IFGAMESTART_NODE_H
#define IFGAMESTART_NODE_H

/*
    游戏是否开始  IfGameStart
    bt树的condition节点，用于判断游戏是否开始，这里采用ros口有无数据
*/
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_behavior_tree
{
    class IfGameStart : public BT::ConditionNode
    {
    public:
        IfGameStart(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);
        IfGameStart() = delete;
        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {BT::OutputPort<bool>("startflag", "游戏开始标志")};
        }

    private:
        bool
        check_condition()
        {
            return true;
        }
        bool startflag{false};

        std::mutex data_mutex;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    };
}
#endif