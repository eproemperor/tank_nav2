#ifndef NAVVELREMAP_NODE_H
#define NAVVELREMAP_NODE_H

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include"geometry_msgs/msg/pose2_d.hpp"
#include"geometry_msgs/msg/pose.hpp"

namespace nav2_behavior_tree
{
    class Navvelremap : public BT::ConditionNode
    {
    public:
        Navvelremap(
            const std::string &condition_name,
            const BT::NodeConfiguration &conf);
        Navvelremap() = delete;
        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {BT::OutputPort<double>("navvelre", "导航速度")};
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr Pos_;
        void setvel();
        double newnavvel=100.0;
        geometry_msgs::msg::Pose2D Pose2D_;
    };
}
#endif