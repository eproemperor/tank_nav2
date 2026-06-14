#ifndef NAVVELREMAP_NODE_H
#define NAVVELREMAP_NODE_H

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include "nav2_core/controller.hpp"
#include <cmath> 

namespace nav2_behavior_tree
{
    class Navvelremap : public BT::SyncActionNode
    {
    public:
        Navvelremap(
            const std::string &action_name,  
            const BT::NodeConfiguration &conf);
        Navvelremap() = delete;
        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {BT::OutputPort<geometry_msgs::msg::Pose2D>("pose", "当前位姿")};
        }

    private:
        void myCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr Pos_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr comwel_sub_;
        
        geometry_msgs::msg::Pose2D Pose2D_;
        rclcpp::Time last_time_;  
    };
}

#endif