#include "navvelremap_node.h"

namespace nav2_behavior_tree
{
    Navvelremap::Navvelremap(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        Pos_ = node_->create_publisher<geometry_msgs::msg::Pose2D>(
            "/pose",
            10);
        RCLCPP_INFO(node_->get_logger(), "速度重映射");
        comwel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            1, 
        std::bind(&Navvelremap::myCallback, this, std::placeholders::_1));
    }

    void Navvelremap::setvel()
    {
        newnavvel = 1.0;
    }

    BT::NodeStatus Navvelremap::tick()
    {
        rclcpp::spin_some(node_);
        Pos_->publish(Pose2D_);
        RCLCPP_INFO(node_->get_logger(), "已发送速度");
        return BT::NodeStatus::SUCCESS;
    }

    void Navvelremap::myCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        msg->linear.x=Pose2D_.x;
        msg->linear.y=Pose2D_.y;
        msg->angular.z=Pose2D_.theta;
    }

}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::Navvelremap>("Navvelremap");
}