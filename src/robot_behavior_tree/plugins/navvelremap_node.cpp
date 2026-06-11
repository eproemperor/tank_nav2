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
    }

    void Navvelremap::setvel()
    {
        newnavvel = 1.0;
    }

    BT::NodeStatus Navvelremap::tick()
    {
        setOutput<double>("navvel", newnavvel);
        Pose2D_.x = newnavvel;
        Pose2D_.y = newnavvel;
        Pose2D_.theta = 90;
        Pos_->publish(Pose2D_);
        RCLCPP_INFO(node_->get_logger(), "速度重映射");
        return BT::NodeStatus::SUCCESS;
    }

}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::Navvelremap>("Navvelremap");
}