#include "navvelremap_node.h"

namespace nav2_behavior_tree
{
    Navvelremap::Navvelremap(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        RCLCPP_INFO(node_->get_logger(), "速度重映射");
    }

    void Navvelremap::setvel()
    {
        newnavvel = 50.0;
    }

    BT::NodeStatus Navvelremap::tick()
    {
        setOutput<double>("navvel", newnavvel);
    }

    BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::Navvelremap>("Navvelremap");
}
}