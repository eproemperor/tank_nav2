#include "ifgamestart_node.h"

namespace nav2_behavior_tree
{

    IfGameStart::IfGameStart(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf),
          startflag(false)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        config().blackboard->get<bool>("startflag", startflag);
        rclcpp::QoS map_qos(10);
        map_qos.transient_local();
        map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            map_qos,
            std::bind(&IfGameStart::mapCallback, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(),"Game start detected /map topic");
    }

    BT::NodeStatus IfGameStart::tick()
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        if (startflag == true)
        {
            config().blackboard->set("startflag", startflag);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        };
    }

    void IfGameStart::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        (void)msg;
        startflag = true;
    }

    BT_REGISTER_NODES(factory)
    {
        factory.registerNodeType<nav2_behavior_tree::IfGameStart>("IfGameStart");
    }
}