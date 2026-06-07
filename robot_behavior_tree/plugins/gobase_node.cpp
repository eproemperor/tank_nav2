#include "gobase_node.h"

namespace gobase
{
    Gobase::Gobase(const std::string &action_name,
                   const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    };
    
    BT::NodeStatus Gobase::tick() {

    };

}