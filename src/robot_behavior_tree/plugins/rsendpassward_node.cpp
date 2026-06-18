#include "rsendpassward_node.h"

namespace nav2_behavior_tree
{
    RSendPassward::RSendPassward(const std::string &action_name,
                                 const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        config().blackboard->get<uint64_t>("Password1", Password1);
        config().blackboard->get<uint64_t>("Password2", Password2);
        config().blackboard->get<int64_t>("Password_rec", Password_rec);
        config().blackboard->get<int>("sendpassmode", sendpasmode);
        config().blackboard->get<int>("enemy_num", enemy_num);
        config().blackboard->get<double>("star_x", star_x);
        config().blackboard->get<double>("star_y", star_y);
        config().blackboard->get<bool>("sentry_is_out_of_center", sentry_is_out_of_center);
        config().blackboard->get<double>("sentry_x", sentry_x);
        config().blackboard->get<double>("sentry_y", sentry_y);
        last_enemy_num = enemy_num;

        Pos_mode_ = node_->create_publisher<std_msgs::msg::Int32>(
            "/send_password_cmd",
            10);
        RCLCPP_INFO(node_->get_logger(), "RSendPassward 节点初始化完成");
    }

    BT::NodeStatus RSendPassward::tick()
    {
        updatemsg();
        if ((enemy_num == 0))
        {
            if (abs(sentry_x - star_x) < 0.5 && abs(sentry_y - star_y) < 0.5)
            {
                controlmode.data = 1;
                Pos_mode_->publish(controlmode);
                RCLCPP_INFO(node_->get_logger(), "发送控制指令");
            }
        }
        // RCLCPP_INFO(node_->get_logger(), "密码发送控制节点");
        return BT::NodeStatus::SUCCESS;
    }

    void RSendPassward::updatemsg()
    {
        config().blackboard->get<int>("sendpassmode", sendpasmode);
        config().blackboard->get<int>("enemy_num", enemy_num);
        config().blackboard->get<double>("star_x", star_x);
        config().blackboard->get<double>("star_y", star_y);
        config().blackboard->get<bool>("sentry_is_out_of_center", sentry_is_out_of_center);
        config().blackboard->get<double>("sentry_x", sentry_x);
        config().blackboard->get<double>("sentry_y", sentry_y);
    }

}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::RSendPassward>("RSendPassward");
}