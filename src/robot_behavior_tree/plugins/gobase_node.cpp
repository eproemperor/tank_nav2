#include "gobase_node.h"

namespace nav2_behavior_tree
{
    Gobase::Gobase(const std::string &action_name,
                   const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        updateBlackboardData();
    };

    void Gobase::updateBlackboardData()
    {
        // 星星信息
        config().blackboard->get<int>("star_name", star_name);
        config().blackboard->get<bool>("star_is_exist", star_is_exist);
        config().blackboard->get<double>("star_x", star_x);
        config().blackboard->get<double>("star_y", star_y);

        // 基地信息
        config().blackboard->get<int>("base_name", base_name);
        config().blackboard->get<bool>("base_is_exist", base_is_exist);
        config().blackboard->get<double>("base_x", base_x);
        config().blackboard->get<double>("base_y", base_y);

        // 敌方基地信息
        config().blackboard->get<int>("enemy_base_name", enemy_base_name);
        config().blackboard->get<bool>("enemy_base_is_exist", enemy_base_is_exist);
        config().blackboard->get<double>("enemy_base_x", enemy_base_x);
        config().blackboard->get<double>("enemy_base_y", enemy_base_y);

        // 哨兵信息
        config().blackboard->get<int>("sentry_name", sentry_name);
        config().blackboard->get<bool>("sentry_is_exist", sentry_is_exist);
        config().blackboard->get<bool>("sentry_is_out_of_center", sentry_is_out_of_center);
        config().blackboard->get<double>("sentry_x", sentry_x);
        config().blackboard->get<double>("sentry_y", sentry_y);

        // 紫色入口信息
        config().blackboard->get<int>("purple_entry_name", purpleentry_name);
        config().blackboard->get<bool>("purple_entry_is_exist", purpleentry_is_exist);
        config().blackboard->get<double>("purple_entry_x", purpleentry_x);
        config().blackboard->get<double>("purple_entry_y", purpleentry_y);

        // 紫色出口信息
        config().blackboard->get<int>("purple_exit_name", purpleexit_name);
        config().blackboard->get<bool>("purple_exit_is_exist", purpleexit_is_exist);
        config().blackboard->get<double>("purple_exit_x", purpleexit_x);
        config().blackboard->get<double>("purple_exit_y", purpleexit_y);

        // 绿色入口信息
        config().blackboard->get<int>("green_entry_name", greenentry_name);
        config().blackboard->get<bool>("green_entry_is_exist", greenentry_is_exist);
        config().blackboard->get<double>("green_entry_x", greenentry_x);
        config().blackboard->get<double>("green_entry_y", greenentry_y);

        // 绿色出口信息
        config().blackboard->get<int>("green_exit_name", greenexit_name);
        config().blackboard->get<bool>("green_exit_is_exist", greenexit_is_exist);
        config().blackboard->get<double>("green_exit_x", greenexit_x);
        config().blackboard->get<double>("green_exit_y", greenexit_y);

        // 敌方单位信息
        config().blackboard->get<int>("enemy_name", enemy_name);
        config().blackboard->get<bool>("enemy_is_exist", enemy_is_exist);
        config().blackboard->get<double>("enemy_x", enemy_x);
        config().blackboard->get<double>("enemy_y", enemy_y);

        // 其他数据
        config().blackboard->get<int>("enemy_num", enemy_num);
        config().blackboard->get<double>("sentry_hp", sentry_hp);
        config().blackboard->get<int>("sendpasmode", sendpasmode);
        config().blackboard->get<bool>("is_bullet_low", is_bullet_low);
    }

    int Gobase::setTargetType()
    {
        if (is_bullet_low || sentry_hp <= 80.0)
        {
            return TargetType::BASE;
        }
        else if (enemy_num != 0 && enemy_is_exist)
        {
            if (abs(sentry_x - enemy_x) > 0.5 && abs(sentry_y - enemy_y) > 0.5)
            {
                return TargetType::ENEMY;
            }
        }
        else if (sendpasmode == 3 && sentry_is_exist && sentry_is_out_of_center)
        {
            return TargetType::GREENENTRY;
        }
        else if (sendpasmode == 3 && sentry_is_exist && !sentry_is_out_of_center)
        {
            return TargetType::STAR;
        }
        else if (sendpasmode == 4 && sentry_is_exist && !sentry_is_out_of_center)
        {
            return TargetType::GREENEXIT;
        }
        else if (sendpasmode == 4 && sentry_is_exist && sentry_is_out_of_center)
        {
            return TargetType::ENEMY_BASE;
        }
        return TargetType::SENTRY;
    }

    BT::NodeStatus Gobase::tick()
    {
        // 更新黑板数据
        updateBlackboardData();

        auto type = setTargetType();
        switch (type)
        {
        case TargetType::STAR:
            goal_.pose.position.x = star_x;
            goal_.pose.position.y = star_y;
            break;
        case TargetType::BASE:
            goal_.pose.position.x = base_x;
            goal_.pose.position.y = base_y;
            break;
        case TargetType::ENEMY_BASE:
            goal_.pose.position.x = enemy_base_x;
            goal_.pose.position.y = enemy_base_y;
            break;
        case TargetType::GREENENTRY:
            goal_.pose.position.x = greenentry_x;
            goal_.pose.position.y = greenentry_y;
            break;
        case TargetType::ENEMY:
            goal_.pose.position.x = enemy_x;
            goal_.pose.position.y = enemy_y;
            break;
        case TargetType::GREENEXIT:
            goal_.pose.position.x = greenexit_x;
            goal_.pose.position.y = greenexit_y;
            break;
        default:
            goal_.pose.position.x = sentry_x;
            goal_.pose.position.y = sentry_y;
            break;
        }
        goal_.header.frame_id = "map";
        goal_.header.stamp = node_->now();
        config().blackboard->set("goal", goal_);
        RCLCPP_INFO(node_->get_logger(), "GoBase: target type=%d, goal=(%.2f, %.2f)", type, goal_.pose.position.x, goal_.pose.position.y);

        return BT::NodeStatus::SUCCESS;
    };

}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::Gobase>("GoBase");
}