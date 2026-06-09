#include "gobase_node.h"

namespace gobase
{
    Gobase::Gobase(const std::string &action_name,
                   const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        config().blackboard->get<TypeMode>("star_info", star);
        config().blackboard->get<TypeMode>("base_info", base);
        config().blackboard->get<TypeMode>("enemy_base_info", enemy_base);
        config().blackboard->get<TypeMode>("sentry_info", sentry);
        config().blackboard->get<TypeMode>("purple_entry_info", purpleentry);
        config().blackboard->get<TypeMode>("purple_exit_info", purpleexit);
        config().blackboard->get<TypeMode>("green_entry_info", greenentry);
        config().blackboard->get<TypeMode>("green_exit_info", greenexit);
        config().blackboard->get<TypeMode>("enemy_info", enemy);
        config().blackboard->get<int>("enemy_num", enemy_num);
        config().blackboard->get<double>("sentry_hp", sentry_hp);
        config().blackboard->get<int>("sendpassmode", sendpasmode);
        config().blackboard->get<bool>("is_bullet_low", is_bullet_low);
    };

    int Gobase::settargetmode()
    {
        if (is_bullet_low || sentry_hp <= 80.0)
        {
            return TargetMode::BASE;
        }
        else if (!(enemy_num == 0))
        {
            return TargetMode::ENEMY;
        }
        else if (sendpasmode == 3 && sentry.is_out_of_center)
        {
            return TargetMode::GREENENTRY;
        }
        else if (sendpasmode == 3 && (!sentry.is_out_of_center))
        {
            return TargetMode::STAR;
        }
        else if (sendpasmode == 4 && (!sentry.is_out_of_center))
        {
            return TargetMode::GREENEXIT;
        }
        else if (sendpasmode == 4 && sentry.is_out_of_center)
        {
            return TargetMode::ENEMY_BASE;
        }
    }

    BT::NodeStatus Gobase::tick()
    {
        auto type = settargetmode();
        switch (type)
        {
        case TargetType::STAR:
            goal_x = star.p_x;
            goal_y = star.p_y;
            break;
        case TargetType::BASE:
            goal_x = base.p_x;
            goal_y = base.p_y;
            break;
        case TargetType::ENEMY_BASE:
            goal_x = enemy_base.p_x;
            goal_y = enemy_base.p_y;
            break;
        case TargetType::GREENENTRY:
            goal_x = greenentry.p_x;
            goal_y = greenentry.p_y;
            break;
        case TargetType::ENEMY:
            goal_x = enemy.p_x;
            goal_y = enemy.p_y;
            break;
        case TargetType::GREENEXIT:
            goal_x = greenexit.p_x;
            goal_y = greenexit.p_y;
            break;
        }
        setOutput("goal_x",goal_x);
        setOutput("goal_y",goal_y);
        return BT::NodeStatus::SUCCESS;
    };

}