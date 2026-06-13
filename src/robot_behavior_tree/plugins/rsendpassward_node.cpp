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

        //Pos_passward = node_->create_publisher<int64_t>(
        //    "/pose",
        //    10);
        //RCLCPP_INFO(node_->get_logger(), "速度重映射");
    }

    BT::NodeStatus RSendPassward::tick()
    {

        config().blackboard->set("sendpassmode", sendpasmode);
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

    void RSendPassward::send()
    {
        switch (sendpasmode)
        {
        case 1:
            RobotMsgProcess_.receive_password();
            Password1 = RobotMsgProcess_.getPassword1();
            config().blackboard->set("Password1", Password1);
            break;
        case 2:
            RobotMsgProcess_.receive_password();
            Password2 = RobotMsgProcess_.getPassword2();
            config().blackboard->set("Password2", Password2);
            RobotMsgProcess_.send_password(Password1, Password2);
            int i = 0;
            RobotMsgProcess_.receive_password();
            Password_rec = RobotMsgProcess_.getPassword_rec();
            config().blackboard->set("Password_rec", Password_rec);
            break;

        //case 3:

            //break;
        }
    }

    void RSendPassward::calsendmode()
    {
        if (enemy_num != last_enemy_num)
        {
            sendpasmode++;
            enemy_num == last_enemy_num;
        }
        if (abs((sentry_x - star_x)) < 1.0)
        {
            sendpasmode = 3;
        }
    }
}
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::RSendPassward>("RSendPassward");
}