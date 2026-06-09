#include "rsendpassward_node.h"
namespace nav2_behavior_tree
{
    RSendPassward::RSendPassward(const std::string &action_name,
                                 const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        config().blackboard->get<uint64_t>("Password1", Password1);
        config().blackboard->get<uint64_t>("Password1", Password2);
        config().blackboard->get<int64_t>("Password_rec", Password_rec);
        config().blackboard->get<int>("sendpassmode", sendpasmode);
    }

    BT::NodeStatus RSendPassward::tick()
    {
        getInput<int>("sendpassmode", sendpasmode);
        switch (sendpasmode)
        {
        case 0:
            RobotMsgProcess_.receive_password();
            Password1 = RobotMsgProcess_.getPassword1();
            setOutput("Password1", Password1);
            sendpasmode++;
            setOutput("sendpassmode", RSendPassward::sendpasmode);
            break;
        case 1:
            RobotMsgProcess_.receive_password();
            Password2 = RobotMsgProcess_.getPassword2();
            setOutput("Password2", Password2);
            sendpasmode++;
            setOutput("sendpassmode", RSendPassward::sendpasmode);
            break;
        case 2:
            RobotMsgProcess_.send_password(Password1, Password2);
            sendpasmode++;
            RobotMsgProcess_.receive_password();
            Password_rec = RobotMsgProcess_.getPassword_rec();
            setOutput("Password_rec", Password_rec);

            setOutput("sendpassmode", RSendPassward::sendpasmode);
            break;
        case 3:
            RobotMsgProcess_.send_password(Password_rec);
            sendpasmode++;
            break;
        default:
            break;
        }
    }
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::RSendPassward>("RSendPassward");
}
}
