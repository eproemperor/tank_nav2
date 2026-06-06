#include "rsendpassward_node.h"
namespace rsendpassward
{
    RSendPassward::RSendPassward(const std::string &action_name,
                                 const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf) {

        }

    BT::NodeStatus RSendPassward::tick()
    {
        getInput<int>("sendpassmode",sendpasmode);
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
            break;
        default:
            break;
        }
    }

}
