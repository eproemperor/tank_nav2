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

        // 打开串口
        if (!RobotMsgProcess_.open())
        {
            RCLCPP_ERROR(node_->get_logger(), "无法打开串口 /dev/pts/2，请检查设备路径");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "串口打开成功");
        }

        Pos_passward = node_->create_publisher<example_interfaces::msg::Int64>(
            "/password",
            10);
        RCLCPP_INFO(node_->get_logger(), "RSendPassward 节点初始化完成");
    }

    BT::NodeStatus RSendPassward::tick()
    {
        updatemsg();
        calsendmode();
        send();
        RCLCPP_DEBUG(node_->get_logger(), "当前模式: %d, 敌方数量: %d", sendpasmode, enemy_num);
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

    void RSendPassward::send()
    {
        switch (sendpasmode)
        {
        case 1:
            if (!password1_received_)
            {
                if (!RobotMsgProcess_.isOpen())
                {
                    RCLCPP_WARN(node_->get_logger(), "串口未打开，无法接收密码1");
                    break;
                }
                if (RobotMsgProcess_.receive_password())
                {
                    Password1 = RobotMsgProcess_.getPassword1();
                    config().blackboard->set("Password1", Password1);
                    password1_received_ = true;
                    RCLCPP_INFO(node_->get_logger(), "接收密码1: %lu", Password1);
                }
                else
                {
                    RCLCPP_WARN(node_->get_logger(), "接收密码1失败");
                }
            }
            break;

        case 2:
            if (!password2_sent_)
            {
                if (!RobotMsgProcess_.isOpen())
                {
                    RCLCPP_WARN(node_->get_logger(), "串口未打开，无法进行密码2收发");
                    break;
                }
                if (RobotMsgProcess_.receive_password())
                {
                    Password2 = RobotMsgProcess_.getPassword2();
                    config().blackboard->set("Password2", Password2);
                    RCLCPP_INFO(node_->get_logger(), "接收密码2: %lu", Password2);

                    if (RobotMsgProcess_.send_password(Password1, Password2))
                    {
                        RCLCPP_INFO(node_->get_logger(), "发送密码片段成功");
                    }
                    else
                    {
                        RCLCPP_WARN(node_->get_logger(), "发送密码片段失败");
                        break;
                    }

                    if (RobotMsgProcess_.receive_password())
                    {
                        Password_rec = RobotMsgProcess_.getPassword_rec();
                        config().blackboard->set("Password_rec", Password_rec);
                        password2_sent_ = true;
                        sendpasmode = 3;   // 收到完整密码后自动进入发布模式
                        RCLCPP_INFO(node_->get_logger(), "接收完整密码: %ld", Password_rec);
                    }
                    else
                    {
                        RCLCPP_WARN(node_->get_logger(), "接收完整密码失败");
                    }
                }
                else
                {
                    RCLCPP_WARN(node_->get_logger(), "接收密码2失败");
                }
            }
            break;

        case 3:
            if (Password_rec != 0)
            {
                example_interfaces::msg::Int64 msg;
                msg.data = Password_rec;
                Pos_passward->publish(msg);
                RCLCPP_INFO(node_->get_logger(), "发布完整密码: %ld", Password_rec);
                sendpasmode = 4;   // 发布后进入完成状态
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "完整密码无效，无法发布");
            }
            break;

        default:
            RCLCPP_DEBUG(node_->get_logger(), "当前模式 %d，无串口操作", sendpasmode);
            break;
        }
    }

    void RSendPassward::calsendmode()
    {
        // 敌方数量变化时递增模式（最多到3）
        if (enemy_num != last_enemy_num)
        {
            if (sendpasmode < 3)
                sendpasmode++;
            last_enemy_num = enemy_num;
        }

        // 到达中央区域且已收到完整密码 -> 强制切换到发布模式（若尚未发布）
        if (std::abs(sentry_x - star_x) < 1.0 && std::abs(sentry_y - star_y) < 1.0&&!sentry_is_out_of_center)
        {
            if (password2_sent_ && sendpasmode != 3 && sendpasmode != 4)
            {
                sendpasmode = 3;
                RCLCPP_INFO(node_->get_logger(), "哨兵到达中央区域，切换到发布模式");
            }
            else if (!password2_sent_)
            {
                RCLCPP_WARN(node_->get_logger(), "到达中央区域但尚未收到完整密码，保持当前模式");
            }
        }

        // 边界保护
        if (sendpasmode > 4) sendpasmode = 4;
        if (sendpasmode < 0) sendpasmode = 0;
    }
}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::RSendPassward>("RSendPassward");
}