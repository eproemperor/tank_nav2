#include "fire_node.h"

namespace nav2_behavior_tree
{
    Fire::Fire(const std::string &action_name,
               const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf)
        , serial_initialized_(false)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        config().blackboard->get<int>("sentry_name", sentry_name);
        config().blackboard->get<bool>("sentry_is_exist", sentry_is_exist);
        config().blackboard->get<double>("sentry_x", sentry_x);
        config().blackboard->get<double>("sentry_y", sentry_y);

        config().blackboard->get<int>("enemy_base_name", enemy_base_name);
        config().blackboard->get<bool>("enemy_base_is_exist", enemy_base_is_exist);
        config().blackboard->get<double>("enemy_base_x", enemy_base_x);
        config().blackboard->get<double>("enemy_base_y", enemy_base_y);

        config().blackboard->get<int>("enemy_name", enemy_name);
        config().blackboard->get<bool>("enemy_is_exist", enemy_is_exist);
        config().blackboard->get<double>("enemy_x", enemy_x);
        config().blackboard->get<double>("enemy_y", enemy_y);

        config().blackboard->get<int>("enemy_num", enemy_num);
        
        std::string serial_port = "/dev/pts/2";  
        int baudrate = 115200;  
        
        if (RobotMsgProcess_.open())
        {
            serial_initialized_ = true;
            RCLCPP_INFO(node_->get_logger(), "Serial port opened successfully: %s", serial_port.c_str());
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to open serial port: %s", serial_port.c_str());
        }
    }

    Fire::~Fire()
    {
        if (serial_initialized_)
        {
            RobotMsgProcess_.close();
        }
    }

    double Fire::calangle(double &x1, double &y1, double &x2, double &y2)
    {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double angle_rad = atan2(dy, dx);
        double angle_deg = angle_rad * 180.0 / M_PI;

        if (angle_deg < 0)
        {
            angle_deg += 360.0;
        }
        if ((angle_deg >= 0 && angle_deg < 45) || (angle_deg >= 315 && angle_deg < 360))
        {
            angle_deg = 0;   // 右
        }
        else if (angle_deg >= 45 && angle_deg < 135)
        {
            angle_deg = 90;  // 上
        }
        else if (angle_deg >= 135 && angle_deg < 225)
        {
            angle_deg = 180; // 左
        }
        else if (angle_deg >= 225 && angle_deg < 315)
        {
            angle_deg = 270; // 下
        }
        
        return angle_deg;
    }

    void Fire::fire(double theta)
    {
        RobotMsgProcess_.SendFireCommand(theta);
        RCLCPP_INFO(node_->get_logger(), "Firing with angle: %.2f degrees", theta);
    }

    void Fire::updateposition()
    {
        config().blackboard->get<bool>("sentry_is_exist", sentry_is_exist);
        config().blackboard->get<double>("sentry_x", sentry_x);
        config().blackboard->get<double>("sentry_y", sentry_y);

        config().blackboard->get<bool>("enemy_base_is_exist", enemy_base_is_exist);
        config().blackboard->get<double>("enemy_base_x", enemy_base_x);
        config().blackboard->get<double>("enemy_base_y", enemy_base_y);

        config().blackboard->get<bool>("enemy_is_exist", enemy_is_exist);
        config().blackboard->get<double>("enemy_x", enemy_x);
        config().blackboard->get<double>("enemy_y", enemy_y);

        config().blackboard->get<int>("enemy_num", enemy_num);
    }

    BT::NodeStatus Fire::tick()
    {
        if (!serial_initialized_)
        {
            RCLCPP_ERROR(node_->get_logger(), "Serial port not initialized");
            return BT::NodeStatus::FAILURE;
        }
        
        updateposition();

        if (enemy_num != 0 && enemy_is_exist)
        {
            theta = calangle(sentry_x, sentry_y, enemy_x, enemy_y);
            RCLCPP_INFO(node_->get_logger(), "Targeting enemy at (%.2f, %.2f), angle: %.2f", 
                        enemy_x, enemy_y, theta);
        }
        else if (enemy_base_is_exist)
        {
            theta = calangle(sentry_x, sentry_y, enemy_base_x, enemy_base_y);
            RCLCPP_INFO(node_->get_logger(), "Targeting enemy base at (%.2f, %.2f), angle: %.2f", 
                        enemy_base_x, enemy_base_y, theta);
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "No target available to fire");
            return BT::NodeStatus::FAILURE;
        }

        fire(theta);
        return BT::NodeStatus::SUCCESS;
    }
}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::Fire>("Fire");
}