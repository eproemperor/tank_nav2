#include "fire_node.h"

namespace nav2_behavior_tree
{
    Fire::Fire(const std::string &action_name,
               const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf), serial_initialized_(false)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        Pos_ = node_->create_publisher<geometry_msgs::msg::Pose2D>(
            "/pose",
            10);
        Pos_angle = node_->create_publisher<geometry_msgs::msg::Pose2D>(
            "/pose_angle",
            10);
        Pos_send_ = node_->create_publisher<example_interfaces::msg::Bool>(
            "/shoot",
            10);
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
        config().blackboard->get<bool>("is_bullet_low", is_bullet_low);
        config().blackboard->get<int>("enemy_num", enemy_num);
        RCLCPP_INFO(node_->get_logger(), "攻击节点就绪");
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
    double dy = -y2 + y1; // y轴向上为正
    double angle_rad = atan2(dy, dx);

    // 归一化到 [-π, π]
    while (angle_rad > M_PI)
        angle_rad -= 2 * M_PI;
    while (angle_rad < -M_PI)
        angle_rad += 2 * M_PI;

    const double eps = 5 * M_PI / 180; // 5°容差

    double axis[4] = {0, M_PI / 2, M_PI, -M_PI / 2};
    for (int i = 0; i < 4; i++) {
        if (fabs(angle_rad - axis[i]) < eps) {
            return axis[i]; // 在坐标轴容差内，返回坐标轴值
        }
    }

    // 再检查对角线方向
    const double diag[4] = {M_PI / 4, 3 * M_PI / 4, -3 * M_PI / 4, -M_PI / 4};
    for (int i = 0; i < 4; i++) {
        if (fabs(angle_rad - diag[i]) < eps) {
            return angle_rad; // 在对角线容差内，返回原值
        }
    }

    // 最后，归类到最近的坐标轴
    int best = 0;
    double min_diff = fabs(angle_rad - axis[0]);
    for (int i = 1; i < 4; i++) {
        double diff = fabs(angle_rad - axis[i]);
        if (diff < min_diff) {
            min_diff = diff;
            best = i;
        }
    }
    return axis[best];
}

    void Fire::fire(double theta)
    {
        sleep(0.1);
        Pose2D_.theta = theta;
        Pose2D_.x = 0;
        Pose2D_.y = 0;
        sendbool.data = true;
        Pos_->publish(Pose2D_);
        Pos_angle->publish(Pose2D_);
        Pos_send_->publish(sendbool);
        Pos_send_->publish(sendbool);
        // RCLCPP_INFO(node_->get_logger(), "攻击角度: %.2f degrees", theta);
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
        config().blackboard->get<bool>("is_bullet_low", is_bullet_low);
    }

    bool Fire::calculatedistance()
    {
        if (!Fire::enemy_num == 0)
        {
            if (abs((sentry_x - enemy_x)) < 0.7 && abs((sentry_y - enemy_y)) < 0.7)
            {
                return true;
            }
        }
        else if (abs((sentry_x - enemy_base_x)) < 2 && abs((sentry_y - enemy_base_y)) < 2)
        {
            return true;
        }
        else
        {
            return FAIL;
        }
    }

    BT::NodeStatus Fire::tick()
    {
        updateposition();
        if (calculatedistance())
        {
            if (enemy_num != 0 && enemy_is_exist)
            {
                theta = calangle(sentry_x, sentry_y, enemy_x, enemy_y);
                RCLCPP_INFO(node_->get_logger(), "计算敌人位置Targeting enemy at (%.2f, %.2f), angle: %.2f",
                            enemy_x, enemy_y, theta);
            }
            else if (enemy_base_is_exist)
            {
                theta = calangle(sentry_x, sentry_y, enemy_base_x, enemy_base_y);
                // RCLCPP_INFO(node_->get_logger(), "计算基地位置Targeting enemy base at (%.2f, %.2f), angle: %.2f",
                //             enemy_base_x, enemy_base_y, theta);
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "No target available to fire");
                return BT::NodeStatus::SUCCESS;
            }

            fire(theta);
            // RCLCPP_INFO(node_->get_logger(), "攻击已发送");
        }
        return BT::NodeStatus::SUCCESS;
    }
}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::Fire>("Fire");
}