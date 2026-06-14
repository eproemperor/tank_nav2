#include "navvelremap_node.h"

namespace nav2_behavior_tree
{
    Navvelremap::Navvelremap(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf)  
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        
        Pose2D_.x = 0.0;
        Pose2D_.y = 0.0;
        Pose2D_.theta = 0.0;
        
        last_time_ = node_->now();
        
        Pos_ = node_->create_publisher<geometry_msgs::msg::Pose2D>(
            "/pose",
            10);

        comwel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            10,
            std::bind(&Navvelremap::myCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(node_->get_logger(), "速度重映射节点已启动");
    }

    BT::NodeStatus Navvelremap::tick()
    {
        rclcpp::spin_some(node_);      
        Pos_->publish(Pose2D_);
        RCLCPP_INFO(node_->get_logger(), "速度重映射已发布");
        
        return BT::NodeStatus::SUCCESS;
    }

    void Navvelremap::myCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        rclcpp::Time current_time = node_->now();
        
        double dt = (current_time - last_time_).seconds();
        
        if (dt > 0.1) {
            dt = 0.1;
        }
        
        if (dt < 0.0001) {
            last_time_ = current_time;
            return;
        }
        
        double linear_vel = msg->linear.x;
        double angular_vel = msg->angular.z;
        
        Pose2D_.x += linear_vel * cos(Pose2D_.theta) * dt;
        Pose2D_.y += linear_vel * sin(Pose2D_.theta) * dt;
        Pose2D_.theta += angular_vel * dt;
        
        while (Pose2D_.theta > M_PI) {
            Pose2D_.theta -= 2 * M_PI;
        }
        while (Pose2D_.theta < -M_PI) {
            Pose2D_.theta += 2 * M_PI;
        }
        
        last_time_ = current_time;
    }
}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::Navvelremap>("Navvelremap");
}