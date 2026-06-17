#ifndef FIRE_NODE_H
#define FIRE_NODE_H

#include "robot_serial.h"
#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include"example_interfaces/msg/bool.hpp"

namespace nav2_behavior_tree
{

    class Fire : public BT::SyncActionNode
    {
    public:
        Fire(const std::string &action_name,
             const BT::NodeConfiguration &conf);
        ~Fire();

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<bool>("is_bullet_low", "子弹是否不足"),
                BT::InputPort<int>("enemy_base_name", "敌方基地类型"),
                BT::InputPort<bool>("enemy_base_is_exist", "敌方基地是否存在"),
                BT::InputPort<double>("enemy_base_x", "敌方基地X坐标"),
                BT::InputPort<double>("enemy_base_y", "敌方基地Y坐标"),
                BT::InputPort<int>("sentry_name", "哨兵类型"),
                BT::InputPort<bool>("sentry_is_exist", "哨兵是否存在"),
                BT::InputPort<double>("sentry_x", "哨兵X坐标"),
                BT::InputPort<double>("sentry_y", "哨兵Y坐标"),
                BT::InputPort<int>("enemy_name", "敌方类型"),
                BT::InputPort<bool>("enemy_is_exist", "敌方是否存在"),
                BT::InputPort<double>("enemy_x", "敌方X坐标"),
                BT::InputPort<double>("enemy_y", "敌方Y坐标"),
                BT::InputPort<int>("enemy_num", "敌人数量")};
        }

    private:
        void fire(double theta);
        void updateposition();
        double calangle(double &x1, double &y1, double &x2, double &y2);
        bool calculatedistance();

        int enemy_num;
        double theta;

        int enemy_base_name;
        bool enemy_base_is_exist;
        double enemy_base_x;
        double enemy_base_y;

        int sentry_name;
        bool sentry_is_exist;
        double sentry_x;
        double sentry_y;

        int enemy_name;
        bool enemy_is_exist;
        double enemy_x;
        double enemy_y;
        bool is_bullet_low;

        rclcpp::Node::SharedPtr node_;
        geometry_msgs::msg::Pose2D direction;
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr Pos_;
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr Pos_angle;
        geometry_msgs::msg::Pose2D Pose2D_;
        rclcpp::Time last_time_;  
        messageprocess::RobotMsgProcess RobotMsgProcess_;
        rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr Pos_send_;
        example_interfaces::msg::Bool sendbool;
        bool serial_initialized_;
    };
}

#endif