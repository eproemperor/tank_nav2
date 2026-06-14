/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *
 * Nav2 (Navigation2) 框架中的纯追踪控制器 的头文件。
 * 它实现了一个路径跟踪算法，用于让机器人沿着规划好的全局路径运动
 */

#ifndef NAV2_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
#define NAV2_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"  

namespace nav2_pure_pursuit_controller
{

  class PurePursuitController : public nav2_core::Controller
  {
  public:
    PurePursuitController() = default;
    ~PurePursuitController() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;
    void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *goal_checker) override;

    void setPlan(const nav_msgs::msg::Path &path) override;

  protected:
    nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped &pose);

    bool transformPose(
        const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::string frame,
        const geometry_msgs::msg::PoseStamped &in_pose,
        geometry_msgs::msg::PoseStamped &out_pose,
        const rclcpp::Duration &transform_tolerance) const;

    // 新增：更新并发布位姿
    void updateAndPublishPose(const geometry_msgs::msg::PoseStamped &pose, const geometry_msgs::msg::Twist &velocity);

    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string plugin_name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    rclcpp::Logger logger_{rclcpp::get_logger("PurePursuitController")};
    rclcpp::Clock::SharedPtr clock_;

    double desired_linear_vel_;
    double lookahead_dist_;
    double max_angular_vel_;
    rclcpp::Duration transform_tolerance_{0, 0};

    nav_msgs::msg::Path global_plan_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;

    // 新增：位姿发布器
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose2D>> pose_pub_;
    
    // 新增：积分位姿
    geometry_msgs::msg::Pose2D integrated_pose_;
    rclcpp::Time last_pose_time_;
  };

} // namespace nav2_pure_pursuit_controller

#endif // NAV2_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_