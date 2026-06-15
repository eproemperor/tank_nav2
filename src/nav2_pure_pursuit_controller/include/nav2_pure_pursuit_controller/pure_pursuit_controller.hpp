#ifndef NAV2_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
#define NAV2_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <memory>
#include <cmath>
#include <queue>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_pure_pursuit_controller
{

class PurePursuitController : public nav2_core::Controller
{
public:
  PurePursuitController() = default;
  ~PurePursuitController() override = default;

  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, 
      const std::shared_ptr<tf2_ros::Buffer> tf,
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
  // 解析路径，生成速度指令队列
  void parsePathToVelocities(const nav_msgs::msg::Path &path);
  
  // 坐标转换：ROS坐标系 → 你的地图坐标系 (X右, Y下)
  void transformToMapFrame(double &x, double &y);

  // ROS 相关成员
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("PurePursuitController")};
  rclcpp::Clock::SharedPtr clock_;

  // 参数
  double speed_scale_;        // 速度缩放系数
  double publish_freq_;       // 发布频率
  
  // 速度指令队列
  std::queue<geometry_msgs::msg::Pose2D> velocity_queue_;
  
  // 当前速度
  geometry_msgs::msg::Pose2D current_velocity_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 发布器（发送速度到 /pose）
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose2D>> pose_pub_;
  
  bool is_active_;
};

} // namespace

#endif