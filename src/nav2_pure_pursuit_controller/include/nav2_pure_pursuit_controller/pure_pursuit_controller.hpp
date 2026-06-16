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
  // 方向指令结构体
  struct DirectionCommand {
    geometry_msgs::msg::Pose2D direction;
    double duration;        // 运动持续时间（秒）
    double distance;        // 运动距离
  };
  
  // 解析路径，生成速度指令队列
  void parsePathToVelocities(const nav_msgs::msg::Path &path);
  
  // 坐标转换
  void transformToMapFrame(double &x, double &y);
  
  // PWM 定时器回调
  void pwmTimerCallback();
  
  // 计算 PWM 参数
  void calculatePwmParams(double speed_ratio);

  // 停止运动
  void stopMovement();

  // ROS 相关成员
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("PurePursuitController")};
  rclcpp::Clock::SharedPtr clock_;

  // 参数
  double speed_ratio_;        // 速度比例 (0.0 ~ 1.0)
  double pwm_freq_;           // PWM 频率 (Hz)
  int pwm_resolution_;        // PWM 分辨率（一个周期的脉冲数）
  double max_speed_;          // 最大速度 (像素/秒)
  
  // 速度指令队列（使用 DirectionCommand）
  std::queue<DirectionCommand> direction_queue_;
  
  // 当前方向指令
  DirectionCommand current_command_;
  
  // 运动控制状态
  bool is_active_;
  bool is_moving_;
  rclcpp::Time command_start_time_;
  
  // PWM 控制
  rclcpp::TimerBase::SharedPtr pwm_timer_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose2D>> pose_pub_;
  
  int pwm_counter_;           // 当前周期内的脉冲计数
  int pwm_on_cycles_;         // 发送方向的脉冲数
  int pwm_off_cycles_;        // 发送0的脉冲数
  bool sending_direction_;    // true=发送方向, false=发送0
};

} // namespace

#endif