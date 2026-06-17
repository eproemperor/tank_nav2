#ifndef NAV2_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
#define NAV2_PURE_PURSUIT_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <memory>
#include <cmath>
#include <queue>
#include <vector>

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
  struct DirectionCommand {
    geometry_msgs::msg::Pose2D direction;
    double duration;
    double distance;
    double speed_ratio;     // 0~1
  };

  void parsePathToVelocities(const nav_msgs::msg::Path &path);
  void transformToMapFrame(double &x, double &y);
  void pwmTimerCallback();
  void calculatePwmParams(double speed_ratio);
  void stopMovement();

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("PurePursuitController")};
  rclcpp::Clock::SharedPtr clock_;

  double speed_ratio_;
  double pwm_freq_;
  int pwm_resolution_;
  double max_speed_;
  double decel_distance_;   // 减速段长度（像素）

  std::queue<DirectionCommand> direction_queue_;
  DirectionCommand current_command_;

  bool is_active_;
  bool is_moving_;
  rclcpp::Time command_start_time_;

  rclcpp::TimerBase::SharedPtr pwm_timer_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose2D>> pose_pub_;

  int pwm_counter_;
  int pwm_on_cycles_;
  int pwm_off_cycles_;
  bool sending_direction_;
  double current_speed_ratio_;
};

} // namespace

#endif