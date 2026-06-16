#include "nav2_pure_pursuit_controller/pure_pursuit_controller.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_pure_pursuit_controller
{

void PurePursuitController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();

  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  costmap_ros_ = costmap_ros;

  // 参数
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".speed_ratio", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".pwm_freq", rclcpp::ParameterValue(20.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".pwm_resolution", rclcpp::ParameterValue(100));

  node->get_parameter(plugin_name_ + ".speed_ratio", speed_ratio_);
  node->get_parameter(plugin_name_ + ".pwm_freq", pwm_freq_);
  node->get_parameter(plugin_name_ + ".pwm_resolution", pwm_resolution_);

  // 限制速度比例范围
  speed_ratio_ = std::max(0.0, std::min(1.0, speed_ratio_));
  
  // 确保分辨率至少为 10
  pwm_resolution_ = std::max(10, pwm_resolution_);

  // 创建发布器
  pose_pub_ = node->create_publisher<geometry_msgs::msg::Pose2D>("/pose", 10);

  is_active_ = false;
  pwm_counter_ = 0;
  sending_direction_ = true;

  // 计算 PWM 参数
  calculatePwmParams(speed_ratio_);

  // 创建 PWM 定时器
  double period = 1.0 / pwm_freq_;
  pwm_timer_ = node->create_wall_timer(
      std::chrono::duration<double>(period),
      [this]() { pwmTimerCallback(); });

  RCLCPP_INFO(logger_, "PWM控制器已配置:");
  RCLCPP_INFO(logger_, "  速度比例: %.2f (最大速度: 300像素/秒 → 实际: %.0f像素/秒)", 
              speed_ratio_, 300.0 * speed_ratio_);
  RCLCPP_INFO(logger_, "  PWM频率: %.1f Hz", pwm_freq_);
  RCLCPP_INFO(logger_, "  PWM分辨率: %d 脉冲/周期", pwm_resolution_);
  RCLCPP_INFO(logger_, "  ON脉冲数: %d, OFF脉冲数: %d", pwm_on_cycles_, pwm_off_cycles_);
}

void PurePursuitController::calculatePwmParams(double speed_ratio)
{
  // 根据速度比例计算 ON/OFF 脉冲数
  // speed_ratio = 0.3 表示 30% 时间发送方向，70% 时间发送 0
  pwm_on_cycles_ = (int)(pwm_resolution_ * speed_ratio);
  pwm_off_cycles_ = pwm_resolution_ - pwm_on_cycles_;
  
  // 确保至少有一个脉冲
  if (pwm_on_cycles_ == 0 && speed_ratio > 0) {
    pwm_on_cycles_ = 1;
    pwm_off_cycles_ = pwm_resolution_ - 1;
  }
  if (pwm_off_cycles_ == 0 && speed_ratio < 1.0) {
    pwm_off_cycles_ = 1;
    pwm_on_cycles_ = pwm_resolution_ - 1;
  }
  
  RCLCPP_DEBUG(logger_, "PWM参数: ON=%d, OFF=%d", pwm_on_cycles_, pwm_off_cycles_);
}

void PurePursuitController::transformToMapFrame(double &x, double &y)
{
  // ROS坐标系 (X前, Y左) → 地图坐标系 (X右, Y下)
  double original_x = x;
  double original_y = y;
  x = original_y;
  y = -original_x;
}

void PurePursuitController::parsePathToVelocities(const nav_msgs::msg::Path &path)
{
  while (!direction_queue_.empty()) {
    direction_queue_.pop();
  }

  if (path.poses.size() < 2) {
    RCLCPP_WARN(logger_, "路径点数不足2个");
    return;
  }

  RCLCPP_INFO(logger_, "解析路径，共 %zu 个点", path.poses.size());

  std::vector<std::pair<double, double>> points;
  for (size_t i = 0; i < path.poses.size(); i++) {
    double x = path.poses[i].pose.position.x;
    double y = path.poses[i].pose.position.y;
    
    RCLCPP_INFO(logger_, "ROS原始点%zu: (%.3f, %.3f)", i, x, y);
    
    // X Y 交换
    transformToMapFrame(x, y);
    
    points.push_back({x, y});
    RCLCPP_INFO(logger_, "地图点%zu: (%.3f, %.3f)", i, x, y);
  }

  // 合并相同方向的连续段
  std::vector<std::pair<double, double>> merged_points;
  merged_points.push_back(points[0]);
  
  for (size_t i = 1; i < points.size(); i++) {
    double dx = points[i].first - merged_points.back().first;
    double dy = points[i].second - merged_points.back().second;
    
    bool is_horizontal = (std::abs(dx) > 0.01 && std::abs(dy) <= 0.01);
    bool is_vertical = (std::abs(dy) > 0.01 && std::abs(dx) <= 0.01);
    
    if (is_horizontal || is_vertical) {
      merged_points.back() = points[i];
    } else {
      merged_points.push_back(points[i]);
    }
  }
  
  RCLCPP_INFO(logger_, "合并后: %zu 个点", merged_points.size());

  for (size_t i = 0; i < merged_points.size() - 1; i++) {
    double dx = merged_points[i+1].first - merged_points[i].first;
    double dy = merged_points[i+1].second - merged_points[i].second;
    
    geometry_msgs::msg::Pose2D dir;
    dir.theta = 0.0;
    
    // 确定方向（只有 ±1 或 0）
    if (std::abs(dx) > std::abs(dy)) {
      dir.x = (dx > 0) ? 1.0 : -1.0;
      dir.y = 0.0;
      RCLCPP_INFO(logger_, "段%zu: X方向 %+.0f, 距离 %.3f", i, dir.x, dx);
    } else if (std::abs(dy) > std::abs(dx)) {
      dir.x = 0.0;
      dir.y = (dy > 0) ? 1.0 : -1.0;
      RCLCPP_INFO(logger_, "段%zu: Y方向 %+.0f, 距离 %.3f", i, dir.y, dy);
    } else {
      dir.x = 0.0;
      dir.y = 0.0;
      RCLCPP_INFO(logger_, "段%zu: 静止", i);
    }
    
    direction_queue_.push(dir);
  }

  RCLCPP_INFO(logger_, "生成 %zu 个方向指令", direction_queue_.size());
}

void PurePursuitController::pwmTimerCallback()
{
  if (!is_active_) {
    return;
  }
  
  geometry_msgs::msg::Pose2D cmd;
  cmd.theta = 0.0;
  
  if (sending_direction_) {
    // 发送方向指令（+1, -1, 或 0）
    cmd.x = -current_direction_.y;
    cmd.y = -current_direction_.x;
    pwm_counter_++;
    
    if (pwm_counter_ >= pwm_on_cycles_) {
      // 切换到发送 0
      sending_direction_ = false;
      pwm_counter_ = 0;
    }
  } else {
    // 发送 0
    cmd.x = 0.0;
    cmd.y = 0.0;
    pwm_counter_++;
    
    if (pwm_counter_ >= pwm_off_cycles_) {
      // 完成一个完整 PWM 周期，检查是否需要切换方向指令
      sending_direction_ = true;
      pwm_counter_ = 0;
      
      // 如果队列中还有下一条方向指令，切换到下一条
      if (!direction_queue_.empty()) {
        current_direction_ = direction_queue_.front();
        direction_queue_.pop();
        RCLCPP_INFO(logger_, "切换到下一条方向: vx=%.0f, vy=%.0f, 剩余 %zu 条",
                    current_direction_.x, current_direction_.y, direction_queue_.size());
      }
    }
  }
  
  pose_pub_->publish(cmd);
}

void PurePursuitController::cleanup()
{
  RCLCPP_INFO(logger_, "清理控制器");
  pose_pub_.reset();
  pwm_timer_.reset();
}

void PurePursuitController::activate()
{
  RCLCPP_INFO(logger_, "激活控制器");
  pose_pub_->on_activate();
  is_active_ = true;
}

void PurePursuitController::deactivate()
{
  RCLCPP_INFO(logger_, "停用控制器");
  pose_pub_->on_deactivate();
  is_active_ = false;
  
  while (!direction_queue_.empty()) {
    direction_queue_.pop();
  }
  
  // 发送零速度
  geometry_msgs::msg::Pose2D zero;
  zero.x = 0.0;
  zero.y = 0.0;
  zero.theta = 0.0;
  pose_pub_->publish(zero);
}

void PurePursuitController::setSpeedLimit(const double &speed_limit, const bool &percentage)
{
  if (percentage) {
    speed_ratio_ = speed_ratio_ * (speed_limit / 100.0);
  } else {
    // 假设 speed_limit 是 0-300 像素/秒
    speed_ratio_ = speed_limit / 300.0;
  }
  speed_ratio_ = std::max(0.0, std::min(1.0, speed_ratio_));
  
  // 重新计算 PWM 参数
  calculatePwmParams(speed_ratio_);
  
  RCLCPP_WARN(logger_, "速度比例: %.2f (实际速度: %.0f 像素/秒)", 
              speed_ratio_, 300.0 * speed_ratio_);
}

void PurePursuitController::setPlan(const nav_msgs::msg::Path &path)
{
  RCLCPP_INFO(logger_, "收到路径，%zu 个点", path.poses.size());
  
  parsePathToVelocities(path);
  
  if (!direction_queue_.empty()) {
    current_direction_ = direction_queue_.front();
    direction_queue_.pop();
    is_active_ = true;
    
    // 重置 PWM 状态
    sending_direction_ = true;
    pwm_counter_ = 0;
    
    RCLCPP_INFO(logger_, "开始运动: vx=%.0f, vy=%.0f, 剩余 %zu 条",
                current_direction_.x, current_direction_.y, direction_queue_.size());
  }
}

geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker *goal_checker)
{
  (void)pose;
  (void)velocity;
  (void)goal_checker;

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "base_link";
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.linear.y = 0.0;
  cmd_vel.twist.angular.z = 0.0;

  return cmd_vel;
}

} // namespace nav2_pure_pursuit_controller

PLUGINLIB_EXPORT_CLASS(nav2_pure_pursuit_controller::PurePursuitController, nav2_core::Controller)