#include "nav2_pure_pursuit_controller/pure_pursuit_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include <algorithm>
#include <cmath>

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
  if (!node) throw std::runtime_error("Failed to lock node");

  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  costmap_ros_ = costmap_ros;
  fire_angle_ = 0.0;  // 初始化角度

  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.description = "Speed ratio (0.0 to 1.0)";
  speed_ratio_ = node->declare_parameter<double>(plugin_name_ + ".speed_ratio", 1.0, desc);

  desc.description = "PWM frequency in Hz";
  pwm_freq_ = node->declare_parameter<double>(plugin_name_ + ".pwm_freq", 50.0, desc);

  desc.description = "PWM resolution (pulses per cycle)";
  pwm_resolution_ = node->declare_parameter<int>(plugin_name_ + ".pwm_resolution", 20, desc); // 改为20提高响应

  desc.description = "Maximum speed in pixels/second (hardware limit)";
  max_speed_ = node->declare_parameter<double>(plugin_name_ + ".max_speed", 300.0, desc);

  desc.description = "Deceleration distance in pixels (for smooth stop)";
  decel_distance_ = node->declare_parameter<double>(plugin_name_ + ".decel_distance", 50.0, desc);

  speed_ratio_ = std::clamp(speed_ratio_, 0.0, 1.0);
  pwm_resolution_ = std::max(10, pwm_resolution_);
  decel_distance_ = std::max(10.0, decel_distance_);

  // 创建发布器
  pose_pub_ = node->create_publisher<geometry_msgs::msg::Pose2D>("/pose", 10);

  // 【修正】使用 node 创建订阅器，而不是 angle_node_
  angle_sub_ = node->create_subscription<geometry_msgs::msg::Pose2D>(
      "/pose_angle",  // 确认话题名称是否正确
      10,
      std::bind(&PurePursuitController::angleCallback, this, std::placeholders::_1));

  is_active_ = false;
  is_moving_ = false;
  pwm_counter_ = 0;
  sending_direction_ = true;
  current_speed_ratio_ = 0.0;
  calculatePwmParams(0.0);

  double period = 1.0 / pwm_freq_;
  pwm_timer_ = node->create_wall_timer(
      std::chrono::duration<double>(period),
      [this]() { pwmTimerCallback(); });

  RCLCPP_INFO(logger_, "PWM控制器已配置:");
  RCLCPP_INFO(logger_, "  速度比例: %.2f (最大速度: %.0f px/s)", speed_ratio_, max_speed_);
  RCLCPP_INFO(logger_, "  减速距离: %.0f px", decel_distance_);
  RCLCPP_INFO(logger_, "  PWM频率: %.1f Hz, 分辨率: %d", pwm_freq_, pwm_resolution_);
}

void PurePursuitController::angleCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
  fire_angle_ = msg->theta;
}

void PurePursuitController::calculatePwmParams(double speed_ratio)
{
  speed_ratio = std::clamp(speed_ratio, 0.0, 1.0);
  pwm_on_cycles_ = (int)(pwm_resolution_ * speed_ratio);
  pwm_off_cycles_ = pwm_resolution_ - pwm_on_cycles_;

  // 全速时彻底关闭 OFF 脉冲
  if (speed_ratio >= 1.0 - 1e-6) {
    pwm_on_cycles_ = pwm_resolution_;
    pwm_off_cycles_ = 0;
  } else {
    if (pwm_on_cycles_ == 0 && speed_ratio > 0.0) {
      pwm_on_cycles_ = 1;
      pwm_off_cycles_ = pwm_resolution_ - 1;
    }
    if (pwm_off_cycles_ == 0 && speed_ratio < 1.0) {
      pwm_off_cycles_ = 1;
      pwm_on_cycles_ = pwm_resolution_ - 1;
    }
  }
 // RCLCPP_DEBUG(logger_, "PWM: ratio=%.2f, ON=%d, OFF=%d", speed_ratio, pwm_on_cycles_, pwm_off_cycles_);
}

void PurePursuitController::transformToMapFrame(double &x, double &y)
{
  double orig_x = x, orig_y = y;
  x = orig_y;
  y = -orig_x;
}

void PurePursuitController::parsePathToVelocities(const nav_msgs::msg::Path &path)
{
  while (!direction_queue_.empty()) direction_queue_.pop();

  if (path.poses.size() < 2) {
    //RCLCPP_WARN(logger_, "路径点数不足2个");
    return;
  }

  // 1. 坐标转换
  std::vector<std::pair<double, double>> points;
  for (const auto &pose : path.poses) {
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    transformToMapFrame(x, y);
    points.push_back({x, y});
  }

  // 2. 简化路径：合并同向段（减少转折）
  std::vector<std::pair<double, double>> simplified;
  simplified.push_back(points[0]);
  for (size_t i = 1; i < points.size(); ++i) {
    double dx = points[i].first - simplified.back().first;
    double dy = points[i].second - simplified.back().second;
    bool same_dir = false;
    if (i > 1) {
      double prev_dx = points[i-1].first - points[i-2].first;
      double prev_dy = points[i-1].second - points[i-2].second;
      double len1 = std::sqrt(prev_dx*prev_dx + prev_dy*prev_dy);
      double len2 = std::sqrt(dx*dx + dy*dy);
      if (len1 > 0.1 && len2 > 0.1) {
        double cos_angle = (prev_dx*dx + prev_dy*dy) / (len1*len2);
        same_dir = (cos_angle > 0.99);
      }
    }
    if (!same_dir) {
      simplified.push_back(points[i]);
    } else {
      simplified.back() = points[i];
    }
  }
  if (simplified.size() < 2) simplified = points;

  // 3. 等距插值（步长 15px，减少指令数）
  const double STEP = 15.0;
  std::vector<std::pair<double, double>> interpolated;
  interpolated.push_back(simplified[0]);
  for (size_t i = 1; i < simplified.size(); ++i) {
    double dx = simplified[i].first - simplified[i-1].first;
    double dy = simplified[i].second - simplified[i-1].second;
    double seg_len = std::sqrt(dx*dx + dy*dy);
    if (seg_len < 1e-6) continue;
    int num_segments = std::max(1, (int)(seg_len / STEP));
    for (int j = 1; j <= num_segments; ++j) {
      double t = (double)j / num_segments;
      interpolated.push_back({simplified[i-1].first + t*dx,
                              simplified[i-1].second + t*dy});
    }
  }

  // 4. 计算总长度
  double total_len = 0.0;
  for (size_t i = 1; i < interpolated.size(); ++i) {
    double dx = interpolated[i].first - interpolated[i-1].first;
    double dy = interpolated[i].second - interpolated[i-1].second;
    total_len += std::sqrt(dx*dx + dy*dy);
  }
  if (total_len < 1e-6) {
    //RCLCPP_WARN(logger_, "路径总长度接近0");
    return;
  }

  // 5. 速度规划：全速 + 末端减速
  double decel_dist = std::min(decel_distance_, total_len * 0.5);
  double cruise_dist = total_len - decel_dist;

  double accumulated = 0.0;

  for (size_t i = 1; i < interpolated.size(); ++i) {
    double dx = interpolated[i].first - interpolated[i-1].first;
    double dy = interpolated[i].second - interpolated[i-1].second;
    double seg_len = std::sqrt(dx*dx + dy*dy);
    if (seg_len < 1e-6) continue;

    double mid_dist = accumulated + seg_len / 2.0;

    double ratio;
    if (mid_dist <= cruise_dist) {
      ratio = 1.0;
    } else {
      double t = (mid_dist - cruise_dist) / decel_dist;
      t = std::clamp(t, 0.0, 1.0);
      ratio = 1.0 - t;
      ratio = std::max(ratio, 0.0);
    }

    if (ratio < 0.01 && seg_len > 0.1) {
      ratio = 0.01;
    }

    double speed = max_speed_ * ratio;
    double duration = (speed > 0.0) ? (seg_len / speed) : 0.001;
    if (duration > 5.0) duration = 5.0;

    DirectionCommand cmd;
    cmd.direction.theta = 0.0;
    if (std::abs(dx) > std::abs(dy)) {
      cmd.direction.x = (dx > 0) ? 1.0 : -1.0;
      cmd.direction.y = 0.0;
    } else if (std::abs(dy) > std::abs(dx)) {
      cmd.direction.x = 0.0;
      cmd.direction.y = (dy > 0) ? 1.0 : -1.0;
    } else {
      cmd.direction.x = 0.0;
      cmd.direction.y = 0.0;
    }
    cmd.duration = duration;
    cmd.distance = seg_len;
    cmd.speed_ratio = ratio;

    direction_queue_.push(cmd);
    accumulated += seg_len;
  }

  //RCLCPP_INFO(logger_, "路径插值点: %zu, 指令数: %zu", interpolated.size(), direction_queue_.size());
}

void PurePursuitController::stopMovement()
{
  is_active_ = false;
  is_moving_ = false;
  while (!direction_queue_.empty()) direction_queue_.pop();

  geometry_msgs::msg::Pose2D stop;
  stop.x = 0.0; stop.y = 0.0; stop.theta = fire_angle_;
  pose_pub_->publish(stop);
  current_speed_ratio_ = 0.0;
  calculatePwmParams(0.0);
}

void PurePursuitController::pwmTimerCallback()
{
  // 【重要】移除 rclcpp::spin()，避免阻塞和死锁

  // 恢复机制
  if (!is_active_ && !direction_queue_.empty()) {
    RCLCPP_WARN(logger_, "恢复执行");
    current_command_ = direction_queue_.front();
    direction_queue_.pop();
    command_start_time_ = clock_->now();
    is_moving_ = true;
    is_active_ = true;
    sending_direction_ = true;
    pwm_counter_ = 0;
    current_speed_ratio_ = current_command_.speed_ratio;
    calculatePwmParams(current_speed_ratio_);
    return;
  }

  if (!is_active_) {
    geometry_msgs::msg::Pose2D stop;
    stop.x = 0.0; stop.y = 0.0; stop.theta = fire_angle_;
    pose_pub_->publish(stop);
    return;
  }

  if (is_moving_) {
    rclcpp::Time now = clock_->now();
    double elapsed = (now - command_start_time_).seconds();
    if (elapsed >= current_command_.duration) {
      is_moving_ = false;
      if (!direction_queue_.empty()) {
        current_command_ = direction_queue_.front();
        direction_queue_.pop();
        command_start_time_ = now;
        is_moving_ = true;
        current_speed_ratio_ = current_command_.speed_ratio;
        calculatePwmParams(current_speed_ratio_);
        //RCLCPP_DEBUG(logger_, "下一条: ratio=%.2f, dur=%.3f, 剩余%zu",
         //            current_speed_ratio_, current_command_.duration, direction_queue_.size());
      } else {
        is_active_ = false;
        current_speed_ratio_ = 0.0;
        calculatePwmParams(0.0);
        geometry_msgs::msg::Pose2D stop;
        stop.x = 0.0; stop.y = 0.0; stop.theta = fire_angle_;
        pose_pub_->publish(stop);
        //RCLCPP_INFO(logger_, "路径完成，停止");
      }
    }
  }

  if (is_moving_) {
    geometry_msgs::msg::Pose2D cmd;
    cmd.theta = fire_angle_;
    if (sending_direction_) {
      cmd.x = -current_command_.direction.y;
      cmd.y = -current_command_.direction.x;
      pwm_counter_++;
      if (pwm_counter_ >= pwm_on_cycles_) {
        sending_direction_ = false;
        pwm_counter_ = 0;
      }
    } else {
      cmd.x = 0.0; cmd.y = 0.0;
      pwm_counter_++;
      if (pwm_counter_ >= pwm_off_cycles_) {
        sending_direction_ = true;
        pwm_counter_ = 0;
      }
    }
    pose_pub_->publish(cmd);
  }
}

void PurePursuitController::cleanup()
{
  //RCLCPP_INFO(logger_, "清理控制器");
  stopMovement();
  pose_pub_.reset();
  pwm_timer_.reset();
  angle_sub_.reset();
}

void PurePursuitController::activate()
{
  //RCLCPP_INFO(logger_, "激活控制器");
  pose_pub_->on_activate();
  is_active_ = true;
}

void PurePursuitController::deactivate()
{
  //RCLCPP_INFO(logger_, "停用控制器");
  pose_pub_->on_deactivate();
  stopMovement();
}

void PurePursuitController::setSpeedLimit(const double &speed_limit, const bool &percentage)
{
  if (percentage) {
    speed_ratio_ = speed_ratio_ * (speed_limit / 100.0);
  } else {
    speed_ratio_ = speed_limit / max_speed_;
  }
  speed_ratio_ = std::clamp(speed_ratio_, 0.0, 1.0);
  //RCLCPP_WARN(logger_, "速度比例更新为 %.2f", speed_ratio_);
}

void PurePursuitController::setPlan(const nav_msgs::msg::Path &path)
{
  //RCLCPP_INFO(logger_, "收到新路径，%zu 个点", path.poses.size());
  stopMovement();

  parsePathToVelocities(path);

  if (!direction_queue_.empty()) {
    current_command_ = direction_queue_.front();
    direction_queue_.pop();
    command_start_time_ = clock_->now();
    is_moving_ = true;
    is_active_ = true;
    sending_direction_ = true;
    pwm_counter_ = 0;
    current_speed_ratio_ = current_command_.speed_ratio;
    calculatePwmParams(current_speed_ratio_);
    //RCLCPP_INFO(logger_, "开始执行: ratio=%.2f, dur=%.3f, 剩余%zu条",
    //            current_speed_ratio_, current_command_.duration, direction_queue_.size());
  } else {
    RCLCPP_WARN(logger_, "无有效指令");
  }
}

geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker *goal_checker)
{
  (void)pose; (void)velocity; (void)goal_checker;
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