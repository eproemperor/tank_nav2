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

    if (!node)
    {
      throw std::runtime_error("Failed to lock node");
    }

    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    costmap_ros_ = costmap_ros;

    // 声明并获取参数 - 使用正确的方式
    // 方法：先声明参数（带默认值），然后获取
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Speed ratio (0.0 to 1.0)";
    speed_ratio_ = node->declare_parameter<double>(
        plugin_name_ + ".speed_ratio", 1.0, desc);

    desc.description = "PWM frequency in Hz";
    pwm_freq_ = node->declare_parameter<double>(
        plugin_name_ + ".pwm_freq", 20.0, desc);

    desc.description = "PWM resolution (pulses per cycle)";
    pwm_resolution_ = node->declare_parameter<int>(
        plugin_name_ + ".pwm_resolution", 100, desc);

    desc.description = "Maximum speed in pixels/second";
    max_speed_ = node->declare_parameter<double>(
        plugin_name_ + ".max_speed", 300.0, desc);

    // 限制参数范围
    speed_ratio_ = std::max(0.0, std::min(1.0, speed_ratio_));
    pwm_resolution_ = std::max(10, pwm_resolution_);

    // 创建发布器
    pose_pub_ = node->create_publisher<geometry_msgs::msg::Pose2D>("/pose", 10);

    // 初始化状态
    is_active_ = false;
    is_moving_ = false;
    pwm_counter_ = 0;
    sending_direction_ = true;

    // 计算 PWM 参数
    calculatePwmParams(speed_ratio_);

    // 创建 PWM 定时器
    double period = 1.0 / pwm_freq_;
    pwm_timer_ = node->create_wall_timer(
        std::chrono::duration<double>(period),
        [this]()
        { pwmTimerCallback(); });

    RCLCPP_INFO(logger_, "PWM控制器已配置:");
    RCLCPP_INFO(logger_, "  速度比例: %.2f (最大速度: %.0f像素/秒 → 实际: %.0f像素/秒)",
                speed_ratio_, max_speed_, max_speed_ * speed_ratio_);
    RCLCPP_INFO(logger_, "  PWM频率: %.1f Hz", pwm_freq_);
    RCLCPP_INFO(logger_, "  PWM分辨率: %d 脉冲/周期", pwm_resolution_);
    RCLCPP_INFO(logger_, "  ON脉冲数: %d, OFF脉冲数: %d", pwm_on_cycles_, pwm_off_cycles_);
  }

  void PurePursuitController::calculatePwmParams(double speed_ratio)
  {
    // 根据速度比例计算 ON/OFF 脉冲数
    pwm_on_cycles_ = (int)(pwm_resolution_ * speed_ratio);
    pwm_off_cycles_ = pwm_resolution_ - pwm_on_cycles_;

    // 确保至少有一个脉冲
    if (pwm_on_cycles_ == 0 && speed_ratio > 0)
    {
      pwm_on_cycles_ = 1;
      pwm_off_cycles_ = pwm_resolution_ - 1;
    }
    if (pwm_off_cycles_ == 0 && speed_ratio < 1.0)
    {
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
    // 清空队列
    while (!direction_queue_.empty())
    {
      direction_queue_.pop();
    }

    if (path.poses.size() < 2)
    {
      RCLCPP_WARN(logger_, "路径点数不足2个");
      return;
    }

    RCLCPP_INFO(logger_, "navtopose运行，解析路径，共 %zu 个点", path.poses.size());

    // 转换坐标
    std::vector<std::pair<double, double>> points;
    for (size_t i = 0; i < path.poses.size(); i++)
    {
      double x = path.poses[i].pose.position.x;
      double y = path.poses[i].pose.position.y;
      transformToMapFrame(x, y);
      points.push_back({x, y});
      RCLCPP_DEBUG(logger_, "地图点%zu: (%.3f, %.3f)", i, x, y);
    }

    // 简化路径：合并同方向连续段
    std::vector<std::pair<double, double>> simplified;
    simplified.push_back(points[0]);

    for (size_t i = 1; i < points.size(); i++)
    {
      double dx = points[i].first - simplified.back().first;
      double dy = points[i].second - simplified.back().second;

      // 检查方向是否改变
      bool is_horizontal = (std::abs(dx) > 0.01 && std::abs(dy) <= 0.01);
      bool is_vertical = (std::abs(dy) > 0.01 && std::abs(dx) <= 0.01);

      if (is_horizontal || is_vertical)
      {
        // 同方向，更新终点
        simplified.back() = points[i];
      }
      else
      {
        // 方向改变，添加新点
        simplified.push_back(points[i]);
      }
    }

    //RCLCPP_INFO(logger_, "简化后路径: %zu 个关键点", simplified.size());

    // 生成带运动时间的指令
    double current_speed = max_speed_ * speed_ratio_; // 实际速度

    for (size_t i = 0; i < simplified.size() - 1; i++)
    {
      double dx = simplified[i + 1].first - simplified[i].first;
      double dy = simplified[i + 1].second - simplified[i].second;

      // 计算距离
      double distance = std::sqrt(dx * dx + dy * dy);

      // 计算运动时间
      double duration = distance / current_speed;

      // 最小时间限制（防止太短的脉冲）
      const double MIN_DURATION = 0.05; // 50ms
      if (duration < MIN_DURATION && distance > 0.01)
      {
        duration = MIN_DURATION;
        RCLCPP_DEBUG(logger_, "距离过小(%.3f)，使用最小时间 %.3fs", distance, duration);
      }

      DirectionCommand cmd;
      cmd.direction.theta = 0.0;
      cmd.duration = duration;
      cmd.distance = distance;

      // 确定方向
      if (std::abs(dx) > std::abs(dy))
      {
        cmd.direction.x = (dx > 0) ? 1.0 : -1.0;
        cmd.direction.y = 0.0;
      }
      else if (std::abs(dy) > std::abs(dx))
      {
        cmd.direction.x = 0.0;
        cmd.direction.y = (dy > 0) ? 1.0 : -1.0;
      }
      else
      {
        cmd.direction.x = 0.0;
        cmd.direction.y = 0.0;
      }

      direction_queue_.push(cmd);

      //RCLCPP_INFO(logger_, "指令 %zu: 方向(%.0f, %.0f), 距离=%.3f, 时间=%.3fs",i, cmd.direction.x, cmd.direction.y, distance, duration);
    }

    //RCLCPP_INFO(logger_, "生成 %zu 个方向指令", direction_queue_.size());
  }

  void PurePursuitController::stopMovement()
  {
    is_active_ = false;
    is_moving_ = false;

    while (!direction_queue_.empty())
    {
      direction_queue_.pop();
    }

    geometry_msgs::msg::Pose2D stop;
    stop.x = 0.0;
    stop.y = 0.0;
    stop.theta = 0.0;
    pose_pub_->publish(stop);

    //RCLCPP_INFO(logger_, "运动已停止");
  }

  void PurePursuitController::pwmTimerCallback()
  {
    if (!is_active_)
    {
      // 发送停止指令
      geometry_msgs::msg::Pose2D stop;
      stop.x = 0.0;
      stop.y = 0.0;
      stop.theta = 0.0;
      pose_pub_->publish(stop);
      return;
    }

    // 检查当前指令是否超时
    if (is_moving_)
    {
      rclcpp::Time now = clock_->now();
      double elapsed = (now - command_start_time_).seconds();

      if (elapsed >= current_command_.duration)
      {
        // 当前指令完成，停止运动
        is_moving_ = false;

        // 发送停止指令
        geometry_msgs::msg::Pose2D stop;
        stop.x = 0.0;
        stop.y = 0.0;
        stop.theta = 0.0;
        pose_pub_->publish(stop);

        //RCLCPP_INFO(logger_, "指令完成: 运动 %.3fs", current_command_.duration);

        // 检查是否有下一条指令
        if (!direction_queue_.empty())
        {
          // 获取下一条指令
          current_command_ = direction_queue_.front();
          direction_queue_.pop();
          command_start_time_ = now;
          is_moving_ = true;

          //RCLCPP_INFO(logger_, "开始下一条指令: 方向(%.0f, %.0f), 距离=%.3f, 时间=%.3fs, 剩余%zu条",
          //            current_command_.direction.x, current_command_.direction.y,
          //            current_command_.distance, current_command_.duration,
          //            direction_queue_.size());
        }
        else
        {
          // 所有指令完成
          is_active_ = false;
          //RCLCPP_INFO(logger_, "所有路径指令完成，停止运动");
        }
      }
    }

    // 如果正在运动，发送当前方向指令（PWM调制）
    if (is_moving_)
    {
      geometry_msgs::msg::Pose2D cmd;
      cmd.theta = 0.0;

      if (sending_direction_)
      {
        // 发送方向指令（坐标转换）
        cmd.x = -current_command_.direction.y;
        cmd.y = -current_command_.direction.x;
        pwm_counter_++;

        if (pwm_counter_ >= pwm_on_cycles_)
        {
          sending_direction_ = false;
          pwm_counter_ = 0;
        }
      }
      else
      {
        // 发送零速度（PWM的OFF阶段）
        cmd.x = 0.0;
        cmd.y = 0.0;
        pwm_counter_++;

        if (pwm_counter_ >= pwm_off_cycles_)
        {
          sending_direction_ = true;
          pwm_counter_ = 0;
        }
      }

      pose_pub_->publish(cmd);
    }
  }

  void PurePursuitController::cleanup()
  {
    RCLCPP_INFO(logger_, "清理控制器");
    stopMovement();
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
    stopMovement();
  }

  void PurePursuitController::setSpeedLimit(const double &speed_limit, const bool &percentage)
  {
    if (percentage)
    {
      speed_ratio_ = speed_ratio_ * (speed_limit / 100.0);
    }
    else
    {
      // 假设 speed_limit 是 0-max_speed_ 像素/秒
      speed_ratio_ = speed_limit / max_speed_;
    }
    speed_ratio_ = std::max(0.0, std::min(1.0, speed_ratio_));

    // 重新计算 PWM 参数
    calculatePwmParams(speed_ratio_);

    //RCLCPP_WARN(logger_, "速度比例: %.2f (实际速度: %.0f 像素/秒)",speed_ratio_, max_speed_ * speed_ratio_);
  }

  void PurePursuitController::setPlan(const nav_msgs::msg::Path &path)
  {
    //RCLCPP_INFO(logger_, "收到新路径，%zu 个点", path.poses.size());

    // 停止当前运动
    stopMovement();

    // 解析新路径
    parsePathToVelocities(path);

    if (!direction_queue_.empty())
    {
      // 获取第一条指令
      current_command_ = direction_queue_.front();
      direction_queue_.pop();
      command_start_time_ = clock_->now();
      is_moving_ = true;
      is_active_ = true;

      // 重置PWM状态
      sending_direction_ = true;
      pwm_counter_ = 0;

      
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