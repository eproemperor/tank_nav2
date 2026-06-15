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
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".speed_scale", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".publish_freq", rclcpp::ParameterValue(50.0));

    node->get_parameter(plugin_name_ + ".speed_scale", speed_scale_);
    node->get_parameter(plugin_name_ + ".publish_freq", publish_freq_);

    // 创建发布器（速度发送到 /pose）
    pose_pub_ = node->create_publisher<geometry_msgs::msg::Pose2D>("/pose", 10);

    is_active_ = false;

    // 定时器：按频率发布速度
    double period = 1.0 / publish_freq_;
    timer_ = node->create_wall_timer(
        std::chrono::duration<double>(period),
        [this]()
        {
          if (is_active_)
          {
            current_velocity_.y = -current_velocity_.x;
            current_velocity_.x = -current_velocity_.y;
            pose_pub_->publish(current_velocity_);
            RCLCPP_INFO(logger_, "速度x: %.2f", current_velocity_.x);
            RCLCPP_INFO(logger_, "速度y: %.2f", current_velocity_.y);
          }
        });

    RCLCPP_INFO(logger_, "控制器已配置: speed_scale=%.2f, publish_freq=%.1fHz",
                speed_scale_, publish_freq_);
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
    while (!velocity_queue_.empty())
    {
      velocity_queue_.pop();
    }

    if (path.poses.size() < 2)
    {
      RCLCPP_WARN(logger_, "路径点数不足2个");
      return;
    }

    RCLCPP_INFO(logger_, "解析路径，共 %zu 个点", path.poses.size());

    // 提取并转换路径点
    std::vector<std::pair<double, double>> points;
    for (size_t i = 0; i < path.poses.size(); i++)
    {
      double x = path.poses[i].pose.position.x;
      double y = path.poses[i].pose.position.y;
      transformToMapFrame(x, y);
      points.push_back({x, y});
      RCLCPP_INFO(logger_, "点%zu: (%.3f, %.3f)", i, x, y);
    }

    // 计算每段的速度（相邻点差值）
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      double dx = points[i + 1].first - points[i].first;
      double dy = points[i + 1].second - points[i].second;

      // 强制单轴（方形迷宫应该只有单轴移动）
      if (dx != 0 && dy != 0)
      {
        // 斜向移动时，选择移动距离更大的轴
        if (std::abs(dx) >= std::abs(dy))
        {
          dy = 0;
        }
        else
        {
          dx = 0;
        }
      }

      geometry_msgs::msg::Pose2D vel;
      vel.x = dx * speed_scale_;
      vel.y = dy * speed_scale_;
      vel.theta = 0.0;

      velocity_queue_.push(vel);
      RCLCPP_INFO(logger_, "速度段%zu: vx=%.3f, vy=%.3f", i, vel.x, vel.y);
    }

    RCLCPP_INFO(logger_, "生成 %zu 个速度指令", velocity_queue_.size());
  }

  void PurePursuitController::cleanup()
  {
    RCLCPP_INFO(logger_, "清理控制器");
    pose_pub_.reset();
    timer_.reset();
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

    while (!velocity_queue_.empty())
    {
      velocity_queue_.pop();
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
    if (percentage)
    {
      speed_scale_ = speed_scale_ * (speed_limit / 100.0);
    }
    else
    {
      speed_scale_ = speed_limit;
    }
    RCLCPP_WARN(logger_, "速度缩放: %.2f", speed_scale_);
  }

  void PurePursuitController::setPlan(const nav_msgs::msg::Path &path)
  {
    RCLCPP_INFO(logger_, "收到路径，%zu 个点", path.poses.size());

    parsePathToVelocities(path);

    if (!velocity_queue_.empty())
    {
      current_velocity_ = velocity_queue_.front();
      velocity_queue_.pop();
      is_active_ = true;
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

    // 更新到下一条速度指令
    if (!velocity_queue_.empty() && is_active_)
    {
      current_velocity_ = velocity_queue_.front();
      velocity_queue_.pop();
    }

    return cmd_vel;
  }

} // namespace nav2_pure_pursuit_controller

PLUGINLIB_EXPORT_CLASS(nav2_pure_pursuit_controller::PurePursuitController, nav2_core::Controller)