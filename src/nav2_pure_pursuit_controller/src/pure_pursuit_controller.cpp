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
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".publish_freq", rclcpp::ParameterValue(10.0));

    node->get_parameter(plugin_name_ + ".speed_scale", speed_scale_);
    node->get_parameter(plugin_name_ + ".publish_freq", publish_freq_);

    // 创建发布器
    pose_pub_ = node->create_publisher<geometry_msgs::msg::Pose2D>("/pose", 10);

    is_active_ = false;

    // 创建定时器，按频率发布速度
    double period = 1.0 / publish_freq_;
    timer_ = node->create_wall_timer(
        std::chrono::duration<double>(period),
        [this]()
        {
          if (is_active_)
          {
            pose_pub_->publish(current_velocity_);
            geometry_msgs::msg::Pose2D zero_vel;
            zero_vel.x = 0.0;
            zero_vel.y = 0.0;
            zero_vel.theta = 0.0;
            pose_pub_->publish(zero_vel);
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

    x = original_y;  // ROS的Y变成地图的X
    y = -original_x; // ROS的X取反变成地图的Y
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
      RCLCPP_WARN(logger_, "路径点数不足2个，无法生成速度指令");
      return;
    }

    RCLCPP_INFO(logger_, "开始解析路径，共 %zu 个点", path.poses.size());

    // 遍历路径中的相邻点对
    for (size_t i = 0; i < path.poses.size() - 1; i++)
    {
      // 获取当前点和下一个点（ROS坐标系）
      double x1 = path.poses[i].pose.position.x;
      double y1 = path.poses[i].pose.position.y;
      double x2 = path.poses[i + 1].pose.position.x;
      double y2 = path.poses[i + 1].pose.position.y;

      RCLCPP_INFO(logger_, "ROS原始点: (%.3f, %.3f) -> (%.3f, %.3f)",
                  x1, y1, x2, y2);

      // 坐标转换到地图坐标系
      transformToMapFrame(x1, y1);
      transformToMapFrame(x2, y2);

      RCLCPP_INFO(logger_, "地图转换后: (%.3f, %.3f) -> (%.3f, %.3f)",
                  x1, y1, x2, y2);

      // 计算两点之间的差值（即速度方向）
      double dx = x2 - x1;
      double dy = y2 - y1;

      RCLCPP_INFO(logger_, "差值: dx=%.3f, dy=%.3f", dx, dy);

      // 生成速度指令
      geometry_msgs::msg::Pose2D vel_cmd;
      vel_cmd.x = dx * speed_scale_; // X轴速度（向右为正）
      vel_cmd.y = dy * speed_scale_; // Y轴速度（向下为正）
      vel_cmd.theta = 0.0;

      velocity_queue_.push(vel_cmd);

      RCLCPP_INFO(logger_, "段 %zu: 速度指令 (%.3f, %.3f)", i, vel_cmd.x, vel_cmd.y);
    }

    RCLCPP_INFO(logger_, "路径解析完成，生成 %zu 个速度指令", velocity_queue_.size());
  }

  void PurePursuitController::cleanup()
  {
    RCLCPP_INFO(logger_, "清理控制器: %s", plugin_name_.c_str());
    pose_pub_.reset();
    timer_.reset();
  }

  void PurePursuitController::activate()
  {
    RCLCPP_INFO(logger_, "激活控制器: %s", plugin_name_.c_str());
    pose_pub_->on_activate();
    is_active_ = true;
  }

  void PurePursuitController::deactivate()
  {
    RCLCPP_INFO(logger_, "停用控制器: %s", plugin_name_.c_str());
    pose_pub_->on_deactivate();
    is_active_ = false;

    // 清空队列
    while (!velocity_queue_.empty())
    {
      velocity_queue_.pop();
    }

    // 发送零速度
    geometry_msgs::msg::Pose2D zero_vel;
    zero_vel.x = 0.0;
    zero_vel.y = 0.0;
    zero_vel.theta = 0.0;
    pose_pub_->publish(zero_vel);
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
    RCLCPP_WARN(logger_, "设置速度缩放系数: %.2f", speed_scale_);
  }

  void PurePursuitController::setPlan(const nav_msgs::msg::Path &path)
  {
    RCLCPP_INFO(logger_, "收到新路径，包含 %zu 个点", path.poses.size());

    // 打印第一个和最后一个点用于调试
    if (!path.poses.empty())
    {
      RCLCPP_INFO(logger_, "路径第一个点 (ROS): (%.3f, %.3f)",
                  path.poses.front().pose.position.x,
                  path.poses.front().pose.position.y);
      RCLCPP_INFO(logger_, "路径最后一个点 (ROS): (%.3f, %.3f)",
                  path.poses.back().pose.position.x,
                  path.poses.back().pose.position.y);
    }

    // 解析路径生成速度指令队列
    parsePathToVelocities(path);

    // 如果有速度指令，立即开始执行第一个
    if (!velocity_queue_.empty())
    {
      current_velocity_ = velocity_queue_.front();
      velocity_queue_.pop();
      is_active_ = true;
      RCLCPP_INFO(logger_, "开始执行速度指令: vx=%.3f, vy=%.3f, 剩余 %zu 个",
                  current_velocity_.x, current_velocity_.y, velocity_queue_.size());
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

    // 更新当前速度指令（由定时器发布到 /pose）
    if (!velocity_queue_.empty() && is_active_)
    {
      current_velocity_ = velocity_queue_.front();
      velocity_queue_.pop();
    }

    return cmd_vel;
  }

} // namespace nav2_pure_pursuit_controller

PLUGINLIB_EXPORT_CLASS(nav2_pure_pursuit_controller::PurePursuitController, nav2_core::Controller)