#include "updatemapinfo_node.h"

namespace updatemap
{

    UpdateMapinfo::UpdateMapinfo(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf)
    {
        //从黑板读取数据
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        config().blackboard->get<TypeMode>("star_info", star);
        config().blackboard->get<TypeMode>("base_info", base);
        config().blackboard->get<TypeMode>("enemy_base_info", enemy_base);
        config().blackboard->get<TypeMode>("sentry_info", sentry);
        config().blackboard->get<TypeMode>("purple_entry_info", purpleentry);
        config().blackboard->get<TypeMode>("green_entry_info", greenentry);
        config().blackboard->get<TypeMode>("purple_exit_info", purpleexit);
        config().blackboard->get<TypeMode>("green_exit_info", greenexit);
        config().blackboard->get<TypeMode>("enemy_info", enemy);
        config().blackboard->get<int>("enemy_num", enemy_num);
        config().blackboard->get<double>("sentry_hp", sentry_hp);
        config().blackboard->get<bool>("is_transfering", is_transfering);
        config().blackboard->get<bool>("is_bullet_low", is_bullet_low);
        config().blackboard->get<nav_msgs::msg::OccupancyGrid>("map_data", latest_map);
        config().blackboard->get<nav_msgs::msg::Odometry>("robot_pose", latest_odom);

        // 1.订阅地图
        rclcpp::QoS map_qos(10);
        map_qos.transient_local();
        map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            map_qos,
            std::bind(&UpdateMapinfo::mapCallback, this, std::placeholders::_1));

        // 2. 订阅 /Odometry (Odometry)
        rclcpp::QoS odom_qos(rclcpp::KeepLast(100000));
        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry",
            odom_qos,
            std::bind(&UpdateMapinfo::odomCallback, this, std::placeholders::_1));

        // 3. 订阅 /map_info
        map_info_sub_ = node_->create_subscription<robot_msgs::msg::MapInfoMsgs>(
            "/map_info",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&UpdateMapinfo::mapInfoCallback, this, std::placeholders::_1));

        RCLCPP_INFO(node_->get_logger(), "MapSubscriber node initialized");
    }

    UpdateMapinfo::~UpdateMapinfo() {}

    void UpdateMapinfo::InitMap()
    {
        RCLCPP_INFO(node_->get_logger(), "Initializing map targets...");

        star = {TargetType::STAR, false, 0, 0};
        base = {TargetType::BASE, false, 0, 0};
        enemy_base = {TargetType::ENEMY_BASE, false, 0, 0};
        purpleentry = {TargetType::PURPLEENTRY, false, 0, 0};
        greenentry = {TargetType::GREENENTRY, false, 0, 0};
        sentry = {TargetType::SENTRY, false, 0, 0};
        enemy = {TargetType::ENEMY, false, 0, 0};
        purpleexit = {TargetType::PURPLEEXIT, false, 0, 0};
        greenexit = {TargetType::GREENEXIT, false, 0, 0};

        RCLCPP_INFO(node_->get_logger(), "Map initialization complete");
    }

    void UpdateMapinfo::updatemsg(TargetType type, double x, double y, bool is_exist)
    {
        switch (type)
        {
        case TargetType::STAR:
            star = {type, is_exist, x, y};
            break;
        case TargetType::BASE:
            base = {type, is_exist, x, y};
            break;
        case TargetType::ENEMY_BASE:
            enemy_base = {type, is_exist, x, y};
            break;
        case TargetType::PURPLEENTRY:
            purpleentry = {type, is_exist, x, y};
            break;
        case TargetType::GREENENTRY:
            greenentry = {type, is_exist, x, y};
            break;
        case TargetType::SENTRY:
            sentry = {type, is_exist, x, y};
            break;
        case TargetType::ENEMY:
            enemy = {type, is_exist, x, y};
            break;
        case TargetType::PURPLEEXIT:
            purpleexit = {type, is_exist, x, y};
            break;
        case TargetType::GREENEXIT:
            greenexit = {type, is_exist, x, y};
            break;
        }
    }

    //========================================================================//
    void UpdateMapinfo::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        latest_map = *msg;
        map_received = true;
        RCLCPP_DEBUG(node_->get_logger(), "Map received: %dx%d",
                     msg->info.width, msg->info.height);
    };

    void UpdateMapinfo::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        latest_odom = *msg;
        odom_received = true;
        RCLCPP_DEBUG(node_->get_logger(), "Odometry received: x=%.2f, y=%.2f",
                     msg->pose.pose.position.x, msg->pose.pose.position.y);
    };

    void UpdateMapinfo::mapInfoCallback(const robot_msgs::msg::MapInfoMsgs::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        static const std::vector<TargetType> index_to_type = {
            TargetType::STAR,
            TargetType::BASE,
            TargetType::ENEMY_BASE,
            TargetType::PURPLEENTRY,
            TargetType::GREENENTRY,
            TargetType::SENTRY,
            TargetType::ENEMY,
            TargetType::PURPLEEXIT,
            TargetType::GREENEXIT};

        for (size_t i = 0; i < msg->map_info.size() && i < index_to_type.size(); i++)
        {
            updatemsg(index_to_type[i],
                      msg->map_info[i].pos.x,
                      msg->map_info[i].pos.y,
                      msg->map_info[i].is_exist);
        }

        enemy_num = msg->enemy_num;
        sentry_hp = msg->sentry_hp;
        is_transfering = msg->is_transfering;
        is_bullet_low = msg->is_bullet_low;

        map_info_received = true;
    };

    BT::NodeStatus UpdateMapinfo::tick()
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        if (!IsInit)
        {
            InitMap();
            IsInit = true;
        }

        if (!map_received || !odom_received || !map_info_received)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Waiting for data: map=%d, odom=%d, map_info=%d",
                        map_received, odom_received, map_info_received);
            return BT::NodeStatus::RUNNING;
        }

        setOutput("star_info", star);
        setOutput("base_info", base);
        setOutput("enemy_base_info", enemy_base);
        setOutput("sentry_info", sentry);
        setOutput("purple_entry_info", purpleentry);
        setOutput("green_entry_info", greenentry);
        setOutput("purple_exit_info", purpleexit);
        setOutput("green_exit_info", greenexit);
        setOutput("enemy_info", enemy);

        setOutput("enemy_num", enemy_num);
        setOutput("sentry_hp", sentry_hp);
        setOutput("is_transfering", is_transfering);
        setOutput("is_bullet_low", is_bullet_low);

        setOutput("map_data", latest_map);
        setOutput("robot_pose", latest_odom);
        setOutput("map_ready", true);

        return BT::NodeStatus::SUCCESS;
    }

}