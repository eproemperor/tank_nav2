#include "updatemapinfo_node.h"

namespace nav2_behavior_tree
{

    UpdateMapinfo::UpdateMapinfo(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf)
    {
        // 从黑板读取数据（拆分为基本类型）
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        // 星星信息
        config().blackboard->get<int>("star_name", star_name);
        config().blackboard->get<bool>("star_is_exist", star_is_exist);
        config().blackboard->get<bool>("star_is_out_of_center", star_is_out_of_center);
        config().blackboard->get<double>("star_x", star_x);
        config().blackboard->get<double>("star_y", star_y);

        // 基地信息
        config().blackboard->get<int>("base_name", base_name);
        config().blackboard->get<bool>("base_is_exist", base_is_exist);
        config().blackboard->get<bool>("base_is_out_of_center", base_is_out_of_center);
        config().blackboard->get<double>("base_x", base_x);
        config().blackboard->get<double>("base_y", base_y);

        // 敌方基地信息
        config().blackboard->get<int>("enemy_base_name", enemy_base_name);
        config().blackboard->get<bool>("enemy_base_is_exist", enemy_base_is_exist);
        config().blackboard->get<bool>("enemy_base_is_out_of_center", enemy_base_is_out_of_center);
        config().blackboard->get<double>("enemy_base_x", enemy_base_x);
        config().blackboard->get<double>("enemy_base_y", enemy_base_y);

        // 哨兵信息
        config().blackboard->get<int>("sentry_name", sentry_name);
        config().blackboard->get<bool>("sentry_is_exist", sentry_is_exist);
        config().blackboard->get<bool>("sentry_is_out_of_center", sentry_is_out_of_center);
        config().blackboard->get<double>("sentry_x", sentry_x);
        config().blackboard->get<double>("sentry_y", sentry_y);

        // 紫色入口信息
        config().blackboard->get<int>("purple_entry_name", purpleentry_name);
        config().blackboard->get<bool>("purple_entry_is_exist", purpleentry_is_exist);
        config().blackboard->get<bool>("purple_entry_is_out_of_center", purpleentry_is_out_of_center);
        config().blackboard->get<double>("purple_entry_x", purpleentry_x);
        config().blackboard->get<double>("purple_entry_y", purpleentry_y);

        // 绿色入口信息
        config().blackboard->get<int>("green_entry_name", greenentry_name);
        config().blackboard->get<bool>("green_entry_is_exist", greenentry_is_exist);
        config().blackboard->get<bool>("green_entry_is_out_of_center", greenentry_is_out_of_center);
        config().blackboard->get<double>("green_entry_x", greenentry_x);
        config().blackboard->get<double>("green_entry_y", greenentry_y);

        // 紫色出口信息
        config().blackboard->get<int>("purple_exit_name", purpleexit_name);
        config().blackboard->get<bool>("purple_exit_is_exist", purpleexit_is_exist);
        config().blackboard->get<bool>("purple_exit_is_out_of_center", purpleexit_is_out_of_center);
        config().blackboard->get<double>("purple_exit_x", purpleexit_x);
        config().blackboard->get<double>("purple_exit_y", purpleexit_y);

        // 绿色出口信息
        config().blackboard->get<int>("green_exit_name", greenexit_name);
        config().blackboard->get<bool>("green_exit_is_exist", greenexit_is_exist);
        config().blackboard->get<bool>("green_exit_is_out_of_center", greenexit_is_out_of_center);
        config().blackboard->get<double>("green_exit_x", greenexit_x);
        config().blackboard->get<double>("green_exit_y", greenexit_y);

        // 敌方单位信息
        config().blackboard->get<int>("enemy_name", enemy_name);
        config().blackboard->get<bool>("enemy_is_exist", enemy_is_exist);
        config().blackboard->get<bool>("enemy_is_out_of_center", enemy_is_out_of_center);
        config().blackboard->get<double>("enemy_x", enemy_x);
        config().blackboard->get<double>("enemy_y", enemy_y);

        // 其他数据
        config().blackboard->get<int>("enemy_num", enemy_num);
        config().blackboard->get<double>("sentry_hp", sentry_hp);
        config().blackboard->get<bool>("is_transfering", is_transfering);
        config().blackboard->get<bool>("is_bullet_low", is_bullet_low);
        config().blackboard->get<nav_msgs::msg::OccupancyGrid>("map_data", latest_map);
        config().blackboard->get<nav_msgs::msg::Odometry>("robot_pose", latest_odom);
        config().blackboard->get<bool>("is_out_of_center", is_out_of_center);

        // 1.订阅地图
        rclcpp::QoS map_qos(10);
        map_qos.transient_local();
        map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            map_qos,
            std::bind(&UpdateMapinfo::mapCallback, this, std::placeholders::_1));

        // 2. 订阅 /Odometry (Odometry)
        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry",
            100000,
            std::bind(&UpdateMapinfo::odomCallback, this, std::placeholders::_1));

        // 3. 订阅 /map_info
        map_info_sub_ = node_->create_subscription<robot_msgs::msg::MapInfoMsgs>(
            "/map_info",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&UpdateMapinfo::mapInfoCallback, this, std::placeholders::_1));

        isblock_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/isblock",
            10,
            std::bind(&UpdateMapinfo::blockCallback, this, std::placeholders::_1));

        RCLCPP_INFO(node_->get_logger(), "MapSubscriber node initialized");
    }

    UpdateMapinfo::~UpdateMapinfo() {}

    void UpdateMapinfo::InitMap()
    {
        RCLCPP_INFO(node_->get_logger(), "Initializing map targets...");

        // 星星信息
        star_name = TargetType::STAR;
        star_is_exist = false;
        star_is_out_of_center = false;
        star_x = 0.0;
        star_y = 0.0;

        // 基地信息
        base_name = TargetType::BASE;
        base_is_exist = false;
        base_is_out_of_center = false;
        base_x = 0.0;
        base_y = 0.0;

        // 敌方基地信息
        enemy_base_name = TargetType::ENEMY_BASE;
        enemy_base_is_exist = false;
        enemy_base_is_out_of_center = false;
        enemy_base_x = 0.0;
        enemy_base_y = 0.0;

        // 紫色入口信息
        purpleentry_name = TargetType::PURPLEENTRY;
        purpleentry_is_exist = false;
        purpleentry_is_out_of_center = false;
        purpleentry_x = 0.0;
        purpleentry_y = 0.0;

        // 绿色入口信息
        greenentry_name = TargetType::GREENENTRY;
        greenentry_is_exist = false;
        greenentry_is_out_of_center = false;
        greenentry_x = 0.0;
        greenentry_y = 0.0;

        // 哨兵信息
        sentry_name = TargetType::SENTRY;
        sentry_is_exist = false;
        sentry_is_out_of_center = false;
        sentry_x = 0.0;
        sentry_y = 0.0;

        // 敌方单位信息
        enemy_name = TargetType::ENEMY;
        enemy_is_exist = false;
        enemy_is_out_of_center = false;
        enemy_x = 0.0;
        enemy_y = 0.0;

        // 紫色出口信息
        purpleexit_name = TargetType::PURPLEEXIT;
        purpleexit_is_exist = false;
        purpleexit_is_out_of_center = false;
        purpleexit_x = 0.0;
        purpleexit_y = 0.0;

        // 绿色出口信息
        greenexit_name = TargetType::GREENEXIT;
        greenexit_is_exist = false;
        greenexit_is_out_of_center = false;
        greenexit_x = 0.0;
        greenexit_y = 0.0;

        RCLCPP_INFO(node_->get_logger(), "Map initialization complete");
    }

    void UpdateMapinfo::updatemsg(TargetType type, double x, double y, bool is_exist, bool is_out_of_center)
    {
        switch (type)
        {
        case TargetType::STAR:
            star_name = type;
            star_is_exist = is_exist;
            star_is_out_of_center = is_out_of_center;
            star_x = x;
            star_y = y;
            break;
        case TargetType::BASE:
            base_name = type;
            base_is_exist = is_exist;
            base_is_out_of_center = is_out_of_center;
            base_x = x;
            base_y = y;
            break;
        case TargetType::ENEMY_BASE:
            enemy_base_name = type;
            enemy_base_is_exist = is_exist;
            enemy_base_is_out_of_center = is_out_of_center;
            enemy_base_x = x;
            enemy_base_y = y;
            break;
        case TargetType::PURPLEENTRY:
            purpleentry_name = type;
            purpleentry_is_exist = is_exist;
            purpleentry_is_out_of_center = is_out_of_center;
            purpleentry_x = x;
            purpleentry_y = y;
            break;
        case TargetType::GREENENTRY:
            greenentry_name = type;
            greenentry_is_exist = is_exist;
            greenentry_is_out_of_center = is_out_of_center;
            greenentry_x = x;
            greenentry_y = y;
            break;
        case TargetType::SENTRY:
            sentry_name = type;
            sentry_is_exist = is_exist;
            sentry_is_out_of_center = is_out_of_center;
            sentry_x = x;
            sentry_y = y;
            break;
        case TargetType::ENEMY:
            enemy_name = type;
            enemy_is_exist = is_exist;
            enemy_is_out_of_center = is_out_of_center;
            enemy_x = x;
            enemy_y = y;
            break;
        case TargetType::PURPLEEXIT:
            purpleexit_name = type;
            purpleexit_is_exist = is_exist;
            purpleexit_is_out_of_center = is_out_of_center;
            purpleexit_x = x;
            purpleexit_y = y;
            break;
        case TargetType::GREENEXIT:
            greenexit_name = type;
            greenexit_is_exist = is_exist;
            greenexit_is_out_of_center = is_out_of_center;
            greenexit_x = x;
            greenexit_y = y;
            break;
        }
    }

    //========================================================================//
    void UpdateMapinfo::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        latest_map = *msg;
        map_received = true;
        // RCLCPP_DEBUG(node_->get_logger(), "Map received: %dx%d",
        //              msg->info.width, msg->info.height);
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
                      msg->map_info[i].is_exist,
                      msg->map_info[i].is_out_of_center);
        }

        enemy_num = msg->enemy_num;
        sentry_hp = msg->sentry_hp;
        is_transfering = msg->is_transfering;
        is_bullet_low = msg->is_bullet_low;

        map_info_received = true;
        // RCLCPP_INFO(node_->get_logger(), "接受信息:x=%.2f, y=%.2f", sentry_x, sentry_y);
    };

    void UpdateMapinfo::blockCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        isblock = msg->data;
    };

    BT::NodeStatus UpdateMapinfo::tick()
    {
        rclcpp::spin_some(node_);
        std::lock_guard<std::mutex> lock(data_mutex);
        if (!IsInit)
        {
            InitMap();
            IsInit = true;
        }

        if (!map_received || !odom_received || !map_info_received)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "等待数据: map=%d, odom=%d, map_info=%d",
                        map_received, odom_received, map_info_received);
            return BT::NodeStatus::SUCCESS;
        }

        // 输出拆分为基本类型的数据
        // 星星信息
        config().blackboard->set("star_name", star_name);
        config().blackboard->set("star_is_exist", star_is_exist);
        config().blackboard->set("star_is_out_of_center", star_is_out_of_center);
        config().blackboard->set("star_x", star_x);
        config().blackboard->set("star_y", star_y);

        // 基地信息
        config().blackboard->set("base_name", base_name);
        config().blackboard->set("base_is_exist", base_is_exist);
        config().blackboard->set("base_is_out_of_center", base_is_out_of_center);
        config().blackboard->set("base_x", base_x);
        config().blackboard->set("base_y", base_y);

        // 敌方基地信息
        config().blackboard->set("enemy_base_name", enemy_base_name);
        config().blackboard->set("enemy_base_is_exist", enemy_base_is_exist);
        config().blackboard->set("enemy_base_is_out_of_center", enemy_base_is_out_of_center);
        config().blackboard->set("enemy_base_x", enemy_base_x);
        config().blackboard->set("enemy_base_y", enemy_base_y);

        // 哨兵信息
        config().blackboard->set("sentry_name", sentry_name);
        config().blackboard->set("sentry_is_exist", sentry_is_exist);
        config().blackboard->set("sentry_is_out_of_center", sentry_is_out_of_center);
        config().blackboard->set("sentry_x", sentry_x);
        config().blackboard->set("sentry_y", sentry_y);

        // 紫色入口信息
        config().blackboard->set("purple_entry_name", purpleentry_name);
        config().blackboard->set("purple_entry_is_exist", purpleentry_is_exist);
        config().blackboard->set("purple_entry_is_out_of_center", purpleentry_is_out_of_center);
        config().blackboard->set("purple_entry_x", purpleentry_x);
        config().blackboard->set("purple_entry_y", purpleentry_y);

        // 绿色入口信息
        config().blackboard->set("green_entry_name", greenentry_name);
        config().blackboard->set("green_entry_is_exist", greenentry_is_exist);
        config().blackboard->set("green_entry_is_out_of_center", greenentry_is_out_of_center);
        config().blackboard->set("green_entry_x", greenentry_x);
        config().blackboard->set("green_entry_y", greenentry_y);

        // 紫色出口信息
        config().blackboard->set("purple_exit_name", purpleexit_name);
        config().blackboard->set("purple_exit_is_exist", purpleexit_is_exist);
        config().blackboard->set("purple_exit_is_out_of_center", purpleexit_is_out_of_center);
        config().blackboard->set("purple_exit_x", purpleexit_x);
        config().blackboard->set("purple_exit_y", purpleexit_y);

        // 绿色出口信息
        config().blackboard->set("green_exit_name", greenexit_name);
        config().blackboard->set("green_exit_is_exist", greenexit_is_exist);
        config().blackboard->set("green_exit_is_out_of_center", greenexit_is_out_of_center);
        config().blackboard->set("green_exit_x", greenexit_x);
        config().blackboard->set("green_exit_y", greenexit_y);

        // 敌方单位信息
        config().blackboard->set("enemy_name", enemy_name);
        config().blackboard->set("enemy_is_exist", enemy_is_exist);
        config().blackboard->set("enemy_is_out_of_center", enemy_is_out_of_center);
        config().blackboard->set("enemy_x", enemy_x);
        config().blackboard->set("enemy_y", enemy_y);

        config().blackboard->set("enemy_num", enemy_num);
        config().blackboard->set("sentry_hp", sentry_hp);
        config().blackboard->set("is_transfering", is_transfering);
        config().blackboard->set("is_bullet_low", is_bullet_low);

        config().blackboard->set("map_data", latest_map);
        config().blackboard->set("robot_pose", latest_odom);
        config().blackboard->set("map_ready", true);
        config().blackboard->set("is_out_of_center", is_out_of_center);
        config().blackboard->set<bool>("isblock", isblock);
        if (sentry_is_out_of_center)
        {
            RCLCPP_INFO(node_->get_logger(), "哨兵在外");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "哨兵在中心");
        }
        RCLCPP_INFO(node_->get_logger(), "目标星星%f,%f",star_x,star_y);
        return BT::NodeStatus::SUCCESS;
    }

}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::UpdateMapinfo>("UpdateMapinfo");
}