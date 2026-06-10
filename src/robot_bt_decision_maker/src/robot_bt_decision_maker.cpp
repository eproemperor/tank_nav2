#include "robot_bt_decision_maker/robot_bt_decision_maker.hpp"

// 构造函数,有一个参数为节点名称
DecisionMakerNode::DecisionMakerNode(std::string name) : Node(name)
{
    RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
    this->declare_parameter("loop_duration_in_millisec", 10);
    this->get_parameter("loop_duration_in_millisec", loop_duration_in_millisec_);
    bt_loop_duration_ = std::chrono::milliseconds(loop_duration_in_millisec_);
    this->declare_parameter("server_timeout_in_millisec", 100);
    this->get_parameter("server_timeout_in_millisec", server_timeout_in_millisec_);
    server_timeout_ = std::chrono::milliseconds(server_timeout_in_millisec_);
    this->declare_parameter("plugin_lib_names", std::vector<std::string>());
    this->get_parameter("plugin_lib_names", plugin_lib_names_);
    this->declare_parameter("bt_xml_filename", std::string(""));
    this->get_parameter("bt_xml_filename", bt_xml_filename_);

    waitNav2();
    client_node_name_ = this->get_name();
    auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args",
         "-r",
         std::string("__node:=") +
             client_node_name_ + "_rclcpp_node",
         "--"});
    client_node_ = std::make_shared<rclcpp::Node>("_", options);

    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("robot_bt_decision_maker");
    bt_xml_filename_ = pkg_share_dir + bt_xml_filename_;
    bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface());
    blackboard_ = BT::Blackboard::create();
    // Put items on the blackboard
    blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);                    // NOLINT
    blackboard_->set<std::chrono::milliseconds>("server_timeout", server_timeout_);     // NOLINT
    blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_); // NOLINT

    // 自定义黑板变量在这边声明，注意声明类型和后续用的类型要一致，无自动类型转换
    // 星星信息
    blackboard_->set<int>("star_name", star_name);
    blackboard_->set<bool>("star_is_exist", star_is_exist);
    blackboard_->set<bool>("star_is_out_of_center", star_is_out_of_center);
    blackboard_->set<double>("star_x", star_x);
    blackboard_->set<double>("star_y", star_y);

    // 基地信息
    blackboard_->set<int>("base_name", base_name);
    blackboard_->set<bool>("base_is_exist", base_is_exist);
    blackboard_->set<bool>("base_is_out_of_center", base_is_out_of_center);
    blackboard_->set<double>("base_x", base_x);
    blackboard_->set<double>("base_y", base_y);

    // 敌方基地信息
    blackboard_->set<int>("enemy_base_name", enemy_base_name);
    blackboard_->set<bool>("enemy_base_is_exist", enemy_base_is_exist);
    blackboard_->set<bool>("enemy_base_is_out_of_center", enemy_base_is_out_of_center);
    blackboard_->set<double>("enemy_base_x", enemy_base_x);
    blackboard_->set<double>("enemy_base_y", enemy_base_y);

    // 哨兵信息
    blackboard_->set<int>("sentry_name", sentry_name);
    blackboard_->set<bool>("sentry_is_exist", sentry_is_exist);
    blackboard_->set<bool>("sentry_is_out_of_center", sentry_is_out_of_center);
    blackboard_->set<double>("sentry_x", sentry_x);
    blackboard_->set<double>("sentry_y", sentry_y);

    // 紫色入口信息
    blackboard_->set<int>("purple_entry_name", purpleentry_name);
    blackboard_->set<bool>("purple_entry_is_exist", purpleentry_is_exist);
    blackboard_->set<bool>("purple_entry_is_out_of_center", purpleentry_is_out_of_center);
    blackboard_->set<double>("purple_entry_x", purpleentry_x);
    blackboard_->set<double>("purple_entry_y", purpleentry_y);

    // 绿色入口信息
    blackboard_->set<int>("green_entry_name", greenentry_name);
    blackboard_->set<bool>("green_entry_is_exist", greenentry_is_exist);
    blackboard_->set<bool>("green_entry_is_out_of_center", greenentry_is_out_of_center);
    blackboard_->set<double>("green_entry_x", greenentry_x);
    blackboard_->set<double>("green_entry_y", greenentry_y);

    // 紫色出口信息
    blackboard_->set<int>("purple_exit_name", purpleexit_name);
    blackboard_->set<bool>("purple_exit_is_exist", purpleexit_is_exist);
    blackboard_->set<bool>("purple_exit_is_out_of_center", purpleexit_is_out_of_center);
    blackboard_->set<double>("purple_exit_x", purpleexit_x);
    blackboard_->set<double>("purple_exit_y", purpleexit_y);

    // 绿色出口信息
    blackboard_->set<int>("green_exit_name", greenexit_name);
    blackboard_->set<bool>("green_exit_is_exist", greenexit_is_exist);
    blackboard_->set<bool>("green_exit_is_out_of_center", greenexit_is_out_of_center);
    blackboard_->set<double>("green_exit_x", greenexit_x);
    blackboard_->set<double>("green_exit_y", greenexit_y);

    // 敌方单位信息
    blackboard_->set<int>("enemy_name", enemy_name);
    blackboard_->set<bool>("enemy_is_exist", enemy_is_exist);
    blackboard_->set<bool>("enemy_is_out_of_center", enemy_is_out_of_center);
    blackboard_->set<double>("enemy_x", enemy_x);
    blackboard_->set<double>("enemy_y", enemy_y);

    blackboard_->set<int>("enemy_num", 2);
    blackboard_->set<double>("sentry_hp", 100);
    blackboard_->set<bool>("is_transfering", false);
    blackboard_->set<bool>("is_bullet_low", false);
    blackboard_->set<nav_msgs::msg::OccupancyGrid>("map_data", latest_map);
    blackboard_->set<nav_msgs::msg::Odometry>("robot_pose", latest_odom);
    blackboard_->set<bool>("map_ready", false);
    blackboard_->set<int>("sendpasmode", 0);
    blackboard_->set<uint64_t>("Password1", Password1);
    blackboard_->set<uint64_t>("Password1", Password2);
    blackboard_->set<int64_t>("Password_rec", Password_rec);
    blackboard_->set<int>("sendpassmode", sendpasmode);
    blackboard_->set<bool>("startflag", startflag);
    blackboard_->set<bool>("is_out_of_center", is_out_of_center);
    blackboard_->set<double>("navvelre", navvelre);
    blackboard_->set<double>("goal_x", goal_x);
    blackboard_->set<double>("goal_y", goal_y);

    if (!loadBehaviorTree(bt_xml_filename_, blackboard_))
    {
        RCLCPP_ERROR(this->get_logger(), "加载行为树失败.");
        return;
    }
}

nav2_behavior_tree::BtStatus DecisionMakerNode::runBehaviorTree()
{
    auto is_canceling = [this]() -> bool
    {
        return false;
    };
    auto on_loop = [this]() -> void
    {
        // RCLCPP_INFO(this->get_logger(), "行为树正在运行...");
        rclcpp::spin_some(this->get_node_base_interface());
    };
    // Run the Behavior Tree
    return bt_->run(&tree_, on_loop, is_canceling, bt_loop_duration_);
}

void DecisionMakerNode::waitNav2()
{
    std::string node_service = "/bt_navigator/get_state";
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client = this->create_client<
        lifecycle_msgs::srv::GetState>(node_service);
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for bt_navigator to be available");
    }
}

bool DecisionMakerNode::loadBehaviorTree(const std::string &bt_xml_filename, BT::Blackboard::Ptr blackboard)
{
    // Read the input BT XML from the specified file into a string
    std::ifstream xml_file(bt_xml_filename);

    if (!xml_file.good())
    {
        RCLCPP_ERROR(this->get_logger(), "Couldn't open input XML file: %s", bt_xml_filename.c_str());
        return false;
    }

    // auto xml_string = std::string(
    // std::istreambuf_iterator<char>(xml_file),
    //  std::istreambuf_iterator<char>());

    // Create the Behavior Tree from the XML input
    try
    {
        BT::BehaviorTreeFactory factory;

        factory.registerNodeType<nav2_behavior_tree::UpdateMapinfo>("UpdateMapinfo");
        factory.registerNodeType<nav2_behavior_tree::IfGameStart>("IfGameStart");
        factory.registerNodeType<nav2_behavior_tree::Gobase>("GoBase");
        factory.registerNodeType<nav2_behavior_tree::Navvelremap>("Navvelremap");
        `factory.registerNodeType<nav2_behavior_tree::RSendPassward>("RSendPassward");
        //factory.registerNodeType<nav2_behavior_tree::Fire>("Fire");

        tree_ = factory.createTreeFromFile(bt_xml_filename, blackboard);

        for (auto &blackboard : tree_.blackboard_stack)
        {
            blackboard->set<rclcpp::Node::SharedPtr>("node", client_node_);                    // NOLINT
            blackboard->set<std::chrono::milliseconds>("server_timeout", server_timeout_);     // NOLINT
            blackboard->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_); // NOLINT

            // 自定义黑板变量在这边要再放一遍
            // 星星信息
            blackboard_->set<int>("star_name", star_name);
            blackboard_->set<bool>("star_is_exist", star_is_exist);
            blackboard_->set<bool>("star_is_out_of_center", star_is_out_of_center);
            blackboard_->set<double>("star_x", star_x);
            blackboard_->set<double>("star_y", star_y);

            // 基地信息
            blackboard_->set<int>("base_name", base_name);
            blackboard_->set<bool>("base_is_exist", base_is_exist);
            blackboard_->set<bool>("base_is_out_of_center", base_is_out_of_center);
            blackboard_->set<double>("base_x", base_x);
            blackboard_->set<double>("base_y", base_y);

            // 敌方基地信息
            blackboard_->set<int>("enemy_base_name", enemy_base_name);
            blackboard_->set<bool>("enemy_base_is_exist", enemy_base_is_exist);
            blackboard_->set<bool>("enemy_base_is_out_of_center", enemy_base_is_out_of_center);
            blackboard_->set<double>("enemy_base_x", enemy_base_x);
            blackboard_->set<double>("enemy_base_y", enemy_base_y);

            // 哨兵信息
            blackboard_->set<int>("sentry_name", sentry_name);
            blackboard_->set<bool>("sentry_is_exist", sentry_is_exist);
            blackboard_->set<bool>("sentry_is_out_of_center", sentry_is_out_of_center);
            blackboard_->set<double>("sentry_x", sentry_x);
            blackboard_->set<double>("sentry_y", sentry_y);

            // 紫色入口信息
            blackboard_->set<int>("purple_entry_name", purpleentry_name);
            blackboard_->set<bool>("purple_entry_is_exist", purpleentry_is_exist);
            blackboard_->set<bool>("purple_entry_is_out_of_center", purpleentry_is_out_of_center);
            blackboard_->set<double>("purple_entry_x", purpleentry_x);
            blackboard_->set<double>("purple_entry_y", purpleentry_y);

            // 绿色入口信息
            blackboard_->set<int>("green_entry_name", greenentry_name);
            blackboard_->set<bool>("green_entry_is_exist", greenentry_is_exist);
            blackboard_->set<bool>("green_entry_is_out_of_center", greenentry_is_out_of_center);
            blackboard_->set<double>("green_entry_x", greenentry_x);
            blackboard_->set<double>("green_entry_y", greenentry_y);

            // 紫色出口信息
            blackboard_->set<int>("purple_exit_name", purpleexit_name);
            blackboard_->set<bool>("purple_exit_is_exist", purpleexit_is_exist);
            blackboard_->set<bool>("purple_exit_is_out_of_center", purpleexit_is_out_of_center);
            blackboard_->set<double>("purple_exit_x", purpleexit_x);
            blackboard_->set<double>("purple_exit_y", purpleexit_y);

            // 绿色出口信息
            blackboard_->set<int>("green_exit_name", greenexit_name);
            blackboard_->set<bool>("green_exit_is_exist", greenexit_is_exist);
            blackboard_->set<bool>("green_exit_is_out_of_center", greenexit_is_out_of_center);
            blackboard_->set<double>("green_exit_x", greenexit_x);
            blackboard_->set<double>("green_exit_y", greenexit_y);

            // 敌方单位信息
            blackboard_->set<int>("enemy_name", enemy_name);
            blackboard_->set<bool>("enemy_is_exist", enemy_is_exist);
            blackboard_->set<bool>("enemy_is_out_of_center", enemy_is_out_of_center);
            blackboard_->set<double>("enemy_x", enemy_x);
            blackboard_->set<double>("enemy_y", enemy_y);

            blackboard_->set<int>("enemy_num", 2);
            blackboard_->set<double>("sentry_hp", 100);
            blackboard_->set<bool>("is_transfering", false);
            blackboard_->set<bool>("is_bullet_low", false);
            blackboard_->set<nav_msgs::msg::OccupancyGrid>("map_data", latest_map);
            blackboard_->set<nav_msgs::msg::Odometry>("robot_pose", latest_odom);
            blackboard_->set<bool>("map_ready", false);
            blackboard_->set<int>("sendpasmode", 0);
            blackboard_->set<uint64_t>("Password1", Password1);
            blackboard_->set<uint64_t>("Password1", Password2);
            blackboard_->set<int64_t>("Password_rec", Password_rec);
            blackboard_->set<int>("sendpassmode", sendpasmode);
            blackboard_->set<bool>("startflag", startflag);
            blackboard_->set<bool>("is_out_of_center", is_out_of_center);
            blackboard_->set<double>("navvelre", navvelre);
            blackboard_->set<double>("goal_x", goal_x);
            blackboard_->set<double>("goal_y", goal_y);
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Exception when loading BT: %s", e.what());
        return false;
    }
    return true;
}
