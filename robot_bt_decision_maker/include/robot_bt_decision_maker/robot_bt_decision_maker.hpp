//
// Created by elsa on 2026/5/17.
//

#ifndef ROBOT_BEHAVIOR_TREE_ROBOT_BT_DECISION_MAKER_HPP
#define ROBOT_BEHAVIOR_TREE_ROBOT_BT_DECISION_MAKER_HPP

#include <nav2_behavior_tree/behavior_tree_engine.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "updatemapinfo_node.h"

class DecisionMakerNode : public rclcpp::Node
{
public:
    DecisionMakerNode(std::string name);

    nav2_behavior_tree::BtStatus runBehaviorTree();

    void waitNav2();

private:
    // Parameters
    int loop_duration_in_millisec_;
    int server_timeout_in_millisec_;
    std::vector<std::string> plugin_lib_names_;
    std::string bt_xml_filename_;

    std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_;
    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;
    std::chrono::milliseconds bt_loop_duration_;
    std::chrono::milliseconds server_timeout_;
    std::string client_node_name_;
    rclcpp::Node::SharedPtr client_node_;
    geometry_msgs::msg::PoseStamped pose;

    // ================自定义类型节点=====================
    // 星星/星标信息
    int star_name;
    bool star_is_exist;
    bool star_is_out_of_center;
    double star_x;
    double star_y;
    
    // 基地信息
    int base_name;
    bool base_is_exist;
    bool base_is_out_of_center;
    double base_x;
    double base_y;
    
    // 敌方基地信息
    int enemy_base_name;
    bool enemy_base_is_exist;
    bool enemy_base_is_out_of_center;
    double enemy_base_x;
    double enemy_base_y;
    
    // 紫色入口信息
    int purpleentry_name;
    bool purpleentry_is_exist;
    bool purpleentry_is_out_of_center;
    double purpleentry_x;
    double purpleentry_y;
    
    // 绿色入口信息
    int greenentry_name;
    bool greenentry_is_exist;
    bool greenentry_is_out_of_center;
    double greenentry_x;
    double greenentry_y;
    
    // 哨兵信息
    int sentry_name;
    bool sentry_is_exist;
    bool sentry_is_out_of_center;
    double sentry_x;
    double sentry_y;
    
    // 敌方单位信息
    int enemy_name;
    bool enemy_is_exist;
    bool enemy_is_out_of_center;
    double enemy_x;
    double enemy_y;
    
    // 紫色出口信息
    int purpleexit_name;
    bool purpleexit_is_exist;
    bool purpleexit_is_out_of_center;
    double purpleexit_x;
    double purpleexit_y;
    
    // 绿色出口信息
    int greenexit_name;
    bool greenexit_is_exist;
    bool greenexit_is_out_of_center;
    double greenexit_x;
    double greenexit_y;


    nav_msgs::msg::OccupancyGrid latest_map;
    nav_msgs::msg::Odometry latest_odom;
    uint64_t Password1;
    uint64_t Password2;
    int64_t Password_rec;
    int sendpasmode;
    bool startflag{false};
    bool is_out_of_center;
    double navvelre;
    double goal_x;
    double goal_y;

    bool loadBehaviorTree(const std::string &bt_xml_filename, BT::Blackboard::Ptr blackboard);
    void registerBehaviorTreeNodes(BT::BehaviorTreeFactory &factory);
};

#endif // ROBOT_BEHAVIOR_TREE_ROBOT_BT_DECISION_MAKER_HPP
