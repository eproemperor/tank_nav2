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

    bool loadBehaviorTree(const std::string &bt_xml_filename, BT::Blackboard::Ptr blackboard);

};

#endif //ROBOT_BEHAVIOR_TREE_ROBOT_BT_DECISION_MAKER_HPP
