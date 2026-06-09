/*输入：目标位置
    流程：前往下一个敌人（每”UPDATEEMERYPOSITION“时间更新目标点）-->检测血量&子弹数量-->（前往补给区）-->寻找第二个敌人（每”UPDATEEMERYPOSITION“时间更新目标点）
        -->前往传送点-->前往密 码发区-->前往传送点-->前往基地
        只给点，路径由规划器决定，移动由规划器控制
*/

#ifndef GOBASE_NODE.H
#define GOBASE_NODE .H

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_behavior_tree
{
    enum TargetType : uint8_t
    {
        STAR = 0,        // 星星/星标
        BASE = 1,        // 基地
        ENEMY_BASE = 2,  // 敌方基地
        PURPLEENTRY = 3, // 紫色入口
        GREENENTRY = 4,  // 绿色入口
        SENTRY = 5,      // 哨兵
        ENEMY = 6,       // 敌方单位
        PURPLEEXIT = 7,  // 紫色出口
        GREENEXIT = 8,   // 绿色出口
    };

    struct TypeMode
    {
        TargetType name;
        bool is_exist = true;
        bool is_out_of_center;
        double p_x;
        double p_y;
        double p_z = 0.0;
    };

    class Gobase : public BT::SyncActionNode
    {
    public:
        Gobase(const std::string &action_name,
               const BT::NodeConfiguration &conf);
        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::OutputPort<double>("goal_x", "·目标点x坐标"),
                BT::OutputPort<double>("goal_y", "目标点y坐标")};
        }

    private:
        TypeMode star;
        TypeMode base;
        TypeMode enemy_base;
        TypeMode purpleentry;
        TypeMode greenentry;
        TypeMode sentry;
        TypeMode enemy;
        TypeMode purpleexit;
        TypeMode greenexit;
        rclcpp::Node::SharedPtr node_;
        int sendpasmode;
        int enemy_num;
        double sentry_hp;
        bool is_bullet_low;
        double goal_x;
        double goal_y;

        int setTargetType();
    };
}

#endif