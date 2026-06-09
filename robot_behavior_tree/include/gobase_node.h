/*输入：目标位置
    流程：前往下一个敌人（每”UPDATEEMERYPOSITION“时间更新目标点）-->检测血量&子弹数量-->（前往补给区）-->寻找第二个敌人（每”UPDATEEMERYPOSITION“时间更新目标点）
        -->前往传送点-->前往密码发区-->前往传送点-->前往基地
        只给点，路径由规划器决定，移动由规划器控制
*/

#ifndef GOBASE_NODE_H
#define GOBASE_NODE_H

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

    class Gobase : public BT::SyncActionNode
    {
    public:
        Gobase(const std::string &action_name,
               const BT::NodeConfiguration &conf);
        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                // 输入端口（从黑板读取数据）
                BT::InputPort<int>("star_name", "星星类型"),
                BT::InputPort<bool>("star_is_exist", "星星是否存在"),
                BT::InputPort<double>("star_x", "星星X坐标"),
                BT::InputPort<double>("star_y", "星星Y坐标"),
                
                BT::InputPort<int>("base_name", "基地类型"),
                BT::InputPort<bool>("base_is_exist", "基地是否存在"),
                BT::InputPort<double>("base_x", "基地X坐标"),
                BT::InputPort<double>("base_y", "基地Y坐标"),
                
                BT::InputPort<int>("enemy_base_name", "敌方基地类型"),
                BT::InputPort<bool>("enemy_base_is_exist", "敌方基地是否存在"),
                BT::InputPort<double>("enemy_base_x", "敌方基地X坐标"),
                BT::InputPort<double>("enemy_base_y", "敌方基地Y坐标"),
                
                BT::InputPort<int>("purple_entry_name", "紫色入口类型"),
                BT::InputPort<bool>("purple_entry_is_exist", "紫色入口是否存在"),
                BT::InputPort<double>("purple_entry_x", "紫色入口X坐标"),
                BT::InputPort<double>("purple_entry_y", "紫色入口Y坐标"),
                
                BT::InputPort<int>("green_entry_name", "绿色入口类型"),
                BT::InputPort<bool>("green_entry_is_exist", "绿色入口是否存在"),
                BT::InputPort<double>("green_entry_x", "绿色入口X坐标"),
                BT::InputPort<double>("green_entry_y", "绿色入口Y坐标"),
                
                BT::InputPort<int>("sentry_name", "哨兵类型"),
                BT::InputPort<bool>("sentry_is_exist", "哨兵是否存在"),
                BT::InputPort<double>("sentry_x", "哨兵X坐标"),
                BT::InputPort<double>("sentry_y", "哨兵Y坐标"),
                
                BT::InputPort<int>("enemy_name", "敌方类型"),
                BT::InputPort<bool>("enemy_is_exist", "敌方是否存在"),
                BT::InputPort<double>("enemy_x", "敌方X坐标"),
                BT::InputPort<double>("enemy_y", "敌方Y坐标"),
                
                BT::InputPort<int>("purple_exit_name", "紫色出口类型"),
                BT::InputPort<bool>("purple_exit_is_exist", "紫色出口是否存在"),
                BT::InputPort<double>("purple_exit_x", "紫色出口X坐标"),
                BT::InputPort<double>("purple_exit_y", "紫色出口Y坐标"),
                
                BT::InputPort<int>("green_exit_name", "绿色出口类型"),
                BT::InputPort<bool>("green_exit_is_exist", "绿色出口是否存在"),
                BT::InputPort<double>("green_exit_x", "绿色出口X坐标"),
                BT::InputPort<double>("green_exit_y", "绿色出口Y坐标"),
                
                BT::InputPort<int>("enemy_num", "敌人数量"),
                BT::InputPort<double>("sentry_hp", "哨兵血量"),
                BT::InputPort<bool>("is_bullet_low", "子弹是否不足"),
                BT::InputPort<int>("sendpasmode", "密码发送模式"),
                
                // 输出端口
                BT::OutputPort<double>("goal_x", "目标点x坐标"),
                BT::OutputPort<double>("goal_y", "目标点y坐标")};
        }

    private:
        // 星星信息
        int star_name;
        bool star_is_exist;
        double star_x;
        double star_y;
        
        // 基地信息
        int base_name;
        bool base_is_exist;
        double base_x;
        double base_y;
        
        // 敌方基地信息
        int enemy_base_name;
        bool enemy_base_is_exist;
        double enemy_base_x;
        double enemy_base_y;
        
        // 紫色入口信息
        int purpleentry_name;
        bool purpleentry_is_exist;
        double purpleentry_x;
        double purpleentry_y;
        
        // 绿色入口信息
        int greenentry_name;
        bool greenentry_is_exist;
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
        double enemy_x;
        double enemy_y;
        
        // 紫色出口信息
        int purpleexit_name;
        bool purpleexit_is_exist;
        double purpleexit_x;
        double purpleexit_y;
        
        // 绿色出口信息
        int greenexit_name;
        bool greenexit_is_exist;
        double greenexit_x;
        double greenexit_y;
        
        rclcpp::Node::SharedPtr node_;
        int sendpasmode;
        int enemy_num;
        double sentry_hp;
        bool is_bullet_low;
        double goal_x;
        double goal_y;

        int setTargetType();
        
        void updateBlackboardData();
    };
}

#endif