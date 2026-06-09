#ifndef FIRE_NODE_H
#define FIRE_NODE_H

#include "robot_serial.h"
#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"

#define YES 1
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

    class Fire : public BT::SyncActionNode
    {

    public:
        Fire(const std::string &action_name,
             const BT::NodeConfiguration &conf);
        ~Fire();

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<bool>("is_bullet_low", "子弹是否不足"),
                // 敌方基地信息
                BT::InputPort<int>("enemy_base_name", "敌方基地类型"),
                BT::InputPort<bool>("enemy_base_is_exist", "敌方基地是否存在"),
                BT::InputPort<double>("enemy_base_x", "敌方基地X坐标"),
                BT::InputPort<double>("enemy_base_y", "敌方基地Y坐标"),
                // 哨兵信息
                BT::InputPort<int>("sentry_name", "哨兵类型"),
                BT::InputPort<bool>("sentry_is_exist", "哨兵是否存在"),
                BT::InputPort<double>("sentry_x", "哨兵X坐标"),
                BT::InputPort<double>("sentry_y", "哨兵Y坐标"),
                // 敌方单位信息
                BT::InputPort<int>("enemy_name", "敌方类型"),
                BT::InputPort<bool>("enemy_is_exist", "敌方是否存在"),
                BT::InputPort<double>("enemy_x", "敌方X坐标"),
                BT::InputPort<double>("enemy_y", "敌方Y坐标"),
                BT::InputPort<int>("enemy_num", "敌人数量")};
        }

    private:
        void fire(double theta);
        void updateposition(); // 更新数据
        double calangle(double &x1, double &y1, double &x2, double &y2);
        
        int enemy_num;
        double theta;
        
        // 敌方基地信息（拆分为基本类型）
        int enemy_base_name;
        bool enemy_base_is_exist;
        double enemy_base_x;
        double enemy_base_y;
        
        // 哨兵信息（拆分为基本类型）
        int sentry_name;
        bool sentry_is_exist;
        double sentry_x;
        double sentry_y;
        
        // 敌方单位信息（拆分为基本类型）
        int enemy_name;
        bool enemy_is_exist;
        double enemy_x;
        double enemy_y;
        
        rclcpp::Node::SharedPtr node_;
        messageprocess::RobotMsgProcess RobotMsgProcess_;
    };
}
#endif