#ifndef FIRE_NODE_H
#define FIRE_NODE_H

#include "robot_serial.h"
#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"

#define YES 1
namespace fire_node
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
        bool is_exist = YES;
        double p_x;
        double p_y;
        double p_z = 0.0;
    };

    class Fire : public BT::SyncActionNode
    {

    public:
        Fire(const std::string &action_name,
             const BT::NodeConfiguration &conf);
        Fire() = delete; // 删除默认构造函数
        ~Fire();

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::OutputPort<bool>("is_bullet_low", "子弹是否不足")};
        }

    private:
        void fire(double theta);
        void updateposition();     //更新数据
        double calangle(double &x1, double &y1, double &x2, double &y2);
        int enemy_num;
        double theta;
        TypeMode enemy_base;
        TypeMode sentry;
        TypeMode enemy;
        messageprocess::RobotMsgProcess RobotMsgProcess_;
    };
}
#endif