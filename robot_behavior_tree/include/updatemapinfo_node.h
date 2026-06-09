/*
"action":
更新地图信息  UpdateMapinfo
    初始化：哨兵出生点，敌方步兵出生点，补给区，中央蓝色区域，传送点，基地，迷宫
    每次轮询：更新敌人位置，更新哨兵位置,发送到黑板上

# MapInfo.msg
bool is_exist
bool is_out_of_center
geometry_msgs/Pose2D pos

# MapInfoMsgs.msg
MapInfo[] map_info
int32 enemy_num
float64 sentry_hp
bool is_transfering
bool is_bullet_low

*/

#ifndef UPDATEMAPINFO_NODEH
#define UPDATEMAPINFO_NODEH

#include <cmath>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_msgs/msg/map_info.hpp"
#include "robot_msgs/msg/map_info_msgs.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

#define FIAl 0
#define MYSUCCESS 1
#define YES 1

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
    bool is_out_of_center;
    double p_x;
    double p_y;
    double p_z = 0.0;
};

namespace updatemap
{

    /* 初始化：哨兵出生点，敌方步兵出生点，补给区，中央蓝色区域，传送点，基地，迷宫
        每次轮询：更新敌人位置，更新哨兵位置*/
    class UpdateMapinfo : public BT::SyncActionNode
    {

    public:
        UpdateMapinfo(const std::string &action_name,
                      const BT::NodeConfiguration &conf);
        UpdateMapinfo() = delete; // 删除默认构造函数

        ~UpdateMapinfo();

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::OutputPort<TypeMode>("star_info", "星星信息"),
                BT::OutputPort<TypeMode>("base_info", "基地信息"),
                BT::OutputPort<TypeMode>("enemy_base_info", "敌方基地信息"),
                BT::OutputPort<TypeMode>("sentry_info", "哨兵信息"),
                BT::OutputPort<TypeMode>("purple_entry_info", "紫色入口信息"),
                BT::OutputPort<TypeMode>("green_entry_info", "绿色入口信息"),
                BT::OutputPort<TypeMode>("purple_exit_info", "紫色出口信息"),
                BT::OutputPort<TypeMode>("green_exit_info", "绿色出口信息"),
                BT::OutputPort<TypeMode>("enemy_info", "敌方单位信息"),
                BT::OutputPort<int>("enemy_num", "敌人数量"),
                BT::OutputPort<double>("sentry_hp", "哨兵血量"),
                BT::OutputPort<bool>("is_transfering", "是否正在传送"),
                BT::OutputPort<bool>("is_bullet_low", "子弹是否不足"),
                BT::OutputPort<nav_msgs::msg::OccupancyGrid>("map_data", "地图数据"),
                BT::OutputPort<nav_msgs::msg::Odometry>("robot_pose", "机器人位姿"),
                BT::OutputPort<bool>("map_ready", "地图是否就绪"),
                BT::OutputPort<int>("sendpasmode", "密码发送模式")};
        }

    private:
        bool IsInit{false};
        bool map_received{false};
        bool odom_received{false};
        bool map_info_received{false};

        std::mutex data_mutex;

        int enemy_num;       // 当前场上的敌人数量
        double sentry_hp;    // 当前哨兵血量（double型）
        bool is_transfering; // 哨兵是否在传送门位置附近
        bool is_bullet_low;  // 当前子弹是否低于一定阈值
        int sendpasmode=0;
        bool is_out_of_center;

        TypeMode star;
        TypeMode base;
        TypeMode enemy_base;
        TypeMode purpleentry;
        TypeMode greenentry;
        TypeMode sentry;
        TypeMode enemy;
        TypeMode purpleexit;
        TypeMode greenexit;

        void InitMap();
        void updatemsg(TargetType type, double x, double y, bool is_exist,bool is_out_of_center);

        // 回调函数声明
        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void mapInfoCallback(const robot_msgs::msg::MapInfoMsgs::SharedPtr msg);

        // 订阅者成员变量
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<robot_msgs::msg::MapInfoMsgs>::SharedPtr map_info_sub_;

        rclcpp::Node::SharedPtr node_;
        nav_msgs::msg::OccupancyGrid latest_map;
        nav_msgs::msg::Odometry latest_odom;
        robot_msgs::msg::MapInfoMsgs latest_map_info;
    };

};

#endif