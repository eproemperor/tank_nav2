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
#define YELLOW "\033[33m"

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

namespace nav2_behavior_tree
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
                // 星星信息
                BT::OutputPort<int>("star_name", "星星类型"),
                BT::OutputPort<bool>("star_is_exist", "星星是否存在"),
                BT::OutputPort<bool>("star_is_out_of_center", "星星是否在中心"),
                BT::OutputPort<double>("star_x", "星星X坐标"),
                BT::OutputPort<double>("star_y", "星星Y坐标"),

                // 基地信息
                BT::OutputPort<int>("base_name", "基地类型"),
                BT::OutputPort<bool>("base_is_exist", "基地是否存在"),
                BT::OutputPort<bool>("base_is_out_of_center", "基地是否在中心"),
                BT::OutputPort<double>("base_x", "基地X坐标"),
                BT::OutputPort<double>("base_y", "基地Y坐标"),

                // 敌方基地信息
                BT::OutputPort<int>("enemy_base_name", "敌方基地类型"),
                BT::OutputPort<bool>("enemy_base_is_exist", "敌方基地是否存在"),
                BT::OutputPort<bool>("enemy_base_is_out_of_center", "敌方基地是否在中心"),
                BT::OutputPort<double>("enemy_base_x", "敌方基地X坐标"),
                BT::OutputPort<double>("enemy_base_y", "敌方基地Y坐标"),

                // 哨兵信息
                BT::OutputPort<int>("sentry_name", "哨兵类型"),
                BT::OutputPort<bool>("sentry_is_exist", "哨兵是否存在"),
                BT::OutputPort<bool>("sentry_is_out_of_center", "哨兵是否在中心"),
                BT::OutputPort<double>("sentry_x", "哨兵X坐标"),
                BT::OutputPort<double>("sentry_y", "哨兵Y坐标"),

                // 紫色入口
                BT::OutputPort<int>("purple_entry_name", "紫色入口类型"),
                BT::OutputPort<bool>("purple_entry_is_exist", "紫色入口是否存在"),
                BT::OutputPort<bool>("purple_entry_is_out_of_center", "紫色入口是否在中心"),
                BT::OutputPort<double>("purple_entry_x", "紫色入口X坐标"),
                BT::OutputPort<double>("purple_entry_y", "紫色入口Y坐标"),

                // 绿色入口
                BT::OutputPort<int>("green_entry_name", "绿色入口类型"),
                BT::OutputPort<bool>("green_entry_is_exist", "绿色入口是否存在"),
                BT::OutputPort<bool>("green_entry_is_out_of_center", "绿色入口是否在中心"),
                BT::OutputPort<double>("green_entry_x", "绿色入口X坐标"),
                BT::OutputPort<double>("green_entry_y", "绿色入口Y坐标"),

                // 紫色出口
                BT::OutputPort<int>("purple_exit_name", "紫色出口类型"),
                BT::OutputPort<bool>("purple_exit_is_exist", "紫色出口是否存在"),
                BT::OutputPort<bool>("purple_exit_is_out_of_center", "紫色出口是否在中心"),
                BT::OutputPort<double>("purple_exit_x", "紫色出口X坐标"),
                BT::OutputPort<double>("purple_exit_y", "紫色出口Y坐标"),

                // 绿色出口
                BT::OutputPort<int>("green_exit_name", "绿色出口类型"),
                BT::OutputPort<bool>("green_exit_is_exist", "绿色出口是否存在"),
                BT::OutputPort<bool>("green_exit_is_out_of_center", "绿色出口是否在中心"),
                BT::OutputPort<double>("green_exit_x", "绿色出口X坐标"),
                BT::OutputPort<double>("green_exit_y", "绿色出口Y坐标"),

                // 敌方单位
                BT::OutputPort<int>("enemy_name", "敌方类型"),
                BT::OutputPort<bool>("enemy_is_exist", "敌方是否存在"),
                BT::OutputPort<bool>("enemy_is_out_of_center", "敌方是否在中心"),
                BT::OutputPort<double>("enemy_x", "敌方X坐标"),
                BT::OutputPort<double>("enemy_y", "敌方Y坐标"),

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
        int sendpasmode = 0;
        bool is_out_of_center;

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

        void InitMap();
        void updatemsg(TargetType type, double x, double y, bool is_exist, bool is_out_of_center);

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