//
// Created by elsa on 2026/5/7.
//

#ifndef SRC_IMG_PROCESS_HPP
#define SRC_IMG_PROCESS_HPP

#include <iostream>
#include <thread>
#include <cmath>
#include <array>
#include <random>
#include <queue>
#include <chrono>


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <example_interfaces/msg/bool.hpp>
#include "opencv4/opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>

#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "robot_msgs/msg/map_info.hpp"
#include "robot_msgs/msg/map_info_msgs.hpp"

enum TargetType : uint8_t {
    STAR = 0,
    BASE = 1,
    ENEMY_BASE = 2,
    PURPLEENTRY = 3,
    GREENENTRY = 4,
    SENTRY = 5,
    ENEMY = 6,
    PURPLEEXIT = 7,
    GREENEXIT = 8,
};

enum PixelStatus {
    REACHABLE = 0,
    UNEXPLORED = 50,
    OBSTACLE = 100
};


class ImgProcess : public rclcpp::Node {
public:
    ImgProcess();

    template<typename T>
    void set_posestamp(T &out);

    void set_map_info(const cv::Mat &Image, uint8_t type);

    void publish_map(const cv::Mat Image, const cv::Vec3b wallColor);

    void publish_map_info();

    void publish_sentry_odom(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped,
                             std::unique_ptr<tf2_ros::TransformBroadcaster> &tf_br);

    void imageCallback(sensor_msgs::msg::Image rosImage);

    void check_callback();

    // 内联函数：检查是否超出范围
    inline bool isOutOfRange(const cv::Point2f &pose) {
        if (mapInfo[STAR].is_out_of_center == false) {
            return true;
        }
        return pose.x / 40 <= mapInfo[STAR].pos.x - 2.5 ||
               pose.x / 40 >= mapInfo[STAR].pos.x + 2.5 ||
               (12.8 - pose.y / 40) <= mapInfo[STAR].pos.y - 1.5 ||
               (12.8 - pose.y / 40) >= mapInfo[STAR].pos.y + 1.5;
    }

    // 内联函数：检查是否
    inline bool isFarFromSentry(const cv::Point2f &pose) {
        return pose.x / 40 <= mapInfo[SENTRY].pos.x - 0.6 ||
               pose.x / 40 >= mapInfo[SENTRY].pos.x + 0.6 ||
               (12.8 - pose.y / 40) <= mapInfo[SENTRY].pos.y - 0.6 ||
               (12.8 - pose.y / 40) >= mapInfo[SENTRY].pos.y + 0.6;
    }

    inline bool isInDoor(const cv::Point2f &pose) {
        return (pose.x / 40 >= mapInfo[PURPLEENTRY].pos.x - 0.4 &&
                pose.x / 40 <= mapInfo[PURPLEENTRY].pos.x + 0.4 &&
                (12.8 - pose.y / 40) >= mapInfo[PURPLEENTRY].pos.y - 0.4 &&
                (12.8 - pose.y / 40) <= mapInfo[PURPLEENTRY].pos.y + 0.4) ||
               (pose.x / 40 >= mapInfo[GREENENTRY].pos.x - 0.4 &&
                pose.x / 40 <= mapInfo[GREENENTRY].pos.x + 0.4 &&
                (12.8 - pose.y / 40) >= mapInfo[GREENENTRY].pos.y - 0.4 &&
                (12.8 - pose.y / 40) <= mapInfo[GREENENTRY].pos.y + 0.4) ||
               (pose.x / 40 >= mapInfo[GREENEXIT].pos.x - 0.4 &&
                pose.x / 40 <= mapInfo[GREENEXIT].pos.x + 0.4 &&
                (12.8 - pose.y / 40) >= mapInfo[GREENEXIT].pos.y - 0.4 &&
                (12.8 - pose.y / 40) <= mapInfo[GREENEXIT].pos.y + 0.4) ||
               (pose.x / 40 >= mapInfo[PURPLEEXIT].pos.x - 0.4 &&
                pose.x / 40 <= mapInfo[PURPLEEXIT].pos.x + 0.4 &&
                (12.8 - pose.y / 40) >= mapInfo[PURPLEEXIT].pos.y - 0.4 &&
                (12.8 - pose.y / 40) <= mapInfo[PURPLEEXIT].pos.y + 0.4);
    }

    inline bool onOutDoor(const cv::Point2f &pose) {
        return (
            pose.x >= one_outdoor_pose.x - 16 &&
            pose.x <= one_outdoor_pose.x + 16 &&
            pose.y >= one_outdoor_pose.y - 16 &&
            pose.y <= one_outdoor_pose.y + 16
        );
    }

    // 内联函数：检查是否远离敌方基地
    inline bool isFarFromEnemyBase(const cv::Point2f &pose) {
        if (mapInfo[ENEMY_BASE].pos.x == 1000) {
            return true;
        }
        return (pose.x / 40 <= mapInfo[ENEMY_BASE].pos.x - 1.0 ||
                pose.x / 40 >= mapInfo[ENEMY_BASE].pos.x + 1.0 ||
                (12.8 - pose.y / 40) <= mapInfo[ENEMY_BASE].pos.y - 1.0 ||
                (12.8 - pose.y / 40) >= mapInfo[ENEMY_BASE].pos.y + 1.0) ||
               (pose.x / 40 <= 24 ||
                pose.x / 40 >= 25);
    }

    // 内联函数：计算两点之间的距离
    inline double distance(const cv::Point2f &pos1, const geometry_msgs::msg::Pose2D &pos2) {
        return std::sqrt(std::pow(pos1.x / 40 - pos2.x, 2) + std::pow(12.8 - pos1.y / 40 - pos2.y, 2));
    }

private:
    //颜色阈值参数
    int B_low_threshold_;
    int B_high_threshold_;
    int G_low_threshold_;
    int G_high_threshold_;
    int R_low_threshold_;
    int R_high_threshold_;

    int if_need_pub_map_cnt_ = 0;

    std::vector<robot_msgs::msg::MapInfo> mapInfo;

    std::vector<std::vector<int> > pixel_status_map;

    bool is_transfering_ = false;
    bool is_bullet_low_ = false;
    bool find_one_outdoor = false;

    bool if_need_pub_map_ = true;

    std::array<std::array<int, 6>, 9> color_threshold = {};

    // 坐标系
    std::string map_frame = "odom";
    std::string robot_frame = "base_link";

    cv::Vec3b wallColor;

    int enemy_num_;
    int last_game_start_;
    double sentry_HP_;

    cv::Point2f shoot_other_enemy_pose;
    cv::Point2f one_outdoor_pose;

    const int dx[4] = {-1, 0, 0, 1};
    const int dy[4] = {0, 1, -1, 0};

    //subpub
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPublisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
    rclcpp::Publisher<robot_msgs::msg::MapInfoMsgs>::SharedPtr pubMapInfo;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::TimerBase::SharedPtr start_check_timer_;

    nav_msgs::msg::Odometry odomAftMapped;
};

#endif //SRC_IMG_PROCESS_HPP
