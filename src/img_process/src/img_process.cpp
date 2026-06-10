//
// Created by elsa on 2026/5/7.
//

#include "img_process/img_process.hpp"

using namespace std::chrono_literals;
using namespace cv;
using namespace std;

ImgProcess::ImgProcess() : Node("img_process_node") {
    RCLCPP_INFO(this->get_logger(), "img_process_node started");
    this->declare_parameter("B_low_threshold", 0);
    this->declare_parameter("G_low_threshold", 0);
    this->declare_parameter("R_low_threshold", 0);
    this->declare_parameter("B_high_threshold", 0);
    this->declare_parameter("G_high_threshold", 0);
    this->declare_parameter("R_high_threshold", 0);

    this->get_parameter("B_low_threshold", B_low_threshold_);
    this->get_parameter("G_low_threshold", G_low_threshold_);
    this->get_parameter("R_low_threshold", R_low_threshold_);
    this->get_parameter("B_high_threshold", B_high_threshold_);
    this->get_parameter("G_high_threshold", G_high_threshold_);
    this->get_parameter("R_high_threshold", R_high_threshold_);

    robot_msgs::msg::MapInfo temp;
    temp.is_exist = false;
    temp.is_out_of_center = false;
    temp.pos.x = 1000.0;
    temp.pos.y = 1000.0;
    temp.pos.theta = 0.0; // 注意Pose2D包含x,y,theta三个字段
    for (int i = 0; i < 9; ++i) {
        // 添加到vector
        mapInfo.push_back(temp);
    }

    map_frame = "odom";
    robot_frame = "base_link";

    wallColor = {58, 58, 58};
    enemy_num_ = 0;
    last_game_start_ = 0;
    shoot_other_enemy_pose.x = 0;
    shoot_other_enemy_pose.y = 0;
    one_outdoor_pose.x = 0;
    one_outdoor_pose.y = 0;
    pixel_status_map.resize(256, std::vector<int>(128, OBSTACLE));
    color_threshold[STAR] = {110, 100, 200, 124, 255, 255};
    color_threshold[BASE] = {100, 43, 46, 110, 220, 145};
    color_threshold[ENEMY_BASE] = {150, 50, 110, 180, 150, 175};
    color_threshold[PURPLEENTRY] = {142, 128, 60, 148, 153, 255};
    color_threshold[GREENENTRY] = {72, 210, 60, 78, 230, 255};
    color_threshold[SENTRY] = {100, 150, 210, 110, 170, 255};
    color_threshold[ENEMY] = {0, 140, 50, 5, 178, 255};
    color_threshold[PURPLEEXIT] = {142, 128, 60, 148, 153, 255};
    color_threshold[GREENEXIT] = {72, 210, 60, 78, 230, 255};

    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        "/image_raw",
        10000,
        std::bind(&ImgProcess::imageCallback, this, std::placeholders::_1)
    );
    //map发布
    rclcpp::QoS map_qos(10);
    map_qos.transient_local();
    mapPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
    pubOdomAftMapped = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 100000);
    pubMapInfo = this->create_publisher<robot_msgs::msg::MapInfoMsgs>("/map_info", rclcpp::SystemDefaultsQoS());
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(300));
    start_check_timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms,
                                              std::bind(&ImgProcess::check_callback, this));
}

void ImgProcess::check_callback() {
    last_game_start_ = sentry_HP_;
}

void ImgProcess::imageCallback(sensor_msgs::msg::Image rosImage) {
    this->get_parameter("B_low_threshold", B_low_threshold_);
    this->get_parameter("G_low_threshold", G_low_threshold_);
    this->get_parameter("R_low_threshold", R_low_threshold_);
    this->get_parameter("B_high_threshold", B_high_threshold_);
    this->get_parameter("G_high_threshold", G_high_threshold_);
    this->get_parameter("R_high_threshold", R_high_threshold_);

    auto cvImage = cv_bridge::toCvCopy(rosImage, rosImage.encoding);
    cv::Mat img = cvImage->image;
    cvtColor(img, img, CV_RGB2BGR);

    //HPupdate
    int cout = 0;
    for (int i = 0; i <= 380; ++i) {
        int x = i;
        int y = 996;
        cv::Vec3b pixel = img.at<cv::Vec3b>(cv::Point(x, y));
        //RCLCPP_INFO(this->get_logger(),"R:%d,G:%d,B:%d",pixel[0],pixel[1],pixel[2]);
        if (pixel[0] == 131 && pixel[1] == 131 && pixel[2] == 131) {
            cout++;
        } else if (pixel[0] == 106 && pixel[1] == 106 && pixel[2] == 106) {
            cout++;
        }
    }
    sentry_HP_ = cout / 3.8;
    // RCLCPP_INFO(this->get_logger(), "HP: %f", sentry_HP_);

    if (sentry_HP_ == 0) {
        //游戏重开
        //    RCLCPP_INFO(this->get_logger(),"game over");
        for (auto &row: pixel_status_map) {
            std::fill(row.begin(), row.end(), OBSTACLE);
        }
        for (int i = 0; i < 9; ++i) {
            mapInfo[i].is_exist = false;
            mapInfo[i].is_out_of_center = false;
            mapInfo[i].pos.x = 1000;
            mapInfo[i].pos.y = 1000;
        }
        one_outdoor_pose.x = 1000;
        one_outdoor_pose.y = 1000;

        if_need_pub_map_ = true;
        if_need_pub_map_cnt_ = 0;
    }

    //bullet update
    cout = 0;
    for (int i = 2032; i <= 2047; ++i) {
        int x = i;
        int y = 1014;
        cv::Vec3b pixel = img.at<cv::Vec3b>(cv::Point(x, y));
        if (pixel[0] >= 200 && pixel[1] >= 200 && pixel[2] >= 200) {
            cout++;
        }
        y = 1013;
        pixel = img.at<cv::Vec3b>(cv::Point(x, y));
        if (pixel[0] >= 200 && pixel[1] >= 200 && pixel[2] >= 200) {
            cout++;
        }
    }
    if (cout == 0) {
        is_bullet_low_ = true;
    } else {
        is_bullet_low_ = false;
    }
    // RCLCPP_INFO(this->get_logger(), "is_bullet_low?: %d", is_bullet_low_);

    //地图处理
    int newWidth = 256; // 新的宽度
    int newHeight = 128; // 新的高度
    cv::Mat mapImage, findingImage, viewImage;
    // 使用cv::resize函数降低分辨率
    cv::resize(img, mapImage, cv::Size(newWidth, newHeight), 0, 0, cv::INTER_NEAREST);
    cv::resize(img, findingImage, cv::Size(newWidth * 4, newHeight * 4), 0, 0, cv::INTER_NEAREST);
    // RCLCPP_INFO(get_logger(), "image size: row=%d, col=%d", findingImage.rows, findingImage.cols);

    for (size_t i = 0; i < 9; i++) {
        set_map_info(findingImage, i);
    }

    double distance_to_purple_entry = sqrt(pow(mapInfo[SENTRY].pos.x - mapInfo[PURPLEENTRY].pos.x, 2)
                                           + pow(mapInfo[SENTRY].pos.y - mapInfo[PURPLEENTRY].pos.y, 2));
    double distance_to_green_entry = sqrt(pow(mapInfo[SENTRY].pos.x - mapInfo[GREENENTRY].pos.x, 2)
                                          + pow(mapInfo[SENTRY].pos.y - mapInfo[GREENENTRY].pos.y, 2));
    double distance_to_purple_exit = sqrt(pow(mapInfo[SENTRY].pos.x - mapInfo[PURPLEEXIT].pos.x, 2)
                                          + pow(mapInfo[SENTRY].pos.y - mapInfo[PURPLEEXIT].pos.y, 2));
    double distance_to_green_exit = sqrt(pow(mapInfo[SENTRY].pos.x - mapInfo[GREENEXIT].pos.x, 2)
                                         + pow(mapInfo[SENTRY].pos.y - mapInfo[GREENEXIT].pos.y, 2));
    is_transfering_ = (distance_to_purple_entry <= 0.3 || distance_to_green_entry <= 0.3 ||
                       distance_to_purple_exit <= 0.3 || distance_to_green_exit <= 0.3);
    // RCLCPP_INFO(get_logger(), "is_transfering?: %d", is_transfering_);

    publish_map(mapImage, wallColor);

    //******************* test ********************
    cv::Mat testImage;
    cv::inRange(findingImage, cv::Scalar(B_low_threshold_, G_low_threshold_, R_low_threshold_),
                cv::Scalar(B_high_threshold_, G_high_threshold_, R_high_threshold_), testImage);
    vector<vector<Point> > test_contours;
    vector<Vec4i> test_hierarchy;
    findContours(testImage, test_contours, test_hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    vector<vector<Point> > test_contours_poly(test_contours.size());
    vector<float> test_radius(test_contours.size());
    vector<Point2f> test_centers(test_contours.size());
    for (size_t i = 0; i < test_contours.size(); i++) {
        approxPolyDP(test_contours[i], test_contours_poly[i], 3, true);
        minEnclosingCircle(test_contours_poly[i], test_centers[i], test_radius[i]);
    }
    // 绘制 ENEMY_BASE 区域的框
    cv::rectangle(img, cv::Point(1900, 1000), cv::Point(2048, 1024), cv::Scalar(0, 0, 255), 2); // 红色框
    cv::rectangle(img, cv::Point(0, 975), cv::Point(390, 1024), cv::Scalar(0, 0, 255), 2); // 红色框

    for (size_t i = 0; i < 9; i++) {
        cv::circle(findingImage, cv::Point2f(40 * mapInfo[i].pos.x, -40 * (mapInfo[i].pos.y - 12.8)), 6,
                   cv::Scalar(30 * i, 20 * i, 255 - 30 * i), 2);
    }
    cv::circle(findingImage, one_outdoor_pose, 6, cv::Scalar(255, 255, 255), 2);
    cv::imshow("view", findingImage);
    waitKey(1);
    //******************* test ********************


    publish_sentry_odom(pubOdomAftMapped, tf_broadcaster);
    publish_map_info();
}

void ImgProcess::publish_map(const cv::Mat resizedImage, const cv::Vec3b wallColor) {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "map";
    map.header.stamp = this->get_clock()->now();
    map.info.resolution = 0.1; // float32
    map.info.width = 256; // uint32
    map.info.height = 128; // uint32
    map.info.origin.position.x = 0.0;
    map.info.origin.position.y = 0.0;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.x = 0.0;
    map.info.origin.orientation.y = 0.0;
    map.info.origin.orientation.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    map.data.resize(map.info.width * map.info.height);
    for (int i = 0; i < resizedImage.rows; ++i) {
        for (int j = 0; j < resizedImage.cols; ++j) {
            cv::Vec3b pixel = resizedImage.at<cv::Vec3b>(cv::Point(j, resizedImage.rows - 1 - i));
            int index = i * resizedImage.cols + j;

            if (pixel[0] == wallColor[0] && pixel[1] == wallColor[1] && pixel[2] == wallColor[2]) {
                map.data[index] = 100;
            } else {
                map.data[index] = 0;
            }

            if (i == 0 || j == 0 || i == resizedImage.rows - 1 || j == resizedImage.cols - 1 ||
                (j >= 237 && i >= 0 && j < 256 && i <= 3) || //子弹区
                (j >= 0 && i >= 0 && j <= 49 && i <= 7) || //血量区
                (j >= 0 && i >= 126 && j <= 3 && i < 128) || //帧率区
                (last_game_start_ == 0 && i >= 0 && i < 128 && j >= 0 && j < 256)
            ) {
                map.data[index] = OBSTACLE;
                pixel_status_map[j][i] = OBSTACLE;
            }
        }
    }

    mapPublisher->publish(map);
    // RCLCPP_INFO(this->get_logger(), "map published");
}

void ImgProcess::publish_sentry_odom(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped,
                                     std::unique_ptr<tf2_ros::TransformBroadcaster> &tf_br) {
    odomAftMapped.header.frame_id = map_frame;
    odomAftMapped.child_frame_id = robot_frame;
    odomAftMapped.header.stamp = this->get_clock()->now();
    set_posestamp(odomAftMapped.pose.pose);
    pubOdomAftMapped->publish(odomAftMapped);

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transformStamped.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transformStamped.transform.translation.z = odomAftMapped.pose.pose.position.z;
    transformStamped.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    transformStamped.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    transformStamped.header.stamp = rclcpp::Time(odomAftMapped.header.stamp);
    transformStamped.header.frame_id = map_frame;
    transformStamped.child_frame_id = robot_frame;
    tf_br->sendTransform(transformStamped);
}

void ImgProcess::publish_map_info() {
    robot_msgs::msg::MapInfoMsgs mapInfoMsgs;
    mapInfoMsgs.map_info = mapInfo;
    if (sentry_HP_ > 0)
        RCLCPP_INFO(this->get_logger(), "sentry: x=%lf, y=%lf", mapInfoMsgs.map_info[SENTRY].pos.x, mapInfoMsgs.map_info[SENTRY].pos.y);
    mapInfoMsgs.enemy_num = enemy_num_;
    mapInfoMsgs.sentry_hp = sentry_HP_;
    // RCLCPP_INFO(get_logger(), "-------------- sentry_hp=%f -----------------", mapInfoMsgs.sentry_hp);
    mapInfoMsgs.is_transfering = is_transfering_;
    mapInfoMsgs.is_bullet_low = is_bullet_low_;
    pubMapInfo->publish(mapInfoMsgs);
}

template<typename T>
void ImgProcess::set_posestamp(T &out) {
    out.position.x = mapInfo[SENTRY].pos.x;
    out.position.y = mapInfo[SENTRY].pos.y;
    out.position.z = 0.0;
    out.orientation.x = 0.0;
    out.orientation.y = 0.0;
    out.orientation.z = 0.0;
    out.orientation.w = 1.0;
}

void ImgProcess::set_map_info(const cv::Mat &Image, uint8_t type) {
    //寻找目标颜色
    cv::Mat binaryImg;
    cv::Mat hsv_image;
    cv::cvtColor(Image, hsv_image, CV_BGR2HSV);
    cv::inRange(hsv_image, cv::Scalar(color_threshold[type][0], color_threshold[type][1], color_threshold[type][2]),
                cv::Scalar(color_threshold[type][3], color_threshold[type][4], color_threshold[type][5]), binaryImg);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(binaryImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    //分类讨论
    if (contours.size() == 0) {
        if (type == ENEMY_BASE) {
            mapInfo[type].is_exist = false;
            mapInfo[type].is_out_of_center = false;
            cv::inRange(hsv_image, cv::Scalar(160, 64, 128), cv::Scalar(163, 90, 178), binaryImg);
            findContours(binaryImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            vector<vector<Point> > contours_poly(contours.size());
            vector<float> radius(contours.size());
            vector<Point2f> centers(contours.size());
            for (size_t i = 0; i < contours.size(); i++) {
                approxPolyDP(contours[i], contours_poly[i], 3, true);
                minEnclosingCircle(contours_poly[i], centers[i], radius[i]);
            }
            if (contours.size() == 1 && radius[0] > 6) {
                mapInfo[ENEMY_BASE].pos.x = centers[0].x / 40;
                mapInfo[ENEMY_BASE].pos.y = 12.8 - centers[0].y / 40;
            }
        } else if (type == ENEMY) {
            enemy_num_ = 0;
            cv::inRange(hsv_image, cv::Scalar(25, 127, 90), cv::Scalar(35, 200, 200), binaryImg);
            findContours(binaryImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            vector<vector<Point> > contours_poly(contours.size());
            vector<float> radius(contours.size());
            vector<Point2f> centers(contours.size());
            for (size_t i = 0; i < contours.size(); i++) {
                approxPolyDP(contours[i], contours_poly[i], 3, true);
                minEnclosingCircle(contours_poly[i], centers[i], radius[i]);
                if (radius[i] > 3) {
                    enemy_num_ = 1;
                    break;
                }
            }
            if (enemy_num_ == 0) {
                mapInfo[type].is_exist = false;
                mapInfo[type].is_out_of_center = false;
            }
        }
        return;
    }
    // 寻找最小外接圆
    vector<vector<Point> > contours_poly(contours.size());
    vector<float> radius(contours.size());
    vector<Point2f> centers(contours.size());
    for (size_t i = 0; i < contours.size(); i++) {
        approxPolyDP(contours[i], contours_poly[i], 3, true);
        minEnclosingCircle(contours_poly[i], centers[i], radius[i]);
    }
    if (contours.size() == 1) {
        // cv::circle(Image, centers[0], radius[0], cv::Scalar(0, 0, 255), 2);
        switch (type) {
            case STAR:
            case BASE:
            case ENEMY_BASE:
                if (radius[0] > 6) {
                    mapInfo[type].pos.x = centers[0].x / 40;
                    mapInfo[type].pos.y = 12.8 - centers[0].y / 40;
                    mapInfo[type].is_exist = true;
                    mapInfo[type].is_out_of_center = false;
                }
                break;
            case SENTRY:
            case GREENENTRY:
            case PURPLEENTRY:
                if (radius[0] > 6) {
                    mapInfo[type].pos.x = centers[0].x / 40;
                    mapInfo[type].pos.y = 12.8 - centers[0].y / 40;
                    mapInfo[type].is_exist = true;
                    mapInfo[type].is_out_of_center = isOutOfRange(centers[0]);
                } else if (type == PURPLEENTRY) {
                    mapInfo[PURPLEEXIT].pos.x = centers[0].x / 40;
                    mapInfo[PURPLEEXIT].pos.y = 12.8 - centers[0].y / 40;
                    mapInfo[PURPLEEXIT].is_exist = true;
                    mapInfo[PURPLEEXIT].is_out_of_center = isOutOfRange(centers[0]);
                } else if (type == GREENENTRY) {
                    mapInfo[GREENEXIT].pos.x = centers[0].x / 40;
                    mapInfo[GREENEXIT].pos.y = 12.8 - centers[0].y / 40;
                    mapInfo[GREENEXIT].is_exist = true;
                    mapInfo[GREENEXIT].is_out_of_center = isOutOfRange(centers[0]);
                }
                break;
            case GREENEXIT:
            case PURPLEEXIT:
                if (radius[0] < 6) {
                    mapInfo[type].pos.x = centers[0].x / 40;
                    mapInfo[type].pos.y = 12.8 - centers[0].y / 40;
                    mapInfo[type].is_exist = true;
                    mapInfo[type].is_out_of_center = isOutOfRange(centers[0]);
                } else if (type == PURPLEEXIT) {
                    mapInfo[PURPLEENTRY].pos.x = centers[0].x / 40;
                    mapInfo[PURPLEENTRY].pos.y = 12.8 - centers[0].y / 40;
                    mapInfo[PURPLEENTRY].is_exist = true;
                    mapInfo[PURPLEENTRY].is_out_of_center = isOutOfRange(centers[0]);
                } else if (type == GREENEXIT) {
                    mapInfo[GREENENTRY].pos.x = centers[0].x / 40;
                    mapInfo[GREENENTRY].pos.y = 12.8 - centers[0].y / 40;
                    mapInfo[GREENENTRY].is_exist = true;
                    mapInfo[GREENENTRY].is_out_of_center = isOutOfRange(centers[0]);
                }
                break;
            case ENEMY:
                mapInfo[type].pos.x = centers[0].x / 40;
                mapInfo[type].pos.y = 12.8 - centers[0].y / 40;
                if (isFarFromEnemyBase(centers[0])) {
                    mapInfo[type].is_exist = true;
                    mapInfo[type].is_out_of_center = isOutOfRange(centers[0]);
                    enemy_num_ = 1;
                } else {
                    mapInfo[type].is_exist = false;
                    mapInfo[type].is_out_of_center = false;
                    enemy_num_ = 0;
                }
                //RCLCPP_WARN(this->get_logger(), "enemy_num_:%d",enemy_num_);
                break;
            default:
                break;
        }
    } else {
        switch (type) {
            case STAR:
            case BASE:
            case ENEMY_BASE:
            case SENTRY:
                // RCLCPP_WARN(this->get_logger(), "more than one object detected,type:%d",type);
                break;
            case GREENENTRY:
            case PURPLEENTRY:
                for (size_t i = 0; i < contours.size(); i++) {
                    if (radius[i] > 6) {
                        mapInfo[type].pos.x = centers[i].x / 40;
                        mapInfo[type].pos.y = 12.8 - centers[i].y / 40;
                        mapInfo[type].is_exist = true;
                        mapInfo[type].is_out_of_center = isOutOfRange(centers[i]);
                    } else if (type == GREENENTRY) {
                        mapInfo[GREENEXIT].pos.x = centers[i].x / 40;
                        mapInfo[GREENEXIT].pos.y = 12.8 - centers[i].y / 40;
                        mapInfo[GREENEXIT].is_exist = true;
                        mapInfo[GREENEXIT].is_out_of_center = isOutOfRange(centers[i]);
                    } else if (type == PURPLEENTRY) {
                        mapInfo[PURPLEEXIT].pos.x = centers[i].x / 40;
                        mapInfo[PURPLEEXIT].pos.y = 12.8 - centers[i].y / 40;
                        mapInfo[PURPLEEXIT].is_exist = true;
                        mapInfo[PURPLEEXIT].is_out_of_center = isOutOfRange(centers[i]);
                    }
                }
                break;
            case GREENEXIT:
            case PURPLEEXIT:
                for (size_t i = 0; i < contours.size(); i++) {
                    if (radius[i] < 6) {
                        mapInfo[type].pos.x = centers[i].x / 40;
                        mapInfo[type].pos.y = 12.8 - centers[i].y / 40;
                        mapInfo[type].is_exist = true;
                        mapInfo[type].is_out_of_center = isOutOfRange(centers[i]);
                    } else if (type == GREENEXIT) {
                        mapInfo[GREENENTRY].pos.x = centers[i].x / 40;
                        mapInfo[GREENENTRY].pos.y = 12.8 - centers[i].y / 40;
                        mapInfo[GREENENTRY].is_exist = true;
                        mapInfo[GREENENTRY].is_out_of_center = isOutOfRange(centers[i]);
                    } else if (type == PURPLEEXIT) {
                        mapInfo[PURPLEENTRY].pos.x = centers[i].x / 40;
                        mapInfo[PURPLEENTRY].pos.y = 12.8 - centers[i].y / 40;
                        mapInfo[PURPLEENTRY].is_exist = true;
                        mapInfo[PURPLEENTRY].is_out_of_center = isOutOfRange(centers[i]);
                    }
                }
                break;
            case ENEMY: {
                double min_dist = 10000.0;
                enemy_num_ = 0;
                for (size_t i = 0; i < contours.size(); i++) {
                    if (radius[i] > 5 &&
                        isFarFromEnemyBase(centers[i])) {
                        enemy_num_ += 1;
                        if (enemy_num_ == 1) {
                            shoot_other_enemy_pose = centers[i];
                        }
                        if (distance(centers[i], mapInfo[SENTRY].pos) - isOutOfRange(centers[i]) * 1000 < min_dist) {
                            mapInfo[type].pos.x = centers[i].x / 40;
                            mapInfo[type].pos.y = 12.8 - centers[i].y / 40;
                            mapInfo[type].is_exist = true;
                            mapInfo[type].is_out_of_center = isOutOfRange(centers[i]);
                            // min_dist = distance(centers[i],mapInfo[SENTRY].pos)-mapInfo[type].is_exist_and_out_range*1000;
                        } else {
                            shoot_other_enemy_pose = centers[i];
                        }
                    }
                }
            }
            break;
            default:
                break;
        }
    }
}
