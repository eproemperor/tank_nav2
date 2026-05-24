//
// Created by elsa on 2026/5/7.
//

#include "img_process/img_process.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImgProcess>());
    if (rclcpp::ok())
        rclcpp::shutdown();
    return 0;
}