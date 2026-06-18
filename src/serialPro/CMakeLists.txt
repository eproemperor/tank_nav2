cmake_minimum_required(VERSION 3.8)
project(serialPro)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 添加包含目录
include_directories(
  include
  msg_serializer
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 添加子目录（串口库）
add_subdirectory(thirdparty/serialib)

# 创建库（可选，如果需要在其他节点中使用）
add_library(serialPro STATIC
  src/robot_serial.cpp
)

target_include_directories(serialPro
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/msg_serializer>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/serialib
)

target_link_libraries(serialPro PUBLIC serialib)

add_executable(serialPro_node 
  src/serialPro_node.cpp 
  src/robot_serial.cpp
)

# 设置节点依赖
ament_target_dependencies(serialPro_node
  rclcpp
  robot_msgs
  example_interfaces
  std_msgs
  geometry_msgs
)

# 链接串口库
target_link_libraries(serialPro_node serialib)

# 安装可执行文件
install(TARGETS serialPro_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装库
install(TARGETS serialPro
  DESTINATION lib
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY msg_serializer/
  DESTINATION include/${PROJECT_NAME}/msg_serializer
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()