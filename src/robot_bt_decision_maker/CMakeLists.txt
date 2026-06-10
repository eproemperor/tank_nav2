cmake_minimum_required(VERSION 3.8)
project(robot_bt_decision_maker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(robot_behavior_tree REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(tf2_ros REQUIRED)

set(SERIAL_PRO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../serialPro)
include_directories(
  ${SERIAL_PRO_DIR}/include
  ${SERIAL_PRO_DIR}/msg_serializer
  ${SERIAL_PRO_DIR}/thirdparty/serialib
)

set(SERIAL_PRO_SOURCES
  ${SERIAL_PRO_DIR}/src/robot_serial.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 创建可执行文件时包含源文件
add_executable(robot_bt_decision_maker_node 
  src/robot_bt_decision_maker_node.cpp 
  src/robot_bt_decision_maker.cpp
  ${SERIAL_PRO_SOURCES}  
)

ament_target_dependencies(robot_bt_decision_maker_node 
  rclcpp 
  behaviortree_cpp_v3 
  robot_behavior_tree 
  nav2_behavior_tree 
  tf2_ros
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../serialPro/thirdparty/serialib ${CMAKE_CURRENT_BINARY_DIR}/serialib)

target_link_libraries(robot_bt_decision_maker_node serialib)

install(TARGETS robot_bt_decision_maker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()