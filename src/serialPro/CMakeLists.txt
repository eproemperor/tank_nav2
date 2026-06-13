cmake_minimum_required(VERSION 3.8)
project(serialPro)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 添加子目录
add_subdirectory(thirdparty/serialib)

# 创建库
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

install(TARGETS serialPro
  DESTINATION lib
)

install(DIRECTORY include/ 
  DESTINATION include/serialPro
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY msg_serializer/
  DESTINATION include/serialPro/msg_serializer
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

ament_package()