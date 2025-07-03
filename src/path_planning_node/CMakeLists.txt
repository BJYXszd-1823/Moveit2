cmake_minimum_required(VERSION 3.8)
project(path_planning_node)

# 默认使用C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 添加头文件目录
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${moveit_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(path_planner src/main.cpp src/path_planner.cpp)
ament_target_dependencies(
  path_planner
  rclcpp
  moveit_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

add_library(${PROJECT_NAME} SHARED src/path_planner.cpp)
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  moveit_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)
# 安装可执行文件
install(TARGETS
  path_planner ${PROJECT_NAME}
  DESTINATION lib
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)
ament_export_dependencies(moveit_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_geometry_msgs)

ament_package()
