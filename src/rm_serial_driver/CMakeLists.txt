cmake_minimum_required(VERSION 3.10)
project(rm_serial_driver)

# 添加项目自己的头文件目录
include_directories(
  include
  /usr/include/opencv4
)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror )

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 确保这些包被找到
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

# 获取MoveIt的包含目录和库
set(MOVEIT_LIBRARIES
  ${moveit_core_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
)

set(MOVEIT_INCLUDE_DIRS
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${moveit_msgs_INCLUDE_DIRS}
)

###########
## Build ##
###########

# 确保库目标也包含头文件目录
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/rm_serial_driver.cpp
  src/crc.cpp
# DIRECTORY src
)

# 添加MoveIt的包含目录
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MOVEIT_INCLUDE_DIRS}
)

# 链接MoveIt库
ament_target_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_serial_driver::RMSerialDriver
  EXECUTABLE ${PROJECT_NAME}_node
)

# 可执行文件也需要包含头文件目录和链接库
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MOVEIT_INCLUDE_DIRS}
)

# 链接MoveIt库到可执行文件
ament_target_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
