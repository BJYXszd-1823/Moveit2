cmake_minimum_required(VERSION 3.10)
project(rm_moveit_node)

# 添加项目自己的头文件目录
include_directories(
  include
  /usr/include/opencv4
)

## Use C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 确保这些包被找到
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

###########
## Build ##
###########

# 创建可执行文件
add_executable(${PROJECT_NAME} 
  src/rm_moveit_node.cpp  # 确保源文件路径正确
)

# 添加MoveIt的包含目录
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)

# 链接MoveIt库
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
  rm_interfaces  # 如果需要rm_interfaces消息
  path_planning_node
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

# 安装可执行文件
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装配置文件和启动文件
install(DIRECTORY
  config
  # launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
