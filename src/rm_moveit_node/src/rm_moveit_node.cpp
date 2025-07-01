#include "rm_moveit_node/rm_moveit_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/logging.hpp>

MoveItNode::MoveItNode()
: Node("rm_moveit_node")
{
  // 初始化MoveGroup
  initMoveGroup();
  
  // 创建服务
  service_ = this->create_service<rm_interfaces::srv::ArmData>(
    "arm_data",
    std::bind(&MoveItNode::handleMotionRequest, this, 
              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "MoveIt node ready!");
}

void MoveItNode::initMoveGroup()
{
  try {
    // 获取规划组名称参数
    std::string group_name = this->declare_parameter<std::string>("group_name", "test");
    
    RCLCPP_INFO(get_logger(), "Initializing MoveGroup for planning group: %s", group_name.c_str());
    
    // 初始化MoveGroup
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this), 
      group_name
    );
    
    // 设置规划参数
    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(3);
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
    
    is_initialized_ = true;
    RCLCPP_INFO(get_logger(), "MoveGroup initialized successfully!");
  } catch (const std::exception& e) {
    RCLCPP_FATAL(get_logger(), "Failed to initialize MoveGroup: %s", e.what());
    rclcpp::shutdown();
  }
}

bool MoveItNode::planTrajectory(
  const geometry_msgs::msg::Pose& target_pose, 
  moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
  if (!is_initialized_) {
    RCLCPP_ERROR(get_logger(), "MoveGroup not initialized, cannot plan trajectory");
    return false;
  }

  // 设置目标位姿
  move_group_->setPoseTarget(target_pose);
  
  // 规划轨迹
  moveit::core::MoveItErrorCode error_code = move_group_->plan(plan);

  if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(get_logger(), "Plan found successfully!");
    return true;
  } 
  
  RCLCPP_ERROR(get_logger(), "Planning failed! Error code: %d", error_code.val);
  return false;
}

bool MoveItNode::executeTrajectory(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
  if (!is_initialized_) {
    RCLCPP_ERROR(get_logger(), "MoveGroup not initialized, cannot execute trajectory");
    return false;
  }

  // 执行轨迹
  moveit::core::MoveItErrorCode error_code = move_group_->execute(plan);

  if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(get_logger(), "Trajectory executed successfully!");
    return true;
  } 
  
  RCLCPP_ERROR(get_logger(), "Execution failed! Error code: %d", error_code.val);
  return false;
}

void MoveItNode::handleMotionRequest(
  const std::shared_ptr<rm_interfaces::srv::ArmData::Request> request,
  std::shared_ptr<rm_interfaces::srv::ArmData::Response> response)
{
  // 使用互斥锁确保线程安全
  std::lock_guard<std::mutex> lock(planning_mutex_);
  
  RCLCPP_INFO(get_logger(), "Received motion request: x=%.2f, y=%.2f, z=%.2f", 
              request->x, request->y, request->z);
  
  // 设置目标位姿
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = request->x;
  target_pose.position.y = request->y;
  target_pose.position.z = request->z;
  
  tf2::Quaternion q;
  q.setRPY(request->roll, request->pitch, request->yaw);
  target_pose.orientation = tf2::toMsg(q);
  
  // 规划轨迹
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  if (!planTrajectory(target_pose, my_plan)) {
    response->success = false;
    return;
  }
  
  // 执行轨迹
  if (!executeTrajectory(my_plan)) {
    response->success = false;
    return;
  }
  
  // 获取当前关节状态
  auto current_state = move_group_->getCurrentState();
  const auto& joint_names = move_group_->getJointNames();
  
  for (const auto& joint_name : joint_names) {
    response->joint_angles.push_back(*current_state->getJointPositions(joint_name));
  }
  
  response->success = true;
  RCLCPP_INFO(get_logger(), "Motion executed successfully!");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveItNode>();
  RCLCPP_INFO(node->get_logger(), "MoveIt node starting up...");  // 添加调试信息
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}