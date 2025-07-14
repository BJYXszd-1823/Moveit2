#include "path_planning_node/path_planner.hpp"
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

PathPlanner::PathPlanner(const rclcpp::NodeOptions &options)
    : Node("path_planner", options)
{
  // 创建服务客户端
  client_ = create_client<moveit_msgs::srv::GetMotionPlan>("plan_kinematic_path");

  // 等待服务可用
  while (!client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service.");
      return;
    }
    RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
  }

  // 初始化请求
  request_ = std::make_shared<moveit_msgs::srv::GetMotionPlan::Request>();
}

void PathPlanner::init(const std::string &group_name)
{

  std::call_once(init_flag_, [&, group_name]()
                 {
                  group_name_ = group_name;
    RCLCPP_INFO(get_logger(), "PathPlanner init, group_name: %s",group_name_.c_str());
    
    move_group_ = std::make_shared< moveit::planning_interface::MoveGroupInterface> (shared_from_this(), group_name_);
    
    auto ret = move_group_->startStateMonitor();
    if (!ret)
    {
      RCLCPP_ERROR(get_logger(), "Failed to start state monitor");
      return;
    }
    auto state = move_group_->getCurrentState();
    if(!state)
    {
      RCLCPP_ERROR(get_logger(), "Failed to get current state");
      return;
    }
    RCLCPP_INFO(get_logger(), "State monitor started"); });
}

bool PathPlanner::plan_path(double x, double y, double z, double roll, double pitch, double yaw)
{
  // 设置目标位姿

  // moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), "test");
  // move_group.setGoalTolerance(0.05);
  init("test");
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link"; // 根据实际机器人修改

  // 转换欧拉角为四元数
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  pose.pose.orientation = tf2::toMsg(quat);

  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;

  // 设置规划请求
  request_->motion_plan_request.group_name = group_name_; // 根据实际机器人修改
  request_->motion_plan_request.num_planning_attempts = 5;
  request_->motion_plan_request.allowed_planning_time = 10.0;
  request_->motion_plan_request.start_state.is_diff = false;
  request_->motion_plan_request.start_state.joint_state.header = pose.header;
  move_group_->getCurrentState();

  request_->motion_plan_request.start_state.joint_state.name = move_group_->getJointNames();
  auto positions = move_group_->getCurrentJointValues();
  RCLCPP_DEBUG(get_logger(), "positions size: %ld", positions.size());
  // if (positions.size() == 0)
  // {
  //   RCLCPP_ERROR(get_logger(), "Failed to get current joint values");
  //   return false;
  // }
  move_group_->setGoalTolerance(0.05);
  request_->motion_plan_request.start_state.joint_state.position = positions;

  // 设置目标约束
  moveit_msgs::msg::Constraints constraints;

  moveit_msgs::msg::PositionConstraint position_constraint;
  position_constraint.header = pose.header;
  position_constraint.link_name = "Minir_Link";
  position_constraint.target_point_offset.x = 0;
  position_constraint.target_point_offset.y = 0;
  position_constraint.target_point_offset.z = 0;
  position_constraint.constraint_region.primitive_poses.resize(1);
  position_constraint.constraint_region.primitive_poses[0].position = pose.pose.position;
  position_constraint.constraint_region.primitives.resize(1);
  position_constraint.constraint_region.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  position_constraint.constraint_region.primitives[0].dimensions = {0.01, 0.01, 0.01}; // 位置容差范围
  position_constraint.weight = 1.0;

  moveit_msgs::msg::OrientationConstraint orientation_constraint;
  orientation_constraint.header = pose.header;
  orientation_constraint.orientation = pose.pose.orientation;
  orientation_constraint.link_name = "Minir_Link"; // 根据实际机器人修改
  orientation_constraint.absolute_x_axis_tolerance = 0.001;
  orientation_constraint.absolute_y_axis_tolerance = 0.001;
  orientation_constraint.absolute_z_axis_tolerance = 0.001;

  orientation_constraint.weight = 1.0;
  // constraints.position_constraints.push_back(position_constraint);
  constraints.orientation_constraints.push_back(orientation_constraint);
  request_->motion_plan_request.goal_constraints.clear();
  request_->motion_plan_request.goal_constraints.push_back(constraints);

  // 发送请求
  auto future = client_->async_send_request(request_);

  // 等待响应
  if (rclcpp::spin_until_future_complete(shared_from_this(), future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Failed to call service plan_kinematic_path");
    return false;
  }

  response_ = future.get();

  // 修复：错误代码现在位于motion_plan_response中
  return (response_->motion_plan_response.error_code.val ==
          moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

bool PathPlanner::get_joint_positions(std::vector<double> &positions)
{
  if (!response_)
  {
    RCLCPP_ERROR(get_logger(), "No response available");
    return false;
  }

  const auto &trajectory = response_->motion_plan_response.trajectory.joint_trajectory;
  if (trajectory.points.empty())
  {
    RCLCPP_ERROR(get_logger(), "Empty trajectory");
    return false;
  }

  // 获取最终关节位置
  positions = trajectory.points.back().positions;
  return true;
}
