#include "rm_moveit_node/rm_moveit_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

MoveItNode::MoveItNode(const rclcpp::NodeOptions &options)
    : Node("rm_moveit_node", options)
{
  // 创建服务
  service_ = create_service<rm_interfaces::srv::ArmData>(
      "arm_data",
      std::bind(&MoveItNode::handleMotionRequest, this,
                std::placeholders::_1, std::placeholders::_2));
  planner_ = std::make_shared<PathPlanner>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(planner_);

  RCLCPP_INFO(get_logger(), "MoveIt node ready!");
}

bool MoveItNode::planTrajectory(const std::shared_ptr<rm_interfaces::srv::ArmData::Request> request)
{
  return planner_->plan_path(request->x, request->y, request->z, request->roll, request->pitch, request->yaw);
}

void MoveItNode::handleMotionRequest(
    const std::shared_ptr<rm_interfaces::srv::ArmData::Request> request,
    std::shared_ptr<rm_interfaces::srv::ArmData::Response> response)
{
  // 使用互斥锁确保线程安全
  std::lock_guard<std::mutex> lock(planning_mutex_);

  RCLCPP_INFO(get_logger(), "Received motion request: X[%.2f], Y[%.2f], Z[%.2f] ROLL[%0.2f] PITCH[%0.2f] YAW[%0.2f]",
              request->x, request->y, request->z, request->roll, request->pitch, request->yaw);

  if (!planner_->plan_path(request->x, request->y, request->z, request->roll, request->pitch, request->yaw))
  {
    RCLCPP_ERROR(get_logger(), "Failed to plan path");
    response->success = false;
    return;
  }
  std::vector<double> joint_positions;
  planner_->get_joint_positions(joint_positions);
  for (auto &joint_position : joint_positions)
  {
    response->joint_angles.push_back((float)joint_position);
  }
  response->success = true;
  RCLCPP_INFO(get_logger(), "Motion executed successfully!");
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);

  auto node = std::make_shared<MoveItNode>(options);
  RCLCPP_INFO(node->get_logger(), "MoveIt node starting up..."); // 添加调试信息
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}