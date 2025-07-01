#ifndef RM_MOVEIT_NODE_HPP_
#define RM_MOVEIT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <mutex>
#include "rm_interfaces/srv/arm_data.hpp"

class MoveItNode : public rclcpp::Node
{
public:
  MoveItNode();

private:

  void initMoveGroup();
  void handleMotionRequest(
    const std::shared_ptr<rm_interfaces::srv::ArmData::Request> request,
    std::shared_ptr<rm_interfaces::srv::ArmData::Response> response);

  bool planTrajectory(const geometry_msgs::msg::Pose& target_pose, 
                     moveit::planning_interface::MoveGroupInterface::Plan& plan);
                     
  bool executeTrajectory(const moveit::planning_interface::MoveGroupInterface::Plan& plan);
  
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Service<rm_interfaces::srv::ArmData>::SharedPtr service_;
  bool is_initialized_ = false;
  std::mutex planning_mutex_;
};

#endif  // RM_MOVEIT_NODE_HPP_