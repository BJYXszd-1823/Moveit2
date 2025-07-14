#ifndef RM_MOVEIT_NODE_HPP_
#define RM_MOVEIT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <mutex>
#include "rm_interfaces/srv/arm_data.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "path_planning_node/path_planner.hpp"
#include "path_planning_node/MoveitPlanner.hpp"
class MoveItNode : public rclcpp::Node
{
public:
  MoveItNode(const rclcpp::NodeOptions &options);

private:
  std::shared_ptr<PathPlanner> planner_;
  void handleMotionRequest(
      const std::shared_ptr<rm_interfaces::srv::ArmData::Request> request,
      std::shared_ptr<rm_interfaces::srv::ArmData::Response> response);

  bool planTrajectory(const std::shared_ptr<rm_interfaces::srv::ArmData::Request> request);
  rclcpp::Service<rm_interfaces::srv::ArmData>::SharedPtr service_;
  std::mutex planning_mutex_;
};

#endif // RM_MOVEIT_NODE_HPP_