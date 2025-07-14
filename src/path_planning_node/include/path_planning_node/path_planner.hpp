#ifndef PATH_PLANNING_NODE__PATH_PLANNER_HPP_
#define PATH_PLANNING_NODE__PATH_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class PathPlanner : public rclcpp::Node
{
public:
  explicit PathPlanner(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~PathPlanner() override = default;

  bool plan_path(double x, double y, double z, double roll, double pitch, double yaw);
  bool get_joint_positions(std::vector<double> &positions);
  void init(const std::string &group_name);

private:
  std::string group_name_;
  std::once_flag init_flag_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Client<moveit_msgs::srv::GetMotionPlan>::SharedPtr client_;
  moveit_msgs::srv::GetMotionPlan::Request::SharedPtr request_;
  moveit_msgs::srv::GetMotionPlan::Response::SharedPtr response_;
};

#endif // PATH_PLANNING_NODE__PATH_PLANNER_HPP_
