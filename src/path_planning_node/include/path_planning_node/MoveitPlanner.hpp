#pragma once
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>

class MoveitPlanner : public rclcpp::Node
{
public:
    explicit MoveitPlanner(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~MoveitPlanner() override = default;

    void init(const std::string &group_name);

    bool plan_path(double x, double y, double z, double roll, double pitch, double yaw);
    bool get_joint_positions(std::vector<double> &positions);

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
};