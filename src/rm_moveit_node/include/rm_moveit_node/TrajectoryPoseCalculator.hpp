#pragma once
#include <rclcpp/subscription.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

class TrajectoryPoseCalculator
{
public:
    TrajectoryPoseCalculator(const std::string &robot_description, const std::string &link_name);
    ~TrajectoryPoseCalculator();

private:
    void callback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg);
    std::unique_ptr<robot_model_loader::RobotModelLoader> model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    std::shared_ptr<rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>> sub_;
    std::string link_name_;
    std::shared_ptr<rclcpp::Node> node_;
};