#include "rm_moveit_node/TrajectoryPoseCalculator.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <Eigen/Geometry>
TrajectoryPoseCalculator::TrajectoryPoseCalculator(const std::string &robot_description, const std::string &link_name)
{
    link_name_ = link_name;
    model_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(robot_description);
    robot_model_ = model_loader_->getModel();
    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    robot_state_->setToDefaultValues();

    node_ = std::make_shared<rclcpp::Node>("trajectory_pose_calculator");
    sub_ = node_->create_subscription<moveit_msgs::msg::DisplayTrajectory>("display_planned_path", 10, std::bind(&TrajectoryPoseCalculator::callback, this, std::placeholders::_1));
}
TrajectoryPoseCalculator::~TrajectoryPoseCalculator()
{
}

void TrajectoryPoseCalculator::callback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg)
{
    auto traj = msg->trajectory[0].joint_trajectory;
    const auto &joint_names = traj.joint_names;
    for (size_t i = 0; i < traj.points.size(); i++)
    {
        auto &point = traj.points[i];
        for (size_t j = 0; j < joint_names.size(); j++)
        {
            robot_state_->setJointPositions(joint_names[j], &point.positions[j]);
        }

        const Eigen::Isometry3d &transform = robot_state_->getGlobalLinkTransform(link_name_);
        geometry_msgs::msg::Pose p;
        p.position.x = transform.translation().x();
        p.position.y = transform.translation().y();
        p.position.z = transform.translation().z();

        Eigen::Quaterniond equat(transform.rotation());
        tf2::convert(equat, p.orientation);

        tf2::Quaternion quat;
        tf2::fromMsg(p.orientation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        RCLCPP_INFO(node_->get_logger(), "Pose: x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f",
                    p.position.x, p.position.y, p.position.z, roll, pitch, yaw);
    }
}