#include "path_planning_node/MoveitPlanner.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <thread>

MoveitPlanner::MoveitPlanner(const rclcpp::NodeOptions &options) : Node("MoveitPlanner", options)
{
}
void MoveitPlanner::init(const std::string &group_name)
{
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), group_name);
    move_group->setGoalTolerance(0.01);
    move_group->startStateMonitor(10.0);
}

bool MoveitPlanner::plan_path(double x, double y, double z, double roll, double pitch, double yaw)
{
    if (!move_group)
        return false;

    move_group->setStartStateToCurrentState();

    //     moveit_msgs::Constraints endEffector_constraints;
    //    moveit_msgs::OrientationConstraint ocm;
    //    ocm.link_name = "gripper";//需要约束的链接
    //    ocm.header.frame_id = "base_link";//基坐标系
    //    //四元数约束
    //    ocm.orientation.w = 1.0;
    //    //欧拉角约束
    //    ocm.absolute_x_axis_tolerance = 0.1;
    //    ocm.absolute_y_axis_tolerance = 0.1;
    //    ocm.absolute_z_axis_tolerance = 2*3.14;
    //    ocm.weight = 1.0;//此限制权重
    //    endEffector_constraints.orientation_constraints.push_back(ocm);//加入限制列表
    //    group.setPathConstraints(endEffector_constraints);//设置约束

    geometry_msgs::msg::Pose pose;
    // 转换欧拉角为四元数
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    pose.orientation = tf2::toMsg(quat);

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    move_group->setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto ret = move_group->plan(plan);
    move_group->clearPathConstraints();

    auto success = (ret == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);

    RCLCPP_INFO(get_logger(), "Visualizing plan (stateCatch pose) %s", ret ? "SUCCESS" : "FAILED");
    return success;
}
bool MoveitPlanner::get_joint_positions(std::vector<double> &positions)
{
    (void)positions;
    return true;
}