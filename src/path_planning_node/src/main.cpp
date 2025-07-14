#include "path_planning_node/path_planner.hpp"
#include "path_planning_node/MoveitPlanner.hpp"
#include <iostream>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // 检查参数
  if (argc != 7)
  {
    std::cerr << "Usage: " << argv[0] << " x y z roll pitch yaw" << std::endl;
    return 1;
  }

  // 解析参数
  double x = std::stod(argv[1]);
  double y = std::stod(argv[2]);
  double z = std::stod(argv[3]);
  double roll = std::stod(argv[4]);
  double pitch = std::stod(argv[5]);
  double yaw = std::stod(argv[6]);

  // 创建节点
  auto node = std::make_shared<MoveitPlanner>();
  node->init("test");

  auto logger = node->get_logger();

  // 执行路径规划
  RCLCPP_INFO(node->get_logger(), "Planning path to: x=%.2f, y=%.2f, z=%.2f, r=%.2f, p=%.2f, y=%.2f",
              x, y, z, roll, pitch, yaw);
  auto ret = node->plan_path(x, y, z, roll, pitch, yaw);

  // if (node->plan_path(x, y, z, roll, pitch, yaw))
  // {
  //   std::vector<double> joint_positions;
  //   if (node->get_joint_positions(joint_positions))
  //   {
  //     RCLCPP_INFO(node->get_logger(), "Path planning succeeded!");
  //     RCLCPP_INFO(node->get_logger(), "Final joint positions:");

  //     for (size_t i = 0; i < joint_positions.size(); ++i)
  //     {
  //       RCLCPP_INFO(node->get_logger(), "  Joint %zu: %.4f rad", i, joint_positions[i]);
  //     }

  //     return 0;
  //   }
  // }

  RCLCPP_ERROR(node->get_logger(), "Path planning %s !", ret ? "succeeded" : "failed");
  rclcpp::shutdown();
  return 1;
}
