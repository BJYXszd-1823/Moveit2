#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    # 配置说明：
    # - my_arm: 主功能包名（您的moveit配置包）
    # - simple_arm: 存放URDF模型的实际包名
    main_package = "my_arm"      # 当前功能包名
    model_package = "simple_arm" # 存放URDF的包名
    robot_name = "simple_arm"    # URDF中<robot name>

    # 跨包引用URDF路径
    urdf_path = os.path.join(
        get_package_share_directory(model_package),  # 关键修改：使用simple_arm包路径
        "urdf",
        f"{robot_name}.urdf"
    )
    
    # SRDF路径仍在主包中
    srdf_path = os.path.join(
        get_package_share_directory(main_package),
        "config",
        f"{robot_name}.srdf"
    )

    # 硬验证路径
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF文件未找到，请检查路径:\n{urdf_path}")
    if not os.path.exists(srdf_path):
        raise FileNotFoundError(f"SRDF文件未找到，请检查路径:\n{srdf_path}")

    # 构建MoveIt配置
    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name=main_package)  # 使用主包名
        .robot_description(file_path=urdf_path)  # 显式指定跨包URDF路径
        .robot_description_semantic(file_path=srdf_path)
        .to_moveit_configs()
    )

    return LaunchDescription([
        LogInfo(msg=f"== 配置验证成功 =="),
        LogInfo(msg=f"URDF来源包: {model_package}"),
        LogInfo(msg=f"URDF路径: {urdf_path}"),
        LogInfo(msg=f"SRDF路径: {srdf_path}"),
        generate_demo_launch(moveit_config)
    ])