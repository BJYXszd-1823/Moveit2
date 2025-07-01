import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取包路径
    rm_serial_driver_dir = get_package_share_directory('rm_serial_driver')
    rm_moveit_node_dir = get_package_share_directory('rm_moveit_node')
    moveit_arm_dir = get_package_share_directory('moveit_arm')  # 获取moveit_arm包路径

    # 配置文件路径
    serial_config = os.path.join(
        rm_serial_driver_dir, 'config', 'serial_driver.yaml')
    moveit_config = os.path.join(
        rm_moveit_node_dir, 'config', 'moveit_config.yaml')

    # 创建节点
    rm_serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[serial_config],
    )

    rm_moveit_node = Node(
        package='rm_moveit_node',
        executable='rm_moveit_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[moveit_config],
    )

    # 包含MoveIt的演示启动文件
    # moveit_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(moveit_arm_dir, 'launch', 'demo.launch.py')
    #     )
    # )

    return LaunchDescription([
        rm_serial_driver_node,
        rm_moveit_node
        # moveit_launch  # 包含MoveIt演示启动文件
    ])
