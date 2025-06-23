import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler,DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch.logging
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils.launch_utils import (
    DeclareBooleanLaunchArg,
)

import xacro

import re
def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)
    
def generate_launch_description():
    robot = "miniarm"
    logger = launch.logging.get_logger(__name__)
    ld = LaunchDescription()
    
    pkg_share = FindPackageShare(package=robot).find(robot) 
    urdf_model_path = os.path.join(pkg_share, f'config/miniarm.urdf.xacro')

    doc = xacro.parse(open(urdf_model_path))
    xacro.process_doc(doc)
    params = {'robot_description': remove_comments(doc.toxml())}
    
    gazebo =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')
    ld.add_action(gazebo)
    

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': False}, params, {"publish_frequency":15.0}],
        output='screen'
    )
    ld.add_action(node_robot_state_publisher)
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', f'{robot}'], 
                        output='screen')
    ld.add_action(spawn_entity)

    # 关节状态发布器
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'Plan1_controller'],
        output='screen'
    )

    close_evt1 =  RegisterEventHandler( 
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
    )
    ld.add_action(close_evt1)

    close_evt2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
    )
    ld.add_action(close_evt2)

    virtual_joints_launch = os.path.join(
        pkg_share , f'launch/static_virtual_joint_tfs.launch.py'
    )

    if os.path.exists(virtual_joints_launch):
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(virtual_joints_launch),
            )
        )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, f'launch/rsp.launch.py')
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share,f'launch/move_group.launch.py')
            ),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Whether to start RVIZ"
    ))
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share,f'launch/moveit_rviz.launch.py')
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )
    
    
    


    return ld
