from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

import launch.logging

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("miniarm", package_name="miniarm").to_moveit_configs()
    logger = launch.logging.get_logger(__name__)
    logger.info("moveit_config: " + str(moveit_config))
    launch_node = generate_demo_launch(moveit_config)
    
    return launch_node
