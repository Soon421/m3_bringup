import os, yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

cfg_path = os.path.join(
    get_package_share_directory("m3_driver"), "config/controller.yaml"
)

PARAMS = yaml.safe_load(open(cfg_path))
PARAMS_JOY = PARAMS["JOY"]
PARAMS_KEYBOARD = PARAMS["KEYBOARD"]

CONTROLLER_NODE = Node(
    package="m3_driver",
    executable="manual_controller_node",
    name="manual_controller_node",
    parameters=[PARAMS_JOY, PARAMS_KEYBOARD],
    output="screen",
)

# ros2 launch m3_driver launch.py
M3_DRIVER_LAUNCH_PATH = PathJoinSubstitution(
    [FindPackageShare("m3_driver"), "launch", "launch.py"]
)
M3_DRIVER_LAUNCH = IncludeLaunchDescription(M3_DRIVER_LAUNCH_PATH)


def generate_launch_description():
    return LaunchDescription([
        M3_DRIVER_LAUNCH,
        CONTROLLER_NODE,
    ])
