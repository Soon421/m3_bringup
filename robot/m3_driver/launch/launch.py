import os, yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

cfg_path = os.path.join(
    get_package_share_directory("m3_driver"), "config/params.yaml"
)

PARAMS = yaml.safe_load(open(cfg_path))
PARAMS_ROBOT = PARAMS["ISR_M3"]

ISR_DRIVER_NODE = Node(
    package="m3_driver",
    executable="isr_m3_driver_node",
    name="isr_m3_driver",
    parameters=[PARAMS_ROBOT],
    output="screen",
)


def generate_launch_description():
    return LaunchDescription([
        ISR_DRIVER_NODE,
    ])
