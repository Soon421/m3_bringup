import os, yaml
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

cfg_path = os.path.join(
    get_package_share_directory("m3_bringup"), "config/sensors.yaml"
)

PARAMS = yaml.safe_load(open(cfg_path))
ENABLE = PARAMS["SENSORS"]
PARAMS_LIDAR = PARAMS["LIDAR"]
PARAMS_IMU = PARAMS["IMU"]
PARAMS_GPS = PARAMS["GPS"]

BASE_FRAME = "base_link"

XSENS_IMU_NODE = Node(
    package="xsens_mti_ros2_driver",
    name="xsens_imu",
    executable="xsens_mti_node",
    parameters=[PARAMS_IMU],
    output="screen",
)

# tf는 M3 수치에 맞게 수정해야함. 아직 반영 안했음.
BASE_TO_LIDAR = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=[
        "0.00",   # x
        "0.00",   # y
        "0.00",   # z (base_link now at lidar position)
        "0.000",  # yaw
        "0.000",  # pitch
        "0.000",  # roll
        BASE_FRAME,
        PARAMS_LIDAR["frame_id"],
    ],
    output="screen",
)
BASE_TO_IMU = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=[
        "-0.12",  # x
        "0.00",   # y
        "-0.18",  # z (0.27 - 0.45, relative to lidar height)
        "0.000",  # yaw
        "0.000",  # pitch
        "0.000",  # roll
        BASE_FRAME,
        PARAMS_IMU["frame_id"],
    ],
    output="screen",
)

# Velodyne calibration 경로
VELODYNE_CALIBRATION = os.path.join(
    get_package_share_directory('velodyne_pointcloud'),
    'params', 'VLP16db.yaml'
)

# Velodyne Driver Node
VELODYNE_DRIVER_NODE = Node(
    package="velodyne_driver",
    executable="velodyne_driver_node",
    name="velodyne_driver_node",
    parameters=[PARAMS_LIDAR],
    output="screen",
)

# Velodyne Transform Node
VELODYNE_TRANSFORM_NODE = Node(
    package="velodyne_pointcloud",
    executable="velodyne_transform_node",
    name="velodyne_transform_node",
    parameters=[{
        'calibration': VELODYNE_CALIBRATION,
        'model': PARAMS_LIDAR['model'],
        'min_range': 0.2,
        'max_range': 130.0,
        'view_direction': 0.0,
        'fixed_frame': '',
        'target_frame': '',
        'organize_cloud': True,
    }],
    output="screen",
)

# Velodyne LaserScan Node
VELODYNE_LASERSCAN_NODE = Node(
    package="velodyne_laserscan",
    executable="velodyne_laserscan_node",
    name="velodyne_laserscan_node",
    parameters=[{
        'min_range': 0.9,
        'max_range': 130.0,
        'ring': 8,
        'resolution': 0.007,
    }],
    output="screen",
)

# Novatel GPS Launch
NOVATEL_LAUNCH_PATH = PathJoinSubstitution(
    [FindPackageShare("novatel_oem7_driver"), "launch", "oem7_port.launch.py"]
)
NOVATEL_LAUNCH = GroupAction([
    SetRemap('/novatel/oem7/fix', '/gps/fix'),
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(NOVATEL_LAUNCH_PATH),
        launch_arguments={
            "oem7_port_name": PARAMS_GPS["port"],
            "oem7_port_baud": str(PARAMS_GPS["baudrate"]),
        }.items(),
    ),
])


def generate_launch_description():
    actions = []
    if ENABLE["lidar"]:
        actions += [
            VELODYNE_DRIVER_NODE,
            VELODYNE_TRANSFORM_NODE,
            VELODYNE_LASERSCAN_NODE,
            BASE_TO_LIDAR,
        ]
    if ENABLE["imu"]:
        actions += [XSENS_IMU_NODE, BASE_TO_IMU]
    if ENABLE["gps"]:
        actions += [NOVATEL_LAUNCH]
    return LaunchDescription(actions)
