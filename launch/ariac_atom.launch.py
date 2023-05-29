import os
import yaml

# from launch import LaunchDescription
# from launch.actions import (
#     DeclareLaunchArgument,
#     OpaqueFunction,
# )

# from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
# from launch.conditions import IfCondition
# from launch_ros.actions import Node

# from launch_ros.substitutions import FindPackageShare

# from ament_index_python.packages import get_package_share_directory


# def load_file(package_name, file_path):
#     package_path = get_package_share_directory(package_name)
#     absolute_file_path = os.path.join(package_path, file_path)

#     try:
#         with open(absolute_file_path, "r") as file:
#             return file.read()
#     except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
#         return None

# def load_yaml(package_name, file_path):
#     package_path = get_package_share_directory(package_name)
#     absolute_file_path = os.path.join(package_path, file_path)

#     try:
#         with open(absolute_file_path, "r") as file:
#             return yaml.safe_load(file)
#     except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
#         return None


# def launch_setup(context, *args, **kwargs):
#     robot_description_content = Command(
#         [
#             PathJoinSubstitution([FindExecutable(name="xacro")]),
#             " ",
#             PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf/ariac_robots", "ariac_robots.urdf.xacro"]), 
#             " "
#         ]
#     )

#     robot_description = {"robot_description": robot_description_content}
    
#     ## Moveit Parameters
#     robot_description_semantic = {"robot_description_semantic": load_file("ariac_moveit_config", "srdf/ariac_robots.srdf")}

#     robot_description_kinematics = {"robot_description_kinematics": load_yaml("ariac_moveit_config", "config/kinematics.yaml")}

#     group7_competitor = Node(
#         package="group7",
#         executable="group7_start_competition",
#         output="screen",
#         parameters=[
#             robot_description,
#             robot_description_semantic,
#             robot_description_kinematics,
#             {"use_sim_time": True},
#         ],
#     )

#     start_rviz = LaunchConfiguration("rviz")

#     rviz_config_file = PathJoinSubstitution(
#         [FindPackageShare("group7"), "rviz", "group7.rviz"]
#     )

#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2_moveit",
#         output="log",
#         arguments=["-d", rviz_config_file],
#         parameters=[
#             robot_description,
#             robot_description_semantic,
#             robot_description_kinematics,
#             {"use_sim_time": True}
#         ],
#         condition=IfCondition(start_rviz)
#     )

#     nodes_to_start = [
#         group7_competitor,
#         rviz_node
#     ]

#     return nodes_to_start

# def generate_launch_description():
#     declared_arguments = []

#     declared_arguments.append(
#         DeclareLaunchArgument("rviz", default_value="false", description="start rviz node?")
#     )

#     return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition

from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)

from ariac_moveit_config.parameters import generate_parameters

def launch_setup(context, *args, **kwargs):
    # Launch arguments
    trial_name = LaunchConfiguration("trial_name")
    
    # Move Group
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_moveit_config"), "/launch", "/ariac_robots_moveit.launch.py"]
        )
    )

    # ARIAC_environment
    ariac_environment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_gazebo"), "/launch", "/ariac.launch.py"]
        ),
        launch_arguments={
            'trial_name': trial_name,
            'competitor_pkg': "group7",
            'sensor_config': "group7_sensors"
        }.items()
    )

    # Competitor node
    group7_competitor = Node(
    package="group7",
    executable="group7_start_competition",
    output="screen",
    parameters=generate_parameters(),
)

    nodes_to_start = [
        group7_competitor,
        ariac_environment,
        moveit
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []


    declared_arguments.append(
        DeclareLaunchArgument("trial_name", default_value="kitting", description="Name of ariac trial")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
