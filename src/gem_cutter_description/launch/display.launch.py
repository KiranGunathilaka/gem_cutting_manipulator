from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("gem_cutter_description")

    default_urdf = os.path.join(pkg, "urdf", "gem_cutter.urdf")
    default_rviz = os.path.join(pkg, "config", "rviz", "config.rviz")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_urdf,
        description="Absolute path to robot URDF file",
    )

    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description="Whether to launch joint_state_publisher_gui",
    )

    rviz_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=default_rviz,
        description="Absolute path to RViz2 config file",
    )

    # --- Read URDF at launch time (plain Python, Jazzy-compatible) ---
    model_file = LaunchConfiguration("model")
    urdf_content = open(default_urdf, "r").read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": urdf_content}],
        output="screen",
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("use_gui")),
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        output="screen",
    )

    return LaunchDescription(
        [
            model_arg,
            use_gui_arg,
            rviz_arg,
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz2,
        ]
    )
