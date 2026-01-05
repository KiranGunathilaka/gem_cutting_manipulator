import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_rsp_launch,
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_static_virtual_joint_tfs_launch,
)

def generate_launch_description():
    # 1) Start Gazebo (your existing sim launch)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gem_cutter_gazebo"),
                "launch",
                "sim.launch.py",
            )
        )
    )

    # 2) Build MoveIt config from your generated package
    moveit_config = (
        MoveItConfigsBuilder("gem_cutter_arm", package_name="gem_cutter_moveit_config")
        .to_moveit_configs()
    )

    ld = LaunchDescription()

    # Important when Gazebo publishes /clock
    ld.add_action(SetParameter(name="use_sim_time", value=True))

    ld.add_action(gazebo_launch)

    # MoveIt side (RViz + move_group + TFs)
    ld.add_action(generate_static_virtual_joint_tfs_launch(moveit_config))
    ld.add_action(generate_rsp_launch(moveit_config))
    ld.add_action(generate_move_group_launch(moveit_config))
    ld.add_action(generate_moveit_rviz_launch(moveit_config))

    return ld
