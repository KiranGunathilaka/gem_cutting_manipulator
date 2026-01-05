from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # Load MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("gem_cutter_arm",
                             package_name="gem_cutter_moveit_config")
        .to_moveit_configs()
    )

    demo_ld = generate_demo_launch(moveit_config)

    # Spawn static environment objects into the MoveIt Planning Scene
    scene_spawner = TimerAction(
        period=2.0, 
        actions=[
            Node(
                package="gem_cutter_moveit_config",
                executable="spawn_scene.py",
                name="gem_cutter_scene_spawner",
                output="screen",
                parameters=[{
                    "frame_id": "world"
                }],
            )
        ],
    )

    # Attach to the generated LaunchDescription
    demo_ld.add_action(scene_spawner)

    return demo_ld
