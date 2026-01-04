from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    gazebo_pkg = get_package_share_directory("gem_cutter_gazebo")
    desc_pkg = get_package_share_directory("gem_cutter_description")

    world_path = os.path.join(gazebo_pkg, "worlds", "gem_cutter_world.sdf")
    urdf_path = os.path.join(desc_pkg, "urdf", "gem_cutter.urdf")

    use_gui_arg = DeclareLaunchArgument(
        "use_gui", default_value="true",
        description="Launch joint_state_publisher_gui"
    )

    # --- Gazebo (gz sim) ---
    gz_sim = ExecuteProcess(
        cmd=["ros2", "launch", "ros_gz_sim", "gz_sim.launch.py",
             "gz_args:=-r -v 4 " + world_path],
        output="screen"
    )

    # --- Robot description -> TF ---
    robot_description = open(urdf_path, "r").read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen"
    )

    # --- Joint GUI (publishes /joint_states) ---
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("use_gui")),
        output="screen"
    )

    # --- Spawn robot into Gazebo from robot_description topic ---
    # This spawns a *visual* model in Gazebo. Dynamics/control comes later (ros2_control).
    spawn_robot = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_sim", "create",
            "-name", "gem_cutter_arm",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.005"
        ],
        output="screen"
    )

    # --- Bridge clock: Gazebo -> ROS (/clock) ---
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen"
    )

    return LaunchDescription([
        use_gui_arg,
        gz_sim,
        clock_bridge,
        robot_state_publisher,
        joint_state_publisher_gui,
        spawn_robot,
    ])
