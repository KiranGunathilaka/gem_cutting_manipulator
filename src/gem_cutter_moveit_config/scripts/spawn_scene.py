#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


def make_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0) -> Pose:
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    p.orientation.x = float(qx)
    p.orientation.y = float(qy)
    p.orientation.z = float(qz)
    p.orientation.w = float(qw)
    return p


class SceneSpawner(Node):
    def __init__(self):
        super().__init__("gem_cutter_scene_spawner")

        # Use "world" if you have a virtual joint world->base_link (you do in SRDF),
        # and static_virtual_joint_tfs.launch.py is running in the demo.
        self.declare_parameter("frame_id", "world")
        frame_id = self.get_parameter("frame_id").value

        self.pub = self.create_publisher(CollisionObject, "/collision_object", 10)

        # Small delay so move_group + PlanningSceneMonitor are fully up.
        time.sleep(2.0)

        self.get_logger().info(f"Publishing collision objects in frame: {frame_id}")

        # ---- Ground plane (MoveIt has no infinite plane; use a thin large box) ----
        ground = CollisionObject()
        ground.header.frame_id = frame_id
        ground.id = "ground_plane"

        ground_prim = SolidPrimitive()
        ground_prim.type = SolidPrimitive.BOX
        # SDF: <size>10 10</size> plane. Approximate with 10x10 box, 2 cm thick.
        ground_prim.dimensions = [10.0, 10.0, 0.02]

        # Place top surface at z=0 like your SDF plane:
        # thickness=0.02 => center at z=-0.01 so top is at 0.
        ground_pose = make_pose(0.0, 0.0, -0.01)

        ground.primitives.append(ground_prim)
        ground.primitive_poses.append(ground_pose)
        ground.operation = CollisionObject.ADD
        self.pub.publish(ground)

        # ---- Gem stand model (SDF model pose) ----
        stand_world_x = 0.25
        stand_world_y = 0.0
        stand_world_z = 0.0

        # Base block
        base = CollisionObject()
        base.header.frame_id = frame_id
        base.id = "gem_stand_base"

        base_prim = SolidPrimitive()
        base_prim.type = SolidPrimitive.BOX
        base_prim.dimensions = [0.12, 0.12, 0.04]

        # SDF link pose: base at z=0.02 relative to model, model at (0.25,0,0)
        base_pose = make_pose(stand_world_x + 0.0, stand_world_y + 0.0, stand_world_z + 0.02)

        base.primitives.append(base_prim)
        base.primitive_poses.append(base_pose)
        base.operation = CollisionObject.ADD
        self.pub.publish(base)

        # Vertical rod (cylinder)
        rod = CollisionObject()
        rod.header.frame_id = frame_id
        rod.id = "gem_stand_rod"

        rod_prim = SolidPrimitive()
        rod_prim.type = SolidPrimitive.CYLINDER
        # SolidPrimitive.CYLINDER dimensions: [height, radius]
        rod_prim.dimensions = [0.1, 0.01]

        rod_pose = make_pose(stand_world_x + 0.0, stand_world_y + 0.0, stand_world_z + 0.17)

        rod.primitives.append(rod_prim)
        rod.primitive_poses.append(rod_pose)
        rod.operation = CollisionObject.ADD
        self.pub.publish(rod)

        # Gem (cone)
        gem = CollisionObject()
        gem.header.frame_id = frame_id
        gem.id = "gem"

        gem_prim = SolidPrimitive()
        gem_prim.type = SolidPrimitive.CONE
        # SolidPrimitive.CONE dimensions: [height, radius]
        gem_prim.dimensions = [0.05, 0.03]

        gem_pose = make_pose(stand_world_x + 0.0, stand_world_y + 0.0, stand_world_z + 0.325)

        gem.primitives.append(gem_prim)
        gem.primitive_poses.append(gem_pose)
        gem.operation = CollisionObject.ADD
        self.pub.publish(gem)

        self.get_logger().info("Scene objects published.")
        # Exit shortly after publishing
        rclpy.shutdown()


def main():
    rclpy.init()
    node = SceneSpawner()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
