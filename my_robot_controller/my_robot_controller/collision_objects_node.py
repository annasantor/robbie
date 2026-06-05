"""
collision_objects_node.py

Adds static collision objects to the MoveIt planning scene at startup:
    1. TV - box, 45 degrees rotation around Z, positioned to the side
    2. Human - sphere, positioned in front of robot

Objects are added once and persist in the planning scene.
MoveIt uses them for collision-aware planning.
CBF node uses their positions for safety filtering.
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import math


class CollisionObjectsNode(Node):

    def __init__(self):
        super().__init__('collision_objects_node')

        # Publisher to MoveIt planning scene
        self._pub = self.create_publisher(
            CollisionObject,
            '/collision_object',
            10,
        )

        # Give MoveIt time to start before publishing
        self._publish_count = 0
        self._timer = self.create_timer(5.0, self._publish_objects)
        self._published = False

        self.get_logger().info('CollisionObjectsNode started — waiting 2s for MoveIt...')

    def _publish_objects(self):
        if self._published:
            return
        self._published = True
        self._timer.cancel()

        self._add_tv()
        self._add_human()
        if self._publish_count >= 3:  # publish 3 times then stop
            self._timer.cancel()

            self.get_logger().info('Collision objects added to MoveIt planning scene.')

    def _add_tv(self):
        """
        TV — box collision object.
        Dimensions: 1.11m (w) × 0.64m (h) × 0.15m (d)
        Center:     x=0.25, y=0.00, z=0.32 in base_link frame
        Rotation:   45° around Z axis
        """
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = 'tv'
        obj.operation = CollisionObject.ADD

        # Box primitive
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [
            1.11,   # x — width
            0.15,   # y — depth
            0.64,   # z — height
        ]
        obj.primitives.append(box)

        # Pose — center of TV with 45° rotation around Z
        pose = Pose()
        pose.position.x = 0.25
        pose.position.y = 0.00
        pose.position.z = 0.32

        # 45° around Z → quaternion
        angle = math.radians(45.0)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(angle / 2.0)
        pose.orientation.w = math.cos(angle / 2.0)

        obj.primitive_poses.append(pose)

        self._pub.publish(obj)
        self.get_logger().info(
            f'Added TV box: center=[0.25, 0.00, 0.32] '
            f'size=[1.11, 0.15, 0.64] rotation=45° around Z'
        )

    def _add_human(self):
        """
        Human — sphere collision object.
        Represents human torso safety zone.
        Center: x=0.0, y=0.5, z=0.5 in base_link frame
        Radius: 0.3m
        """
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = 'human'
        obj.operation = CollisionObject.ADD

        # Sphere primitive
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.3]   # radius
        obj.primitives.append(sphere)

        # Pose — center of human torso
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.5
        pose.position.z = 0.5
        pose.orientation.w = 1.0   # no rotation
        obj.primitive_poses.append(pose)

        self._pub.publish(obj)
        self.get_logger().info(
            f'Added Human sphere: center=[0.0, 0.5, 0.5] radius=0.3m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CollisionObjectsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
