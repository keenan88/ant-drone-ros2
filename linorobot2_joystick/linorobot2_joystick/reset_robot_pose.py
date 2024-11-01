import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
import math
from nav_msgs.msg import Odometry

class VelocityIntegratorNode(Node):
    def __init__(self):
        super().__init__('reset_robot_pose')
        
        # Publisher for the integrated position as a Transform message on /desired_drone_poses
        self.publisher = self.create_publisher(
            TransformStamped,
            '/drone_teleport_pose',
            10
        )

        transform_msg = TransformStamped()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = 'map'  # Set appropriate frame ID
        transform_msg.child_frame_id = 'base_link'

        # Set translation (position) values
        transform_msg.transform.translation.x = 0.0
        transform_msg.transform.translation.y = 0.0
        transform_msg.transform.translation.z = 0.0

        # Set rotation (orientation) values
        transform_msg.transform.rotation.x = 0.0
        transform_msg.transform.rotation.y = 0.0
        transform_msg.transform.rotation.z = 0.0
        transform_msg.transform.rotation.w = 1.0

        self.publisher.publish(transform_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityIntegratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


