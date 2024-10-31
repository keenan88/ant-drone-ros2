import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
import math

class VelocityIntegratorNode(Node):
    def __init__(self):
        super().__init__('velocity_integrator_node')
        
        # Subscribe to the Twist messages on /nav2/cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/nav2/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher for the integrated position as a Transform message on /desired_drone_poses
        self.publisher = self.create_publisher(
            TransformStamped,
            '/drone_teleport_pose',
            10
        )
        
        # Initial position and orientation
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0  # Only yaw is tracked for 2D rotation

        # Store the previous time for integration
        self.prev_time = self.get_clock().now()
    
    def cmd_vel_callback(self, msg: Twist):
        # Get current time and calculate time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        # Integrate linear velocities to update position
        self.x += msg.linear.x * math.cos(self.yaw) * dt
        self.y += msg.linear.x * math.sin(self.yaw) * dt
        self.z += msg.linear.z * dt
        
        # Integrate angular velocity to update yaw
        self.yaw += msg.angular.z * dt
        
        # Create and publish the Transform message
        transform_msg = TransformStamped()
        transform_msg.header.stamp = current_time.to_msg()
        transform_msg.header.frame_id = 'map'  # Set appropriate frame ID
        transform_msg.child_frame_id = 'base_link'

        # Set translation (position) values
        transform_msg.transform.translation.x = self.x
        transform_msg.transform.translation.y = self.y
        transform_msg.transform.translation.z = self.z

        # Set rotation (orientation) values
        transform_msg.transform.rotation.x = 0.0
        transform_msg.transform.rotation.y = 0.0
        transform_msg.transform.rotation.z = math.sin(self.yaw / 2)
        transform_msg.transform.rotation.w = math.cos(self.yaw / 2)
        
        self.publisher.publish(transform_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityIntegratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
