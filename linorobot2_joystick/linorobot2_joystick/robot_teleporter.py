import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
import math
from nav_msgs.msg import Odometry
import signal, sys
from rosgraph_msgs.msg import Clock

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

        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        
        # Initial position and orientation
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.is_robot_pose_reset = False

        # Store the previous time for integration
        self.prev_time = self.get_clock().now()

    def clock_callback(self, msg: Clock):
        if msg.clock.sec < 1 and not self.is_robot_pose_reset: # Reset robot pose at the start of the simulation
            self.x = 0
            self.y = 0
            self.yaw = 0

            self.get_logger().info('Reset robot pose')

            self.update_robot_pose(0, 0, 0)

            self.is_robot_pose_reset = True
    
    def cmd_vel_callback(self, msg: Twist):

        vx = msg.linear.x
        vy = msg.linear.y
        vtheta = msg.angular.z

        self.update_robot_pose(vx, vy, vtheta)


    def update_robot_pose(self, vx, vy, vtheta):

        # Get current time and calculate time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        delta_x = (vx * math.cos(vtheta) - vy * math.sin(vtheta)) * dt
        delta_y = (vx * math.sin(vtheta) + vy * math.cos(vtheta)) * dt
        delta_theta = vtheta * dt

        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_theta
        
        # Create and publish the Transform message
        transform_msg = TransformStamped()
        transform_msg.header.stamp = current_time.to_msg()
        transform_msg.header.frame_id = 'map'  # Set appropriate frame ID
        transform_msg.child_frame_id = 'base_link'

        # Set translation (position) values
        transform_msg.transform.translation.x = self.x
        transform_msg.transform.translation.y = self.y
        transform_msg.transform.translation.z = 0.0

        # Set rotation (orientation) values
        transform_msg.transform.rotation.x = 0.0
        transform_msg.transform.rotation.y = 0.0
        transform_msg.transform.rotation.z = math.sin(self.yaw / 2)
        transform_msg.transform.rotation.w = math.cos(self.yaw / 2)

        self.is_robot_pose_reset = False
        
        self.publisher.publish(transform_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityIntegratorNode()
    
    signal.signal(signal.SIGINT, lambda s, f: rclpy.shutdown())
    signal.signal(signal.SIGTERM, lambda s, f: rclpy.shutdown())

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

