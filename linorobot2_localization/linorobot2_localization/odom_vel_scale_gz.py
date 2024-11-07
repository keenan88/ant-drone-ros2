#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class OdomScaler(Node):
    def __init__(self):
        super().__init__('odom_scaler')

        # Create subscriber for unscaled odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_unscaled',
            self.odom_callback,
            10
        )

        # Create publisher for scaled odometry
        self.scaled_pub = self.create_publisher(
            Odometry,
            '/nav2/odom',
            10
        )

    def odom_callback(self, msg):
        scaled_msg = Odometry()
        scaled_msg.header = msg.header
        scaled_msg.child_frame_id = msg.child_frame_id
        scaled_msg.pose = msg.pose

        in_lin_vel_range_x = abs(msg.twist.twist.linear.x) >= 0.05
        in_lin_vel_range_y = abs(msg.twist.twist.linear.y) >= 0.05
        in_lin_vel_range_yaw = abs(msg.twist.twist.angular.z) >= 0.05

        do_pub = False

        if in_lin_vel_range_x:
            scalar = -1 if msg.twist.twist.linear.x < 0 else 1
            scaled_msg.twist.twist.linear.x = 0.993 * msg.twist.twist.linear.x - scalar * 0.054
        elif msg.twist.twist.linear.x == 0:
            scaled_msg.twist.twist.linear.x = 0.0
            do_pub = True
        if in_lin_vel_range_y:
            scalar = -1 if msg.twist.twist.linear.y < 0 else 1
            scaled_msg.twist.twist.linear.y = 0.978 * msg.twist.twist.linear.y - scalar * 0.0516
        elif msg.twist.twist.linear.y == 0:
            scaled_msg.twist.twist.linear.y = 0.0
            do_pub = True

        if in_lin_vel_range_yaw:
            scalar = -1 if msg.twist.twist.angular.z < 0 else 1
            scaled_msg.twist.twist.angular.z = 0.986 * msg.twist.twist.angular.z - scalar * 0.0816
        elif msg.twist.twist.angular.z == 0:
            scaled_msg.twist.twist.angular.z = 0.0
            do_pub = True

        if do_pub or in_lin_vel_range_x or in_lin_vel_range_y or in_lin_vel_range_yaw:
            self.scaled_pub.publish(scaled_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomScaler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
