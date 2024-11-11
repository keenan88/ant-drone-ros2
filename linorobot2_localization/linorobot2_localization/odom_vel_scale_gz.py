#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from math import isnan

class OdomScaler(Node):
    def __init__(self):
        super().__init__('odom_scaler')

        self.drone_name = self.declare_parameter('drone_name', '').value

        # Create subscriber for unscaled odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom_unscaled',
            self.odom_callback,
            10
        )

        # Create publisher for scaled odometry
        self.scaled_pub = self.create_publisher(
            Odometry,
            'odom',
            10
        )

        self.first_x = None
        self.first_y = None

        # Create static transform broadcaster for map to odom transform
        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg):

        if not (
            isnan(msg.pose.pose.position.x) or 
            isnan(msg.pose.pose.position.y) or
            isnan(msg.pose.pose.position.z) or
            isnan(msg.pose.pose.orientation.x) or
            isnan(msg.pose.pose.orientation.y) or
            isnan(msg.pose.pose.orientation.z) or
            isnan(msg.pose.pose.orientation.w)
        ):


            if self.first_x is None and msg.pose.pose.position.x != 0:
                if self.first_y is None and msg.pose.pose.position.y != 0:
                    self.first_x = msg.pose.pose.position.x
                    self.first_y = msg.pose.pose.position.y


            if not (self.first_x is None and self.first_y is None):
                scaled_msg = Odometry()
                scaled_msg.header = msg.header
                scaled_msg.child_frame_id = msg.child_frame_id
                scaled_msg.pose = msg.pose

                

                in_lin_vel_range_x = abs(msg.twist.twist.linear.x) >= 0.05
                in_lin_vel_range_y = abs(msg.twist.twist.linear.y) >= 0.05
                in_lin_vel_range_yaw = abs(msg.twist.twist.angular.z) >= 0.05

                do_pub = False

                if in_lin_vel_range_x:
                    scalar = 1 if msg.twist.twist.linear.x < 0 else -1
                    scaled_msg.twist.twist.linear.x = 0.994 * msg.twist.twist.linear.x + scalar * 0.0546
                    # scaled_msg.twist.twist.linear.x = msg.twist.twist.linear.x
                elif msg.twist.twist.linear.x == 0:
                    scaled_msg.twist.twist.linear.x = 0.0
                    do_pub = True
                if in_lin_vel_range_y:
                    scalar = 1 if msg.twist.twist.linear.y < 0 else -1
                    scaled_msg.twist.twist.linear.y = 0.978 * msg.twist.twist.linear.y + scalar * 0.0516
                    # scaled_msg.twist.twist.linear.y = msg.twist.twist.linear.y
                elif msg.twist.twist.linear.y == 0:
                    scaled_msg.twist.twist.linear.y = 0.0
                    do_pub = True

                if in_lin_vel_range_yaw:
                    scalar = 1 if msg.twist.twist.angular.z < 0 else -1
                    scaled_msg.twist.twist.angular.z = 0.986 * msg.twist.twist.angular.z + scalar * 0.0816
                    # scaled_msg.twist.twist.angular.z = msg.twist.twist.angular.z
                elif msg.twist.twist.angular.z == 0:
                    scaled_msg.twist.twist.angular.z = 0.0
                    do_pub = True

                if do_pub or in_lin_vel_range_x or in_lin_vel_range_y or in_lin_vel_range_yaw:

                    scaled_msg.pose.pose.position.x = msg.pose.pose.position.x #- self.first_x
                    scaled_msg.pose.pose.position.y = msg.pose.pose.position.y# -self.first_y
                    self.scaled_pub.publish(scaled_msg)

                    # Create and initialize the static transform
                    map_odom_tf = TransformStamped()
                    map_odom_tf.header.frame_id = self.drone_name + '_odom'
                    map_odom_tf.header.stamp = msg.header.stamp
                    map_odom_tf.child_frame_id = self.drone_name + '_base_link'
                    map_odom_tf.transform.translation.x = scaled_msg.pose.pose.position.x - self.first_x
                    map_odom_tf.transform.translation.y = scaled_msg.pose.pose.position.y - self.first_y
                    map_odom_tf.transform.rotation.x = scaled_msg.pose.pose.orientation.x
                    map_odom_tf.transform.rotation.y = scaled_msg.pose.pose.orientation.y
                    map_odom_tf.transform.rotation.z = scaled_msg.pose.pose.orientation.z
                    map_odom_tf.transform.rotation.w = scaled_msg.pose.pose.orientation.w

                    # Publish the static transform
                    self.tf_broadcaster.sendTransform(map_odom_tf)

def main(args=None):
    rclpy.init(args=args)
    node = OdomScaler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
