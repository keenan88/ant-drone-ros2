#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelScaler(Node):
    def __init__(self):
        super().__init__('cmd_vel_scaler')

        # Create subscribers for both cmd_vel topics
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.nav_cmd_vel_sub = self.create_subscription(
            Twist,
            '/nav2/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Create publisher for scaled velocities
        self.scaled_pub = self.create_publisher(
            Twist,
            'cmd_vel_scaled',
            10
        )

    def cmd_vel_callback(self, msg):
        scaled_msg = Twist()

        if msg.linear.x != 0:
            if abs(msg.linear.x) < 0.1:
                msg.linear.x = 0.1 if msg.linear.x > 0 else -0.1

        if msg.linear.y != 0:
            if abs(msg.linear.y) < 0.1:
                msg.linear.y = 0.1 if msg.linear.y > 0 else -0.1
        
        if msg.angular.z != 0:
            if abs(msg.angular.z) < 0.1:
                msg.angular.z = 0.1 if msg.angular.z > 0 else -0.1

        # Gazebo planar move plugin does not move robot quite as much as it should, so velocity needs to be scaled up.
        # Scaling up is linear, as long as velocity is fast enough (see above 'if' conditions)
        in_lin_vel_range_x = abs(msg.linear.x) >= 0.1
        in_lin_vel_range_y = abs(msg.linear.y) >= 0.1
        in_lin_vel_range_yaw = abs(msg.angular.z) >= 0.1
        zero_vel_cmd = False

        if in_lin_vel_range_x:
            scalar = 1 if msg.linear.x > 0 else -1 
            scaled_msg.linear.x = (msg.linear.x + scalar * 0.0546)/0.994
        elif msg.linear.x == 0:
            scaled_msg.linear.x = 0.0
            zero_vel_cmd = True

        if in_lin_vel_range_y:
            scalar = 1 if msg.linear.y > 0 else -1
            scaled_msg.linear.y = (msg.linear.y + scalar * 0.0516)/0.978
        elif msg.linear.y == 0:
            scaled_msg.linear.y = 0.0
            zero_vel_cmd = True

        if in_lin_vel_range_yaw:
            scalar = -1 if msg.angular.z < 0 else 1
            scaled_msg.angular.z = (msg.angular.z + scalar * 0.0816)/0.986
        elif msg.angular.z == 0:
            scaled_msg.angular.z = 0.0
            zero_vel_cmd = True

        if zero_vel_cmd or in_lin_vel_range_x or in_lin_vel_range_y or in_lin_vel_range_yaw:
            self.scaled_pub.publish(scaled_msg)
                    
                    
                    
                    
                    

                    
                    

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelScaler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
