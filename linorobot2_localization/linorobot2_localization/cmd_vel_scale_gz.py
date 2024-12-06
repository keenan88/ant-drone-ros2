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

        # Create publisher for scaled velocities
        self.scaled_pub = self.create_publisher(
            Twist,
            'cmd_vel_scaled',
            10
        )

    def cmd_vel_callback(self, msg):
        scaled_msg = Twist()

        # Gazebo's planar movement plugin is far slower than commanded at low velocities
        nonlinear_regime_limit = 0.04

        if abs(msg.linear.y) <= nonlinear_regime_limit:
            
            if msg.linear.y > 0:
                scaled_msg.linear.y = 3.3 * msg.linear.y - 68.4 * msg.linear.y * msg.linear.y
            else:
                scaled_msg.linear.y = 3.3 * msg.linear.y + 68.4 * msg.linear.y * msg.linear.y
        else:
            scaled_msg.linear.y = msg.linear.y


        if abs(msg.linear.x) <= nonlinear_regime_limit:
            if msg.linear.x > 0:
                scaled_msg.linear.x = 3.3 * msg.linear.x - 68.4 * msg.linear.x * msg.linear.x
            else:
                scaled_msg.linear.x = 3.3 * msg.linear.x + 68.4 * msg.linear.x * msg.linear.x
        else:
            scaled_msg.linear.x = msg.linear.x

        if abs(msg.angular.z) >= 0.1:
            scalar = -1 if msg.angular.z < 0 else 1
            scaled_msg.angular.z = (msg.angular.z + scalar * 0.0816)/0.986
        elif msg.angular.z == 0:
            scaled_msg.angular.z = 0.0
        

        self.scaled_pub.publish(scaled_msg)
                    
                    
                    
                    
                    

                    
                    

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelScaler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
