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

        nonlin_regime_lim_yaw = 0.045

        if msg.angular.z == 0.0:
            scaled_msg.angular.z = 0.0
        elif abs(msg.angular.z) <= nonlin_regime_lim_yaw:
            # self.get_logger().info(f"lower regime {msg.linear.y}")
            if msg.angular.z > 0:
                scaled_msg.angular.z = 8 * msg.angular.z - 15 * msg.angular.z * msg.angular.z
            else:
                scaled_msg.angular.z = 8 * msg.angular.z + 15 * msg.angular.z * msg.angular.z
        else:
            if msg.angular.z > 0:
                scaled_msg.angular.z = 1.03 * msg.angular.z + 0.293
            else:
                scaled_msg.angular.z = 1.03 * msg.angular.z - 0.293



        

        self.scaled_pub.publish(scaled_msg)
                    
                    
                    
                    
                    

                    
                    

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelScaler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
