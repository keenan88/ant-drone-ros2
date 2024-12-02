#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ant_fleet_interfaces.srv import MoveOut
from geometry_msgs.msg import Twist
import time
from rclpy.executors import MultiThreadedExecutor


# In simulation, depth camera cannot see underside of worker when robot is trying to navigate out from under the worker after placing it down, the worker underbelly
# is too close to the depth cameras (setting clipping distance to 0 does not work)
# So we have this simple moveout server that just blindly drives the robot forward for approx. 2 metres to be between the next 2 pipe rails.
class Moveout(Node):

    def __init__(self):
        super().__init__('moveout')

        self.moveout_srv = self.create_service(MoveOut, 'moveout', self.moveout_cb)

        self.twist_pub = self.create_publisher(Twist, '/drone_boris/cmd_vel', 10)


    def moveout_cb(self, req, res):

        moveout_twist = Twist()
        moveout_twist.linear.x = 0.25

        t0 = self.get_clock().now().to_msg().sec
        tdiff = self.get_clock().now().to_msg().sec - t0

        t_move = 2.0 / moveout_twist.linear.x

        while tdiff < t_move:
            tdiff =  self.get_clock().now().to_msg().sec - t0
            self.twist_pub.publish(moveout_twist)
            
            time.sleep(0.25)

        stop_twist = Twist()
        self.twist_pub.publish(stop_twist)

        res.success = True 

        return res






def main(args=None):
    rclpy.init(args=args)
    moveout = Moveout()
    # rclpy.spin(moveout)
    # rclpy.shutdown()

    executor = MultiThreadedExecutor(num_threads = 2)
    executor.add_node(moveout)
    executor.spin()

if __name__ == '__main__':
    main()
