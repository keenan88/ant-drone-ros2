#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from antdrone_interfaces.srv import MissionHeartbeatSrv
from ant_queen_interfaces.msg import MissionHeartbeatMsg
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor




# Since executing nodes at a regular interval isnt something that can be done in Bt.CPP behavior tree, we have a node
# that will regularly communicate heartbeat with the queen. The Bt.CPP nodes can then talk with this ros2 node before executing
# a node, to make sure that the heartbeat is still alive.
class Heartbeat(Node):

    def __init__(self):
        super().__init__('heartbeat')

        self.drone_name = self.declare_parameter('DRONE_NAME', '').get_parameter_value().string_value

        # Use pub/sub for heartbeat instead of a server, is more async. Can still use reliable QOS to ensure message delivery.
        self.heartbeat_pub = self.create_publisher(MissionHeartbeatMsg, '/mission_robot_heartbeat', 25)
        self.heartbeat_sub = self.create_subscription(MissionHeartbeatMsg, '/mission_queen_heartbeat', self.handle_incoming_heartbeat, 25)

        self.heartbeat_timer = self.create_timer(2, self.heartbeat_timed_cb)
        self.last_queen_heartbeat_s = -1
        self.heartbeat_timeout_s = 10.0
        self.heartbeat_timeout_healthy = False # Assume no heartbeat until recieved
        self.heartbeat_state_healthy = False


        # drone behavior tree needs to check if the heartbeat is still healthy before executing calls to the queen, so 
        # we will make a server here that it can call to validate heartbeat is healthy
        self.heartbeat_server = self.create_service(MissionHeartbeatSrv, 'mission_heartbeat', self.heartbeat_srv_cb)

        self.get_logger().info(f'heartbeat node started')



    def heartbeat_timed_cb(self):
        # Send and check heartbeat run async of each other, its just convenience to put them in the same callback
        self.send_heartbeat()
        self.check_heartbeat()

        

    def send_heartbeat(self):
        heartbeat_msg = MissionHeartbeatMsg()
        heartbeat_msg.sender_name = self.drone_name
        heartbeat_msg.heartbeat_s = self.get_clock().now().to_msg().sec

        self.heartbeat_pub.publish(heartbeat_msg)



    def check_heartbeat(self):
        # Periodically check if heartbeat as fallen out of timeout period
        
        # Only validate heartbeat time if it has been set
        if self.last_queen_heartbeat_s != -1:
            curr_time_s = self.get_clock().now().to_msg().sec
            self.heartbeat_timeout_healthy = (curr_time_s - self.last_queen_heartbeat_s <= self.heartbeat_timeout_s)

            # self.get_logger().info(f"heartbeat_timeout_healthy? {self.heartbeat_timeout_healthy}, {curr_time_s}, {self.last_queen_heartbeat_s}")
        

    def handle_incoming_heartbeat(self, msg: MissionHeartbeatMsg):

        if msg.sender_name == "queen":

            self.last_queen_heartbeat_s = msg.heartbeat_s


            # Update heartbeat as soon as heartbeat message recieved
            curr_time_s = self.get_clock().now().to_msg().sec
            self.heartbeat_timeout_healthy = (curr_time_s - self.last_queen_heartbeat_s <= self.heartbeat_timeout_s)

            # self.get_logger().info(f"heartbeat_timeout_healthy? {self.heartbeat_timeout_healthy}, {curr_time_s}, {self.last_queen_heartbeat_s}")

    def heartbeat_srv_cb(self, req, res):


        res.heartbeat_timeout_healthy = self.heartbeat_timeout_healthy

        return res



def main(args=None):
    rclpy.init(args=args)
    heartbeat = Heartbeat()
    rclpy.spin(heartbeat)
    rclpy.shutdown()

    # executor = MultiThreadedExecutor(num_threads = 4)
    # executor.add_node(heartbeat)
    # executor.spin()

if __name__ == '__main__':
    main()
