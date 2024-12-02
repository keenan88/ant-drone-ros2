#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ant_fleet_interfaces.srv import MissionHeartbeatSrv
from ant_fleet_interfaces.msg import MissionHeartbeatMsg
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor




# Since executing nodes at a regular interval isnt something that can be done in Bt.CPP behavior tree, we have a node
# that will regularly communicate heartbeat with the queen. The Bt.CPP nodes can then talk with this ros2 node before executing
# a node, to make sure that the heartbeat is still alive.
class Heartbeat(Node):

    def __init__(self):
        super().__init__('heartbeat')

        self.drone_name = self.get_namespace()[1:] # Remove leading forward slash

        qos_reliable = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 10
        )

        # Use pub/sub for heartbeat instead of a server, is more async. Can still use reliable QOS to ensure message delivery.
        self.heartbeat_pub = self.create_publisher(MissionHeartbeatMsg, '/mission_heartbeat', qos_reliable)
        self.heartbeat_sub = self.create_subscription(MissionHeartbeatMsg, '/mission_heartbeat', self.handle_incoming_heartbeat, qos_reliable)

        self.heartbeat_timer = self.create_timer(2, self.heartbeat_timed_cb)
        self.last_queen_heartbeat_s = -1
        self.heartbeat_timeout_s = 10.0
        self.heartbeat_timeout_healthy = False # Assume no heartbeat until recieved

        # Both queen and drone track the drone's floor mission status. Use this node to validate that the statuses match.
        self.last_queen_drone_floor_mission_status = 'IDLE' # Assume starts idle
        self.last_drone_drone_floor_mission_status = 'IDLE' # Assume starts idle

        # drone behavior tree needs to check if the heartbeat is still healthy before executing calls to the queen, so 
        # we will make a server here that it can call to validate heartbeat and drone_floor_mission_status are health
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
        heartbeat_msg.drone_floor_mission_status

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
            # self.last_queen_drone_floor_mission_status = msg.drone_floor_mission_status

            # Update heartbeat as soon as heartbeat message recieved
            curr_time_s = self.get_clock().now().to_msg().sec
            self.heartbeat_timeout_healthy = (curr_time_s - self.last_queen_heartbeat_s <= self.heartbeat_timeout_s)

            # self.get_logger().info(f"heartbeat_timeout_healthy? {self.heartbeat_timeout_healthy}, {curr_time_s}, {self.last_queen_heartbeat_s}")

    def heartbeat_srv_cb(self, req, res):

        # self.last_drone_drone_floor_mission_status = req.drone_floor_mission_status

        res.robot_floor_mission_status_healthy = True #(self.last_drone_drone_floor_mission_status == self.last_queen_drone_floor_mission_status)
    
        res.heartbeat_timeout_healthy = self.heartbeat_timeout_healthy

        self.get_logger().info(f"returning: {res}")

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
