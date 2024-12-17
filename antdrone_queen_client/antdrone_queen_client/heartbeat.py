import rclpy
from rclpy.node import Node
from antdrone_interfaces.srv import MissionHeartbeatSrv
from ant_queen_interfaces.msg import MissionHeartbeatMsg


class Heartbeat(Node):

    # Heartbeat sends regular heartbeats to queen. 
    # Behavior tree can sample heartbeat state.

    def __init__(self):
        super().__init__('heartbeat')

        self.drone_name = self.declare_parameter('DRONE_NAME', '').get_parameter_value().string_value

        self.heartbeat_pub = self.create_publisher(MissionHeartbeatMsg, '/mission_robot_heartbeat', 25)
        self.heartbeat_sub = self.create_subscription(MissionHeartbeatMsg, '/mission_queen_heartbeat', self.handle_incoming_heartbeat, 25)

        self.heartbeat_timer = self.create_timer(2, self.heartbeat_timed_cb)
        self.last_queen_heartbeat_s = None
        self.heartbeat_timeout_s = 10.0
        self.heartbeat_timeout_healthy = False # Assume no heartbeat until recieved

        self.heartbeat_server = self.create_service(MissionHeartbeatSrv, 'mission_heartbeat', self.heartbeat_srv_cb)

        self.get_logger().info(f'heartbeat node started')

    def heartbeat_timed_cb(self):
        self.send_heartbeat()
        self.check_heartbeat()

    def send_heartbeat(self):
        heartbeat_msg = MissionHeartbeatMsg()
        heartbeat_msg.sender_name = self.drone_name
        heartbeat_msg.heartbeat_s = self.get_clock().now().to_msg().sec

        self.heartbeat_pub.publish(heartbeat_msg)

    def check_heartbeat(self):
        if self.last_queen_heartbeat_s:
            curr_time_s = self.get_clock().now().to_msg().sec
            self.heartbeat_timeout_healthy = (curr_time_s - self.last_queen_heartbeat_s <= self.heartbeat_timeout_s)
        

    def handle_incoming_heartbeat(self, msg: MissionHeartbeatMsg):

        if msg.sender_name == "queen":

            self.last_queen_heartbeat_s = msg.heartbeat_s

            curr_time_s = self.get_clock().now().to_msg().sec
            self.heartbeat_timeout_healthy = (curr_time_s - self.last_queen_heartbeat_s <= self.heartbeat_timeout_s)

    def heartbeat_srv_cb(self, req, res):
        res.heartbeat_timeout_healthy = self.heartbeat_timeout_healthy
        return res

def main(args=None):
    rclpy.init(args=args)
    heartbeat = Heartbeat()
    rclpy.spin(heartbeat)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
