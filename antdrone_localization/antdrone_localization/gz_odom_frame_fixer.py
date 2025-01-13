import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class FrameFixer(Node):
    def __init__(self):
        super().__init__('gz_odom_frame_fixer')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.camera_info_sub = self.create_subscription(
            Odometry,
            'odom_wrong_frame',
            self.camera_info_cb,
            qos_profile
        )

        self.camera_info_pub = self.create_publisher(
            Odometry,
            'odom',
            # qos_profile
            10
        )

    def camera_info_cb(self, msg):
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        self.camera_info_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    frame_fixer = FrameFixer()
    rclpy.spin(frame_fixer)
    frame_fixer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
