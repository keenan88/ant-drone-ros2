import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# The gazebo/ros camera plugin says the points' frame is base_link, even though the points are still expressed in the camera frame.
# The easiest solution (instead of a chain of transforms) is to keep the frame as the camera frame, and then change the frame id in the pointcloud msg.
# We use the filter_box_crop_node later to properly transform the points to the base link.

class FrameFixer(Node):
    def __init__(self):
        super().__init__('frame_fixer')

        self.camera_pos = self.declare_parameter('camera_pos', '').get_parameter_value().string_value

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            self.camera_pos + '/' + self.camera_pos + '/depth/color/points',
            self.pointcloud_callback,
            qos_profile
        )

        self.publisher = self.create_publisher(
            PointCloud2,
            self.camera_pos + '/' + self.camera_pos + '/depth/color/points', 
            qos_profile
        )

    def pointcloud_callback(self, msg):
        msg.header.frame_id = self.camera_pos + '_depth_optical_frame'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    frame_fixer = FrameFixer()
    rclpy.spin(frame_fixer)
    frame_fixer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
