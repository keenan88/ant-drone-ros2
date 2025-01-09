import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
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

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_pos + '/' + self.camera_pos + '/color/camera_info_wrong_frame',
            self.camera_info_cb,
            qos_profile
        )

        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            self.camera_pos + '/' + self.camera_pos + '/color/camera_info', 
            # qos_profile
            10
        )


        self.color_img_sub = self.create_subscription(
            Image,
            self.camera_pos + '/' + self.camera_pos + '/color/image_raw_wrong_frame',
            self.color_img_cb,
            qos_profile
        )

        self.color_img_pub = self.create_publisher(
            Image,
            self.camera_pos + '/' + self.camera_pos + '/color/image_raw', 
            # qos_profile,
            10
        )

    def camera_info_cb(self, msg):
        msg.header.frame_id = self.camera_pos + '_depth_optical_frame'
        self.camera_info_pub.publish(msg)

    def color_img_cb(self, msg):
        msg.header.frame_id = self.camera_pos + '_depth_optical_frame'
        self.color_img_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    frame_fixer = FrameFixer()
    rclpy.spin(frame_fixer)
    frame_fixer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
