#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# The gazebo/ros camera plugin tries to put the camera frame in the base_link frame.
# But it doesnt actually do a transform, it just says the frame is base_link, even though the points are still expressed in the camera frame.
# The easiest solution (instead of a chain of transforms) is to keep the frame as the camera frame, and then change the frame id.
# We use the filter_box_crop_node later to properly transform the points to the base link.

class FrameFixer(Node):
    def __init__(self):
        super().__init__('frame_fixer')

        self.declare_parameter('camera_pos', 'front')
        self.camera_pos = self.get_parameter('camera_pos').get_parameter_value().string_value

        self.declare_parameter('drone_name', '')
        self.drone_name = self.get_parameter('drone_name').get_parameter_value().string_value

        print(self.camera_pos)


        # Create QoS profile for better point cloud handling
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            self.camera_pos + '_camera/points',
            self.pointcloud_callback,
            qos_profile
        )

        # Create publisher
        self.publisher = self.create_publisher(
            PointCloud2,
            self.camera_pos + '_camera/frame_fixed/points',
            qos_profile
        )

    def pointcloud_callback(self, msg):
        # Change the frame id
        msg.header.frame_id = self.drone_name + '_' + self.camera_pos + '_rs_depth_optical_frame'
        
        # Publish the modified message
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    frame_fixer = FrameFixer()
    rclpy.spin(frame_fixer)
    frame_fixer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
