import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.qos import qos_profile_system_default

class GZFrameNameFixer(Node):
    def __init__(self):
        super().__init__('tf_filter_republisher')

        self.drone_name = self.declare_parameter('drone_name', '').get_parameter_value().string_value
        
        # Subscriber to the /tf_gz topic
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf_gz',
            self.tf_callback,
            10
        )

        # Publisher for the filtered transforms
        self.publisher = self.create_publisher(
            TFMessage,
            '/tf',
            10
        )

        self.subscription = self.create_subscription(
            TFMessage,
            '/tf_static_gz',
            self.tf_static_callback,
            QoSProfile(
                history=History.KEEP_LAST,
                depth=1,
                reliability=Reliability.RELIABLE,
                durability=Durability.TRANSIENT_LOCAL
            )
        )

        # Publisher for the filtered transforms
        self.static_publisher = self.create_publisher(
            TFMessage,
            '/tf_static',
            QoSProfile(
                history=History.KEEP_LAST,
                depth=1,
                reliability=Reliability.RELIABLE,
                durability=Durability.TRANSIENT_LOCAL
            )
        )

    def tf_callback(self, msg):
        filtered_transforms = []

        for transform in msg.transforms:
            if self.drone_name in transform.header.frame_id or self.drone_name in transform.child_frame_id:
                new_transform = TransformStamped()
                new_transform.header = transform.header
                new_transform.header.frame_id = transform.header.frame_id.replace(self.drone_name + '_', '')
                new_transform.child_frame_id = transform.child_frame_id.replace(self.drone_name + '_', '')
                new_transform.transform = transform.transform

                filtered_transforms.append(new_transform)

        if filtered_transforms:
            filtered_msg = TFMessage(transforms=filtered_transforms)
            self.publisher.publish(filtered_msg)


    def tf_static_callback(self, msg):
        filtered_transforms = []

        for transform in msg.transforms:
            if self.drone_name in transform.header.frame_id or self.drone_name in transform.child_frame_id:
                new_transform = TransformStamped()
                new_transform.header = transform.header
                new_transform.header.frame_id = transform.header.frame_id.replace(self.drone_name + '_', '')
                new_transform.child_frame_id = transform.child_frame_id.replace(self.drone_name + '_', '')
                new_transform.transform = transform.transform

                filtered_transforms.append(new_transform)

        if filtered_transforms:
            filtered_msg = TFMessage(transforms=filtered_transforms)
            self.static_publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GZFrameNameFixer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()