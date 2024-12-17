import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from nav2_msgs.srv import SetInitialPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

class GZFrameNameFixer(Node):
    def __init__(self):
        super().__init__('tf_filter_republisher')

        self.drone_name = self.declare_parameter('DRONE_NAME', '').get_parameter_value().string_value
        
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

        starting_pose = PoseWithCovarianceStamped()
        starting_pose.header.stamp = self.get_clock().now().to_msg()
        starting_pose.header.frame_id = 'map'
        starting_pose.pose.pose.orientation.x = 0.0
        starting_pose.pose.pose.orientation.y = 0.0
        starting_pose.pose.pose.orientation.z = 0.0
        starting_pose.pose.pose.orientation.w = 1.0
        
        starting_pose.pose.pose.position.z = 0.0

        

         # Publishing starting pose is just a convenience measure in simulation
        self.set_initial_pose_client = self.create_client(SetInitialPose, 'set_initial_pose')
        while not self.set_initial_pose_client.wait_for_service(timeout_sec=1):
            self.get_logger().info('set_initial_pose service not available, waiting again...')

        if self.drone_name == 'drone_boris':
            starting_pose.pose.pose.position.x = 16.5 
            starting_pose.pose.pose.position.y = -18.6 

        req = SetInitialPose.Request()
        req.pose = starting_pose
        future = self.set_initial_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

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