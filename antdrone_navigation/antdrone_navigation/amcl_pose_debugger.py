import rclpy
from rclpy.node import Node
from nav2_msgs.msg import ParticleCloud
from geometry_msgs.msg import PoseArray, Pose
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class ParticleCloudListener(Node):
    def __init__(self):
        super().__init__('particle_cloud_listener')

        # Set QoS to Best Effort
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber to /particle_cloud topic
        self.subscription = self.create_subscription(
            ParticleCloud,
            '/particle_cloud',
            self.particle_cloud_callback,
            qos_profile
        )

        # Publisher to /pose_array topic
        self.pose_array_publisher = self.create_publisher(
            PoseArray,
            '/pose_array',
            qos_profile
        )

    def particle_cloud_callback(self, msg: ParticleCloud):
        # Create a PoseArray message
        pose_array = PoseArray()
        pose_array.header = msg.header

        # Populate PoseArray from ParticleCloud
        for particle in msg.particles:
            pose = Pose()
            pose.position.x = particle.pose.position.x
            pose.position.y = particle.pose.position.y
            pose.position.z = particle.pose.position.z
            pose.orientation = particle.pose.orientation
            pose_array.poses.append(pose)

        # Publish the PoseArray
        self.pose_array_publisher.publish(pose_array)
        self.get_logger().info('Published PoseArray with {} poses.'.format(len(pose_array.poses)))


def main(args=None):
    rclpy.init(args=args)
    
    node = ParticleCloudListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
