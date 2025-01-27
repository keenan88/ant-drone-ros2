import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, Twist
from antdrone_interfaces.srv import GoUnder
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation as R
import numpy as np
import time



class GoUnderWorker(Node):
    def __init__(self):
        super().__init__('go_under')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer__ = self.create_timer(1, self.get_transform)


    def get_transform(self):

        try:
            tag_id = 0
            now = rclpy.time.Time()
            timeout = rclpy.duration.Duration(seconds=0.05)
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link', 'tag16h5:' + str(tag_id), now, timeout
            )

            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            quaternion = [qx, qy, qz, qw]  # Replace with your quaternion values

            # Create a Rotation object from the quaternion
            rotation = R.from_quat(quaternion)  # Note the order: [x, y, z, w]

            # Convert to Euler angles (in radians)
            # 'xyz' means rotation about x, then y, then z in that order
            euler_angles = rotation.as_euler('xyz', degrees=True)

            self.get_logger().info(f"Euler angles (degrees): {euler_angles}")

            

        except Exception as e:
            self.get_logger().info(f"Could not get transform: {e}")



def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor(num_threads = 2)
    executor.add_node(GoUnderWorker())
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
