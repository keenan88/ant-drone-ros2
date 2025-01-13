import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, Twist
from std_srvs.srv import SetBool
import math
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation as R
import numpy as np



class GoUnder(Node):
    def __init__(self):
        super().__init__('go_under')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.asdf = self.create_timer(1.0, self.get_transform)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.go_under_service = self.create_service(SetBool, 'go_under', self.go_under)

        self.x = None
        self.y = None
        self.base_link_yaw = None
        self.t = None

        self.x_tol = 0.03
        self.y_tol = 0.02
        self.y_goal = 0.4
        self.angle_tol = 0.05
        self.t_tol = 0.5

    def get_transform(self):
        try:
            now = rclpy.time.Time()
            timeout = rclpy.duration.Duration(seconds=0.1)
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link', 'tag36h11:1', now, timeout
            )

            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            rotation = R.from_quat([qx, qy, qz, qw])

            # Original X-axis
            original_x = np.array([1, 0, 0])

            # Rotate the X-axis using the quaternion
            rotated_x = rotation.apply(original_x)

            # Compute the angle using the dot product
            dot_product = np.dot(original_x, rotated_x)
            dot_product = np.clip(dot_product, -1.0, 1.0)  # Clamp for numerical stability
            angle = np.arccos(dot_product)

            # Determine the sign using the cross product
            cross_product = np.cross(original_x, rotated_x)
            sign = np.sign(cross_product[2])  # Use Z-component for 2D rotation (about Z-axis)

            self.angle_alignment = sign * angle

            self.x = round(transform.transform.translation.x, 2)
            self.y = round(transform.transform.translation.y, 2)
            self.t = round(transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9, 2)

            self.get_logger().info(f"angle_difference: {self.angle_alignment}, y: {self.y}")

        except Exception as e:
            self.get_logger().info(f"Could not get transform: {e}")


    def get_corrective_angle_vel(self):

        angle_vel = 0.0

        if self.angle_alignment > self.angle_tol:
            angle_vel = 5/180*3.14
        elif self.angle_alignment < -self.angle_tol:
            angle_vel = -5/180*3.14

        return angle_vel
    
    def get_corrective_y_vel(self):

        y_vel = 0.0
        y_err = self.y - self.y_goal

        if y_err > self.y_tol:
            y_vel = 0.05
        elif y_err < - self.y_tol:
            y_vel = -0.05

        return y_vel



    def go_under(self, req, res):
        # TODO - potentially turn movements into a PID controlled movement, might be necessary, might not.

        twist = Twist()
        check_angle = True
        check_y = True

        while check_angle or check_y:

            self.get_transform()

            twist.angular.z = self.get_corrective_angle_vel()
            twist.linear.y = self.get_corrective_y_vel()

            check_angle = twist.angular.z != 0
            check_y = twist.linear.y != 0

            self.cmd_vel_pub.publish(twist)

        res.success = True

        return res

    

    

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor(num_threads = 2)
    executor.add_node(GoUnder())
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
