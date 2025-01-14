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

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.go_under_service = self.create_service(GoUnder, 'go_under_worker', self.go_under_worker)

        self.x_tol = 0.03
        self.x_goal = 0.31
        self.y_tol = 0.01
        self.y_goal = 0.56/2
        self.angle_tol = 0.05

        self.vy_max = 0.05
        self.vx_max = 0.05
        self.vyaw_max = 5 / 180 * 3.14

    def get_transform(self, tag_id):
        x_to_tag = None
        y_to_tag = None
        t = None
        angle_to_tag = None

        try:
            now = rclpy.time.Time()
            timeout = rclpy.duration.Duration(seconds=0.05)
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link', 'tag36h11:' + str(tag_id), now, timeout
            )

            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            rotation = R.from_quat([qx, qy, qz, qw])

            original_x = np.array([1, 0, 0])

            rotated_x = rotation.apply(original_x)

            dot_product = np.dot(original_x, rotated_x)
            dot_product = np.clip(dot_product, -1.0, 1.0)
            angle = np.arccos(dot_product)

            cross_product = np.cross(original_x, rotated_x)
            sign = np.sign(cross_product[2]) 

            angle_to_tag = round(sign * angle, 2)

            x_to_tag = round(transform.transform.translation.x, 2)
            y_to_tag = round(transform.transform.translation.y, 2)
            t = round(transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9, 2)

            self.get_logger().info(f"angle_difference: {angle_to_tag}, y: {y_to_tag}, x: {x_to_tag}")

        except Exception as e:
            self.get_logger().info(f"Could not get transform: {e}")

        return x_to_tag, y_to_tag, angle_to_tag, t


    def get_corrective_angle_vel(self, angle_to_tag):
        angle_vel = 0.0

        if abs(angle_to_tag) < 1 / 180 * 3.14:
            pass
        elif angle_to_tag > 0:
            angle_vel = min(self.vyaw_max, 1.0 * angle_to_tag)
        elif angle_to_tag < 0:
            angle_vel = max(-self.vyaw_max, 1.0 * angle_to_tag)

        return angle_vel
    
    def get_corrective_y_vel(self, y_err):
        y_vel = 0.0

        if y_err < 0.01:
            pass
        elif y_err > self.y_tol:
            y_vel = min(0.05, 1.0 * y_err)
        elif y_err < -self.y_tol:
            y_vel = max(-0.05, 1.0 * y_err)

        return y_vel
    
    def get_x_vel(self, x_err):

        x_vel = 0.0
    
        if x_err > self.x_tol:
            x_vel = 0.1 * np.sqrt(abs(x_err))

        return x_vel



    def go_under_worker(self, req, res):
        # Assumes an apriltag is visible in front cam when called

        res.success = False

        self.tf_buffer = Buffer() # Clear buffer
        self.tf_listener = TransformListener(self.tf_buffer, self)
        time.sleep(3) # Allow a few seconds to find tags
        x_to_tag0, y_to_tag0, angle_to_tag0, t_tag0 = self.get_transform(tag_id=0)
        x_to_tag1, y_to_tag1, angle_to_tag1, t_tag1 = self.get_transform(tag_id=1)

        if t_tag0 != None:
            side_to_enter = 'L'
            tag_id = 0
            self.get_logger().info("Found tag 0")

        elif t_tag1 != None:
            side_to_enter = 'R'
            tag_id = 1
            self.get_logger().info("Found tag 1")

        else:
            self.get_logger().info("Found No tags")
            return res

        twist = Twist()

        in_position = False
        while not in_position:

            x_to_tag, y_to_tag, angle_to_tag, t = self.get_transform(tag_id)

            y_err = y_to_tag - self.y_goal
            x_err = x_to_tag - self.x_goal

            twist.angular.z = self.get_corrective_angle_vel(angle_to_tag)
            twist.linear.y = self.get_corrective_y_vel(y_err)

            # Only go forward if well-aligned
            if abs(angle_to_tag) <= self.angle_tol:
                if abs(y_err) <= self.y_tol: 
                    twist.linear.x = self.get_x_vel(x_err)

            in_position = abs(x_err) <= self.x_tol

            self.cmd_vel_pub.publish(twist)
            time.sleep(0.01)

        # Send stop command
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

        res.success = True
        res.side_entered = side_to_enter

        return res

    

    

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor(num_threads = 2)
    executor.add_node(GoUnderWorker())
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
