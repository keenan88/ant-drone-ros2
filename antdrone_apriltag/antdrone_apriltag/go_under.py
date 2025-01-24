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

        self.transforms = {
            '0': {'x': None, 'y': None, 'yaw': None},
            '1': {'x': None, 'y': None, 'yaw': None},
            '2': {'x': None, 'y': None, 'yaw': None},
            '3': {'x': None, 'y': None, 'yaw': None},
            '4': {'x': None, 'y': None, 'yaw': None},
            '5': {'x': None, 'y': None, 'yaw': None},
        }

        # Setup the desired distances from the tag's to the drone's base link
        tag_spread_m = 0.33
        self.target_x_dists = {
            '0': 0, # Tag 0 is centered on worker's rear side
            '1': tag_spread_m, # Tag 1 is far on worker's rear side
            '2': -tag_spread_m, # Tag 0 is near on worker's rear side

            # Worker front side tags not placed yet
            '3': 0,
            '4': 0,
            '5': 0,
        }
        self.target_y_dist = 0.56/2 # Worker underbelly is 0.56m wide, aim for drone base link to be in middle        

        self.x_tol = 0.03
        self.y_tol = 0.01
        self.angle_tol = 0.05

        self.vy_max = 0.05
        self.vx_max = 0.05
        self.vyaw_max = 5 / 180 * 3.14

    def update_transforms(self):

        for tag_id in range(6):
            self.get_transform(str(tag_id))

    def get_transform(self, tag_id):
        x_to_tag = None
        y_to_tag = None
        t = None
        angle_to_tag = None

        try:
            now = rclpy.time.Time()
            timeout = rclpy.duration.Duration(seconds=0.05)
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link', 'tag16h5:' + tag_id, now, timeout
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

            self.transforms[tag_id]['x'] = x_to_tag
            self.transforms[tag_id]['y'] = y_to_tag
            self.transforms[tag_id]['yaw'] = angle_to_tag

            # self.get_logger().info(f"tag {tag_id} x: {round(x_to_tag,2)}, y: {round(y_to_tag,2)}, yaw: {round(angle_to_tag,2)}")

        except Exception as e:
            pass
            # self.get_logger().info(f"Could not get transform: {e}")



    def get_corrective_angle_vel(self, angle_to_tag):
        angle_vel = 0.0

        if angle_to_tag > 0:
            angle_vel = min(self.vyaw_max, 1.0 * angle_to_tag)
        elif angle_to_tag < 0:
            angle_vel = max(-self.vyaw_max, 1.0 * angle_to_tag)

        return angle_vel
    
    def get_corrective_y_vel(self, y_err):
        y_vel = 0.0

        if y_err > self.y_tol:
            y_vel = min(0.05, 1.0 * y_err)
        elif y_err < -self.y_tol:
            y_vel = max(-0.05, 1.0 * y_err)

        return y_vel
    
    def get_x_vel(self, x_err):

        x_vel = 0.0
    
        if x_err > self.x_tol:
            x_vel = 0.15 * np.sqrt(abs(x_err))

        return x_vel

    def determine_worker_approach_side(self):

        self.get_logger().info(f"Transforms at count time {self.transforms}")

        left_tag_ids =  ['0','1','2']
        right_tag_ids = ['3','4','5']

        left_tags_visible_cnt =  sum(1 for key in left_tag_ids if self.transforms[key]['x'] is not None)
        right_tags_visible_cnt = sum(1 for key in right_tag_ids if self.transforms[key]['x'] is not None)

        approach_side = 'N'
        useable_tags = []

        if left_tags_visible_cnt + right_tags_visible_cnt > 0:  
            if left_tags_visible_cnt > right_tags_visible_cnt:
                useable_tags = left_tag_ids
                approach_side = 'L'
            else:
                useable_tags = right_tag_ids
                approach_side = 'R'

        return useable_tags, approach_side
    
    def get_goal_err(self, useable_tags):

        x_avg_err = 0
        y_avg_err = 0
        yaw_avg_err = 0
        found_tags_cnt = 0

        for tag_id in useable_tags:

            if self.transforms[tag_id]['x'] != None:

                x_avg_err += self.transforms[tag_id]['x'] - self.target_x_dists[tag_id]
                y_avg_err += self.transforms[tag_id]['y'] - self.target_y_dist
                yaw_avg_err += self.transforms[tag_id]['yaw'] # Target yaw is 0, no subtraction necessary
                found_tags_cnt += 1

        if found_tags_cnt > 0:
            x_avg_err /= found_tags_cnt
            y_avg_err /= found_tags_cnt
            yaw_avg_err /= found_tags_cnt

        return found_tags_cnt, x_avg_err, y_avg_err, yaw_avg_err

        

    def go_under_worker(self, req, res):
        # Assumes an apriltag is visible in apriltag cam FOV when called

        res.success = False

        # # Clear buffer & transforms
        self.tf_buffer = Buffer() 
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.transforms = {
            '0': {'x': None, 'y': None, 'yaw': None},
            '1': {'x': None, 'y': None, 'yaw': None},
            '2': {'x': None, 'y': None, 'yaw': None},
            '3': {'x': None, 'y': None, 'yaw': None},
            '4': {'x': None, 'y': None, 'yaw': None},
            '5': {'x': None, 'y': None, 'yaw': None},
        }
        time.sleep(3) # Allow a few seconds to find tags after clearing
        self.update_transforms()
        

        # Determine if left or right side tags on worker will be visible/used during go under operation
        useable_tags, approach_side = self.determine_worker_approach_side()

        if approach_side != 'N':

            twist = Twist()

            in_position = False
            while not in_position:

                # Update transforms each iteration. I tried using a timer, it is blocked by this loop.
                self.update_transforms()

                n_tags_found, x_err, y_err, yaw_err = self.get_goal_err(useable_tags)

                self.get_logger().info(f"{n_tags_found, round(x_err, 2), round(y_err, 2), round(yaw_err, 2)}")

                if n_tags_found > 0:

                    twist.angular.z = self.get_corrective_angle_vel(yaw_err)
                    twist.linear.y = self.get_corrective_y_vel(y_err)

                    # Only go forward if well-aligned
                    if abs(yaw_err) <= self.angle_tol:
                        if abs(y_err) <= self.y_tol: 
                            twist.linear.x = self.get_x_vel(x_err)

                    in_position = (abs(x_err) <= self.x_tol) and (abs(yaw_err) <= self.angle_tol) and (abs(y_err) <= self.y_tol)

                    self.cmd_vel_pub.publish(twist)

                else:
                    # TODO - trigger recovery behaviors
                    pass

                time.sleep(0.01)

            # Send stop command
            twist = Twist()
            self.cmd_vel_pub.publish(twist)

            res.success = True
            res.side_entered = approach_side
            self.get_logger().info(f"side entered: {res.side_entered}")

        return res

    

    

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor(num_threads = 3)
    executor.add_node(GoUnderWorker())
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
