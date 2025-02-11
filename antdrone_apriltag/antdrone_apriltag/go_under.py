import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, Twist, Transform
from geometry_msgs.msg import Vector3, Quaternion, Point
from nav_msgs.msg import Odometry
from rclpy.time import Time
from std_msgs.msg import Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import transformations
from rclpy.action import ActionServer
from tf2_msgs.msg import TFMessage

from antdrone_interfaces.action import GoUnder
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation as R
import numpy as np
import time
from time import sleep
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup


class GoUnderWorker(Node):
    def __init__(self):
        super().__init__('go_under')

        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()

        # Using subscription instead of TFListener allows us to get transforms instantly and without missing TFs (like an interrupt instead of polling with TF Listener on a timer)
        self.tf_apriltag_sub = self.create_subscription(TFMessage, '/tf', self.record_tag_tf, 10, callback_group=self.sub_callback_group)

        self.go_under_action = ActionServer(self, GoUnder, 'go_under_worker', self.go_under_worker, callback_group=self.action_callback_group)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)


        # Transforms needed to get full transform from drone base link to worker pickup link
        self.base_link_to_cam_tf = TransformStamped(
            header = Header(stamp=Time(seconds=0).to_msg(), frame_id='base_link'),
            child_frame_id='apriltag_cam',
            transform=Transform(
                translation = Vector3(x=0.0, y=0.216, z=0.195),
                rotation = Quaternion(x=-0.329, y=0.329, z=-0.625, w=0.625)
            )
        )

        self.worker_to_tag_static_tfs = [

            # Tag 0 is outward facing on worker
            TransformStamped(
                header = Header(stamp=Time(seconds=0).to_msg(), frame_id='tag16h5:0'),
                child_frame_id='worker_pickup_frame_0',
                transform=Transform(
                    translation = Vector3(x=0.385 + 0.02, y=0.124+0.01, z=-0.4),
                    rotation = Quaternion(x=0.5, y=-0.5, z=-0.5, w=-0.5)
                )
            ),

            # Tag 1 is facing down on the far side of the worker
            TransformStamped(
                header = Header(stamp=Time(seconds=0).to_msg(), frame_id='tag16h5:1'),
                child_frame_id='worker_pickup_frame_1',
                transform=Transform(
                    translation = Vector3(x=0.232 + 0.02, y=0.24, z=-0.05),
                    rotation = Quaternion(x=-0.707, y=0.707, z=0.0, w=0.0)
                )
            ),

            # Tag 2 is facing down in the middle of the worker
            TransformStamped(
                header = Header(stamp=Time(seconds=0).to_msg(), frame_id='tag16h5:2'),
                child_frame_id='worker_pickup_frame_2',
                transform=Transform(
                    translation = Vector3(x=0.21+0.025, y= 0.0, z=-0.05),
                    rotation = Quaternion(x=-0.707, y=0.707, z=0.0, w=0.0)
                )
            ),
        ]

        self.drone_to_worker_tfs = [
            TransformStamped(),
            TransformStamped(),
            TransformStamped(),
            TransformStamped(),
            TransformStamped(),
            TransformStamped(),
        ]

        self.drone_to_worker_avg_tf = TransformStamped()

        self.tf_broadcaster = StaticTransformBroadcaster(self)     

        # Tolerances
        self.x_tol = 0.04
        self.y_tol = 0.02
        self.yaw_tolerance = 4 / 180*3.14
        
        # Maximum velocities
        self.vy_max = 0.01
        self.vx_max = 0.05
        self.vyaw_max = 3.0 / 180 * 3.14

    def record_tag_tf(self, tf_message: TFMessage):

        for tf in tf_message.transforms:

            if tf.header.frame_id == 'apriltag_cam':

                tag_id = int(tf.child_frame_id.split(':')[1])

                tag_to_worker = self.worker_to_tag_static_tfs[tag_id]
                tag_to_worker.header.stamp = self.get_clock().now().to_msg()
                self.base_link_to_cam_tf.header.stamp = self.get_clock().now().to_msg()

                # self.tf_broadcaster.sendTransform(self.base_link_to_cam_tf)
                # self.tf_broadcaster.sendTransform(drone_cam_to_tag)
                base_link_to_tag = self.get_transform_a_to_c(self.base_link_to_cam_tf, tf)
                drone_to_worker_tf = self.get_transform_a_to_c(base_link_to_tag, tag_to_worker)
                self.tf_broadcaster.sendTransform(drone_to_worker_tf)

                roll, pitch, yaw = self.euler_from_quat(drone_to_worker_tf.transform.rotation)
                
                x = drone_to_worker_tf.transform.translation.x
                y = drone_to_worker_tf.transform.translation.y
                z = drone_to_worker_tf.transform.translation.z

                angular_outlier = abs(pitch) >= 15/180*3.14 or abs(roll)  >= 15/180*3.14 # Drone to worker TF should be flat.
                
                linear_outlier = abs(x) >= 2.0 or abs(y) >= 1.0 or abs(z) >= 1.0

                if not (angular_outlier or linear_outlier):
                    
                    self.drone_to_worker_tfs[tag_id] = drone_to_worker_tf
                    self.drone_to_worker_tfs[tag_id].transform.translation.z = yaw # Using z coord for yaw to avoid transforming between RPY and Quaternion all the time
                

    def euler_from_quat(self, quat):

        qx = quat.x
        qy = quat.y
        qz = quat.z
        qw = quat.w

        rotation = R.from_quat([qx, qy, qz, qw])

        euler_angles = rotation.as_euler('xyz', degrees=False)

        roll = euler_angles[0]
        pitch = euler_angles[1]
        yaw = euler_angles[2]

        return roll, pitch, yaw

    def get_corrective_angle_vel(self, yaw):
        angle_vel = 0.0

        if yaw > self.yaw_tolerance:
            angle_vel = min(self.vyaw_max, yaw)          
        elif yaw < -self.yaw_tolerance:
            angle_vel = max(-self.vyaw_max, yaw)  

        return float(angle_vel)
    
    def get_corrective_y_vel(self, y_err):
        y_vel = 0.0

        if y_err > self.y_tol:
            y_vel = min(self.vy_max, y_err)
        elif y_err < -self.y_tol:
            y_vel = max(-self.vy_max, y_err)

        return float(y_vel)
    
    def get_x_vel(self, x_err):

        x_vel = 0.0
    
        if abs(x_err) > 0:
            x_vel = min(self.vx_max, x_err)

        if x_err < 0:
            x_vel *= -1

        return float(x_vel)

    def determine_worker_approach_side(self):

        left_tag_ids =  [0, 1, 2]
        right_tag_ids =   [3, 4, 5]

        curr_s = self.get_clock().now().to_msg().sec
        left_tags_visible_cnt =  sum(1 for key in left_tag_ids if curr_s - self.drone_to_worker_tfs[key].header.stamp.sec < 3)
        right_tags_visible_cnt = sum(1 for key in right_tag_ids if curr_s - self.drone_to_worker_tfs[key].header.stamp.sec < 3)

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
    
    def get_goal_err(self):

        x_avg_err = 0.0
        y_avg_err = 0.0
        yaw_avg_err = 0.0
        found_tags_cnt = 0
        tags = []

        for tag_id in range(len(self.drone_to_worker_tfs)):

            drone_to_worker_tf = self.drone_to_worker_tfs[tag_id]

            curr_t = self.get_clock().now().to_msg()
            curr_s = curr_t.sec + curr_t.nanosec / 1e9

            tag_t = drone_to_worker_tf.header.stamp
            tag_s = tag_t.sec + tag_t.nanosec / 1e9

            tag_life_length_s = curr_s - tag_s

            if tag_life_length_s <= 0.1: # Only use fairly recent tag captures for extrapolation.

                x_extrapolated = drone_to_worker_tf.transform.translation.x
                y_extrapolated = drone_to_worker_tf.transform.translation.y
                z_extrapolated = drone_to_worker_tf.transform.translation.z

                x_avg_err += x_extrapolated
                y_avg_err += y_extrapolated
                yaw_avg_err += z_extrapolated

                found_tags_cnt += 1
                tags.append(tag_id)
                

            elif tag_s > 1: # Filter out no-transform tags
                pass
            

        self.drone_to_worker_avg_tf.header.stamp = self.get_clock().now().to_msg()

        if found_tags_cnt > 0:
            x_avg_err /= found_tags_cnt
            y_avg_err /= found_tags_cnt
            yaw_avg_err /= found_tags_cnt
            
            self.drone_to_worker_avg_tf.transform.translation.x = x_avg_err
            self.drone_to_worker_avg_tf.transform.translation.y = y_avg_err
            self.drone_to_worker_avg_tf.transform.translation.z = yaw_avg_err # Using z translation as yaw rotation, avoiding quaternion in internal use.

            self.get_logger().info(f"tags {tags}, x: {round(self.drone_to_worker_avg_tf.transform.translation.x, 3)}, y: {round(self.drone_to_worker_avg_tf.transform.translation.y, 3)}, yaw: {round(self.drone_to_worker_avg_tf.transform.translation.z, 3)}")
            
        else:
            self.get_logger().info("No Tags found within 0.1s timeout")

        return found_tags_cnt

        

    def go_under_worker(self, goal_handle):
        # Assumes an apriltag is visible in apriltag cam FOV when called

        result = GoUnder.Result()
        result.success = False
    
        # Determine if left or right side tags on worker will be visible/used during go under operation
        useable_tags, approach_side = self.determine_worker_approach_side()

        curr_t = self.get_clock().now().to_msg()
        curr_s = curr_t.sec + curr_t.nanosec / 1e9

        last_publish_t = self.get_clock().now().to_msg()
        last_publish_s = last_publish_t.sec + last_publish_t.nanosec / 1e9

        if approach_side != 'N':

            in_position = False
            no_tags_cnt = 0

            while not in_position and no_tags_cnt < 50:

                found_tags_cnt = self.get_goal_err()

                if found_tags_cnt > 0:
                    no_tags_cnt = 0
            
                    twist = Twist() # reset twist every time

                    yaw_err = self.drone_to_worker_avg_tf.transform.translation.z
                    x_err = self.drone_to_worker_avg_tf.transform.translation.x
                    y_err = self.drone_to_worker_avg_tf.transform.translation.y

                    twist.angular.z = self.get_corrective_angle_vel(yaw_err)

                    twist.linear.y = self.get_corrective_y_vel(y_err)

                    well_aligned = abs(y_err) <= self.y_tol and abs(yaw_err) <= self.yaw_tolerance
                    front_under_worker = x_err <=  0.7
                    far_away = x_err >= 0.9
                    if well_aligned or front_under_worker or far_away:
                        pass
                        twist.linear.x = self.get_x_vel(x_err)

                    in_position = (abs(x_err) <= self.x_tol) and (abs(yaw_err) <= self.yaw_tolerance) and (abs(y_err) <= self.y_tol)

                    curr_t = self.get_clock().now().to_msg()
                    curr_s = curr_t.sec + curr_t.nanosec / 1e9

                    if curr_s - last_publish_s >= 0.016: # Throttle publishing to max 60hz
                        last_publish_s = curr_s
                        self.cmd_vel_pub.publish(twist)       

                else:
                    no_tags_cnt += 1
                    sleep(0.1)


            # Send stop command
            twist = Twist()
            self.cmd_vel_pub.publish(twist)

            

            if in_position and no_tags_cnt < 50:
                goal_handle.succeed()
                result.success = True
                result.side_entered = approach_side
            else:
                goal_handle.abort()
                result.success = False


        return result



    def get_transform_a_to_c(self, tf_a_b: TransformStamped, tf_b_c: TransformStamped) -> TransformStamped:
        """
        Given:
        tf_a_b: TransformStamped representing the transform T_{A->B}
        tf_b_c: TransformStamped representing the transform T_{B->C}
        
        This function computes T_{A->C} by:
        
            T_{A->C} = T_{A->B} * T_{B->C}
        
        using homogeneous matrices.
        """
        # --- Create homogeneous transformation matrix for T_{A->B} ---
        t_ab = np.array([
            tf_a_b.transform.translation.x,
            tf_a_b.transform.translation.y,
            tf_a_b.transform.translation.z
        ])
        q_ab = np.array([
            tf_a_b.transform.rotation.w,
            tf_a_b.transform.rotation.x,
            tf_a_b.transform.rotation.y,
            tf_a_b.transform.rotation.z,
            
        ])
        # Create a 4x4 rotation matrix from the quaternion and insert translation.
        M_ab = transformations.quaternion_matrix(q_ab)
        M_ab[:3, 3] = t_ab

        # --- Create homogeneous transformation matrix for T_{B->C} ---
        t_bc = np.array([
            tf_b_c.transform.translation.x,
            tf_b_c.transform.translation.y,
            tf_b_c.transform.translation.z
        ])
        q_bc = np.array([
            tf_b_c.transform.rotation.w,
            tf_b_c.transform.rotation.x,
            tf_b_c.transform.rotation.y,
            tf_b_c.transform.rotation.z,
            
        ])
        M_bc = transformations.quaternion_matrix(q_bc)
        M_bc[:3, 3] = t_bc

        # --- Compose the transforms: T_{A->C} = T_{A->B} * T_{B->C} ---
        M_ac = np.dot(M_ab, M_bc)

        # --- Extract the translation and rotation from the resulting matrix ---
        t_ac = M_ac[:3, 3]
        q_ac = transformations.quaternion_from_matrix(M_ac)

        # --- Build the resulting TransformStamped message ---
        tf_a_c = TransformStamped()
        # You can choose which timestamp to use (here we use tf_b_c's timestamp)
        tf_a_c.header.stamp = tf_b_c.header.stamp  
        # The resulting transform is expressed in frame A (parent) to frame C (child)
        tf_a_c.header.frame_id = tf_a_b.header.frame_id  
        tf_a_c.child_frame_id = tf_b_c.child_frame_id

        tf_a_c.transform.translation.x = t_ac[0]
        tf_a_c.transform.translation.y = t_ac[1]
        tf_a_c.transform.translation.z = t_ac[2]

        tf_a_c.transform.rotation.x = q_ac[1]
        tf_a_c.transform.rotation.y = q_ac[2]
        tf_a_c.transform.rotation.z = q_ac[3]
        tf_a_c.transform.rotation.w = q_ac[0]

        return tf_a_c



def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads = 3) # 3 threads are needed to run subscribers & timers while action callback is running
    executor.add_node(GoUnderWorker())
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
