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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_callback_group = ReentrantCallbackGroup()
        self.sub_callback_group2 = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()

        # Using subscription instead of TFListener allows us to get transforms instantly (like an interrupt instead of polling with TF Listener on a timer)
        self.tf_apriltag_sub = self.create_subscription(TFMessage, '/tf_apriltag', self.record_tag_tf, 10, callback_group=self.sub_callback_group)
        self.wheel_odom_subscriber = self.create_subscription(Odometry, '/odom', self.record_odom, 10, callback_group=self.sub_callback_group2)

        self.go_under_action = ActionServer(self, GoUnder, 'go_under_worker', self.go_under_worker, callback_group=self.action_callback_group)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # List of drone's previous odom -> base link TFs, subtract recent from current to get displacement since last tag capture.
        self.pose_chain = [] 
        self.poses_at_tag_capture = [ # Holds odom -> base link TF at capture time for each tag, to be used in extrapolation.
            TransformStamped(),
            TransformStamped(),
            TransformStamped(),
            TransformStamped(),
            TransformStamped(),
            TransformStamped()
        ]

        self.base_link_to_cam_tf = TransformStamped(
            header = Header(stamp=Time(seconds=0).to_msg(), frame_id='base_link'),
            child_frame_id='apriltag_cam',
            transform=Transform(
                translation = Vector3(x=0.0, y=0.216, z=0.195),
                rotation = Quaternion(x=-0.329, y=0.329, z=-0.625, w=0.625)
            )
        )

        # Do not publish these tags! They are for internal use of the go-under algorithm only. Publishing will confuse Ros2 TF system.
        # These allow us to transform worker tags to the base link for comparison to drone base link, without having to send 
        # TF frames from the actual worker to the drone.
        # worker pickup link is -90 degrees yaw rotated from worker base link. This aligns it with drone base link when drone is driving at side of worker.
        # worker_pickup_frame_x (x=1,2,3) are all the same frame IRL, but TF2 cant handle closed loops, so one frame per tag.
        self.worker_to_tag_static_tfs = [

            # Tag 0 is outward facing
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
                    translation = Vector3(x=0.21, y=0.205, z=-0.05),
                    rotation = Quaternion(x=-0.707, y=0.707, z=0.0, w=0.0)
                )
            ),

            # Tag 2 is facing down in the middle of the worker
            TransformStamped(
                header = Header(stamp=Time(seconds=0).to_msg(), frame_id='tag16h5:2'),
                child_frame_id='worker_pickup_frame_2',
                transform=Transform(
                    translation = Vector3(x=0.21, y= 0.0, z=-0.05),
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

        self.asdf = StaticTransformBroadcaster(self)     

        # Tolerances
        self.x_tol = 0.01
        self.y_tol = 0.01
        self.yaw_tolerance = 2 / 180*3.14
        
        # Maximum velocities
        self.vy_max = 0.05
        self.vx_max = 0.05
        self.vyaw_max = 5.0 / 180 * 3.14

    def record_tag_tf(self, tf_message: TFMessage):

        for drone_cam_to_tag in tf_message.transforms:
            
            tag_id = int(drone_cam_to_tag.child_frame_id.split(':')[1])

            tag_to_worker = self.worker_to_tag_static_tfs[tag_id]
            tag_to_worker.header.stamp = self.get_clock().now().to_msg()
            self.base_link_to_cam_tf.header.stamp = self.get_clock().now().to_msg()

            # self.asdf.sendTransform(self.base_link_to_cam_tf)
            # self.asdf.sendTransform(drone_cam_to_tag)
            base_link_to_tag = self.get_transform_a_to_c(self.base_link_to_cam_tf, drone_cam_to_tag)
            drone_to_worker_tf = self.get_transform_a_to_c(base_link_to_tag, tag_to_worker)
            self.asdf.sendTransform(drone_to_worker_tf)

            roll, pitch, yaw = self.euler_from_quat(drone_to_worker_tf.transform.rotation)
            
            x = drone_to_worker_tf.transform.translation.x
            y = drone_to_worker_tf.transform.translation.y
            z = drone_to_worker_tf.transform.translation.z

            angular_outlier = abs(pitch) >= 15/180*3.14 or abs(roll)  >= 15/180*3.14 # Drone to worker TF should be flat.
            
            linear_outlier = abs(x) >= 2.0 or abs(y) >= 1.0 or abs(z) >= 1.0

            if not (angular_outlier or linear_outlier):
                
                for pos_in_odom_at_tag_capture_time in reversed(self.pose_chain): # Check last items first, most recent poses at end of list.

                    odom_capture_s = pos_in_odom_at_tag_capture_time.header.stamp.sec + pos_in_odom_at_tag_capture_time.header.stamp.nanosec / 1e9
                    tag_capture_s = drone_to_worker_tf.header.stamp.sec + drone_to_worker_tf.header.stamp.nanosec / 1e9

                    dt = odom_capture_s - tag_capture_s

                    # self.get_logger().info(f"{dt}, {odom_capture_s}, {tag_capture_s}")

                    if abs(dt) < 0.166*2:
                        self.poses_at_tag_capture[tag_id] = pos_in_odom_at_tag_capture_time
                        self.drone_to_worker_tfs[tag_id] = drone_to_worker_tf
                        self.drone_to_worker_tfs[tag_id].transform.translation.z = yaw # Using z coord for yaw to avoid transforming between RPY and Quaternion all the time
                        self.get_logger().info(f"updated {round(x, 2), round(y, 2), round(yaw, 2)}")
                        break

                    elif dt < -0.166*2:
                        self.get_logger().info("no odom within time lim")
                        break



    def record_odom(self, odom: Odometry):

        # Record drone poses in odom frame. 
        # We will subtract poses in odom frame at tag capture time from poses in odom frame at current time to extrapolate robot position until next tag capture.

        pos_in_odom_frame = TransformStamped()
        pos_in_odom_frame.child_frame_id = 'base_link'
        pos_in_odom_frame.header.frame_id = 'odom'
        pos_in_odom_frame.header.stamp = odom.header.stamp
        
        pos_in_odom_frame.transform.translation.x = odom.pose.pose.position.x
        pos_in_odom_frame.transform.translation.y = odom.pose.pose.position.y
        pos_in_odom_frame.transform.translation.z = odom.pose.pose.position.z

        pos_in_odom_frame.transform.rotation.x = odom.pose.pose.orientation.x
        pos_in_odom_frame.transform.rotation.y = odom.pose.pose.orientation.y
        pos_in_odom_frame.transform.rotation.z = odom.pose.pose.orientation.z
        pos_in_odom_frame.transform.rotation.w = odom.pose.pose.orientation.w

        self.pose_chain.append(pos_in_odom_frame) # Append newest odom message to end of list

        # self.get_logger().info("odom")

        if len(self.pose_chain) > 50:
            self.pose_chain.pop(0) # Remove oldest TF from start of list

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
            x_vel = min(self.vx_max, np.sqrt(abs(x_err)))

        if x_err < 0:
            x_vel *= -1

        return float(x_vel)

    def determine_worker_approach_side(self):

        left_tag_ids =  [0, 1, 2]
        right_tag_ids = [3,4]

        # TODO - update to check recency of TFs instead of None check
        left_tags_visible_cnt =  sum(1 for key in left_tag_ids if self.drone_to_worker_tfs[key] is not None)
        right_tags_visible_cnt = sum(1 for key in right_tag_ids if self.drone_to_worker_tfs[key] is not None)

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

        if not len(self.pose_chain):
            self.get_logger().info("pos chain 0 length")
            return

        x_avg_err = 0.0
        y_avg_err = 0.0
        yaw_avg_err = 0.0
        found_tags_cnt = 0

        for tag_id in range(len(self.drone_to_worker_tfs)):

            drone_to_worker_tf = self.drone_to_worker_tfs[tag_id]
            odom_to_drone_at_tag_capture_tf = self.poses_at_tag_capture[tag_id]

            tag_life_length_s = self.get_clock().now().to_msg().sec - drone_to_worker_tf.header.stamp.sec

            if tag_life_length_s <= 2: # Only use fairly recent tag captures for extrapolation.

                odom_pos_last = self.pose_chain[-1]

                dx = odom_pos_last.transform.translation.x - odom_to_drone_at_tag_capture_tf.transform.translation.x
                dy = odom_pos_last.transform.translation.y - odom_to_drone_at_tag_capture_tf.transform.translation.y

                _, _, yaw0 = self.euler_from_quat(odom_to_drone_at_tag_capture_tf.transform.rotation)
                _, _, yaw1 = self.euler_from_quat(odom_pos_last.transform.rotation)
                dyaw = yaw1 - yaw0


                x_extrapolated = drone_to_worker_tf.transform.translation.x  - dx
                y_extrapolated = drone_to_worker_tf.transform.translation.y  - dy
                z_extrapolated = drone_to_worker_tf.transform.translation.z  - dyaw # Subtraact to work with the frames

                x_avg_err += x_extrapolated
                y_avg_err += y_extrapolated
                yaw_avg_err += z_extrapolated

                found_tags_cnt += 1
            else:
                pass
            

        self.drone_to_worker_avg_tf.header.stamp = self.get_clock().now().to_msg()

        if found_tags_cnt > 0:
            x_avg_err /= found_tags_cnt
            y_avg_err /= found_tags_cnt
            yaw_avg_err /= found_tags_cnt
            
            self.drone_to_worker_avg_tf.transform.translation.x = x_avg_err
            self.drone_to_worker_avg_tf.transform.translation.y = y_avg_err
            self.drone_to_worker_avg_tf.transform.translation.z = yaw_avg_err # Using z translation as yaw rotation, avoiding quaternion in internal use.

            # self.get_logger().info(f"x: {round(self.drone_to_worker_avg_tf.transform.translation.x, 3)}, y: {round(self.drone_to_worker_avg_tf.transform.translation.y, 3)}, yaw: {round(self.drone_to_worker_avg_tf.transform.translation.z, 3)}")
            
        return found_tags_cnt

        

    def go_under_worker(self, goal_handle):
        # Assumes an apriltag is visible in apriltag cam FOV when called

        result = GoUnder.Result()
        result.success = False
    
        # Determine if left or right side tags on worker will be visible/used during go under operation
        useable_tags, approach_side = self.determine_worker_approach_side()

        if approach_side != 'N':

            in_position = False
            while not in_position:

                self.get_goal_err()
            
                twist = Twist() # reset twist every time

                yaw_err = self.drone_to_worker_avg_tf.transform.translation.z
                x_err = self.drone_to_worker_avg_tf.transform.translation.x
                y_err = self.drone_to_worker_avg_tf.transform.translation.y

                twist.angular.z = self.get_corrective_angle_vel(yaw_err)

                twist.linear.y = self.get_corrective_y_vel(y_err)

                well_aligned = abs(y_err) <= self.y_tol and abs(yaw_err) <= self.yaw_tolerance
                front_under_worker = x_err <=  0.7
                far_away = x_err >= 1.1
                if well_aligned or front_under_worker or far_away:
                    pass
                    # twist.linear.x = self.get_x_vel(x_err)

                in_position = (abs(x_err) <= self.x_tol) and (abs(yaw_err) <= self.yaw_tolerance) and (abs(y_err) <= self.y_tol)

                self.cmd_vel_pub.publish(twist) 
                # sleep(0.05) # Throttle to max 20hz

            # Send stop command
            twist = Twist()
            self.cmd_vel_pub.publish(twist)

            goal_handle.succeed()
            
            result.success = True
            result.side_entered = approach_side

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
    # rclpy.init(args=args)
    # go_under = GoUnderWorker()
    # rclpy.spin(go_under)
    # go_under.destroy_node()
    # rclpy.shutdown()

    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads = 3) # 3 threads are needed to run subscribers & timers while action callback is running
    executor.add_node(GoUnderWorker())
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
