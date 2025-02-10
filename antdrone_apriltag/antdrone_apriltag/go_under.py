import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, Twist, Transform
from geometry_msgs.msg import Vector3, Quaternion
from nav_msgs.msg import Odometry
from rclpy.time import Time
from std_msgs.msg import Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import transformations
from rclpy.action import ActionServer

from antdrone_interfaces.action import GoUnder
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation as R
import numpy as np
import time
from time import sleep




class GoUnderWorker(Node):
    def __init__(self):
        super().__init__('go_under')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.wheel_odom_subscriber = self.create_subscription(Odometry, '/odom', self.record_odom, 10)
        self.update_tag_pos_timer = self.create_timer(1.0, self.update_tag_transforms) # Regularly check for transforms to tags
        self.update_pos_timer = self.create_timer(0.1, self.get_goal_err) # Regularly use tags & wheel odometry to smoothly update drone pos relative to worker

        self.go_under_action = ActionServer(self, GoUnder, 'go_under_worker', self.go_under_worker)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)




        self.asdf = StaticTransformBroadcaster(self)        
        # Do not publish these tags! They are for internal use of the go-under algorithm only. Publishing will confuse Ros2 TF system.
        # These allow us to transform worker tags to the base link for comparison to drone base link, without having to send 
        # TF frames from the actual worker to the drone.
        # worker pickup link is -90 degrees yaw rotated from worker base link. This aligns it with drone base link when drone is driving at side of worker.
        # worker_pickup_frame_x (x=1,2,3) are all the same frame IRL, but TF2 cant handle closed loops, so one frame per tag.
        self.worker_to_tag_static_tfs = [
            # Tag 0 is facing down in the middle of the worker
            TransformStamped(
                header = Header(stamp=Time(seconds=0).to_msg(), frame_id='tag16h5:0'),
                child_frame_id='worker_pickup_frame_0',
                transform=Transform(
                    translation = Vector3(x=0.0, y=0.215, z=-0.01),
                    rotation = Quaternion(x=-1.0, y=0.0, z=0.0, w=0.0)
                )
            ),

            # Tag 1 is facing down on the far side of the worker
            TransformStamped(
                header = Header(stamp=Time(seconds=0).to_msg(), frame_id='tag16h5:1'),
                child_frame_id='worker_pickup_frame_1',
                transform=Transform(
                    translation = Vector3(x=-0.34, y=0.225, z=-0.05),
                    rotation = Quaternion(x=-1.0, y=0.0, z=0.0, w=0.0)
                )
            ),

            # Tag 2 is outward facing
            TransformStamped(
                header = Header(stamp=Time(seconds=0).to_msg(), frame_id='tag16h5:2'),
                child_frame_id='worker_pickup_frame_2',
                transform=Transform(
                    translation = Vector3(x=0.345, y=0.06, z=-0.4),
                    rotation = Quaternion(x=0.5, y=-0.5, z=-0.5, w=-0.5)
                )
            ),
        ]

        self.drone_to_worker_tfs = [
            TransformStamped(),
            TransformStamped(),
            TransformStamped(),
            TransformStamped(),
            TransformStamped(),
            TransformStamped()
        ]

        self.drone_to_worker_avg_tf = TransformStamped()
        self.prev_odom_poses = [] 
        self.prev_odom_ns = self.get_clock().now().nanoseconds


        self.x_tol = 0.03
        self.y_tol = 0.01
        self.yaw_tolerance = 0.05

        self.vy_max = 0.01
        self.vx_max = 0.05
        self.vyaw_max = 5 / 180 * 3.14

    def record_odom(self, wheel_odom: Odometry):

        self.prev_odom_poses.append(wheel_odom) # Append newest odom message to end of list

        if len(self.prev_odom_poses) > 500:
            self.prev_odom_poses.pop(0) # Remove oldest TF from start of list

    def update_tag_transforms(self):

        for tag_id in range(3):
            
            self.get_transform(tag_id)

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

    def get_transform(self, tag_id):


        try:
            # Publish transform from worker pickup frame to tag to keep it fresh
            self.worker_to_tag_static_tfs[tag_id].header.stamp = self.get_clock().now().to_msg()
            self.asdf.sendTransform(self.worker_to_tag_static_tfs[tag_id])
            
            # Read Full transform from drone -> tag -> worker. tag -> worker TF is published by apriltag node.
            now = rclpy.time.Time()
            timeout = rclpy.duration.Duration(seconds=0.05)
            drone_to_worker: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link', 'worker_pickup_frame_' + str(tag_id), now, timeout
            )

            roll, pitch, yaw = self.euler_from_quat(drone_to_worker.transform.rotation)
            
            x = drone_to_worker.transform.translation.x
            y = drone_to_worker.transform.translation.y
            z = drone_to_worker.transform.translation.z

            angular_outlier = abs(pitch) >= 15/180*3.14 or abs(roll)  >= 15/180*3.14 # Drone to worker TF should be flat.
            
            linear_outlier = abs(x) >= 2.0 or abs(y) >= 1.0 or abs(z) >= 1.0

            if not (angular_outlier or linear_outlier):
                self.drone_to_worker_tfs[tag_id].header.stamp = drone_to_worker.header.stamp
                self.drone_to_worker_tfs[tag_id].transform.translation.x = x
                self.drone_to_worker_tfs[tag_id].transform.translation.y = y
                self.drone_to_worker_tfs[tag_id].transform.translation.z = yaw
        
        except Exception as e:
            pass

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
            y_vel = min(self.vy_max, 0.1 * y_err)
        elif y_err < -self.y_tol:
            y_vel = max(-self.vy_max, 0.1 * y_err)

        return float(y_vel)
    
    def get_x_vel(self, x_err):

        x_vel = 0.0
    
        if x_err > self.x_tol:
            x_vel = 0.05 * np.sqrt(abs(x_err))

        return float(x_vel)

    def determine_worker_approach_side(self):

        left_tag_ids =  [0, 1, 2]
        right_tag_ids = [3, 4, 5]

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

        x_avg_err = 0.0
        y_avg_err = 0.0
        yaw_avg_err = 0.0
        found_tags_cnt = 0

        curr_ns = self.get_clock().now().nanoseconds

        for tf in self.drone_to_worker_tfs:

            tf_ns = tf.header.stamp.sec * 1e9 + tf.header.stamp.nanosec

            for odom_msg in self.prev_odom_poses:

                msg_ns = odom_msg.header.stamp.sec * 1e9 + odom_msg.header.stamp.nanosec
                dt = abs(msg_ns - tf_ns) / 1e9

                if dt < 0.1:

                    odom_pos_last = self.prev_odom_poses[-1].pose.pose
                    odom_pos_at_tf = odom_msg.pose.pose

                    x_since_tf = odom_pos_at_tf.position.x - odom_pos_last.position.x
                    y_since_tf = odom_pos_at_tf.position.y - odom_pos_last.position.y

                    _, _, yaw0 = self.euler_from_quat(odom_pos_at_tf.orientation)
                    _, _, yaw1 = self.euler_from_quat(odom_pos_last.orientation)
                    yaw_since_tf = yaw0 - yaw1

                    x_updated_time = tf.transform.translation.x  + x_since_tf
                    y_updated_time = tf.transform.translation.y  + y_since_tf
                    z_updated_time = tf.transform.translation.z  + yaw_since_tf

                    x_avg_err += x_updated_time
                    y_avg_err += y_updated_time
                    yaw_avg_err += z_updated_time

                    found_tags_cnt += 1
                    break

        self.drone_to_worker_avg_tf.header.stamp = self.get_clock().now().to_msg()

        if found_tags_cnt > 0:
            x_avg_err /= found_tags_cnt
            y_avg_err /= found_tags_cnt
            yaw_avg_err /= found_tags_cnt
            
            self.drone_to_worker_avg_tf.transform.translation.x = x_avg_err
            self.drone_to_worker_avg_tf.transform.translation.y = y_avg_err
            self.drone_to_worker_avg_tf.transform.translation.z = yaw_avg_err # Using z translation as yaw rotation, avoiding quaternion in internal use.

            self.get_logger().info(f"x: {round(self.drone_to_worker_avg_tf.transform.translation.x, 3)}, y: {round(self.drone_to_worker_avg_tf.transform.translation.y, 3)}, yaw: {round(self.drone_to_worker_avg_tf.transform.translation.z, 3)}")
            

            

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
            
                twist = Twist() # reset twist every time

                yaw_err = self.drone_to_worker_avg_tf.transform.translation.z
                x_err = self.drone_to_worker_avg_tf.transform.translation.x
                y_err = self.drone_to_worker_avg_tf.transform.translation.y

                twist.angular.z = self.get_corrective_angle_vel(yaw_err)

                twist.linear.y = self.get_corrective_y_vel(y_err)
                # Only go forward if well-aligned
                if abs(yaw_err) <= self.yaw_tolerance:
                    if abs(y_err) <= self.y_tol: 
                        pass
                        # twist.linear.x = self.get_x_vel(x_err)

                in_position = (abs(x_err) <= self.x_tol) and (abs(yaw_err) <= self.yaw_tolerance) and (abs(y_err) <= self.y_tol)

                self.cmd_vel_pub.publish(twist) 
                sleep(1)

            # Send stop command
            twist = Twist()
            self.cmd_vel_pub.publish(twist)

            goal_handle.succeed()
            
            result.success = True
            result.side_entered = approach_side

        return result


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor(num_threads = 3)
    executor.add_node(GoUnderWorker())
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
