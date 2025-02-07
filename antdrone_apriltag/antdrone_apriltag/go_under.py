import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, Twist, Transform
from geometry_msgs.msg import Vector3, Quaternion
from rclpy.time import Time
from std_msgs.msg import Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import transformations

from antdrone_interfaces.srv import GoUnder
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
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.go_under_service = self.create_service(GoUnder, 'go_under_worker', self.go_under_worker)


        self.asdf = StaticTransformBroadcaster(self)        
        # Do not publish these tags! They are for internal use of the go-under algorithm only. Publishing will confuse Ros2 TF system.
        # These allow us to transform worker tags to the base link for comparison to drone base link, without having to send 
        # TF frames from the actual worker to the drone.
        # worker pickup link is -90 degrees yaw rotated from worker base link. This aligns it with drone base link when drone is driving at side of worker.
        # worker_pickup_frame_x (x=1,2,3) are all the same frame IRL, but TF2 cant handle closed loops, so one frame per tag.
        self.worker_to_tag_tfs = [
            # Tag 0 is facing down in the middle of the worker
            TransformStamped(
                header = Header(stamp=Time(seconds=0).to_msg(), frame_id='worker_pickup_frame_0'),
                child_frame_id='tag16h5:0',
                transform=Transform(
                    translation = Vector3(x=0.0, y=0.22, z=-0.01),
                    rotation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
                )
            ),

            # Tag 1 is facing down on the far side of the worker
            TransformStamped(
                header = Header(stamp=Time(seconds=0).to_msg(), frame_id='worker_pickup_frame_1'),
                child_frame_id='tag16h5:1',
                transform=Transform(
                    translation = Vector3(x=0.34, y=0.22, z=-0.05),
                    rotation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
                )
            ),

            # Tag 2 is outward facing
            TransformStamped(
                header = Header(stamp=Time(seconds=0).to_msg(), frame_id='tag16h5:2'),
                child_frame_id='worker_pickup_frame_2',
                transform=Transform(
                    translation = Vector3(x=0.34, y=0.06, z=-0.4),
                    rotation = Quaternion(x=0.5, y=-0.5, z=-0.5, w=-0.5)
                )
            ),
        ]

        self.drone_to_worker_tfs = [
            None,
            None,
            None,
            None,
            None,
            None
        ]


        self.x_tol = 0.03
        self.y_tol = 0.01
        self.yaw_tolerance = 0.05

        self.vy_max = 0.01
        self.vx_max = 0.05
        self.vyaw_max = 5 / 180 * 3.14

    def update_transforms(self, useable_tags):

        for tag_id in useable_tags:
            
            self.get_transform(tag_id)

    def get_transform(self, tag_id):

        if tag_id == 2:

            try:
                # Publish transform from worker pickup frame to tag to keep it fresh
                self.asdf.sendTransform(self.worker_to_tag_tfs[tag_id])
                
                # Read Full transform from drone -> tag -> worker. tag -> worker TF is published by apriltag node.
                now = rclpy.time.Time()
                timeout = rclpy.duration.Duration(seconds=0.05)
                drone_to_worker: TransformStamped = self.tf_buffer.lookup_transform(
                    'base_link', 'worker_pickup_frame_' + str(tag_id), now, timeout
                )
                
                # self.get_logger().info(f"Transform:: {drone_to_worker}")

                self.drone_to_worker_tfs[tag_id] = drone_to_worker
                    

            except Exception as e:
                self.get_logger().info(f"Coudl not get transform for tag {tag_id}: {e}")
                self.drone_to_worker_tfs[tag_id] = None
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
            x_vel = 0.15 * np.sqrt(abs(x_err))

        return float(x_vel)

    def determine_worker_approach_side(self):

        self.get_logger().info(f"Transforms at count time {self.drone_to_worker_tfs}")

        left_tag_ids =  [0, 1, 2]
        right_tag_ids = [3, 4, 5]

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
    
    def get_goal_err(self, useable_tags):

        x_avg_err = 0.0
        y_avg_err = 0.0
        yaw_avg_err = 0.0
        found_tags_cnt = 0

        for tag_id in useable_tags:

            if self.drone_to_worker_tfs[tag_id] != None:

                transform = self.drone_to_worker_tfs[tag_id]

                qx = transform.transform.rotation.x
                qy = transform.transform.rotation.y
                qz = transform.transform.rotation.z
                qw = transform.transform.rotation.w

                rotation = R.from_quat([qx, qy, qz, qw])

                euler_angles = rotation.as_euler('xyz', degrees=False)

                roll = euler_angles[0]
                pitch = euler_angles[1]
                yaw = euler_angles[2]
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                z = transform.transform.translation.z

                angular_outlier = abs(pitch) >= 15/180*3.14 or abs(roll)  <= 170/180*3.14
                
                linear_outlier = abs(x) >= 2.0 or abs(y) >= 1.0 or abs(z) >= 1.0

                if not ( linear_outlier): #angular_outlier or

                    if tag_id == 2:
                        self.get_logger().info(f"tag_id: {tag_id}, x: {round(x, 2)}, y: {round(y, 2)}, roll: {round(roll, 2)}, pitch: {round(pitch, 2)}, yaw: {round(yaw, 2)}")
                
                    x_avg_err += x
                    y_avg_err += y
                    yaw_avg_err += yaw
                    found_tags_cnt += 1
                else:
                    if tag_id == 2:
                        self.get_logger().info(f"tag_id: {tag_id}, OUTLIER x: {round(x, 2)}, y: {round(y, 2)}, roll: {round(roll, 2)}, pitch: {round(pitch, 2)}, yaw: {round(yaw, 2)}")

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
        self.drone_to_worker_tfs = [
            None,
            None,
            None,
            None,
            None,
            None
        ]
        time.sleep(3) # Allow a few seconds to find tags after clearing
        self.update_transforms([0,1,2,3,4,5])
        
        # Determine if left or right side tags on worker will be visible/used during go under operation
        useable_tags, approach_side = self.determine_worker_approach_side()
        useable_tags = [0, 1, 2] # Only have rear-side tags mounted right now

        if approach_side != 'N':

            in_position = False
            while not in_position:

                # Update transforms each iteration. I tried using a timer, it is blocked by this loop.
                self.update_transforms(useable_tags)

                n_tags_found, x_err, y_err, yaw_err = self.get_goal_err(useable_tags)
 
                twist = Twist() # Refresh twist every time

                if n_tags_found > 0:

                    twist.angular.z = self.get_corrective_angle_vel(yaw_err)

                    twist.linear.y = self.get_corrective_y_vel(y_err)

                    # self.get_logger().info(f"{n_tags_found, round(x_err, 2), round(y_err, 2), round(yaw_err, 2)}")

                    # Only go forward if well-aligned
                    if abs(yaw_err) <= self.yaw_tolerance:
                        if abs(y_err) <= self.y_tol: 
                            pass
                            # twist.linear.x = self.get_x_vel(x_err)

                    in_position = (abs(x_err) <= self.x_tol) and (abs(yaw_err) <= self.yaw_tolerance) and (abs(y_err) <= self.y_tol)

                else:
                    self.get_logger().info(f"No tags found")

                    # TODO - trigger recovery behaviors
                    pass

                self.cmd_vel_pub.publish(twist)
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
