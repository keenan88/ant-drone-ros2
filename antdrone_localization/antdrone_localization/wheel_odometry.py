import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math
import numpy as np
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


# Kinematics source: https://www.researchgate.net/publication/308570348_Inverse_kinematic_implementation_of_four-wheels_mecanum_drive_mobile_robot_using_stepper_motors
        
class MecanumStateEstimator(Node):
    def __init__(self):
        super().__init__('mecanum_state_estimator')

        self.wheel_radius_m = 0.0762
        self.lx = 0.193
        self.ly = 0.258

        self.x = 0.0 
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0

        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            JointState,
            'wheel_joint_states',
            self.joint_state_callback,
            qos_best_effort
        )

        self.odom_publish_timer = self.create_timer(1, self.publish_odometry)

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.odom_base_link_tf = TransformStamped()
        self.odom_base_link_tf.header.frame_id = 'odom'
        self.odom_base_link_tf.child_frame_id = 'base_link'
        
        self.odometry_publisher = self.create_publisher(Odometry, 'odom', 10)
        
        self.prev_ns = None

    def joint_state_callback(self, msg):
        if not self.prev_ns:
            self.prev_ns = self.get_clock().now().nanoseconds
            self.x = 0.0 
            self.y = 0.0
            self.theta = 0.0
            self.vx = 0.0
            self.vy = 0.0
            self.vtheta = 0.0
        else:
            # Assuming joint names are ordered: front_left, front_right, back_left, back_right
            wheels_rad_vels = np.array(msg.velocity[0:4])

            # Deadband
            if np.all(np.abs(wheels_rad_vels) < 0.05):
                wheels_rad_vels = np.array([0,0,0,0])
        
            self.vx, self.vy, self.vtheta = self.inverse_kinematics(wheels_rad_vels)

            # self.get_logger().info(f"vx: {round(self.vx, 2)}, vy: {round(self.vy, 2)}, wz: {round(self.vtheta, 2)}, x: {round(self.x, 2)}, y: {round(self.y, 2)}, yaw: {round(self.theta, 2)}")

            # Integrate velocities to update position
            curr_ns = self.get_clock().now().nanoseconds
            dt = (curr_ns - self.prev_ns) / 1e9
            self.prev_ns = curr_ns

            # Update the robot's position based on the velocities
            # delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
            # delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
            # delta_theta = self.vtheta * dt

            z_rot_matrix = np.array([
                [np.cos(self.theta), -np.sin(self.theta), 0],
                [np.sin(self.theta),  np.cos(self.theta), 0],
                [0,                  0,                   1]
            ])

            velocity_vector = np.array([self.vx, self.vy, self.vtheta])

            delta_x, delta_y, delta_theta = np.dot(z_rot_matrix, velocity_vector) * dt

            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta

            

    def inverse_kinematics(self, wheels_rad_vels):
        fl, fr, bl, br = wheels_rad_vels

        # Right side wheels have negative rotation when drone driving forward.
        fr = -fr
        br = -br

        # self.get_logger().info(f"{round(fl, 2), round(fr, 2), round(bl, 2), round(br, 2)}")

        inv_kin_mat = np.array([
            [1, -1, -(self.lx + self.ly)],
            [1,  1,  (self.lx + self.ly)],
            [1,  1, -(self.lx + self.ly)],
            [1, -1,  (self.lx + self.ly)]
        ])
        
        w = np.array([fl, fr, bl, br])
        
        pseudo_inv = np.linalg.pinv(inv_kin_mat)
        vxytheta = np.matmul(pseudo_inv, w) * self.wheel_radius_m

        return vxytheta

    def quaternion_from_euler_0_0(self, theta):
        cy = np.cos(theta * 0.5)
        sy = np.sin(theta * 0.5)

        q = np.array([0.0, 0.0, sy, cy])

        return q

    def publish_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        quat = self.quaternion_from_euler_0_0(self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = self.vtheta

        self.odometry_publisher.publish(odom_msg)

        self.odom_base_link_tf.transform.translation.x = self.x
        self.odom_base_link_tf.transform.translation.y = self.y

        self.odom_base_link_tf.transform.rotation.x = quat[0]
        self.odom_base_link_tf.transform.rotation.y = quat[1]
        self.odom_base_link_tf.transform.rotation.z = quat[2]
        self.odom_base_link_tf.transform.rotation.w = quat[3]

        self.odom_base_link_tf.header.stamp = self.get_clock().now().to_msg()
        self.tf_static_broadcaster.sendTransform(self.odom_base_link_tf)

        asdf = TransformStamped()
        asdf.header.frame_id = 'map'
        asdf.child_frame_id = 'odom'
        asdf.header.stamp = self.get_clock().now().to_msg()
        asdf.transform.translation.x = 5.0
        asdf.transform.translation.y = -3.5



def main(args=None):
    rclpy.init(args=args)
    mecanum_state_estimator = MecanumStateEstimator()
    rclpy.spin(mecanum_state_estimator)
    mecanum_state_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
