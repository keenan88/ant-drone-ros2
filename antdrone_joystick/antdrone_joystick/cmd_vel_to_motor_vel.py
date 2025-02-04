import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Forward/Inverse kinematics for mecanum wheeled robot: https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf

class MecanumRobotController(Node):
    def __init__(self):
        super().__init__('mecanum_robot_controller')

        self.wheel_radius_m = 0.0762
        self.lx = 0.193
        self.ly = 0.258

        # Subscriber for cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.joint_state_publisher = self.create_publisher(
            JointState,
            'wheel_joint_cmds',
            qos_best_effort # Set QOS to best effort since dropping some commanded velocities is ok and we dont want to flood uros with re-sent messages
        )




    def cmd_vel_callback(self, msg):
        deadband_vx = 0.01
        vx = msg.linear.x

        if vx > -deadband_vx and vx < 0:
            vx = -deadband_vx
        elif vx > 0 and vx < deadband_vx:
            vx = deadband_vx

        vy = msg.linear.y
        deadband_vy = 0.05

        if vy > -deadband_vy and vy < 0:
            vy = -deadband_vy
        elif vy > 0 and vy < deadband_vy:
            vy = deadband_vy

        deadband_wz = 0.02
        wz = msg.angular.z

        if wz > -deadband_wz and wz < 0:
            wz = -deadband_wz
        elif wz > 0 and wz < deadband_wz:
            wz = deadband_wz

        body_vels = np.matrix(
            [
                [vx],
                [vy],
                [wz]
            ]
        )        

        forward_kin_mat = np.matrix(
            [
                [1, -1, -(self.lx + self.ly)],
                [1,  1,  (self.lx + self.ly)],
                [1,  1, -(self.lx + self.ly)],
                [1, -1,  (self.lx + self.ly)]
            ]
        ) / self.wheel_radius_m

        wheel_rad_vels = np.matmul(forward_kin_mat, body_vels)
    
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.velocity = [
             wheel_rad_vels[0, 0],
            -wheel_rad_vels[1, 0],
             wheel_rad_vels[2, 0],
            -wheel_rad_vels[3, 0],
        ]

        self.joint_state_publisher.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    mecanum_robot_controller = MecanumRobotController()
    
    rclpy.spin(mecanum_robot_controller)

    # Cleanup
    mecanum_robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
