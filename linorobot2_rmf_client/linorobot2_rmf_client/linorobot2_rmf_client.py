#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rmf_fleet_msgs.msg import RobotState, PathRequest
from geometry_msgs.msg import PoseStamped
import math
from tf2_ros import Buffer, TransformListener
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from geometry_msgs.msg import TransformStamped
import os
import numpy as np
from time import sleep


# Intended to run on each individual robot

MODE_IDLE=0
MODE_CHARGING=1
MODE_MOVING=2
MODE_PAUSED=3
MODE_WAITING=4
MODE_EMERGENCY=5
MODE_GOING_HOME=6
MODE_DOCKING=7
MODE_ADAPTER_ERROR=8
MODE_CLEANING=9

class Linorobot2RMF(Node):

    def __init__(self):
        super().__init__('linorobot2_rmf_client')

        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.robot_name = self.get_namespace()[1:]  # Remove leading forward slash from robot name

        # TODO - change robot naming to read from a yaml file
        self.fleet_name = os.getenv('FLEET_NAME')

        self.robot_state = RobotState()
        self.robot_state.battery_percent = 100.0 # TODO - implement hardware checking for battery percent
        self.robot_state.location.level_name = "L1" # TODO - implement level updating later, if need be
        self.robot_state.location.index = 0
        self.robot_state.name = self.robot_name
        self.robot_state.mode.mode = MODE_IDLE
        self.robot_state.seq = 0

        # TODO - add a bridge to bridge in and out robot_state and robot_path_requests from queen DOMAIN ID to individual robot DOMAIN ID

        self.rmf_robot_state_publisher = self.create_publisher(RobotState, '/robot_state', 10)
        
        self.rmf_path_request_subscription = self.create_subscription(
            PathRequest,
            '/robot_path_requests',
            self.execute_path_request,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.5, self.get_transform)
        self.first_tf_set = False

        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        while not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().info('Waiting for navigate_to_pose action server to become available...')
        sleep(3) # Navigate to pose server in unknown state for a few seconds after it becomes available

        self.state_timer = self.create_timer(1.0, self.publish_state)

        self.row_L_xy_poses = np.array([
            [26.049999999999997, -14.299999999999999],
            [24.65, -14.299999999999999],
            [23.2, -14.299999999999999],
        ])

        self.get_logger().info(f"{self.robot_name}_rmf_client started")
        
    def get_transform(self):
        try:
            now = rclpy.time.Time()
            timeout = rclpy.duration.Duration(seconds=1.0)
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'map', self.robot_name + '_base_link', now, timeout
            )

            x = transform.transform.rotation.x
            y = transform.transform.rotation.y
            z = transform.transform.rotation.z
            w = transform.transform.rotation.w

            self.robot_state.location.x = transform.transform.translation.x
            self.robot_state.location.y = transform.transform.translation.y
            self.robot_state.location.yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            self.robot_state.location.t = transform.header.stamp
            self.first_tf_set = True

        except Exception as e:
            self.get_logger().info(f"Could not get transform: {e}")

    def send_nav_to_pose_goal(self, pose):
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.nav_to_pose_response_cb)

    def nav_to_pose_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.robot_state.mode.mode = MODE_IDLE # Have to set mode to IDLE to recieve new path requests
            self.robot_state.path = []
            self.robot_state.task_id = '' # Empty task ID to tell RMF fleet adapter that task was not completed
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.nav_to_pose_result_cb)

    def nav_to_pose_result_cb(self, future):
        nav_movement_succeeded = future.result().result

        if nav_movement_succeeded:
            self.robot_state.mode.mode = MODE_IDLE # Have to set mode to IDLE to recieve new path requests
            self.robot_state.path = []
        else:
            self.robot_state.task_id = '' # Empty task ID to tell RMF fleet adapter that task was not completed
        
        
        self.get_logger().info("nav_to_pose_result_cb")

    def execute_path_request(self, path_request):
        if path_request.fleet_name == self.fleet_name and path_request.robot_name == self.robot_name:

            if self.robot_state.task_id != path_request.task_id: # Do not interrupt task underway

                self.robot_state.mode.mode = MODE_MOVING
                        
                self.robot_state.path = [path_request.path[1]]
                self.robot_state.task_id = path_request.task_id

                waypoint = path_request.path[1]
                for row_pose in self.row_L_xy_poses:
                    dist = np.linalg.norm(np.array([waypoint.x, waypoint.y]) - row_pose)
                    if dist < 0.05:
                        # self.get_logger().info(f"Row L pose: {row_pose} straightened")
                        waypoint.yaw = 0.0
                        break

                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = waypoint.t

                pose.pose.position.x = waypoint.x
                pose.pose.position.y = waypoint.y
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
                pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)

                self.get_logger().info(f"goal pose: {pose}")

                self.send_nav_to_pose_goal(pose)

    def publish_state(self):

        if self.first_tf_set:

            self.robot_state.seq += 1

            self.rmf_robot_state_publisher.publish(self.robot_state)
    

def main(args=None):
    rclpy.init(args=args)
    linorobot2_rmf = Linorobot2RMF()
    rclpy.spin(linorobot2_rmf)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
