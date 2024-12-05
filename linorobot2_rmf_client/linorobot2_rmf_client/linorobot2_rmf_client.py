#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rmf_fleet_msgs.msg import RobotState, PathRequest
from geometry_msgs.msg import PoseStamped, TransformStamped
from ant_fleet_interfaces.srv import SuspendRMFPathing, CheckDroneIdle
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener

import os, math
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

        # TODO - add a bridge to bridge in and out robot_state and robot_path_requests from queen DOMAIN ID to individual robot DOMAIN ID

        self.robot_name = self.get_namespace()[1:]  # Remove leading forward slash from robot name
        self.fleet_name = os.getenv('FLEET_NAME')

        # RMF fleet adapter sends paths to robots over /robot_path_requests topic
        self.rmf_path_request_subscription = self.create_subscription(
            PathRequest,
            '/robot_path_requests',
            self.execute_path_request,
            10
        )

        # drone_rmf_state_publisher gives feedback to RMF fleet adapter as drone executes paths from RMF fleet adapter
        rmf_fleet_adapter_robot_state_topic_name = '/robot_state'
        self.drone_rmf_state_publisher = self.create_publisher(RobotState, rmf_fleet_adapter_robot_state_topic_name, 10)
        self.state_timer = self.create_timer(0.1, self.publish_state)
        self.drone_rmf_state = RobotState()
        self.drone_rmf_state.battery_percent = 100.0 # TODO - implement hardware checking for battery percent
        self.drone_rmf_state.location.level_name = "L1" # TODO - implement level updating later, if need be
        self.drone_rmf_state.location.index = 0
        self.drone_rmf_state.name = self.robot_name
        self.drone_rmf_state.mode.mode = MODE_IDLE
        self.drone_rmf_state.seq = 0

        # We need a seperate publisher to inform the drone's behavior tree when the robot is idle after the previous task. drone_rmf_state needs to be set to IDLE to get 
        # the next waypoint in a string of waypoints, so a seperate state that is only set to IDLE once all movement is completed is necessary.
        self.drone_queen_state = RobotState()
        self.drone_queen_state.mode.mode = MODE_IDLE
        self.last_movement_end_time_s = self.get_clock().now().to_msg().sec
        self.idle_timeout_s = 3 # If RMF doesnt send a new waypoint within timeout period, inform drone behavior tree that drone is free for new tasks
        self.check_drone_idle_srv = self.create_service(CheckDroneIdle, 'check_drone_idle', self.check_drone_idle_cb)
        
        # Robot position is needed for drone_rmf_state
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.25, self.get_transform)
        self.first_tf_set = False

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        while not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().info('Waiting for navigate_to_pose action server to become available...')
        sleep(3) # Navigate to pose server in unknown state for a few seconds after it becomes available

        # RMF pathing needs to be suspended while robot does pickup and dropoff routines to avoid conflicts in desired robot position
        self.suspend_rmf_pathing_srv = self.create_service(SuspendRMFPathing, 'suspend_rmf_pathing', self.suspend_rmf_pathing_cb)
        self.is_rmf_pathing_suspended = False

        self.get_logger().info(f"{self.robot_name}_rmf_client started")
    
    def check_drone_idle_cb(self, request, response):

        response.is_drone_idle = self.drone_queen_state.mode.mode == MODE_IDLE

        return response

    def suspend_rmf_pathing_cb(self, request, response):

        self.is_rmf_pathing_suspended = request.is_rmf_pathing_suspended

        self.get_logger().info(f"self.is_rmf_pathing_suspended: {self.is_rmf_pathing_suspended}")

        response.success = True

        return response
        
    def get_transform(self):
        try:
            now = rclpy.time.Time()
            timeout = rclpy.duration.Duration(seconds=0.1)
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'map', self.robot_name + '_base_link', now, timeout
            )

            x = transform.transform.rotation.x
            y = transform.transform.rotation.y
            z = transform.transform.rotation.z
            w = transform.transform.rotation.w

            self.drone_rmf_state.location.x = transform.transform.translation.x
            self.drone_rmf_state.location.y = transform.transform.translation.y
            self.drone_rmf_state.location.yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            self.drone_rmf_state.location.t = transform.header.stamp

            # self.drone_queen_state.location = self.drone_rmf_state.location
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
            self.drone_rmf_state.mode.mode = MODE_IDLE # Have to set mode to IDLE to recieve new path requests
            self.drone_rmf_state.path = []
            self.drone_rmf_state.task_id = '' # Empty task ID to tell RMF fleet adapter that task was not completed
            self.last_movement_end_time_s = self.get_clock().now().to_msg().sec
            self.get_logger().info("Navigation not accepted")
            return

        self.get_logger().info("Navigation accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.nav_to_pose_result_cb)

    def nav_to_pose_result_cb(self, future):
        nav_movement_succeeded = future.result().result

        if nav_movement_succeeded:
            self.drone_rmf_state.mode.mode = MODE_IDLE # Have to set mode to IDLE to recieve new path requests
            self.drone_rmf_state.path = []
            self.get_logger().info("Navigation succeeded")
        else:
            self.drone_rmf_state.task_id = '' # Empty task ID to tell RMF fleet adapter that task was not completed
            self.get_logger().info("Navigation failed")

        self.last_movement_end_time_s = self.get_clock().now().to_msg().sec
        

    def execute_path_request(self, path_request):

        if path_request.fleet_name == self.fleet_name and path_request.robot_name == self.robot_name: # Only execute paths intended for this drone

            if not self.is_rmf_pathing_suspended: # Only execute paths if rmf pathing is not suspended

                # Do not interrupt task underway. This is NECESSARY since RMF may try sending path requests before the previous path request finished.
                if self.drone_rmf_state.task_id != path_request.task_id and self.drone_rmf_state.mode.mode == MODE_IDLE: 

                    self.get_logger().info(f"Accepted path request #{path_request.task_id} x: {path_request.path[0].x} -> {path_request.path[1].x}, y: {path_request.path[0].y} -> {path_request.path[1].y}")

                    self.drone_rmf_state.mode.mode = MODE_MOVING
                    self.drone_queen_state.mode.mode = MODE_MOVING
                            
                    self.drone_rmf_state.path = [path_request.path[1]]
                    self.drone_rmf_state.task_id = path_request.task_id

                    waypoint = path_request.path[1]

                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.header.stamp = waypoint.t

                    pose.pose.position.x = waypoint.x
                    pose.pose.position.y = waypoint.y
                    pose.pose.orientation.x = 0.0
                    pose.pose.orientation.y = 0.0
                    pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
                    pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)

                    self.send_nav_to_pose_goal(pose)

                else:

                    self.get_logger().info(f"Ignored path request #{path_request.task_id} x: {path_request.path[0].x} -> {path_request.path[1].x}, y: {path_request.path[0].y} -> {path_request.path[1].y}")

    def publish_state(self):

        if self.first_tf_set:

            self.drone_rmf_state.seq += 1

            self.drone_rmf_state_publisher.publish(self.drone_rmf_state)

            if self.drone_rmf_state.mode.mode == MODE_IDLE:

                if self.get_clock().now().to_msg().sec >= self.last_movement_end_time_s + self.idle_timeout_s:

                    self.drone_queen_state.mode.mode = MODE_IDLE                
    

def main(args=None):
    rclpy.init(args=args)
    linorobot2_rmf = Linorobot2RMF()
    rclpy.spin(linorobot2_rmf)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
