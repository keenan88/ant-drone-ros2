
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ant_fleet_interfaces.srv import RequestWorkerPickup
from linkattacher_msgs.srv import AttachLink, DetachLink
from ant_fleet_interfaces.msg import WorkerPickupState
from std_msgs.msg import String, Bool
from rclpy.executors import MultiThreadedExecutor
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy




class PickupCoordinator(Node):

    def __init__(self):
        super().__init__('pickup_coordinator')

        self.worker_name = self.get_namespace()[1:]  # Remove leading forward slash from robot name

        qos_reliable = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability = QoSDurabilityPolicy.VOLATILE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 10
        )

        self.suspend_rmf_pathing_pub = self.create_publisher(Bool, 'suspend_rmf_pathing', qos_reliable)

        # self.request_worker_pickup_sub = self.create_subscription(WorkerPickupState, 'request_worker_pickup', self.request_worker_pickup_callback, 10)

        # self.request_worker_pickup_srv = self.create_service()

        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ATTACHLINK service not available, waiting 1s...')


        self.picked_up_drone_worker_combos = []
        self.drone_worker_attachment_pub = self.create_publisher(WorkerPickupState, '/worker_pickup_state', 10)
        self.attachment_pub_timer = self.create_timer(0.1, self.publish_worker_pickup_state)


    def request_worker_pickup_callback(self, msg: String):

        drone_name = msg.data.split('-')[0]
        worker_name = msg.data.split('-')[1]

        if drone_name == self.worker_name:

            suspend_rmf_msg = Bool()
            suspend_rmf_msg.data = True



            attach_request = AttachLink.Request()
            attach_request.model1_name = drone_name
            attach_request.model2_name = worker_name
            attach_request.link1_name = attach_request.model1_name + '_attachment_point'
            attach_request.link2_name = attach_request.model2_name + '_base_link'

            self.get_logger().warn('Attempting attach')

            # Try attaching worker to drone
            try:
                future = self.attach_client.call_async(attach_request)
                
                time.sleep(2.0)

                # Assumed picked up, even if service call times out. /ATTACH gazebo service call always times out, even when pickup successful.
                combined_name = drone_name + '-' + worker_name
                if combined_name not in self.picked_up_drone_worker_combos:
                    self.picked_up_drone_worker_combos.append(combined_name)
                    self.get_logger().info(f"appended combined_name: {combined_name}")

                msg = WorkerPickupState()
                msg.worker_name = worker_name
                msg.drone_name = drone_name
                self.drone_worker_attachment_pub.publish(msg)

                time.sleep(2.0)

                if not future.done():
                    self.get_logger().warn(f'Attach service call timed out')
                else:
                    self.get_logger().warn(f'Attach service call succeeded, future: {future.result()}')
                
            except Exception as e:
                self.get_logger().error(f'Attach service call failed: {str(e)}')

    def publish_worker_pickup_state(self):

        # self.get_logger().info(f"self.picked_up_drone_worker_combos: {self.picked_up_drone_worker_combos}")

        for worker_drone_combo_combnined_name in self.picked_up_drone_worker_combos:

            msg = WorkerPickupState()
            msg.worker_name = worker_drone_combo_combnined_name.split('-')[1]
            msg.drone_name = worker_drone_combo_combnined_name.split('-')[0]

            self.drone_worker_attachment_pub.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    pickup_coordinator = PickupCoordinator()

    executor = MultiThreadedExecutor(num_threads = 2)
    executor.add_node(pickup_coordinator)
    executor.spin()

if __name__ == '__main__':
    main()
