import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv2
import os
from cv_bridge import CvBridge
import time


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        
        # Parameters
        self.declare_parameter('image_folder', '/home/humble_ws/src/antdrone_bringup/calibrationdata')
        self.declare_parameter('topic_name', '/apriltag_cam/apriltag_cam/color/image_raw')
        
        # Load parameters
        self.image_folder = self.get_parameter('image_folder').value
        self.topic_name = self.get_parameter('topic_name').value
        
        # Create a publisher
        self.publisher_ = self.create_publisher(Image, self.topic_name, 10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Get the list of PNG images
        self.image_files = sorted([
            os.path.join(self.image_folder, f) 
            for f in os.listdir(self.image_folder) 
            if f.endswith('.png')
        ])
        
        if not self.image_files:
            self.get_logger().error(f"No PNG images found in {self.image_folder}. Exiting.")
            rclpy.shutdown()
            return
        
        self.get_logger().info(f"Found {len(self.image_files)} PNG images in {self.image_folder}.")
        
        # Initialize index for images
        self.current_index = 0
        
        # Timer to publish images every second
        self.timer = self.create_timer(0.25, self.publish_image)

    def publish_image(self):
        # Read the current image
        if self.current_index < len(self.image_files):
            image_path = self.image_files[self.current_index]
            cv_image = cv2.imread(image_path, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                self.get_logger().error(f"Failed to read image: {image_path}")
                return
            
            # Convert to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            
            # Add a timestamp
            ros_image.header = Header()
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_frame'
            
            # Publish the image
            self.publisher_.publish(ros_image)
            self.get_logger().info(f"Published image: {image_path}")
            
            # Update to the next image
            self.current_index = (self.current_index + 1)


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
