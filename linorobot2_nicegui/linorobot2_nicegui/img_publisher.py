import base64
import json
import rospy  # Assuming you're using ROS2 with Python
from std_msgs.msg import String  # Import the appropriate message type
import time

def image_to_json(image_path):
    # Step 1: Read the image file
    with open(image_path, 'rb') as img_file:
        image_bytes = img_file.read()

    # Step 2: Convert bytes to Base64
    image_base64 = base64.b64encode(image_bytes).decode('utf-8')

    # Step 3: Create a JSON object
    json_data = json.dumps({
        'op': 'png',
        'data': image_base64
    })

    return json_data

def publish_image_as_json(image_path):
    # Initialize the ROS2 node
    rospy.init_node('image_publisher', anonymous=True)
    pub = rospy.Publisher('image_topic', String, queue_size=10)

    # Convert image to JSON
    json_data = image_to_json(image_path)

    # Step 4: Publish the JSON data
    rospy.loginfo("Publishing JSON data...")
    while 1:

        pub.publish(json_data)
        time.sleep(1)

    rospy.spin()  # Keep the node running

if __name__ == '__main__':
    image_path = './asdf.png'  # Update this to your image path
    publish_image_as_json(image_path)
