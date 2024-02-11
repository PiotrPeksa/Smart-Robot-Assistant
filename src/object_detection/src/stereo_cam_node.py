import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')
        self.pub_left = self.create_publisher(Image, 'stereo/left/image_raw', 10)
        self.pub_right = self.create_publisher(Image, 'stereo/right/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.capture_and_publish)  # 10 Hz
        self.initialize_camera()

    def initialize_camera(self):
        global left_camera, right_camera
        left_camera = cv2.VideoCapture(0) #left camera
        right_camera = cv2.VideoCapture(1) #right camera
    
    def capture_and_publish(self):
        global left_camera, right_camera
        ret_left, left_image = left_camera.read()
        ret_right, right_image = right_camera.read()

        if not ret_left or not ret_right:
            self.get_logger().warn("Failed to capture stereo images.")
            return

        # Convert OpenCV images to ROS Image messages
        ros_left_image = self.bridge.cv2_to_imgmsg(left_image, "bgr8")
        ros_right_image = self.bridge.cv2_to_imgmsg(right_image, "bgr8")

        # Publish the images
        self.pub_left.publish(ros_left_image)
        self.pub_right.publish(ros_right_image)

def main(args=None):
    rclpy.init(args=args)
    stereo_camera_node = StereoCameraNode()
    rclpy.spin(stereo_camera_node)
    stereo_camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


