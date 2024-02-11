import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math
from ultralytics import YOLO  
import numpy as np
import glob
from calibrate import disparity_map
from calibrate import cameraMatrixL
from geometry_msgs.msg import Pose,PoseStamped
from std_msgs.msg import String



class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.current_object = None
        self.subscription_left = self.create_subscription(Image, 'stereo/left/image_raw', self.listener_callback_left, 10)
        self.subscription_right = self.create_subscription(Image, 'stereo/right/image_raw', self.listener_callback_right, 10)
        self.bridge = CvBridge()
        self.model = YOLO("/home/ppeksa/rosDetect/src/models/newmodel.pt")  
        self.classNames = ["door", "glass", "door-knob", "door-handle", "door-handle-pull", "refrigerator", "waterbottle"]
        self.confidence_threshold = 0.2
        self.bbox = []
        self.baseline = 60
        self.pose_pub = self.create_publisher(PoseStamped, '/object_pose', 10)
        self.voice_command_subscriber = self.create_subscription(String, '/voice_command', self.voice_command_callback, 10)


    def process_and_visualize(self, cv_image):
        results = self.model(cv_image, stream=True)
        detected_object = None

        for r in results:
            boxes = r.boxes
            for box in boxes:
                confidence = math.ceil((box.conf[0] * 100)) / 100
                if confidence >= self.confidence_threshold:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls = int(box.cls[0])
                    detected_object = self.classNames[cls]

                    if detected_object == self.current_object:
                        self.bbox = map(int, box.xyxy[0])

                        cv2.rectangle(cv_image, (x1, y1), (x2, y2), (255, 0, 255), 3)
                        cv2.putText(cv_image, f"{self.classNames[cls]} {confidence}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        return cv_image

    def listener_callback_left(self, msg):
            if self.activate_object_detection:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                output_image = self.process_and_visualize(cv_image)
                if self.bbox:
                    pose = self.estimate_pose(self.bbox)
                    pose_stamped = PoseStamped()
                    pose_stamped.header.stamp = self.get_clock().now().to_msg()
                    pose_stamped.header.frame_id = "left_camera_frame"
                    pose_stamped.pose = pose
                    self.pose_pub.publish(pose_stamped)
                    self.activate_object_detection = False
                cv2.imshow("Object Detection", output_image)
                cv2.waitKey(1)


        
    

    def listener_callback_right(self, msg):
            if self.activate_object_detection: # If the object detection is activated
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # Convert ROS Image message to OpenCV image
                output_image = self.process_and_visualize(cv_image) # Process and visualize the image
                if self.bbox: # If the bounding box is not empty
                    pose = self.estimate_pose(self.bbox)# Estimate the pose of the object
                    pose_stamped = PoseStamped()# Create a PoseStamped message
                    pose_stamped.header.stamp = self.get_clock().now().to_msg() # Set the timestamp
                    pose_stamped.header.frame_id = "left_camera_frame" # Set the frame ID
                    pose_stamped.pose = pose # Set the pose
                    self.pose_pub.publish(pose_stamped) # Publish the pose
                    self.activate_object_detection = False # Deactivate the object detection
                cv2.imshow("Object Detection", output_image) # Display the image
                cv2.waitKey(1) # 1 ms delay
        
          
    def voice_command_callback(self, msg):
        if msg.data == "pick up a glass":
            self.current_object = "glass"
       
            print("Glass detected! Ready to pick up.")
       
            
        elif msg.data == "open the door":
            self.current_object = "door"
            print("Door detected! Ready to open.")
        else:
            print("Unknown command: %s" % msg.data)


    def estimate_pose(self, bbox):
        x_min, y_min, x_max, y_max = bbox
        bbox_disparity = disparity_map[y_min:y_max, x_min:x_max]
        avg_disparity = np.mean(bbox_disparity[bbox_disparity > 0])
        focal_length = cameraMatrixL[0,2]
        depth = focal_length * self.baseline / avg_disparity

        center_x = (x_min + x_max) / 2
        center_y = (y_min + y_max) / 2
        K = cameraMatrixL
        cx = K[0, 2]
        cy = K[1, 2]
        fx = K[0, 0]
        fy = K[1, 1]
        x_3d = (center_x - cx) * depth / fx
        y_3d = (center_y - cy) * depth / fy
        z_3d = depth
        pose = Pose()
        pose.position.x = x_3d
        pose.position.y = y_3d
        pose.position.z = z_3d

        # Assuming no orientation information
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1

        return pose



def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()
    rclpy.spin(object_detection_node)
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()