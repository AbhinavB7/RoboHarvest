#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')
        
        # Create a subscriber to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            'camera1/image_raw', 
            self.image_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )
        self.bridge = CvBridge()
        self.image_counter = 0
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.get_logger().info('running') 
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        # Convert the image to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the range of the red color in HSV
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv_image, lower_red, upper_red)
        result_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # Display the original and result images
        cv2.imshow('Original Image', cv_image)
        cv2.imshow('Result Image', result_image)
        cv2.waitKey(1000)  # delay
        
        # Save the images
        original_image_filename = f'original_image_{self.image_counter}.png'
        result_image_filename = f'result_image_{self.image_counter}.png'
        cv2.imwrite(original_image_filename, cv_image)
        cv2.imwrite(result_image_filename, result_image)
        self.get_logger().info(f'Saved images: {original_image_filename}, {result_image_filename}')
        self.image_counter += 1

        
def main(args=None):
    rclpy.init(args=args)
    color_detection_node = ColorDetectionNode()
    rclpy.spin(color_detection_node)
    color_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
