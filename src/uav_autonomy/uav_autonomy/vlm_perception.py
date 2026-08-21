#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class VLMPerception(Node):
    def __init__(self):
        super().__init__('vlm_perception')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.detection_pub = self.create_publisher(String, '/rescue/vlm_detection', 10)
        self.get_logger().info("VLM Perception Node Active.")
        self.frame_count = 0

    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % 10 != 0: 
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 120, 70])
            upper_red = np.array([10, 255, 255])
            mask = cv2.inRange(hsv, lower_red, upper_red)
            
            if cv2.countNonZero(mask) > 500:
                self.get_logger().info("VLM ALERT: Survivor detected in frame!")
                msg_out = String()
                msg_out.data = "SURVIVOR_FOUND"
                self.detection_pub.publish(msg_out)
        except Exception as e:
            self.get_logger().error(str(e))

def main(args=None):
    rclpy.init(args=args)
    node = VLMPerception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
