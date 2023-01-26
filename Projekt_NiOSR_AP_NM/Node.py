#!/usr/bin/env python3
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # ROS2 package to convert between ROS and OpenCV Images
import cv2 # Python OpenCV library
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist

global dir 
dir = 0.0

class MinimalPublisher(Node):
    def __init__(self):
        self.dir = 0.0
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    def callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
           if x>350: 
            self.dir = 1.0
           else:
            self.dir = -1.0
            
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.dir
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        img = np.zeros((350,700,3), dtype = np.uint8)
        cv2.imshow("Projekt",img)
        cv2.setMouseCallback("Projekt",self.callback)
        #self.get_logger().info(dir)
        self.i += 1
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
