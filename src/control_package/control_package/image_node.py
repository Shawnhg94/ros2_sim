#!/usr/bin/env python3
import math
import threading
from enum import Enum
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, LaserScan, NavSatFix
from std_msgs.msg import Float32, Int32, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# from image_manager import ImageManager

import os
import shutil
import numpy as np
import cv2


W = 400
H = 240

def numerical_sort(filename):
    """Extracts the numerical part of the filename for sorting."""
    #print('filename:', filename)
    name = filename.split('_')[-1]
    return int(name.split('.')[0])

class ImageManager:
    def __init__(self, path: str):
        print('Create ImageManger {}'.format(path))
        self.img_path = path
        self.imgs = list(sorted(os.listdir(path), key= numerical_sort))

    
    def get_num(self):
        return len(self.imgs)

    
    def get_img(self, idx: int):
        img_path = os.path.join(self.img_path, self.imgs[idx])
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE) #Image.open(img_path)
        # if (img.height > H and img.width > W):
        #     img = self.resize_image(img)
        return img



class ImageNode(Node):
    def __init__(self):
        super().__init__('image_node')
        self.publisher_ = self.create_publisher(Image, 'CameraFront', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.033, self.timer_cb)
        self.image_manager =ImageManager('/home/forev/project/ros2_sim/input/')
        self.index = 0
        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        self.started = False

    def timer_cb(self):
        if (not self.started):
            return
        if (self.index >= self.image_manager.get_num()):
            return

        cv_ori_img = self.image_manager.get_img(self.index)
        cv_img = cv2.resize(cv_ori_img, (W, H))

        
        ros_image_message = self.bridge.cv2_to_imgmsg(cv_img, "mono8")
        ros_image_message.header.stamp = self.get_clock().now().to_msg()
        ros_image_message.header.frame_id = "{}".format(self.index) 
        self.index += 1
        self.publisher_.publish(ros_image_message)
    
    def cmd_cb(self, msg):
        self.started = True
        


def main():
    rclpy.init()
    rclpy.spin(ImageNode())
    rclpy.shutdown()
