#!/usr/bin/env python3
import math
import threading
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, LaserScan, NavSatFix
from std_msgs.msg import Float32, Int32, Float32MultiArray
from tf_transformations import euler_from_quaternion

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.mutex = threading.Lock()

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
    
        # Subscribers
        self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        self.create_subscription(LaserScan, 'scan', self.lidar_cb, 10)
        
        
    def getRangeAtDegree(self, scan: LaserScan, angle_degrees: float):
        if (angle_degrees > 180.0):
            angle_degrees -= 360.0
        
        angle_radians = angle_degrees * math.pi / 180.0
        center_index = int(- scan.angle_min / scan.angle_increment)
        index = center_index + int(angle_radians / scan.angle_increment)
        
        if (index >= 0 and index < len(scan.range)):
            return scan.ranges[index]
        return -1.0
    
    def getMinRange(angle_list: list):
        min_range = math.inf
        min_index = 0
        
        for i, range in enumerate(angle_list):
            if (range < min_range and range > 0.1):
                min_range = range
                min_index = i
        
        angle = -30.0 + float(min_index)
        return min_range, angle

def main():
    print('Hi from control_package.')


if __name__ == '__main__':
    main()

