#!/usr/bin/env python3
import math
import threading
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, LaserScan, NavSatFix
from std_msgs.msg import Float32, Int32, Float32MultiArray


class ControlNode(Node):
    def __init__(self):
        print('Create ControlNode')
        super().__init__('control_node')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
    
        # Subscribers
        self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        self.create_subscription(LaserScan, 'scan', self.lidar_cb, 10)
        
        # Timer
        self.create_timer(0.1, self.timer_cb)
        
        # Variables
        self.is_obstacle = False
        self.minRange = 0.0
        self.minAngle = 0.0
        self.turning_counter = 0
        
        self.angle_counter = -1
        self.step_counter = 0
        self.is_turning = False
        self.auto_mode = True
        self.publish_twist(0.7, 0.0)
        
    def joy_cb(self, msg: Joy):
        if (self.auto_mode):
            return
        if msg.buttons[0]:  # X (PS4)
            self.auto_mode = True
            self.publish_twist(0.7, 0.0)
    
    def lidar_cb(self, msg: LaserScan):
        if (self.is_turning):
            return
        
        dist = []
        angle = -30.0
        
        for i in range(len(msg.ranges)):
            angle = angle + i
            dist.append(self.getRangeAtDegree(msg, angle))
            
        self.minRange, self.minAngle = self.getMinRange(dist)
        if (self.minRange < 0.7):
            self.is_obstacle = True
        else:
            self.is_obstacle = False
            
        if (self.is_obstacle):
            self.publish_twist(0.0, 0.0)
            if (self.step_counter % 2 == 0):
                self.turn_angle(-90.0)
            else:
                self.turn_angle(90.0)
            self.step_counter += 1
            
        
        
    def getRangeAtDegree(self, scan: LaserScan, angle_degrees: float):
        if (angle_degrees > 180.0):
            angle_degrees -= 360.0
        
        angle_radians = angle_degrees * math.pi / 180.0
        center_index = int(- scan.angle_min / scan.angle_increment)
        index = center_index + int(angle_radians / scan.angle_increment)
        
        if (index >= 0 and index < len(scan.ranges)):
            return scan.ranges[index]
        return -1.0
    
    def getMinRange(self, angle_list: list):
        min_range = math.inf
        min_index = 0
        
        for i, range in enumerate(angle_list):
            if (range < min_range and range > 0.1):
                min_range = range
                min_index = i
        
        angle = -30.0 + float(min_index)
        return min_range, angle
    
    def timer_cb(self):
        if self.angle_counter < 0:
            return
        
        if self.angle_counter >= self.target_counter:
            self.angle_counter = -1
            self.is_turning = False
            self.publish_twist(0.7, 0.0)
        
        if (self.step_counter > 10):
            self.auto_mode = False
            self.publish_twist(0.0, 0.0)
            
        self.angle_counter += 1
        
            
    def turn_angle(self, angle):
        angular_sp = 0.314
        if angle < 0.0:
            angular_sp = -0.314
        else:
            angular_sp = 0.314
            
        self.target_counter = int(abs(angle) / 1.8)
            
        self.is_turning = True
        
        self.publish_twist(0.0, angular_sp)
        
    def publish_twist(self, lin: float, ang: float):
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        self.cmd_pub.publish(t)
        

def main():
    rclpy.init()
    rclpy.spin(ControlNode())
    rclpy.shutdown()


