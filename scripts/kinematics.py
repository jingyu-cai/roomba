#!/usr/bin/env python3

import rospy
import numpy as np
import math
import time

import moveit_commander
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image, LaserScan

class RobotMovement(object):
    def __init__(self):

        # init LIDAR
        rospy.Subscriber("scan", LaserScan, self.update_distance)
        rospy.Subscriber('camera/rgb/image_raw', Image, self.update_image)
        # init arm
        self.move_arm  = moveit_commander.MoveGroupCommander("arm")
        self.move_grip = moveit_commander.MoveGroupCommander("gripper")
        # init movement
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
        rospy.init_node('turtlebot3_movement')

    def lower_arm(self):
        lower_pos = [0, .85, -.3, -.35]

        self.move_arm.go(lower_pos, wait=True)
        self.move_arm.stop()
    def raise_arm(self):
        arm = {
            "upper": [0.3, 0.5, -0.3, -0.5],
            "upper_diag": [0.65, 0.25, -0.3, -0.5],
            "side": [1.5, 0.25, -0.3, -0.5],
        }

        self.move_arm.go(arm["upper"], wait=True)
        self.move_arm.stop()
        self.move_arm.go(arm["upper_diag"], wait=True)
        self.move_arm.stop()
        self.move_arm.go(arm["side"], wait=True)
        self.move_arm.stop()

    def open_grip(self):
        open_grip = [0.016, 0.016]
        self.move_grip.go(open_grip)

    def update_distance(self, data):
        self.distance = data.ranges[0]
    def update_image(self, data):
        self.image    = data

    def run(self):
        self.open_grip()

        while True:
            self.lower_arm()
            time.sleep(2)
            self.raise_arm()
        rospy.spin()

if __name__ == "__main__":
    node = RobotMovement()
    node.run()

