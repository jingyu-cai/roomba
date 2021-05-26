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
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
        rospy.init_node('turtlebot3_movement')

    # GRANULAR GRIP MOVEMENT FUNCS
    def open_grip(self):
        open_grip = [0.016, 0.016]
        self.move_grip.go(open_grip)
    def close_grip(self):
        close_grip = [0.010, 0.010]
        self.move_grip.go(close_grip)

    # GRANULAR ARM MOVEMENT FUNCS
    def lower_arm(self):
        lower_pos = [0, 0.7, 0, -0.65]
        self.move_arm.go(lower_pos, wait=True)
    def upper_arm(self):
        upper_pos = [0, 0.05, -0.5, -0.65]
        self.move_arm.go(upper_pos, wait=True)
    def angle_arm(self):
        angle_pos = [0.75, 0.05, -0.5, -0.65]
        self.move_arm.go(angle_pos, wait=True)

    # GRANULAR TWIST MOVEMENT FUNCS
    def stop(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
    def move_forward(self, secs=3):
        self.twist.linear.x = 0.1
        self.cmd_vel_pub.publish(self.twist)
        time.sleep(secs)
        self.stop()
    def move_back(self, secs=3):
        self.twist.linear.x = -0.1
        self.cmd_vel_pub.publish(self.twist)
        time.sleep(secs)
        self.stop()

    # COMPOUND MOVEMENTS
    def pick_up(self):
        """ Picks up dumbbell and angles out of camera POV
        """
        self.lower_arm()
        self.close_grip()
        self.upper_arm()
        self.angle_arm()
    def let_go(self):
        """ Loosens grip and backs away from dumbbell.
        """
        self.upper_arm()
        self.lower_arm()
        self.open_grip()
        self.move_back()

    # STATE UPDATE FUNCS
    def update_distance(self, data):
        self.distance = data.ranges[0]
    def update_image(self, data):
        self.image = data

    def run(self):
        while True:
            # self.open_grip()
            self.open_grip()
            while True:
                self.pick_up()
                time.sleep(3)
                self.let_go()
            # self.move_fwd()

        rospy.spin()

if __name__ == "__main__":
    node = RobotMovement()
    node.run()

