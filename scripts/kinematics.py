#!/usr/bin/env python3

import rospy
import numpy as np
import math
import time
import os

import moveit_commander
from geometry_msgs.msg import Twist, Vector3, Pose
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from distances.compute_distances import floyd_warshall
from action_states.generate_action_states import read_objects_and_bins

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__)

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) 
    and returns yaw """

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw

def get_target_angle(p1, p2):
    """ Gets the target angle for the robot to turn to based on robot position (p1)
    and node position (p2) """

    target_angle = 0

    x1 = p1.x
    y1 = p1.y
    x2 = p2[0]
    y2 = p2[1]

    raw_angle = math.atan(abs(x2 - x1) / abs(y2 - y1))

    print("raw", raw_angle)

    if x2 < x1 and y2 > y1:
        target_angle = -1 * (math.radians(90) - raw_angle)
    elif x2 > x1 and y2 > y1:
        target_angle = math.radians(90) - raw_angle
    elif x2 > x1 and y2 < y1:
        target_angle = raw_angle - math.radians(90)
    elif x2 < x1 and y2 < y1:
        target_angle = -1 * (math.radians(180) - raw_angle)

    return target_angle

class RobotMovement(object):
    def __init__(self):

        rospy.init_node('turtlebot3_movement')

        # init LIDAR
        rospy.Subscriber("scan", LaserScan, self.update_distance)
        rospy.Subscriber('camera/rgb/image_raw', Image, self.update_image)
        # init odom
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        # init arm
        self.move_arm  = moveit_commander.MoveGroupCommander("arm")
        self.move_grip = moveit_commander.MoveGroupCommander("gripper")
        # init movement
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-5 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][1] = 5
        self.action_matrix = np.loadtxt(path_prefix + "/action_states/" + "action_matrix.csv", delimiter = ',')

        # Fetch actions. These are the only 6 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: "dumbbell", color: "red", node: 1}
        self.actions = np.genfromtxt(path_prefix + "/action_states/" + "objects.csv", dtype = 'str', delimiter = ',')
        self.actions = list(map(
            lambda x: {"object": str(x[0]), "color": str(x[1]), "node": int(x[2])},
            self.actions
        ))

        # Fetch bins. There are 2 bins, each with a different color and at a different node
        objects, self.bins = read_objects_and_bins(path_prefix + "/action_states/")

        # set up graph-related variables
        self.origin_state = 256
        self.origin_node = 4

        self.num_nonobstacles = 0
        for obj in objects:
            if obj[0] != "obstacle":
                self.num_nonobstacles += 1

        # Set up a list to store the trained Q-matrix
        self.q_matrix = []
        self.load_q_matrix()

        self.locations = []
        self.load_locations()

        self.curr_pose = Pose()

        # Get the array of shortest paths from node to node
        weight_mat = np.genfromtxt(path_prefix + "/distances/" + "map1_matrix.csv", delimiter=',')
        n = weight_mat.shape[0]
        for i in range(weight_mat.shape[0]):
            for j in range(weight_mat.shape[1]):
                if weight_mat[i,j] == 0:
                    weight_mat[i,j] = 1e9
        distance, self.shortest_paths = floyd_warshall(n, weight_mat)

        # set up action sequence
        self.action_sequence = []
        self.get_action_sequence()

        # set up the node sequence
        self.node_sequence = []
        self.get_node_sequence()

    def load_locations(self):
        """ Loads locations of each node"""
        self.locations = np.loadtxt(path_prefix + "/locations.csv", delimiter = ',')

    def load_q_matrix(self):
        """ Load the trained Q-matrix csv file """

        # Store the file into self.q_matrix
        self.q_matrix = np.loadtxt(path_prefix + "/q_matrix.csv", delimiter = ',')

    def get_action_sequence(self):
        """ Get the sequence of actions for the robot to move the dumbbells
        to the correct blocks based on the trained Q-matrix """
        
        # Start at the origin
        curr_state = self.origin_state

        # Loop through a specific amount of times to get the action sequence
        for i in range(self.num_nonobstacles):

            # Get row in matrix and select the best action to take with the
            #   maximum Q-value
            q_matrix_row = self.q_matrix[curr_state]
            selected_action = np.where(q_matrix_row == max(q_matrix_row))[0][0]

            # Store the object, color, and node for the action as a tuple
            obj = self.actions[selected_action]["object"]
            clr = self.actions[selected_action]["color"]
            node = self.actions[selected_action]["node"]
            self.action_sequence.append((obj, clr, node))

            # Update the current state
            curr_state = np.where(self.action_matrix[curr_state] == selected_action)[0][0]
                
        print(self.action_sequence)

    def get_node_sequence(self):
        """ Get the sequence of nodes for each action from the action sequence
        with the help of shortest_paths.txt, e.g. the first index is the sequences
        [[4, 1], [1, 4, 5, 6]], meaning the robot has to go from node 4 -> 1 to
        pick up the red dumbbell, and go from node 1 -> 4 -> 5 -> 6 to place the
        red dumbbell at the red bin  """

        # First we start at the origin node
        curr_node = self.origin_node

        for i in range(len(self.action_sequence)):
            # Define an array to hold the pick up object + place object node seqs
            one_sequence = []

            # Find the node of interest and append that shortest path from curr_node
            next_node = self.action_sequence[i][2]
            one_sequence.append(self.shortest_paths[curr_node][next_node])

            # Find where the object belong to and append that shortest path from next_node
            bin_node = self.bins[self.action_sequence[i][1]]
            one_sequence.append(self.shortest_paths[next_node][bin_node])

            # Append the sequence to node sequence and update curr_node to bin_node
            self.node_sequence.append(one_sequence)
            curr_node = bin_node
        
        print(self.node_sequence)

    def odom_callback(self, data):
        """ Saves the odometry data """
        self.curr_pose = data.pose.pose
        '''
        kp = 0.5
        target_angle = get_target_angle(data.pose.pose.position, (1.4, 1.4))
        self.twist.linear.x = 0
        self.twist.angular.z = kp * (target_angle - get_yaw_from_pose(data.pose.pose))
        self.cmd_vel_pub.publish(self.twist)
        '''

    def orient(self, p):
        """ given a node number, orients robot to face that node"""
        kp = 0.2
        point = self.locations[p]
        target_angle = get_target_angle(self.curr_pose.position, (point[0], point[1]))
        #print("pose:", self.curr_pose)
        print("target ang", target_angle)
        self.twist.linear.x = 0
        self.twist.angular.z = kp * (target_angle - get_yaw_from_pose(self.curr_pose))
        print(kp * (target_angle - get_yaw_from_pose(self.curr_pose)))
        self.cmd_vel_pub.publish(self.twist)

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
        #rospy.spin()

        #self.orient(1)
        
        r = rospy.Rate(1)
        
        while not rospy.is_shutdown():
            self.orient(1)
            r.sleep()
        '''
        while True:
            # self.open_grip()
            self.open_grip()
            while True:
                self.pick_up()
                time.sleep(3)
                self.let_go()
            # self.move_fwd()
        '''

if __name__ == "__main__":
    node = RobotMovement()
    node.run()

