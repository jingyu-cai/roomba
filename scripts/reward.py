#!/usr/bin/env python3

import rospy
import os


from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from roomba.msg import QLearningReward
from roomba.msg import RobotAction
import numpy as np

from std_msgs.msg import Header

from random import shuffle
from tf.transformations import quaternion_from_euler, euler_from_quaternion

path_prefix = os.path.dirname(__file__) + "/distances/"
class Reward(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('virtual_reset_world_q_learning')

        # reward amounts
        self.dumbbell_reward = 100
        self.ball_reward = 50
        self.negative_reward = -100

        self.non_obstacles_inplace = 0

        #this will change with the map
        self.num_nonobstacles = 2
        self.robot_location = 4

        #need to change this based on graph
        self.bin_nodes = {'red' : 6, 'blue' : 0, 'green' : -1}

        self.distance = np.load(path_prefix + "distances.npy")


        # keep track of the iteration number
        self.iteration_num = 0

        # ROS subscribe to the topic publishing actions for the robot to take
        rospy.Subscriber("/roomba/robot_action", RobotAction, self.send_reward)

        # ROS publishers
        self.reward_pub = rospy.Publisher("/roomba/reward", QLearningReward, queue_size=10)

        self.run()

    def send_reward(self, data):
        print(data)
        obj = data.object
        color = data.color
        node = data.node
        reset_world = False

        #set reward
        if obj == 'dumbbell':
            reward_amount = self.dumbbell_reward
            self.non_obstacles_inplace += 1
        
        elif obj == 'ball':
            reward_amount = self.ball_reward
            self.non_obstacles_inplace += 1

        elif obj == 'obstacle':
            reward_amount = self.negative_reward
        
        else:
            raise Exception("Wrong object type")

        #adjust with respect to distance
        reward_amount -= self.distance[node, self.bin_nodes[color]] + self.distance[node, self.robot_location]
        self.robot_location = self.bin_nodes[color]

        
        if self.non_obstacles_inplace == self.num_nonobstacles:
            reset_world = True


        # prepare reward msg
        reward_msg = QLearningReward()
        reward_msg.header = Header(stamp=rospy.Time.now())
        reward_msg.reward = reward_amount
        reward_msg.iteration_num = self.iteration_num
        reward_msg.reset_world = reset_world
        reward_msg.robot_location = self.robot_location
        self.reward_pub.publish(reward_msg)
        print("Published reward: ", reward_amount)

        # increment iteration if world needs to be reset
        #reset obj positions 
        if reset_world:
            print("reseting the world")
            self.iteration_num += 1
            self.non_obstacles_inplace = 0



    def run(self):
        rospy.spin()

if __name__=="__main__":

    node = Reward()
