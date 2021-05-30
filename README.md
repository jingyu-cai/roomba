# roomba

## Team Members
Karhan Kayan, Jingyu Cai, Zack Wang, Yoon Jeong

## Project Description

### Goal of the project
Given a fixed graph of yellow lines and marked out locations for objects of interest, the goal of this project is to make the robot take the most optimal path in the graph to pick up an object and drop it off in its corresponding bin to maximize reward, which takes into account the type of the object as well as the distance it took for the robot to complete the action. In particular, we will use the Q-learning algorithm to train the action sequence and manipulate the kinematics of the robot along with computer vision to execute the actions.

### Why it's interesting?
We think that this kind of robot would be helpful in the real world - picking and sorting clothes and trash in someone's room, or even picking up garbage in a less-controlled environment. Additionally, this is a good synthesis of all the mechanics we've learned in the class, including computer vision, kinematics, and a reward-based algorithm.

### What were you able to make the robot do?
TODO: include gif/screenshots

### What are the main components of your project and how they fit together?
The three main components of the project are 1) developing a fixed map with known distances between nodes and locations of objects, 2) training a Q-matrix with the Q-learning algorithm on the map, and 3) executing the most optimal actions based on robot kinematics and computer vision. First, given the map and the states of the robot and the objects, we were able to create an action space and a state space to train the Q-matrix with. For reward, we took into account the type of object the robot picks up (dumbbell has the highest reward, kettlebell second, and you can't pick up a cubic obstacle) and the time it took for the robot to complete the pick up-drop off action. With the converged Q-matrix, we selected the most optimal action from each given state the robot is in to create an action sequence for the robot to execute so to maximize the reward. The robot will use computer vision to follow the yellow lines to traverse the map and use its arm to pick up and drop off objects at a bin corresponding to the color of the object. To add complexity to the project, we also used computer vision to identify the type of object the robot is going to pick up without previous knowledge of what object is at that node (in other words, the robot only knows the  path sequence but doesn't know what arm action to take to handle the object before the robot identifies it).

## System Architecture

### Describe in detail the robotics algorithm you implemented

### Describe each major component of your project

### Highlight what pieces of code contribute to these main components

## Challenges, Future Work, and Takeaways

### Challenges
TODO: 1 paragraph

### Future work
TODO: 1 paragraph

### Takeaways
TODO: A few bullet points
