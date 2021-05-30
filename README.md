# roomba

## Team Members
Karhan Kayan, Jingyu Cai, Zack Wang, Yoon Jeong

## Project Description

### Goal of the project
Given a fixed graph of yellow lines and marked-out locations for objects of interest, the goal of this project is to make the robot take the most optimal path in the graph to pick up objects and drop them off in their corresponding bins to maximize reward, which takes into account the type of the object as well as the distance it took for the robot to complete an action. In particular, we will use the Q-learning algorithm to generate an action sequence and manipulate the kinematics of the robot along with perception and computer vision to execute the actions.

### Why it's interesting?
We think that this kind of robot would be helpful in the real world - picking and sorting clothes and trash in someone's room, or even picking up garbage in a less-controlled environment. Additionally, this is a good synthesis of all the mechanics we've learned in the class, including perception, kinematics, and a reward-based algorithm.

### What were you able to make the robot do?
TODO: include gif/screenshots

### What are the main components of your project and how they fit together?
The three main components of the project are 1) developing a fixed map with known distances between nodes and locations of objects, 2) training a Q-matrix with the Q-learning algorithm on the map, and 3) executing the most optimal actions based on robot perception and kinematics. First, given the map and the states of the robot and the objects, we were able to create an action space and a state space to train the Q-matrix with. For reward, we took into account the type of object the robot picks up (dumbbell has the highest reward, ball/kettlebell second, and you can't pick up a cubic obstacle) and the distance it took for the robot to complete the pick up-drop off action (so the robot would need to choose between prioritizing high-value objects or shortest paths). With the converged Q-matrix, we selected the most optimal action from each given state the robot is in to create an action sequence for the robot to execute so to maximize the reward. The robot will use perception to follow the yellow lines to traverse the map in a path calculated with the Floyd-Warshall algorithm, and use its arm to pick up and drop off objects at bins with the same colors. To add complexity to the project, we also used computer vision to identify the type of object the robot is going to pick up without previous knowledge of what object is at each node (in other words, the robot only knows what path to take but doesn't know what arm action to take to handle the object before the robot identifies it).

## System Architecture

### Describe in detail the robotics algorithm you implemented
The main algorithm we implemented is the Q-learning algorithm, where we trained a Q-matrix by randomly choosing actions for the robot to take with a given state. We defined the action and state spaces discretely and minimalistically since the number of permutations can get really large. The actions specify the type, color, and location of the object to be picked up and states specify the location of the robot (which node it is on) and the states of each object (whether it is at its starting place or in a bin), and we integrated them into an action matrix so we can easily identify which action transitions which state. We also have a reward callback that updates the Q-matrix values with each action, and we iterate through the Q-learning algorithm until the Q-matrix converges under a certain threshold. With the trained Q-matrix (which is a matrix of action-state pairs), we start from the origin position of the robot and chooses the action that has the highest Q-value in a given state and move onto the next state, and we continue iterating until all non-obstacle objects are successfully sorted. Therefore, we have generated an action sequence, which we converted into a sequence of nodes that produces the shortest path for the robot to follow with the Floyd-Warshall algorithm.

### Describe each major component of your project

### Highlight what pieces of code contribute to these main components

## Challenges, Future Work, and Takeaways

### Challenges
One of the biggest challenges we faced in the beginning was defining the state space. We wanted to take into account the location of the robot in figuring out the most optimal path, but the robot could also be anywhere in space, so we decided to base everything on a fixed graph to avoid dealing with a continuous state space. But even when creating the state space discretely, we found that it may still produce a really large number of permutations, which may make the Q-matrix take a very long time to converge. So we discussed a lot amongst ourselves and with Sarah to come up with the most optimal way to define the state space with limited but sufficient information. Also, although the robot considers many aspects during the training phase, we didn't want to give away too much information to it during the execution phase, which we were able to solve by adding complexity to the project with computer vision.

### Future work
TODO: 1 paragraph

### Takeaways
TODO: A few bullet points
