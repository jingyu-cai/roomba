# roomba

## Team Members
Karhan Kayan, Jingyu Cai, Zack Wang, Yoon Jeong

## Motivation
We think that this kind of robot would be helpful in the real world - picking and sorting clothes and trash in someone's room, or even picking up garbage in a less-controlled environment. Additionally, this is a good synthesis of all the mechanics we've learned in the class, including computer vision, kinematics, and a reward-based algorithm.

## Main Components
The primary topics will be computer vision and q-learning. The project will involve a fixed environment (room) with three types of objects (trash) at fixed locations: dumbbells, balls, and obstacles. Connecting these objects with the bins is a graph of yellow lines on which the Turtlebot will navigate. We will use a computer vision algorithm to identify which type of object we are dealing with, a q-learning algorithm to determine which actions should be done to optimize, and a shortest-path algorithm to determine the path the Turtlebot needs to take to complete an action.

Placing an object into a bin will return a computed reward from a reward algorithm. This algorithm takes into account if the object was returned to the correctly-colored bin and how “long” it took to deliver it. This factor roughly translates to time spent, and will be calculated using distance from the shortest-path algorithm; if it takes longer, it will be deducted some points. We hope this will lead to interesting situations, in which the q-learning algorithm learns that some trash isn’t worth the effort, or it finds a more optimal pathing to minimize this distance cost (placing any obstacles in a bin will also deduct points instead of rewarding points). 

For defining Q-matrix states, we hope to keep the space limited since the number of permutations can get very large. For each object (dumbbell, ball, or obstacle), it can either be in its starting position, or in its corresponding bin. We also have the location of the robot (at discrete nodes of the yellow line graph) as part of the state. Ex: for just three garbage objects, we could have the state: “obj1: node 1, obj2: in bin, obj3: in bin, turtlebot: at node 3”. We hope that this would take into account the most optimal action for the robot to execute based on its position and what’s left to pick up.

## Final Product 
**MVP**: Given a fixed room and marked out locations for objects “of interest”, the robot can use computer vision to identify types of objects, and move them accordingly. Given a command, it will navigate the “highway” of yellow lines and move that object to the specified bin.
Solid end result: We can train and execute a q-matrix for rewards, meaning the robot The robot should not pick up any of the dud objects (these will incur a negative reward) and also pick up the optimal number of objects in a quick period of time to maximize its rewards.

**Stretch**: It would be above and beyond if we could give more freedom to the robot to move around without following the yellow lines. This may result in a continuous and more complex state space, but it may also increase the efficiency of the robot since it doesn’t have to follow set paths.

## Timeline
**Short term goal**: pick up the object that is right in front of it and put it into a trash bin.
**Long term goal**: pick up every single object in the entire room with high efficiency.

### Tentative Plan
- Finish creating Gazebo world with the objects and the bins by **Wednesday, May 19**. 
- Finish line following and kinematics by **Friday, May 21**.
- Finish object recognition by **Monday, May 24**. 
- Shortest path algorithm by **Wednesday, May 26**. 
- Q learning and the remaining parts by the deadline (**Friday, June 4**), we will also work on defining and testing the action and state space throughout the project.

## Risks
There would likely be a lot of difficulty in telling the difference between an obstacle and an object that needs to be thrown away. Given how tedious keras can be, we may need to explore alternatives for computer vision. Additionally, building our own action and Q-matrices will be taxing. For each additional object to consider, we need an exponentially larger table. Building an algorithm to do this automatically may help significantly. 
