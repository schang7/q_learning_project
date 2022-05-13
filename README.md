# q_learning_project


## Part 1 Writeup

### Objectives description, high-level description, Q-learning algorithm description

- Objectives description (2-3 sentences): Describe the goal of this project.:

  To implement a Q-Learning algorithm that gives the robot the ability to organize objects in a given environment. Our goal is to train our Q-Matrix using reinforcement learning, and stopping when we have a converged Q-matrix that will be used to guide the robot to execute the best set of actions in placing different colored objects in front of corresponding AR tags in space. 

- High-level description (1 paragraph): At a high-level, describe how you used reinforcement learning to solve the task of determining which colored objects belong in front of each AR tag.:

  We used reinforcement learning in the Q-learning algorithm. For each iteration of the Q-learning algorithm, we randomly chose an action of moving one of three colored objects (pink, blue, and green) to one of three AR tags. After performing this action, we received a reward that would result from placing the objects in the post-action state (which was either 0 or +100; +100 corresponds to when all three objects are all placed in front of the correct tags). We used this reward to calculate the q-value using the Bellman Equation. In this equation, we set our learning rate to 1 and our discount factor to 0.1. We then updated the Q-matrix with this calculated q-value. We determined convergence of the q matrix by a rule that compared the difference between consecutive q-values, and if the difference was negligible for a certain number of iterations, we determined that the Q Matrix had converged. This converged Q-matrix allows us to use reinforcement learning to determine the best set of actions the robot should take to perform a goal because we will ultimately use it to find the best set of actions (a best policy) that maximizes the reward obtained.
  
- Q-learning algorithm description: Describe how you accomplished each of the following components of the Q-learning algorithm in 1-3 sentences, and also describe what functions / sections of the code executed each of these components (1-3 sentences per function / portion of code):
- 1) Selecting and executing actions for the robot (or phantom robot) to take
We took the current state at the beginning of the q-learning iteration and found all valid potential next states that could be attained (given the current state) using the action_matrix. We then randomly selected one state from this array of potential next valid states, and found the corresponding action that would get us to that state. If there are no valid next states to take from the current state (i.e. all three objects are in front of a tag), then there are no valid states and we reset the current state to be 0 (corresponding to when all three objects are at the origin). 
  Location: Lines 85-103, 168-172 (for going back to origin if we’ve reached an end state) 

- 2) Updating the Q-matrix:
In order to update the Q matrix, we first found the Q matrix entries corresponding to all actions (columns) given the chosen future state (row), which is stored in max_Q_Candidates. We then took the max Q-value from all these entries (max_Q). We then used this as the Q(S_t+1, at) input in the Bellman Equation, and along with the reward we obtained from performing the action and our discount factor parameter, we calculated the Q value of (S_t, a_t). We then updated the corresponding element of the Q matrix to this value. 
  Location: Lines 123-136 

- 3) Determining when to stop iterating through the Q-learning algorithm
We determined our q matrix had converged by comparing the calculated q values in consecutive iterations. (abs(q_value-current_q_value)). If the difference between these q values was less than some tolerance level, n times consecutively, then we determined that the Q matrix was not being updated significantly and so it had converged. In testing for optimizing our parameters, we found that in our best iteration, the q matrix converged in 6700 iterations, after the q_value remained unchanged for 60 iterations total. Thus, we set our upper bound for the number of consecutive iterations the q matrix should remain unchanged for, to be 60. This determined that our q matrix converged, and we tested this by manually confirming that the best policy in the converged q-matrix csv file led to the goal state.
  Location: Lines 155-162 

## Part 2 Write Up: 
- Executing the path most likely to lead to receiving a reward after the Q-matrix has converged on the simulated Turtlebot3 robot
  
**Description**: For finding the policy, we started at the index of the origin state (where all objects are not placed in front of any tags). We found the row in the converged Q-matrix which corresponded to that state and then found the column at which it had the maximum q-value, which was the next best action to take in the policy. We stored that action and updated the current state to a new state that the environment would be in after executing that action and found the next best action until all the q-values were equal in a given row (which means we have reached the final action). We stored this into a list and ran through each action (putting an object in front of a given tag) in our run function, popping off actions in the list after they were succesfully executed. 
  **Location**: find_best_policy in robot_perception.py
  
- Robot perception description: Describe how you accomplished each of the following components of the perception elements of this project in 1-3 sentences, any online sources of information/code that helped you to recognize the objects, and also describe what functions / sections of the code executed each of these components (1-3 sentences per function / portion of code):

- Identifying the locations and identities of each of the colored objects


  **Description**: We first had the robot spin around until it found an object that was within the optimal hsv range for one of the three color identities it would be looking for depending on the action being executed in the best policy (pink, green, or blue). We found these ranges through online color picker applications and also by putting the objects in front of the camera and observing the reported hsv values. We then created a mask to remove all the color pixel values that were not in the range of the color we were considering and found the central location of the desired color pixels’ in the image. This central location was considered to be the location of the colored object.
    **Location**: within robot_perception.py, the find_object method in the ObjectIdentifier class.
  
- Identifying the locations and identities of each of the AR tags


  **Description**: We programmed the robot such that it turns around until it detects the desired AR tag id within the ARUCO library, and then moves to the AR tag using proportional control. We detected the correct AR tag by matching the id of a detected tag with the corresponding tag from the action output of our converged Q Matrix. 
    **Location**: find_tag method in robot_perception.py 

- Robot manipulation and movement: Describe how you accomplished each of the following components of the robot manipulation and movement elements of this project in 1-3 sentences, and also describe what functions / sections of the code executed each of these components (1-3 sentences per function / portion of code):
- Moving to the right spot in order to pick up a colored object


  **Description**: We used proportional control to angle the robot towards the location of the object (central location of the pink/blue/green pixels) so that the robot faced the object head on. If the front of the robot (0 degrees) faced the object within a certain angular tolerance from the object, we considered it as facing the object and initiated forward linear movement towards it using proportional control. We used LIDAR scan range data from within 8 degrees of the front of the robot to see how far away it was from the object, and if it was close enough within a certain tolerance such that the object would be inside the gripper, we stopped all movement. 
   **Location**: find_object method in robot_perception.py
  
- Picking up the colored object


  **Description**: We tested out different joint angle and gripper width configurations using the manipulation GUI to see how the arm should be positioned to pick up the object. The robot is initially set to have its arm extended in front of it (without blocking the LIDAR) and gripper wide open. Once the robot was positioned such that the object was inside this gripper, we initiated movements that closed the gripper to grab the object and lift the arm up so that the object is high in the air (so that nothing interferes with the LiDAR scan). 
  **Location**: pick_up_object method in robot_perception.py

- Moving to the desired destination (AR tag) with the colored object


  **Description**: While the arm was still gripping the object we had the robot turn around and then identify the corresponding AR tag. We used proportional control again to angle the robot towards the correct AR tag so that it would face it head on, but this time initiated constant linear movement (not using proportional control) towards the object if the front of the robot was within an angular tolerance from the center of the AR tag. Once the robot got close enough to the AR tag within a set linear distance tolerance, we stopped all movement. 
  **Location**: find_tag method in robot_perception.py

- Putting the colored object back down at the desired destination


  **Description**: We had the robot first lower its arm back to its initial position (extended outwards) and let the program sleep for a second to make sure all the joint arm movements were executed. We then made the gripper open up again while also having the robot linearly move backwards for one second so that it would let go of the object, and then get the object out of the vicinity of the gripper so that when it starts exploring for another colored object, it does not knock the other one over. 
  **Location**: drop_object method in robot_perception.py

 
### Challenges (1 paragraph): Describe the challenges you faced and how you overcame them.

- A persistent challenge we had was our measurements in general being very noisy or laggy. For instance, our LIDAR scan was noisy so we continuously had to modify our parameters in the proportional control to ensure the robot reaches the correct distance away from the objects to pick it up (e.g. changing the range of degrees considered for the lidar scan, updating the angular or linear tolerances, decreasing our k-values to make updates slower and more precise). 
- For the camera, the images would often be very laggy such that the correct actions were not initiated in time and the color measurements would be impacted by other objects in the real world environment. We overcame this by conducting our tests in a space closer to the intro_robo wifi connection and controlling our environment by ensuring no other colors were in the view of the robot. Continuing from above, the presence of noise in detecting the AR tag was also an issue. We overcame this by moving the robot slower while it was turning to explore the different tags and making it move sooner towards the tag in the forwards linear direction if it had detected it (by increasing the angular tolerance for find_tag) because the detection ability would get better as the robot got closer.  
- For the big picture, figuring out how to structure our code in enacting different modular step of robot movement was a challenge we faced at first. We first wrote logic within callback functions for LIDAR scan and image receiving topics but realized that it was too complicated, so we wrote them in separate functions and were able to figure out a sound logic for executing each action within the run function. 

### Future work (1 paragraph): If you had more time, how would you improve your implementation?
- We would make our logic more robust to noise and include faster movements, since we had to make things a bit slower in order to make sure that it properly recognized and moved towards objects due to the noise. One idea that could help with recognition issues would be to have the robot go back to the center using odometry information after it correctly picks up an object or drops it off at a tag so that it detects either colored objects or tags more easily.
- Also, for our q-learning portion our algorithm converged a bit slowly. The reason why is because we included sleeps in our logic to make sure that the correct reward was being published and received after different actions (since shorter sleeps led to the program receiving the wrong rewards). However, we would improve the q-learning implementation by having the q-values for the Q-matrix be calculated within the callback function that receives the rewards and use booleans to detect whether updates to the matrix had been made so that we would avoid having to handle so many sleep statements. 

### Takeaways (at least 2 bullet points with 2-3 sentences per bullet point): What are your key takeaways from this project that would help you/others in future robot programming assignments working in pairs? For each takeaway, provide a few sentences of elaboration.

- One key takeaway was how to create modular structure for code in logical ways. Since this project had so many moving parts, it gave us experience in breaking down a large objective into smaller goals, and tackling them one at a time. This will be especially useful for the team final project, where we might be implementing different interesting modules for our project and can divide and conquer for writing and managing different types of algorithms.
- Incorporating arm control was another key takeaway of the project. We learned that a lot of pick-up/drop-off movements can be perfected by tweaking the gripper joint parameters and gained experience in learning about how different joint angles affect the position of the robot arm. This will help in future projects where we hope to use the robot arm for dextrous movements that allow you to pick up and manipulate things in the environment. 
- Another key takeaway from this project was again the importance of accounting for noise in real-world applications of robotics and budgeting time for optimizing parameters. As we gain more experience in implementing complex forms of color/object/distance recognition, sensor measurements play a greater role in robot control and we will need to be experienced in handling noise in these situations. 

## Video of the program running


https://user-images.githubusercontent.com/65791750/168212855-c622305f-5003-4afe-a011-87e4d3f0f8c1.mp4



Implementation plan:
The names of your team members:
Suha Chang
Ana Rath

A 1-2 sentence description of how your team plans to implement each of the following components of this project as well as a 1-2 sentence description of how you will test each component:

Q-learning algorithm
Executing the Q-learning algorithm
- Implementation: We would implement the q-learning algorithm as discussed in class. In our Q matrix we will make sure to choose an action at random that is a valid transition with self.action_matrix and update the Q value for each of the states we enter through every iteration until convergence. 
- Testing: We will use the phantom_robot movement node to make sure that actions are being published properly and and movements from certain states are being executed as expected depending on a starting state. We will also check each step of the algorithm to make sure invalid transitions aren’t being included and calculations are correct.

Determining when the Q-matrix has converged
- Implementation: Store the Q matrix between iterations, and find the difference. Once the difference between consecutive Q matrices is smaller than some threshold that we set, we will determine that it has converged. 
- Testing: Run some iterations and see if we reach a point where our Q matrix converges. Also see what the consecutive Q matrix values are at this point. Running the debugging scripts and seeing if there are no bugs will also help us run a successful test. 

Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward
- Implementation: Loading in Q-matrix, taking the current state and looking up the corresponding row in the Q-matrix. In this row we can find the action (column) that corresponds to the highest Q-value. This is the action that will lead to the highest expected future reward.
- Testing: Checking what our function returns for the highest Q-value with the actual Q matrix. We can sort the Q values and see if the highest values match. This may have a high run-time depending on the size of the Q matrix. 

Robot perception
Determining the identities and locations of the three colored objects
- Implementation: We will use /scan and /camera/rgb/image_raw + the line follower code we implemented in lab to first have the robot orient towards close objects in its environment and then go towards the ones that match the desired color based on camera feed data.
- Testing: Similar to the line-follower we implemented in lab, we will place an object within the camera view of the robot, and see if the robot correctly identifies the object and color. A similar process will be followed for the AR tags. 

Determining the identities and locations of the three AR tags
- Implementation:  We will use the aruco module to detect the AR tags by matching what the robot is seeing in the camera feed to tags 1, 2, and 3 from the dictionary. 
- Testing: As mentioned above, we will place an AR tag within the camera view of the robot, and see if the robot correctly identifies the tag properties. 

Robot manipulation & movement
Picking up and putting down the colored objects with the OpenMANIPULATOR arm
- Implementation:  We will follow the instructions in Lab F, Simulation in the Robotis e-manual and MoveIt Tutorials. We will use the ROBOTIS GUI to control the arm by determining the joint angle and position information and moving the actual arm to the desired joint angles/positions that will grab the colored objects with the MoveIt package.
- Testing: We will use trial and error to find the right set of parameters for movements and grips to initiate in order to ideally grab onto the colored object’s shape without it slipping from the robot’s grasp, but also making sure the robot isn’t gripping the object too tight. We will have the robot move around while holding the object and optimize the picking up and putting down initial processes to be steady. 

Navigating to the appropriate locations to pick up and put down the colored objects
- Implementation: Using the knowledge about the AR tag identities and locations we will have the robot navigate to the appropriate location depending on the best actions obtained from the Q matrix using /cmd_vel.
- Testing: Testing with one object at a close location and seeing if the navigation is working successfully.
A brief timeline sketching out when you would like to have accomplished each of the components listed above.

Timeline:
Work on Q learning algorithm: April 27 - May 3:
- Executing Q Learning Algorithm: Complete by May 1
- Determining when Q Matrix has converged: May 1 
- Determining highest future reward: May 1 
Generating a converged Q matrix and respective write up by May 3rd 
Work on arm manipulation and picking/dropping objects: May 3 - May 12:
- Robot perception: Complete by May 3
- Arm manipulation: Complete by May 5
- Navigation: Complete by May 8
Full project and write up by May 12 

