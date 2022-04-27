# q_learning_project

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

