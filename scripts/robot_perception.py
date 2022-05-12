#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import os
# import the moveit_commander, which allows us to control the arms
import moveit_commander

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)



# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"
matrix_prefix = os.path.dirname(__file__) + '/'

class ObjectIdentifier:

    def __init__(self):
        rospy.init_node('object_identifier')

        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # initalize the debugging window
        cv2.namedWindow("window", 1)
     

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',
                Image, self.image_callback)
        
        # subscribe to the robot's lidar scan data
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # movement publisher 
        self.movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        

        # initializing movement publishing
        self.movement = Twist(linear=Vector3(), angular=Vector3())
        # for proportional control
        self.min_dist_away = .5

        self.scans = None
        self.images = None

        ### Parameters for finding the best set of options given a state
        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the pink, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))
        self.states_array = np.asarray(self.states)

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: "pink", tag: 1}
        self.colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": self.colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))

        # getting the converged q_matrix to obtain a best policy
        self.converged_q_matrix = np.loadtxt(matrix_prefix +'converged_q_matrix.csv', delimiter=',')

        # a list of dictionaries with keys 'object' and 'tag'
        self.best_policy = self.find_best_policy()
        
        #control variables to keep track of the state the robot is in 
        self.object_found = 0
        self.object_picked_up = 0
        self.tag_found = 0
        self.object_dropped = 0

        self.start_moving_forward = 0
        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Resetting the arm position
        self.move_group_arm.go([0,0.4,0.1,-0.65], wait=True)
        self.move_group_arm.stop()
        self.move_group_gripper.go([0.019,0.019], wait=True)
        self.move_group_gripper.stop()
        
        print("ready")

    #method for the robot to find the correct object 
    def find_object(self, color):
        #if it finds the image 
        if self.images is not None:
            print('find object ran')

            #setting the angular z value parameters found from trial and error 
            self.movement.angular.z = 0.2
            image = self.bridge.imgmsg_to_cv2(self.images,desired_encoding='bgr8')
            
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            #setting up the ranges for the different colored objects 
            if color == 'blue':
                lower_bound = numpy.array([95, 90, 100]) 
                upper_bound = numpy.array([105, 110, 150]) 
            elif color == 'pink':
                lower_bound= numpy.array([155, 140, 120]) 
                upper_bound= numpy.array([165, 160, 170]) 
            elif color == 'green':
                lower_bound = numpy.array([30, 130, 90]) 
                upper_bound = numpy.array([40, 150, 150]) 
            
            # erases all the pixels in the image that aren't in that range
            mask = cv2.inRange(hsv, lower_bound, upper_bound)
            
            # determines the center of the colored pixels
            M = cv2.moments(mask)

            #dimensions of the image, used later for proportional control 
            h, w, d = image.shape


            # if it detected the color
            if M['m00'] > 0:
                print('detected the color')
                # center of the colored pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                
                # a red circle is visualized in the debugging window to indicate
                # the center point of the colored pixels
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                # proportional control to orient towards the colored object
                angular_error = ((w/2) - (cx))
                angular_k = 0.001
                self.movement.angular.z = angular_k * angular_error
                
                #if the robot is within a certain tolerance of the object, then it should stop moving forward 
                tol = 30
                print(angular_error)
                if abs(angular_error) < tol:
                    self.start_moving_forward = 1

                if self.start_moving_forward:
                    # extracting distances of the objects or tags located 
                    # -10 to 10 degrees in front of the bot
                    print('should start moving forward')
                    if self.scans is not None:
                        ranges = np.asarray(self.scans)
                        ranges[ranges == 0.0] = np.nan
                        slice_size = int(16)
                        first_half_angles = slice(0,int(slice_size/2))
                        second_half_angles = slice(int(-slice_size/2),0)
                    
                        # this is the mean distance of likely the object that has been detected
                        # we want to minimize this distance once the robot has detected the object 
                        # through the camera and we want to start moving close to it
                        slice_mean = np.nanmean(np.append(ranges[first_half_angles],ranges[second_half_angles]))
                        if np.isnan(slice_mean):
                            print('empty slice')
                            self.movement.linear.x = 0.0
                        else:
                            linear_error = slice_mean - self.min_dist_away 
                            print(linear_error) 
                            linear_k = 0.04
                            
                            self.movement.linear.x = linear_k * linear_error
                        
                            linear_tol = 0.20
                            if linear_error < linear_tol:
                                self.start_moving_forward = 0
                                self.object_found = 1
                                self.movement.linear.x = 0
                                self.movement.angular.z = 0
                        
                        
                else:
                    self.movement.linear.x = 0.0
                    

                # shows the debugging window
                # hint: you might want to disable this once you're able to get a red circle in the debugging window
                cv2.imshow("window", image)
                cv2.waitKey(3)

                
            #publish the movement 
            self.movement_pub.publish(self.movement) 

    #now that the robot has found the object, method to pick it up 
    def pick_up_object(self):
        print('pick up object running')
        #parameters for the gripper joint found through trial and error testing 
        gripper_joint_goal = [-0.006, -0.006]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

        #parameters for the arm joint found through trial and error testing 
        arm_joint_goal = [0.0, -.7, .1, -0.65]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()

 
        rospy.sleep(1)
        #updating control variables 
        self.object_picked_up = 1

    #now that the robot has picked up the object, the method to find the correct AR tag 
    def find_tag(self, tag):
        print('find tag running')
        self.movement.angular.z = 0.1
        # corners is a 4D array of shape (n, 4, 2), where n is the number of tags detected
        # each entry is a set of four (x, y) pixel coordinates corresponding to the
        # location of a tag's corners in the image
        
        # ids is a 2D array of shape (n, 1)
        # each entry is the id of a detected tag in the same order as in corners
        
        # tags are 1, 2, and 3 
        # pink should go to 3
        if self.images is not None:
            grayscale_image = self.bridge.imgmsg_to_cv2(self.images,desired_encoding='mono8')
            corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, aruco_dict)
            
            #if it finds ids and corners, matching the tag from the best policy to the tag it finds 
            if ids is not None and len(corners) != 0:
                ids = ids[0]
                corners = corners[0]
                tag_idx = np.argwhere(ids == tag)
                #moving toward the correct tag 
                if tag_idx.size > 0:
                    #print('found the right tag')
                    tag_idx = tag_idx[0]
                    left_upper_corner = corners[tag_idx,0,:]
                    right_upper_corner = corners[tag_idx,1,:]
                    right_bottom_corner = corners[tag_idx,2,:]
                    left_bottom_corner = corners[tag_idx,3,:]
                
                    width = right_upper_corner[0,0] - left_upper_corner[0,0]
                    height = left_upper_corner[0,1] - left_bottom_corner[0,1]
                    cx = int(left_upper_corner[0,0] + width/2)
                    cy = int(left_upper_corner[0,1] + height/2)
                    cv2.circle(grayscale_image, (cx,cy),20,(0,0,255),-1)
                    h, w  = grayscale_image.shape
                    angular_error = ((w/2) - (cx))
                    #angular k values found through testing 
                    angular_k = 0.001
                    print(angular_error)
                    self.movement.angular.z = angular_k*angular_error
                    tol = 30
                    if abs(angular_error) < tol:
                        self.start_moving_forward = 1

                    if self.start_moving_forward:
                        # extracting distances of the objects or tags located 
                        # -10 to 10 degrees in front of the bot
                        print('should start moving forward')
                        if self.scans is not None:
                            ranges = np.asarray(self.scans)
                            ranges[ranges == 0.0] = np.nan
                            slice_size = int(20)
                            first_half_angles = slice(0,int(slice_size/2))
                            second_half_angles = slice(int(-slice_size/2),0)
                    
                            # this is the mean distance of likely the object that has been detected
                            # we want to minimize this distance once the robot has detected the object 
                            # through the camera and we want to start moving close to it
                            slice_mean = np.nanmean(np.append(ranges[first_half_angles],ranges[second_half_angles]))
                            if np.isnan(slice_mean):
                                self.movement.linear.x = 0
                            else:
                                linear_error = slice_mean - self.min_dist_away 
                                
                                print(linear_error)
                                #setting a constant linear velocity to move towards the tag, until it reaches within a certain tolerance 
                                self.movement.linear.x = 0.04
                                linear_tol = 0.005
                                if linear_error < linear_tol:
                                    self.start_moving_forward = 0
                                    self.tag_found = 1
                                    self.movement.linear.x = 0
                                    self.movement.angular.z = 0
                    else:
                        self.movement.linear.x = 0.0
                     

            cv2.imshow("window", grayscale_image)
            cv2.waitKey(3)


        self.movement_pub.publish(self.movement) 

#once the robot has found the correct tag, method to drop the object 
    def drop_object(self):
        print('drop object running')
        #arm joint parameters found through testing 
        arm_joint_goal = [0.0, 0.4, 0.1, -0.65]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        rospy.sleep(5)
        gripper_joint_goal = [0.019,0.019]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()
        
        #moving the robot back once it has dropped the object, so that there is space for it to continue spinning and find the next object 
        self.movement.linear.x = -0.1
        self.movement_pub.publish(self.movement)
        rospy.sleep(1)
        self.movement.linear.x = 0.0
        self.movement_pub.publish(self.movement)

        rospy.sleep(1)
        #updating control variables 
        self.object_dropped = 1

    def scan_callback(self, data):
        self.scans = data.ranges 
    
    def image_callback(self, msg):
        self.images = msg
    
    def find_best_policy(self):
        # find best actions to take based on max value
        # at the beginning we want to start at the origin for all objects (state = 0,0,0)

        # initializing current state as the origin
        current_state = np.asarray([0,0,0])
        reached_final_state = 0
        # a list of dictionaries with keys 'object' and 'tag'
        best_policy = []

        while not reached_final_state:
            # get index of current state
            current_state_idx = np.where((self.states_array == current_state).all(axis=1))[0][0]
            
            q_vals_current_state = self.converged_q_matrix[current_state_idx,:]

            if np.min(q_vals_current_state) == np.max(q_vals_current_state):
                # all values are equal for the q_vals, there is nowhere else to jump to next
                reached_final_state = 1
                break

            best_action = np.argmax(q_vals_current_state)
            best_policy.append(self.actions[best_action])
            
            # figure out our next state
            next_state = current_state.copy()
            robot_obj = self.actions[best_action]["object"]
            to_tag_id = self.actions[best_action]["tag"]
            if robot_obj == 'pink':
                next_state[0] = to_tag_id
            elif robot_obj == 'green':
                next_state[1] = to_tag_id
            elif robot_obj == 'blue':
                next_state[2] = to_tag_id

            current_state = next_state

        return best_policy

    def run(self):
        # while the list of remaining actions to take is not empty
        while len(self.best_policy) > 0: #or not rospy.is_shutdown():
            current_action = self.best_policy[0]
            color = current_action["object"]
            tag = current_action["tag"]
            #statements to execute the different methods 
            if not self.object_found:
                self.find_object(color)
            elif not self.object_picked_up:
                self.pick_up_object()
            elif not self.tag_found:
                self.find_tag(tag)
            elif not self.object_dropped:
                self.drop_object()
            #popping off the list of remaining actions to take once the action has been completed 
            else:
                self.best_policy.pop(0)
                self.object_found = 0
                self.object_picked_up = 0
                self.tag_found = 0
                self.object_dropped = 0
            rospy.sleep(1)
       
        #rospy.spin()
                
if __name__ == '__main__':
    object_identifier = ObjectIdentifier()
    object_identifier.run()
