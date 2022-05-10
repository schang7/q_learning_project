#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import os
# import the moveit_commander, which allows us to control the arms
import moveit_commander

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
#from q_learning_project.msg import Traffic

# TODO: import aruco stuff

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
        ### Different states
        #to keep track of whether the robot is holding an object or not - aka is it looking for an object or an AR tag 
        #self.finding_color = 1

        # this applies to either an object or an ar tag
        # if 1, then we are still looking for an object or ar tag
        # if 0, then we are 
        #self.is_exploring = 1

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',
                Image, self.image_callback)
        
        # subscribe to the robot's lidar scan data
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # movement publisher 
        self.movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # TODO: traffic publisher
        #self.traffic_status_pub = rospy.Publisher("/traffic_status", Traffic)

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
        self.move_group_arm.go([0,0.5,0.1,-0.65], wait=True)
        self.move_group_gripper.go([0.0,0.0], wait=True)

        print("ready")

    def find_object(self, color):
        if self.images is not None:
            print('find object ran')

            self.movement.angular.z = -0.1
            image = self.bridge.imgmsg_to_cv2(self.images,desired_encoding='bgr8')
            
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
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
                angular_k = 0.003
                self.movement.angular.z = angular_k * angular_error
                
                tol = 5
                if abs(angular_error) < tol:
                    self.start_moving_forward = 1

                if self.start_moving_forward:
                    # extracting distances of the objects or tags located 
                    # -10 to 10 degrees in front of the bot
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
                        linear_error = slice_mean - self.min_dist_away 
                        linear_k = 0.07
                        print('this is the linear error')
                        print(linear_error)
                        print('is starting to move forward towards goal')
                        self.movement.linear.x = linear_k * linear_error
                        
                        linear_tol = 0.30
                        if abs(linear_error) < linear_tol:
                            self.start_moving_forward = 0
                            self.object_found = 1
                            self.movement.linear.x = 0
                            self.movement.angular.z = 0
                        #TODO: if within tolerance, say that you are finished
                        # ready to pick up object and then set start_moving_forward to 0 
                else:
                    self.movement.linear.x = 0.0
                    

                # shows the debugging window
                # hint: you might want to disable this once you're able to get a red circle in the debugging window
                cv2.imshow("window", image)
                cv2.waitKey(3)

                
            self.movement_pub.publish(self.movement) 

    def pick_up_object(self):
        gripper_joint_goal = [-0.006, -0.006]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

        arm_joint_goal = [0.0, -.7, .1, -0.65]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()

        rospy.sleep(1)
        self.object_picked_up = 1

    def find_tag(self, tag):
        self.movement.angular.z = -0.1
        # corners is a 4D array of shape (n, 1, 4, 2), where n is the number of tags detected
        # each entry is a set of four (x, y) pixel coordinates corresponding to the
        # location of a tag's corners in the image
        
        # ids is a 2D array of shape (n, 1)
        # each entry is the id of a detected tag in the same order as in corners
        
        # tags are 1, 2, and 3 
        # pink should go to 3
        if self.images is not None:
            grayscale_image = self.bridge.imgmsg_to_cv2(self.images,desired_encoding='mono8')
            corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, aruco_dict)
            print('ids', ids)
            print('corners', corners)
            #tag_idx = np.argwhere(ids == tag)
            # do things with proportional control
            #if tag_idx is : # this might not work for list syntax
            #    tag_idx = np.argwhere(ids == tag)
                
            #    self.tag_found = 1
     
        self.movement_pub.publish(self.movement) 


    def drop_object(self):
        arm_joint_goal = [0.0, 0.5, 0.1, -0.65]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()

        gripper_joint_goal = [0.0,0.0]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

        rospy.sleep(1)
        self.object_dropped = 1

    def scan_callback(self, data):
        self.scans = data.ranges 
    def image_callback(self, msg):
        self.images = msg
    '''    
    def image_callback(self, msg):
        if self.is_exploring:
            # start turning 
            self.movement.angular.z = -0.1
            
            # TODO: using scan to also figure out where stuff is, but we could also use the color
             
            if self.finding_color:
                # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                lower_blue = numpy.array([95, 90, 100]) 
                upper_blue = numpy.array([105, 110, 150]) 
                lower_pink= numpy.array([155, 140, 120]) 
                upper_pink= numpy.array([165, 160, 170]) 
                lower_green = numpy.array([30, 130, 90]) 
                upper_green = numpy.array([40, 150, 150]) 
            
                # TODO!!!!!!: need to find a way to not consider the other colors when 
                # we've already placed a pink, green, or blue object. using self.placed_color could be advantageous here
                # We also need to utilize our policy from the q-matrix somehow
                # this erases all pixels that aren't the color in question
                pink_mask = cv2.inRange(hsv, lower_pink, upper_pink)
                blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
                green_mask = cv2.inRange(hsv, lower_green, upper_green)
                # TODO: figure out if we need to limit out search scope, but probably not
                # currently commented out
                # this limits our search scope to only view a slice of the image near the ground
                h, w, d = image.shape
                #search_top = int(3*h/4)
                #search_bot = int(3*h/4 + 20)
                #mask[0:search_top, 0:w] = 0
                #mask[search_bot:h, 0:w] = 0
                # using moments() function, the center of the colored pixels is determined
                pink_M = cv2.moments(pink_mask)
                blue_M = cv2.moments(blue_mask)
                green_M = cv2.moments(green_mask)
                # TODO: apply this to all the colors, currently just to pink mask
                M = blue_M
                # if there are any colored pixels found
                if M['m00'] > 0:
                        #print('detected the color pink')
                        # center of the colored pixels in the image
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        # a red circle is visualized in the debugging window to indicate
                        # the center point of the yellow pixels
                        # hint: if you don't see a red circle, check your bounds for what is considered 'yellow'
                        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                        # proportional control to orient towards the colored object
                        error = ((w/2) - (cx))
                        k = 0.003
                        self.movement.angular.z = k * error
                        
                        # TODO: maybe set a tolerance for judging whether the robot is facing the 
                        # pink/blue/green object head on, so that we can set a boolean for
                        # starting to move towards the object with linear acceleration
                        # e.g.
                        print('angle error')
                        print(error)
                        tol = 1
                        if abs(error) < tol:
                            self.is_exploring = 0
                        else:
                            self.is_exploring = 1
                # shows the debugging window
                # hint: you might want to disable this once you're able to get a red circle in the debugging window
                cv2.imshow("window", image)
                cv2.waitKey(3)
            else: # detecting an AR tag
                # corners is a 4D array of shape (n, 1, 4, 2), where n is the number of tags detected
                # each entry is a set of four (x, y) pixel coordinates corresponding to the
                # location of a tag's corners in the image
                # ids is a 2D array array of shape (n, 1)
                # each entry is the id of a detected tag in the same order as in corners
                # tags are 1, 2, and 3 
                # pink should go to 3
                grey_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='mono8')
                corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, aruco_dict)
        else: # if we are not exploring, we want to stop turning 
            self.movement.angular.z = 0.0
        # publishing either a movement to keep exploring (turning in circles)
        # or proportional control to orient the robot to the object/AR tag
        self.movement_pub.publish(self.movement)
    
    def scan_callback(self, data):
        self.scans = data
        # extracting distances of the objects or tags located 
        # -10 to 10 degrees in front of the bot
        ranges = np.asarray(data.ranges)
        ranges[ranges == 0.0] = np.nan
        slice_size = int(20)
        first_half_angles = slice(0,int(slice_size/2))
        second_half_angles = slice(int(-slice_size/2),0)
        
        # this is the mean distance of likely the object that has been detected
        # we want to minimize this distance once the robot has detected the object 
        # through the camera and we want to start moving close to it
        slice_mean = np.nanmean(np.append(ranges[first_half_angles],ranges[second_half_angles]))
        error = slice_mean - self.min_dist_away # TODO: this might be flipped
        k = 0.05
        if not self.is_exploring:
            print('is starting to move forward towards goal')
            self.movement.linear.x = k*error
            print(error)
            
            # TODO: need to figure out when to actually publish traffic messages for arm movement
            # e.g.
            #tol = 1e-2
            #if error < tol:
            #    self.movement.linear.x = 0 # quit all cmd_vel movement
            #   if self.finding_color: # if we were just looking for an object
            #       traffic_msg = Traffic(pick_up_object = 1, put_down_object = 0)
            #       rospy.sleep(1) # ?
            #       finding_color = 0 # now want to look for ar tag
            #   elif:
            #       traffic_msg = Traffic(put_down_object = 0, pick_up_object = 1) # if we were just looking for a tag
            #       rospy.sleep(1) # ?
            #       finding_color = 1 # start looking for another object again
            #   self.traffic_status_pub.publish(traffic_msg) # publish traffic message to enact arm movement, for different script/module to initiate
            #   TODO!!!!!: # after robot has finished picking up or putting stuff down, 
            #              # need to signal something to move back to the center, which we haven't implemented yet
            #              # This would be set as self.movement.linear.x = ___, self.movement.angular.z = ___ inside this control structure
        else:
            self.movement.linear.x = 0
        
        self.movement_pub.publish(self.movement) 
    '''
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
        #while self.best_policy:
        while True:
            current_action = self.best_policy[0]
            #color = current_action["object"]
            #tag = current_action["tag"]
            color = 'blue'
            tag = 1
            if not self.tag_found:
                self.find_tag(tag)
            #if not self.object_found:
            #    self.find_object(color)
            #elif not self.object_picked_up:
            #    self.pick_up_object()
            #elif not self.tag_found:
            #    self.find_tag(tag)
            #elif not self.object_dropped:
            #    self.drop_object()
            # change all these booleans to 0 after everything is complete
            # remove action so that we can perform the next one
            #else:
            #    self.best_policy.pop(0)
            #    self.object_found = 0
            #    self.object_picked_up = 0
            #    self.tag_found = 0
            #    self.object_dropped = 0
            rospy.sleep(1)
       
        rospy.spin()
                
if __name__ == '__main__':
    object_identifier = ObjectIdentifier()
    object_identifier.run()
