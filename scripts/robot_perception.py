#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3
import numpy as np

# TODO
# import aruco stuff

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
        
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # movement publishing 
        self.movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.movement = Twist(linear=Vector3(), angular=Vector3())

        self.min_dist_away = 0.1
        # TODO: traffic publishing

        #to keep track of whether the robot is holding an object or not - aka is it looking for an object or an AR tag 
        self.finding_color = 1

        # this applies to either an object or an ar tag
        self.is_exploring = 1

    def image_callback(self, msg):
        #print('entered the callback')
        if self.is_exploring:
            #start turning 
            #self.movement.angular.z = -0.1
            # set up movement commands for turning
            # publish at the very end, but update movement commands within later if statements and then publish those if there are other movements to make
            
            # using scan to also figure out where stuff is, but we could also use the color

             
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
            
                # TODO apply this for all of the pixels
                # this erases all pixels that aren't yellow
                mask = cv2.inRange(hsv, lower_pink, upper_pink)

                # this limits our search scope to only view a slice of the image near the ground
                h, w, d = image.shape
                search_top = int(3*h/4)
                search_bot = int(3*h/4 + 20)
                #mask[0:search_top, 0:w] = 0
                #mask[search_bot:h, 0:w] = 0

                # using moments() function, the center of the yellow pixels is determined
                M = cv2.moments(mask)
                # if there are any yellow pixels found
                if M['m00'] > 0:
                        print('detected the color')
                        # center of the yellow pixels in the image
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])

                        # a red circle is visualized in the debugging window to indicate
                        # the center point of the yellow pixels
                        # hint: if you don't see a red circle, check your bounds for what is considered 'yellow'
                        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

                        # TODO: based on the location of the line (approximated
                        #       by the center of the yellow pixels), implement
                        #       proportional control to have the robot follow
                        #       the yellow line
                        error = ((w/2) - (cx))
                        k = 0.003
                        
                        #self.movement.linear.x = 0.2
                        self.movement.angular.z = k*error
                        
                        # TODO: maybe set a tolerance for judging whether the robot is facing the 
                        # pink/blue/green object head on, so that we can set a boolean for
                        # starting to move towards the object with linear acceleration
                        # e.g.
                        # if error < tol:
                        #    self.forward_towards_object = 1
                        #else:
                        #    self.forward_towards_object = 0
                        
                        #self.movement_pub.publish(self.movement)


                # shows the debugging window
                # hint: you might want to disable this once you're able to get a red circle in the debugging window
                cv2.imshow("window", image)
                cv2.waitKey(3)
            else: # detecting an AR tag
                # TODO: code that 
                print('whatever')
        self.movement_pub.publish(self.movement)
    
    def scan_callback(self, data):
        ## TODO: extracting distances of the objects or tags located 
        # -10 to 10 degrees in front of the bot
        ranges = np.asarray(data.ranges)
        ranges[ranges == 0.0] = np.nan
        slice_size = int(20)
        first_half_angles = slice(0,int(slice_size/2))
        second_half_angles = slice(int(-slice_size/2),0)
        
        # this is the mean distance of likely the object that has been detected
        # we want to minimize this distance once the robot has detected the object 
        # through the camera and we want to start moving close to it
        slice_mean = np.nanmean(np.append(ranges[first_half_angles],ranges[second_half_angles])
        error = slice_mean - self.min_dist_away # TODO: this might be flipped
        k = 0.003

        if self.forward_towards_object:    
            self.movement.linear.x = k*error
         
            # TODO: need some kind of boolean to indicate whether it is time to start picking up the object
            # based on a tolerance level
            # e.g.
            # if error < tol:
            #   quit all cmd_vel movement (self.movement.linear.x = 0)
            #   publish traffic message to enact arm movement, for different script/module to initiate
            #   after robot has finished picking up robot, need to signal something to move back to the center, which we haven't implemented yet

        else:
            self.movement.linear.x = k*error
        self.movement_pub.publish(movement)    
        

    def run(self):
        rospy.spin()
                
if __name__ == '__main__':
    object_identifier = ObjectIdentifier()
    object_identifier.run()
