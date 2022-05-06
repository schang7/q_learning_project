#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3

# TODO
# import aruco stuff

class ObjectIdentifier:

        def __init__(self):

                # set up ROS / OpenCV bridge
                self.bridge = cv_bridge.CvBridge()

                # initalize the debugging window
                cv2.namedWindow("window", 1)

                # subscribe to the robot's RGB camera data stream
                self.image_sub = rospy.Subscriber('/raspicam_node/image',
                        Image, self.image_callback)
                
                # movement publishing 
                self.movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
                self.movement = Twist(linear=Vector3(), angular=Vector3())

                # TODO: traffic publishing
                
                #to keep track of whether the robot is holding an object or not - aka is it looking for an object or an AR tag 
                self.finding_color = 0

                # this applies to either an object or an ar tag
                self.is_exploring = 0

        def image_callback(self, msg):
                if self.is_exploring:
                    #start turning 
                    self.movement.angular.z = -0.1
                    # set up movement commands for turning
                    # publish at the very end, but update movement commands within later if statements and then publish those if there are other movements to make
                    
                    # using scan to also figure out where stuff is, but we could also use the color
        
                     
                    if self.finding_color:

                        # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
                        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                        # TODO: Define the lower and upper bounds for what should be considered
                        # blue, pink, and green
                        
                        lower_blue = numpy.array([17, 76, 204]) #TODO
                        upper_blue = numpy.array([20, 102, 230]) #TODO
                        lower_pink= numpy.array([17, 76, 204]) #TODO
                        upper_pink= numpy.array([20, 102, 230]) #TODO
                        lower_green = numpy.array([17, 76, 204]) #TODO
                        upper_green = numpy.array([20, 102, 230]) #TODO
                        
                        # TODO apply this for all of the pixels
                        # this erases all pixels that aren't yellow
                        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

                        # this limits our search scope to only view a slice of the image near the ground
                        h, w, d = image.shape
                        search_top = int(3*h/4)
                        search_bot = int(3*h/4 + 20)
                        mask[0:search_top, 0:w] = 0
                        mask[search_bot:h, 0:w] = 0

                        # using moments() function, the center of the yellow pixels is determined
                        M = cv2.moments(mask)
                        # if there are any yellow pixels found
                        if M['m00'] > 0:
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
                                
                                self.movement.linear.x = 0.2
                                self.movement.angular.z = k*error
                                
                                self.movement_pub.publish(self.movement)


                        # shows the debugging window
                        # hint: you might want to disable this once you're able to get a red circle in the debugging window
                        cv2.imshow("window", image)
                        cv2.waitKey(3)
                    else: # detecting an AR tag
                        # TODO: code that 


        def run(self):
                rospy.spin()
                
if __name__ == '__main__':

        rospy.init_node('line_follower')
        follower = Follower()
        follower.run()
