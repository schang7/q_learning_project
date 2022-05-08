#!/usr/bin/env python3

import rospy
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math
# import the custom message

# TODO: create custom message and then set the right imports here
# traffic message is likely going to be published in robot_perception.py
# the traffic will have two booleans, pick_up_object and put_down_object
from q_learning_project.msg import Traffic

class Robot(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('turtlebot3_traffic_controller')

        # Traffic status subscriber
        rospy.Subscriber("/traffic_status", Traffic, self.traffic_dir_received)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        # TODO: NEED TO SPECIFY THIS AS SLIGHLTY OUTWARDS! 
        self.move_group_arm.go([0,0,0,0], wait=True)
        print("ready")

    def traffic_dir_received(self, data: Traffic):

        # First get gripper to grab the object
        if data.pick_up_object:
            # gripper_joint_goal is a list of 2 radian values, 1 for the left gripper and 1 for the right gripper
            # TODO: this requires testing for the best set of gripper radians to execute
            # arm_joint_goal is a list of 4 radian values, 1 for each joint
            gripper_joint_goal = [0.0, 0.0]
        elif data.put_down_object:
            # TODO: this requires testing for the best set of gripper radians to execute
            gripper_joint_goal = [-0.009,0.0009]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

        # Then move the arm to correctly pick up or drop the object
        if data.pick_up_object:
            # TODO: this requires testing for the best set of joint angles to execute
            # arm_joint_goal is a list of 4 radian values, 1 for each joint
            # SHOULD MOVE UP
            arm_joint_goal = [0.0, math.radians(5.0), math.radians(10.0), math.radians(-20.0)]
        elif data.put_down_object:
            # TODO: this requires testing for the best set of joint angles to execute
            # SHOULD MOVE BACK TO RESET ARM POSITION
            arm_joint_goal = [0.0, math.radians(5.0), math.radians(10.0), math.radians(-20.0)]
        # wait=True ensures that the movement is synchronous
        self.move_group_arm.go(arm_joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group_arm.stop()

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    robot = Robot()
    robot.run()
