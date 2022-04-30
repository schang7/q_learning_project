#!/usr/bin/env python3

import rospy
import numpy as np
import os
from q_learning_project.msg import RobotMoveObjectToTag, QLearningReward, QMatrix

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: "pink", tag: 1}
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))


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

        self.Q = np.zeros((len(self.states), len(self.actions)))

        # ROS publisher to publish action 
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)

        # ROS publisher to publish Q matrix 
        self.q_matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)

        # ROS subscriber to get the reward
        self.reward_sub = rospy.Subscriber("/q_learning/reward",  QLearningReward, self.receive_reward)

        # initializing different values
        self.reward = 0
        self.action_was_pubbed = 0
        self.best_policy = []


    def train_q_matrix(self):
        t = 0
        is_converged = 0
        previous_state = np.asarray([0, 0, 0])
        previous_q_value = -1
        epsilon = 1e-10
        while not is_converged:
            ## Selecting a valid action

            # get index of previous state
            previous_state_idx = np.where((self.states_array == previous_state).all(axis=1))
            previous_state_idx = previous_state_idx[0][0]
            # search action matrix for a good set of actions given our previous state
            potential_actions = self.action_matrix[previous_state_idx,:]
            valid_actions = np.where(potential_actions >= 0)[0]
            next_action = np.random.choice(valid_actions)
            
            action_dict = self.actions[next_action]
            robot_obj = action_dict["object"]
            to_tag_id = action_dict["tag"]

            # publish action
            action_msg = RobotMoveObjectToTag(robot_object=robot_obj, tag_id=to_tag_id)
            self.action_pub.publish(action_msg)
            self.action_was_pubbed = 1

            rospy.sleep(1)

            # reward was stored into self.reward, due to the callback function
            reward_after_action = self.reward
            gamma = 0.1

            # previous state refers to state t, current state refers to state t+1
            current_state = previous_state.copy()
            # figure out our new state
            if robot_obj == 'pink':
                current_state[0] = to_tag_id 
            elif robot_obj == 'blue':
                current_state[1] = to_tag_id
            elif robot_obj == 'green':
                current_state[2] = to_tag_id
            current_state_idx = np.where((self.states_array == current_state).all(axis=1))
            current_state_idx = current_state_idx[0][0]
            max_Q_candidates = self.Q[current_state_idx,:]
            max_Q = np.max(max_Q_candidates)

            # calculate q_value and update the Q matrix
            q_value = reward_after_action + (gamma * max_Q)
            self.Q[previous_state_idx, next_action] = q_value
            Q_matrix_msg = QMatrix(self.Q)
            self.q_matrix_pub.publish(Q_matrix_msg)

            # checking for convergence 
            if t > 1:
                if abs(q_value - previous_q_value) < epsilon: 
                    is_converged = 1

            # updating stuff for the next iteration!
            previous_q_value = q_value
            previous_state = current_state
            self.action_was_pubbed = 0
            t = t + 1

    def find_best_policy(self):
        # TODO: finish writing this function
        # find best actions to take based on max value
        # at the beginning we want to start at the origin for all objects (state = 0,0,0)
        goal_state_idx = 0 # TODO find the right goal state

        previous_state = np.asarray([0,0,0])
        while current_state_idx != goal_state_idx:
            # get index of previous state
            previous_state_idx = np.where((self.states_array == previous_state).all(axis=1))
            previous_state_idx = previous_state_idx[0][0]

            q_vals_for_initial_state = self.Q[0,:]
            best_action = np.argmax(q_vals_for_initial_state)

            #storing current state

            current_state = previous_state.copy()
            current_state_idx = np.where((self.states_array == current_state).all(axis=1))
            current_state_idx = current_state_idx[0][0]







        self.policy.append(best_action)
        #self.best_policy = ________


    def receive_reward(self, data):
        if self.action_was_pubbed:
            self.reward = data.reward

    def save_q_matrix(self):
        # TODO: You'll want to save your q_matrix to a file once it is done to
        # avoid retraining
        return
    
    def run(self):
        self.train_q_matrix()
        self.find_best_policy()


if __name__ == "__main__":
    node = QLearning()
