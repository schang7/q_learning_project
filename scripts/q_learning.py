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
        current_state = np.asarray([0, 0, 0])
        current_q_value = -1
        epsilon = 1e-1

        num_iterations_unchanged = 0 #no of iterations to determine q matrix convergence 
        upper_bound_convergence = 60
        while not is_converged:
            print('iteration: ', t)
            print('num_iterations_unchanged: ', num_iterations_unchanged)
            ## Selecting a valid action

            # get index of current state
            current_state_idx = np.where((self.states_array == current_state).all(axis=1))
            current_state_idx = current_state_idx[0][0]
            
            # search action matrix for a good set of actions given our current state
            potential_next_states = self.action_matrix[current_state_idx,:]
            valid_next_states = np.where(potential_next_states >= 0)[0]
            
            next_state_idx = np.random.choice(valid_next_states)
            next_action = int(self.action_matrix[current_state_idx,next_state_idx])
            
            action_dict = self.actions[next_action]
            robot_obj = action_dict["object"]
            to_tag_id = action_dict["tag"]

            # publish action
            action_msg = RobotMoveObjectToTag(robot_object=robot_obj, tag_id=to_tag_id)
            self.action_pub.publish(action_msg)
            
            self.action_was_pubbed = 1
            rospy.sleep(1.5)
            # reward was stored into self.reward, due to the callback function
            gamma = 0.1


            # current state refers to state t, future state refers to state t+1
            future_state = current_state.copy()
            # figure out our new state
            if robot_obj == 'pink':
                future_state[0] = to_tag_id 
            elif robot_obj == 'green':
                future_state[1] = to_tag_id
            elif robot_obj == 'blue':
                future_state[2] = to_tag_id
            future_state_idx = np.where((self.states_array == future_state).all(axis=1))
            future_state_idx = future_state_idx[0][0]
            print('future state: ', future_state_idx)
            max_Q_candidates = self.Q[future_state_idx,:]
            
            max_Q = np.max(max_Q_candidates)
            
            # calculate q_value and update the Q matrix
            reward_after_action = self.reward
            
            
            q_value = reward_after_action + (gamma * max_Q)
            self.Q[current_state_idx, next_action] = q_value
            Q_matrix_msg = QMatrix(q_matrix = self.Q)
            self.q_matrix_pub.publish(Q_matrix_msg)

            # checking for convergence 
            #checking if the q value has not changed in n iterations 
            if abs(q_value - current_q_value) < epsilon: 
                num_iterations_unchanged += 1
            else:
                num_iterations_unchanged = 0 
            
            # the q matrix has converged
            if(num_iterations_unchanged == upper_bound_convergence):
                is_converged = 1
                    
            # updating stuff for the next iteration!
            current_q_value = q_value
            current_state = future_state
            #checking if all objects is in front of a tag (reset current state to 0)
            future_states = self.action_matrix[future_state_idx,:]
            valid_future_states = np.where(future_states >= 0)[0]
            if(valid_future_states.size == 0):
                current_state = np.asarray([0, 0, 0])
            self.action_was_pubbed = 0
            t = t + 1
    

    def find_best_policy(self):
        # find best actions to take based on max value
        # at the beginning we want to start at the origin for all objects (state = 0,0,0)
        
        # initializing current state as the origin
        current_state = np.asarray([0,0,0])
        reached_final_state = 0

        while not reached_final_state:
            # get index of current state
            current_state_idx = np.where((self.states_array == current_state).all(axis=1))[0][0]

            q_vals_current_state = self.Q[current_state_idx,:]
            best_action = np.argmax(q_vals_current_state)
            self.best_policy.append(best_action)

            if np.min(q_vals_current_state) == np.max(q_vals_current_state):
                # all values are equal for the q_vals, there is nowhere else to jump to next
                reached_final_state = 1
                print(self.best_policy)

    def receive_reward(self, data):
        if self.action_was_pubbed:
            self.reward = data.reward

    def save_q_matrix(self):
        np.savetxt('converged_q_matrix.csv', self.Q, delimiter=",")
        return
    
    def run(self):
        self.train_q_matrix()
        self.save_q_matrix()

if __name__ == "__main__":
    node = QLearning()
    node.run()
