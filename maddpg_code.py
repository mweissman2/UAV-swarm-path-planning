import numpy as np
import matplotlib.pyplot as plt
import random
import math
from MultiAgentEnvironment import *


# Constants
WIDTH = 800  # Width of the simulation window
HEIGHT = 600  # Height of the simulation window
AGENT_RADIUS = 10  # Radius of the agent
OBSTACLE_RADIUS = 30  # Radius of the obstacles
MOVEMENT_SPEED = 3  # Movement speed of the agent


# class for making individual agents
class MADDPG_agent:
    def __init__(self, initial, goal, count, temp, cool, e_th, obstacles, ID):
        self.position = initial             # Current position of the agent, 1x2 list
        self.goal = goal                   # Target position to reach, 1x2 list
        self.path =[]                       # initialize the path
        self.count = count
        self.path_length = 0
        self.obstacles = obstacles
        self.agent_id = ID

        # algorithm parameters
        self.temp = temp
        self.cool_rate = cool
        self.e_th = e_th

    def is_valid(self, node):
        x, y = node
        if x < 0 or x >= WIDTH or y < 0 or y >= HEIGHT:
            return False
        for obstacle in self.obstacles:
            if ((x - obstacle.x) ** 2 + (y - obstacle.y) ** 2) ** 0.5 <= AGENT_RADIUS + obstacle.radius:
                return False
        return True

    def get_neighbors(self, node):
        x, y = node
        neighbors = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]  # 4-connected grid
        valid_neighbors = []
        for neighbor in neighbors:
            if self.is_valid(neighbor):
                valid_neighbors.append(neighbor)
        return valid_neighbors

    def euclid_distance(self, position_now):  # next needs to be a list with two numbers, one x and one y
        x1, y1 =  position_now[0], position_now[1]
        x2, y2 =  self.goal[0], self.goal[1]
        return ((x1 - x2)**2 + (x2 - y2)**2) ** 0.5

    def replay_buffer(self, current, next):   #replay buffer stores the total distance achieved during the episode
        self.path_length += ((current[0]-next[0])**2+(current[1]-next[1])**2)**0.5


    def update_critic(self,e_update, cool_update, temp_update,count_update):
                                 # update reward function
        self.count = count_update
        self.cool_rate = cool_update
        self.temp = temp_update
        self.e_th = e_update


    def count_critic(self):        #agent count down critic
        countdown = self.count
        countdown -= 1
        if countdown == 0:      #maybe make this some other variable instead of bool
            return False
        if countdown > 0:
            return True

    def get_param(self):
        return self.e_th, self.temp, self.cool_rate


    def reward(self,final_x,final_y):
        # used to define the reward for the agents, defined as the euclidean distance- cost
        x1, y1 = final_x, final_y
        x2, y2 = self.goal[0], self.goal[1]
        # Debugging code
        # print(self.goal[0])
        # print(self.goal)
        # print(x1)
        return (abs(x1 - x2) + abs(y1 - y2)) ** 0.5         # needs to change to manhattan heuristic



    # actions are Defined via simulated annealing algorithm, which determines which action to take locally
    # members of the swarm will have to use their own action
    # when transmitting, the replay buffer will contain a list of 5 past positions
    # an episode will contain 5 transitions

    # the agent's internal critic is written into the if statements for the algo
    def action(self, temp=1500, cool_rate=0.25, e_th=.56):   # these params will be updated by critic
        # set city and count parameters
        current_pos = self.position   # the new initial position will be wherever the robot is after 5 transitions
        sa_path = [current_pos]        #overwrite the old sa_path

        for i in range(0,4):                        #every episode has 5 transitions
            # later, the action choice will be based on something else
            neighbor = random.choice(self.get_neighbors(current_pos))  # choose a random neighbor, from the surrounding grid
            current_energy = self.euclid_distance(current_pos)          #find current energy by approximating distance to target
            next_energy = ((current_pos[0]-neighbor[0])**2+(current_pos[1]-neighbor[1])**2)**.5
            delta = current_energy - next_energy                        # calculating energy cost delta for criterion calculation

            # Occasionally the probability calc is not a real number result, if so probability is set to 0
            try:
                prob = math.exp(-delta / temp)
            except:
                prob = 0

            if self.goal == current_pos:
                return sa_path, self.path_length

            if delta > 0:
                self.replay_buffer(current_pos, neighbor)
                current_pos = neighbor
                sa_path.append(current_pos)         # choosing best neighbor as next node if energy delta > 0


            elif prob > e_th:
                self.replay_buffer(current_pos, neighbor)
                current_pos = neighbor
                sa_path.append(current_pos) # choosing best neighbor as next node if probability of delta > 0 is higher

            self.count_critic()
            temp *= cool_rate

        return sa_path, self.path_length                         # return path route and distance to the target, euclid distance



# # Below would be implemented in the MultiAgentEnvironment.py file under function mad_search
# # ------------------------------------------------------------------------------------------------------------------
#
# N = 100                     # number of episodes
# y = .45                     # reward discount, basically minimizes the max reward in the Q equation
# q_1 = []                    # list of q-values
# q_2 = []
#
# path1_store = []            # storing the total path
# path2_store = []
#
# e_th = .56
# cool = 0.05
# temp = 1500
# count = 50
#
# initial = [5,4]
# final = [14,6]
#
# # create agents
#
# agent1 = MADDPG(initial,final,e_th,cool,temp,count)  # argument order is currently incorrect
# agent2 = MADDPG(initial,final,e_th,cool,temp,count)
#
# for episode in range(1, N):
#
#
#     #command agents to commit actions, 5 transitions/ actions per episode = receive 5 transitions/actions as a result
#     path1, length1 = agent1.action()
#     path2, length2 = agent2.action()
#
#     path1_store += path1
#     path2_store += path2
#
#     #compute the value of each Q-value
#     #one for simulated annealing
#     # formula is Q = R + y*max(reward)
#     # R is calculating immediate reward of an action, so reward function calls one of the action algos
#     #the sampling of different actions will be thru using different parameters of the SA function
#     #y is the discount rate, which is multiplied by the maximum reward
#     q_1[episode] = agent1.reward(path1[-1],path1[-2])   #the reward, argument will be current position of agent at end of the action
#     q_2[episode] = agent2.reward(path2[-1], path2[-2])
#     #this q-value will determine the critic update
#     # which modifies the threshold  param of the SA algo
#
#     #sample a previous slope
#         #if new slope is greater, q values are rising and temperature can cool even more --> higher cooling rate, higher threshold
#     gradient_1 = (q_1[episode-2]-q_1[episode-1])/(2)       #simple rise over run calculation
#     gradient_2 = (q_2[episode-2]-q_2[episode-1])/(2)
#
#     new_gradient1 = (q_1[episode-1]-q_1[episode])/(2)
#     new_gradient2 = (q_2[episode-1]-q_2[episode])/(2)
#
#     e_th_1, temp_1, cool_rate_1 = agent1.get_param()
#     e_th_2, temp_2, cool_rate_2 = agent2.get_param()
#
#     if new_gradient1 < gradient_1:
#         e_update_1 = e_th_1 - .03
#         cool_update_1 = cool_rate_1 - .1
#         temp_update_1 = temp_1 - 100
#         # add count update too?
#
#     if new_gradient1 > gradient_1:
#         e_update_1 = e_th_1 + .03
#         cool_update_1 = cool_rate_1 + .1
#         temp_update_1 = temp_1 + 100
#
#     if new_gradient2 < gradient_2:
#         e_update_2 = e_th_2-.03
#         cool_update_2 = cool_rate_2-.1
#         temp_update_2 = temp_2 -100
#         # add count update too?
#
#     if new_gradient2 > gradient_2:
#         e_update_2 = e_th_2 + .03
#         cool_update_2 = cool_rate_2 + .1
#         temp_update_2 = temp_2 + 100
#         # add count update too?
#
#
#     # update critic according to conditions
#     agent1.update_critic(e_update_1,cool_update_1, temp_update_1)
#     agent2.update_critic(e_update_2,cool_update_2, temp_update_2)






