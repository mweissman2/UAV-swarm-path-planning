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

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

# class for making individual agents
class MADDPG_agent:
    def __init__(self, initial, goal, temp, e_th, obstacles, ID):
        self.position = initial  # Current position of the agent, 1x2 list
        self.start = initial
        self.goal = goal  # Target position to reach, 1x2 list
        self.path = []  # initialize short term memory
        self.path_length = 0
        self.obstacles = obstacles
        self.agent_id = ID
        self.next_reward_neighbor = []
        self.reward_mem = []
        self.disp_path = []
        self.long_mem = []  # long term memory path
        self.disp_goal_reached = False

        # algorithm parameters
        self.temp = temp
        self.e_th = e_th

    def move(self):
        if self.long_mem:
            next_pos = self.long_mem[0]
            dx = next_pos[0] - self.position[0]
            dy = next_pos[1] - self.position[1]
            distance = (dx ** 2 + dy ** 2) ** 0.5
            self.position = (next_pos[0], next_pos[1])
            self.long_mem.pop(0)

            # if distance <= MOVEMENT_SPEED:

            # else:
            #     direction_x = int(dx / distance * MOVEMENT_SPEED)
            #     direction_y = int(dy / distance * MOVEMENT_SPEED)
            #     self.position = (self.position[0] + direction_x, self.position[1] + direction_y)

            # print("dist: " + str(distance))
            # print("position: " + str(self.position))

    def draw(self, screen):
        pygame.draw.circle(screen, GREEN, self.position, AGENT_RADIUS)

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

    def euclid_distance(self, position_now, target):  # next needs to be a list with two numbers, one x and one y
        x1, y1 = position_now[0], position_now[1]
        x2, y2 = target[0], target[1]
        return ((x1 - x2) ** 2 + (x2 - y2) ** 2) ** 0.5

    def update_critic(self, e_update, temp_update):
        # update reward function
        self.temp = temp_update
        self.e_th = e_update

    # def count_critic(self):  # agent count down critic
    #     countdown = self.count
    #     countdown -= 1
    #     if countdown == 0:  # maybe make this some other variable instead of bool
    #         return False
    #     if countdown > 0:
    #         return True

    def get_param(self):
        return self.e_th, self.temp

    def reward(self, x, y):
        # used to define the reward for the agents, defined as the euclidean distance- cost
        x1, y1 = x, y
        x2, y2 = self.goal[0], self.goal[1]

        max_reward = (WIDTH ** 2 + HEIGHT ** 2) ** 0.5
        reward = 0
        for obstacle in self.obstacles:
            p0 = AGENT_RADIUS + obstacle.radius  # Influence radius of F_rep
            distance = self.euclid_distance((x, y), (obstacle.x, obstacle.y))
            if distance < p0:
                reward = 0
            else:
                reward = max_reward - (abs(x1 - x2) + abs(y1 - y2)) ** 0.5

        return reward

    # actions are Defined via simulated annealing algorithm, which determines which action to take locally
    # members of the swarm will have to use their own action
    # when transmitting, the replay buffer will contain a list of 5 past positions
    # an episode will contain 5 transitions

    # the agent's internal critic is written into the if statements for the algo
    def action(self):  # these params will be updated by critic
        # set city and count parameters
        sa_path = [self.position]  # overwrite the old sa_path

        for i in range(0, 4):  # every episode has 5 transitions
            # later, the action choice will be based on something else
            neighbor = random.choice(
                self.get_neighbors(self.position))  # choose a random neighbor, from the surrounding grid

            current_energy = self.reward(self.position[0], self.position[1])
            next_energy = self.reward(neighbor[0], neighbor[1])

            delta = current_energy - next_energy  # calculating energy cost delta for criterion calculation

            # Debugging/Tuning print statements
            # print("temp: " + str(self.temp))
            # print("delta: " + str(delta))
            # print("pos 1: " + str(self.position) + " pos 2: " + str(neighbor))
            # print("curr reward: " + str(current_energy))
            # print("next reward: " + str(next_energy))

            # Occasionally the probability calc is not a real number result, if so probability is set to 0
            try:
                prob = math.exp(-delta / self.temp)
            except:
                # print("Didn't Calculate")
                prob = 0

            if delta < 0:
                # print("accepted best option")
                self.position = neighbor

            elif prob > self.e_th:
                self.position = neighbor

                # Debugging/Tuning print statements
                # print("accepted possibly worse option")
                # print("prob: " + str(prob))
                # print("eth: " + str(self.e_th))

            # else:
                # Debugging/Tuning print statements
                # print("stayed")
                # print("prob: " + str(prob))
                # print("eth: " + str(self.e_th))

            sa_path.append(self.position)
            self.long_mem.append(self.position)
            self.disp_path.append(self.position)

        self.next_reward_neighbor = self.position
        return sa_path, self.path_length, self.reward(self.position[0], self.position[1])  # return path route and distance to the target, euclid distance

    def next_reward(self):
        # maybe add obstacle repulsion later

        max_reward = (WIDTH ** 2 + HEIGHT ** 2) ** 0.5

        # sorry I overwrote ur code sean just cleaned the function up, does the exact same thing

        # iterate over all possible next positions
        x1, y1 = self.next_reward_neighbor[0] + 1, self.next_reward_neighbor[1]
        reward_1 = self.reward(x1, y1)

        x2, y2 = self.next_reward_neighbor[0] - 1, self.next_reward_neighbor[1]
        reward_2 = self.reward(x2, y2)

        x3, y3 = self.next_reward_neighbor[0], self.next_reward_neighbor[1] + 1
        reward_3 = self.reward(x3, y3)

        x4, y4 = self.next_reward_neighbor[0], self.next_reward_neighbor[1] - 1
        reward_4 = self.reward(x4, y4)

        return max(reward_1, reward_2, reward_3, reward_4)

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
