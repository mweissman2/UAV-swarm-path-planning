import numpy as np
import matplotlib.pyplot as plt
import random
import math
import pygame
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
        self.swarm_pos = []  # memory for storing swarm agent locations

        # algorithm parameters
        self.temp = temp
        self.e_th = e_th
        self.e_counter = 0

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

    # Checking if available neighbors are near the simulation boundary or obstacle
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

    def get_pos(self):
        return self.position

    def goal_test(self):
        if self.position == self.goal:
            return True
        else:
            return False

    def feed_pos(self, agent_pos_in):
        self.swarm_pos = agent_pos_in

    def reward(self, x, y):
        # used to define the reward for the agents, defined as the euclidean distance- cost
        x1, y1 = x, y
        x2, y2 = self.goal[0], self.goal[1]

        max_reward = (WIDTH ** 2 + HEIGHT ** 2) ** 0.5
        obstacle_reward = 0
        goal_reward = 0
        total_reward = 0
        for obstacle in self.obstacles:
            p0 = AGENT_RADIUS + obstacle.radius  # Influence radius of F_rep
            p1 = p0 + 3
            p2 = p0 + 6
            p3 = p0 + 12
            distance = self.euclid_distance((x, y), (obstacle.x, obstacle.y))

            # set negative reward for being close to the obstacle
            # if p0 < distance < p1:
            #     obstacle_reward = -(max_reward - (abs(x1 - x2) + abs(y1 - y2)) ** 0.5)*1.6
            #
            # elif p1 < distance < p2:
            #     obstacle_reward = -(max_reward - (abs(x1 - x2) + abs(y1 - y2)) ** 0.5)*1.3
            #
            # elif p2 < distance < p3:
            #     obstacle_reward = -(max_reward - (abs(x1 - x2) + abs(y1 - y2)) ** 0.5)*1.2

            multiplier = 1.75
            threshold = 50
            goal_r0 = AGENT_RADIUS * 5  # assumes agent radius is same as goal collision radius
            goal_r = [goal_r0]
            # goal_r1 = goal_r0+multiplier
            # goal_r2 = goal_r1*multiplier
            # goal_r3 = goal_r2*multiplier
            # goal_r4 = goal_r3*multiplier
            # goal_r5 = goal_r4*multiplier

            # Creating distance radii from goal, each element is an increasing distance away from goal
            for i in range(1, 16):              # distance increase as index increases
                goal_r.append(goal_r[i - 1] + threshold)

            goal_distance = self.euclid_distance((x, y), self.goal)

            # compare distances and assign appropriate rewards
            for i in range(0,16):       # index 0 has smallest distance, and index 15 has greatest distance
                if goal_r[i] <= goal_distance < goal_r[i + 1] and i != 15:
                    goal_reward = (max_reward - (abs(x1 - x2) + abs(y1 - y2)) ** 0.5) * (17- i)
                    break
                elif i == 15:
                    goal_reward = -(max_reward - (abs(x1 - x2) + abs(y1 - y2)) ** 0.5)



            # if goal_distance <= goal_r[15]:
            #     goal_reward = (max_reward - (abs(x1 - x2) + abs(y1 - y2)) ** 0.5)
            # elif goal_r[14] <= goal_distance < goal_r[15]:
            #     goal_reward = (max_reward - (abs(x1 - x2) + abs(y1 - y2)) ** 0.5) * 1.5
            # elif goal_r[13] <= goal_distance < goal_r[14]:
            #
            #
            # elif goal_r4 <= goal_distance < goal_r5:
            #     goal_reward = (max_reward - (abs(x1 - x2) + abs(y1 - y2)) ** 0.5) * 2
            # elif goal_r3 <= goal_distance < goal_r4:
            #     goal_reward = (max_reward - (abs(x1 - x2) + abs(y1 - y2)) ** 0.5) * 4
            # elif goal_r2 <= goal_distance < goal_r3:
            #     goal_reward = (max_reward - (abs(x1 - x2) + abs(y1 - y2)) ** 0.5) * 4
            # elif goal_r1 <= goal_distance < goal_r2:
            #     goal_reward = (max_reward - (abs(x1 - x2) + abs(y1 - y2)) ** 0.5) * 4
            # elif goal_distance < goal_r1:
            #     goal_reward = (max_reward - (abs(x1 - x2) + abs(y1 - y2)) ** 0.5) * 4
            # else:
            #     goal_reward = (max_reward - (abs(x1 - x2) + abs(y1 - y2)) ** 0.5) * 0.5

            # this directs agents on straight line path
            # else:
            #     reward = max_reward - (abs(x1 - x2) + abs(y1 - y2)) ** 0.5

            reward = obstacle_reward + goal_reward
            # print(reward)
            # Setting reward to 0 for collision case
            if distance <= p0:
                reward = 0

        # test if the drone is stuck in a local maxima
        # if self.position in self.long_mem[-3:]:
        #      reward = -reward

        # test how close the agents are to each other
        # R0 = AGENT_RADIUS*2
        # if len(self.swarm_pos) > 1:
        #     for agents in self.swarm_pos:
        #         col_distance = self.euclid_distance((x, y), (agents[0], agents[1]))
        #         if col_distance <= R0:
        #             reward = 0

        return reward

    # actions are Defined via simulated annealing algorithm, which determines which action to take locally
    # members of the swarm will have to use their own action
    # when transmitting, the replay buffer will contain a list of 5 past positions
    # an episode will contain 5 transitions

    # the agent's internal critic is written into the if statements for the algo
    def action(self):  # these params will be updated by critic
        # set city and count parameters
        sa_path = [self.position]  # overwrite the old sa_path
        self.e_counter += 1
        for i in range(0, 5):  # every episode has 5 transitions
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
        if self.position == self.goal:
            msg_out = f'Agent {self.agent_id} has reached goal at episode {self.e_counter}'
            print(msg_out)


        return sa_path, self.path_length, self.reward(self.position[0], self.position[
            1])  # return path route and distance to the target, euclid distance

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
