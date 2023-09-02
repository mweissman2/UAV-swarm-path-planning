import pygame
import math
import random
import numpy as np
import heapq

# Constants
WIDTH = 800  # Width of the simulation window
HEIGHT = 608  # Height of the simulation window
AGENT_RADIUS = 5  # Radius of the agent
OBSTACLE_RADIUS = 30  # Radius of the obstacles
MOVEMENT_SPEED = 3  # Movement speed of the agent

# For APF
SEARCH_RADIUS = 20

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)


class Agent:
    def __init__(self, agent_id, x, y):
        self.agent_id = agent_id
        self.x = x
        self.y = y
        self.start = (self.x, self.y)
        self.path = []
        self.disp_goal_reached = False
        self.temp_path = []
        self.initial_pos = (x, y)  # For permanent pos storage

    def get_id(self):
        print(self.agent_id)

    def move(self):
        if self.path:
            next_pos = self.path[0]
            dx = next_pos[0] - self.x
            dy = next_pos[1] - self.y
            distance = (dx ** 2 + dy ** 2) ** 0.5

            self.x = next_pos[0]
            self.y = next_pos[1]
            self.path.pop(0)
            if len(self.path) == 0:
                print("agent path completed")

    def draw(self, screen):
        pygame.draw.circle(screen, GREEN, (self.x, self.y), AGENT_RADIUS)

    def reset(self):
        self.x, self.y = self.initial_pos
