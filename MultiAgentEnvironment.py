import random

import pygame
import heapq
from maddpg_code import *
import statistics
import numpy

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


class Agent:
    def __init__(self, agent_id, x, y):
        self.agent_id = agent_id
        self.x = x
        self.y = y
        self.start = (self.x, self.y)
        self.path = []

    def move(self):
        if self.path:
            next_pos = self.path[0]
            dx = next_pos[0] - self.x
            dy = next_pos[1] - self.y
            distance = (dx ** 2 + dy ** 2) ** 0.5
            if distance <= MOVEMENT_SPEED:
                self.x = next_pos[0]
                self.y = next_pos[1]
                self.path.pop(0)
            else:
                direction_x = int(dx / distance * MOVEMENT_SPEED)
                direction_y = int(dy / distance * MOVEMENT_SPEED)
                self.x += direction_x
                self.y += direction_y

    def draw(self, screen):
        pygame.draw.circle(screen, GREEN, (self.x, self.y), AGENT_RADIUS)


class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

    def draw(self, screen):
        pygame.draw.circle(screen, BLACK, (self.x, self.y), self.radius)


# ******* new addition below   *******************

def create_mad_agents(min_x, max_x, min_y, max_y, num_agents):
    # list to contain all agents
    agent_objects = []
    #

    for agent_id in range(1, num_agents + 1):
        x = int(random.uniform(min_x, max_x))
        y = int(random.uniform(min_y, max_y))
        e_th = .56 * random.uniform(2, max_y)
        cool = 0.05 * random.uniform(min_y, max_y)
        temp = 150
        count = 50
        mad_agent = MADDPG_agent(agent_id, x, y, count, temp, cool, e_th)
        agent_objects.append(mad_agent)
    return agent_objects


def create_mad_agents_from_agents(agents_in, goal, obstacles):
    # list to contain all agents
    agent_objects = []
    #
    for agent in agents_in:
        # edit parameter initialization at some point
        # might be randomized later
        e_th = .56
        cool = 0.15
        temp = 1500
        count = 50
        mad_agent = MADDPG_agent((agent.x, agent.y), goal, count, temp, cool, e_th, obstacles, agent.agent_id)
        agent_objects.append(mad_agent)
    return agent_objects


# ************************************************************

# Generate random agents
def create_random_agents(min_x, max_x, min_y, max_y, num_agents):
    agent_objects = []
    for agent_id in range(1, num_agents + 1):
        x = int(random.uniform(min_x, max_x))
        y = int(random.uniform(min_y, max_y))
        agent = Agent(agent_id, x, y)
        agent_objects.append(agent)
    return agent_objects


def create_agent_line(right_x, right_y, num_agents):
    agent_objects = []
    for agent_id in range(1, num_agents + 1):
        agent = Agent(agent_id, right_x - 2 * agent_id, right_y)
        agent_objects.append(agent)
    return agent_objects


class Algorithm:
    def __init__(self, list_of_agents, obstacles):
        self.list_of_agents = list_of_agents
        self.obstacles = obstacles

    def a_star_search(self, goal):
        # Heuristic function (Euclidean distance)
        def heuristic(node, goal):
            x, y = node
            goal_x, goal_y = goal
            return ((x - goal_x) ** 2 + (y - goal_y) ** 2) ** 0.5

        # Check if a given node is valid
        def is_valid(node):
            x, y = node
            if x < 0 or x >= WIDTH or y < 0 or y >= HEIGHT:
                return False
            for obstacle in self.obstacles:
                if ((x - obstacle.x) ** 2 + (y - obstacle.y) ** 2) ** 0.5 <= AGENT_RADIUS + obstacle.radius:
                    return False
            return True

        # Generate valid neighbor nodes
        def get_neighbors(node):
            x, y = node
            neighbors = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]  # 4-connected grid
            valid_neighbors = []
            for neighbor in neighbors:
                if is_valid(neighbor):
                    valid_neighbors.append(neighbor)
            return valid_neighbors

        # A* search algorithm
        paths = []

        for agent in self.list_of_agents:
            frontier = [(0, agent.start)]  # Priority queue of nodes to explore
            came_from = {}  # Dictionary to store the parent of each node
            cost_so_far = {agent.start: 0}  # Dictionary to store the cost to reach each node

            while frontier:
                _, current = heapq.heappop(frontier)

                if current == goal:
                    break

                for next_node in get_neighbors(current):
                    new_cost = cost_so_far[current] + 1  # Assuming uniform cost
                    if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                        cost_so_far[next_node] = new_cost
                        priority = new_cost + heuristic(next_node, goal)
                        heapq.heappush(frontier, (priority, next_node))
                        came_from[next_node] = current

            # Reconstruct the paths for each agent in the swarm
            agent.path = []
            temp_path = []
            current = goal
            while current != agent.start:
                agent.path.append(current)
                temp_path.append(current)
                current = came_from[current]
            agent.path.append(agent.start)
            temp_path.append(agent.start)
            agent.path.reverse()
            temp_path.reverse()
            paths.append(temp_path)

        return paths

    def apf_search(self, goal):
        raise NotImplementedError

    def mad_search(self):
        q = []
        y = .54  # discount value, also currently
        paths = []
        new_gradient = []
        past_gradient = []
        for agent in self.list_of_agents:
            new_gradient.append(0)
            past_gradient.append(0)

        for episode in range(0, 1000):

            for agent in self.list_of_agents:
                path, length, reward = agent.action()

                # update rewards and gradients
                agent.reward_mem.append(reward + y * agent.next_reward())  # this is q-value stored in agent
                if episode > 1:
                    past_gradient[agent.agent_id - 1] = (agent.reward_mem[episode - 2] - agent.reward_mem[episode - 1]) / 2
                    new_gradient[agent.agent_id - 1] = (agent.reward_mem[episode - 1] - agent.reward_mem[episode]) / 2

                # update total paths var

            if episode > 1:

                compare = statistics.mean(new_gradient) - statistics.mean(past_gradient)

                if compare < 0:
                    for mad_agent in self.list_of_agents:
                        # currently arbitrary
                        e_update = mad_agent.e_th - .03
                        cool_update = mad_agent.cool_rate - .05
                        temp_update = mad_agent.temp - 100
                        count_update = mad_agent.count - 5
                        mad_agent.update_critic(e_update, cool_update, temp_update, count_update)

        for agent in self.list_of_agents:
            paths.append(agent.long_mem)

        return paths

    def grey_wolf_search(self, goal):
        raise NotImplementedError


def run_scenario_multi_agent(obstacles_in, agents_in, goal_in, algorithm_type):
    # Initialize Pygame
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Path Planning Simulation")
    clock = pygame.time.Clock()

    # input variables
    agents = agents_in
    obstacles = obstacles_in
    goal_position = goal_in

    # Find paths for each agent depending on search method
    # Add the way your algorithm is accessed here
    if algorithm_type == "A Star":
        # Create an instance of the Algorithm class
        algorithm = Algorithm(agents, obstacles)

        paths = algorithm.a_star_search(goal_position)
        print(paths)
    elif algorithm_type == "APF":
        raise NotImplementedError
    elif algorithm_type == "Grey Wolf":
        raise NotImplementedError
    elif algorithm_type == "MAD":
        mad_agents = create_mad_agents_from_agents(agents, goal_position, obstacles)
        mad_algorithm = Algorithm(mad_agents, obstacles)
        paths = mad_algorithm.mad_search()
        print(paths)
    else:
        print("invalid algorithm")

    # Game loop
    running = True
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Update the agent's position
        for agent in agents:
            agent.move()

        # Clear the screen
        screen.fill(WHITE)

        # Draw the agent
        # Draw the start and goal positions
        for agent in agents:
            agent.draw(screen)
            pygame.draw.circle(screen, BLUE, agent.start, 5)
            pygame.draw.circle(screen, BLUE, goal_position, 5)

        # Draw the obstacles
        for obstacle in obstacles:
            obstacle.draw(screen)

        # Draw the path
        for path in paths:
            pygame.draw.lines(screen, BLUE, False, path)

        # Update the display
        pygame.display.flip()
        clock.tick(60)

    # Quit the simulation
    pygame.quit()
