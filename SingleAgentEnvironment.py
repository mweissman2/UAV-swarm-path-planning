import random

import pygame
import heapq

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
    def __init__(self, x, y):
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


def create_obstacle_objects(min_x, max_x, min_y, max_y, min_size, max_size, num_obstacles):
    obstacle_objects = []
    for _ in range(num_obstacles):
        x = int(random.uniform(min_x, max_x))
        y = int(random.uniform(min_y, max_y))
        size = int(random.randint(min_size, max_size))
        obstacle = Obstacle(x, y, size)
        obstacle_objects.append(obstacle)
    return obstacle_objects


class Algorithm:
    def __init__(self, agent, obstacles):
        self.agent = agent
        self.obstacles = obstacles

    def a_star_search(self, goal):
        # Heuristic function (Euclidean distance)
        def heuristic(node):
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
        frontier = [(0, self.agent.start)]  # Priority queue of nodes to explore
        came_from = {}  # Dictionary to store the parent of each node
        cost_so_far = {self.agent.start: 0}  # Dictionary to store the cost to reach each node

        while frontier:
            _, current = heapq.heappop(frontier)

            if current == goal:
                break

            for next_node in get_neighbors(current):
                new_cost = cost_so_far[current] + 1  # Assuming uniform cost
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + heuristic(next_node)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current

        # Reconstruct the path
        path = []
        current = goal
        while current != self.agent.start:
            path.append(current)
            current = came_from[current]
        path.append(self.agent.start)
        path.reverse()

        return path

    def apf_search(self, goal):
        raise NotImplementedError

    def mad_search(self, goal):
        raise NotImplementedError

    def grey_wolf_search(self, goal):
        raise NotImplementedError


def run_scenario_single_agent(obstacles_in, agent_in, goal_in, algorithm_type):
    # Initialize Pygame
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Path Planning Simulation")
    clock = pygame.time.Clock()

    # Create agent and obstacles
    agent = agent_in  # Initialize agent at the start point
    obstacles = obstacles_in

    # Set goal position
    goal = goal_in

    # Create an instance of the Algorithm class
    algorithm = Algorithm(agent, obstacles)

    # Find paths for each agent depending on search method
    # Add the way your algorithm is accessed here
    if algorithm_type == "A Star":
        # Find path using A* search algorithm
        path = algorithm.a_star_search(goal)
        agent.path = path.copy()
    elif algorithm_type == "APF":
        raise NotImplementedError
    elif algorithm_type != "Grey Wolf":
        raise NotImplementedError
    elif algorithm_type != "MAD":
        raise NotImplementedError
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
        agent.move()

        # Clear the screen
        screen.fill(WHITE)

        # Draw the agent and obstacles
        agent.draw(screen)
        for obstacle in obstacles:
            obstacle.draw(screen)

        # Draw the start and goal positions
        pygame.draw.circle(screen, BLUE, agent.start, 5)
        pygame.draw.circle(screen, BLUE, goal, 5)

        # Draw the path
        if path:
            pygame.draw.lines(screen, BLUE, False, path)

        # Update the display
        pygame.display.flip()
        clock.tick(60)

    # Quit the simulation
    pygame.quit()
