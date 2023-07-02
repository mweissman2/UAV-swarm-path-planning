import pygame
import heapq
import math

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
        # Compute the attractive force between the agent and the goal
        def attractive_force(agent_pos, goal_pos):
            k_att = 1.0  # Attractive force gain
            dx = goal_pos[0] - agent_pos[0]
            dy = goal_pos[1] - agent_pos[1]
            angle = math.atan2(dy, dx)
            return (k_att * dx, k_att * dy)

        # Compute the repulsive force between the agent and an obstacle
        def repulsive_force(agent_pos, obstacle_pos, obstacle_radius):
            k_rep = 100.0  # Repulsive force gain
            min_dist = AGENT_RADIUS + obstacle_radius
            dx = agent_pos[0] - obstacle_pos[0]
            dy = agent_pos[1] - obstacle_pos[1]
            dist = math.sqrt(dx ** 2 + dy ** 2)
            if dist < min_dist:
                angle = math.atan2(dy, dx)
                return (k_rep * (1.0 / dist - 1.0 / min_dist) * math.cos(angle),
                        k_rep * (1.0 / dist - 1.0 / min_dist) * math.sin(angle))
            else:
                return (0.0, 0.0)

        # Compute the total force acting on the agent at its current position
        def total_force(agent_pos, goal_pos, obstacles):
            force_x, force_y = attractive_force(agent_pos, goal_pos)
            for obstacle in obstacles:
                rep_force_x, rep_force_y = repulsive_force(agent_pos, (obstacle.x, obstacle.y), obstacle.radius)
                force_x += rep_force_x
                force_y += rep_force_y
            return (force_x, force_y)

        # Move the agent towards the goal position based on the total force
        def move_towards(agent_pos, goal_pos, obstacles):
            force_x, force_y = total_force(agent_pos, goal_pos, obstacles)
            force_magnitude = math.sqrt(force_x ** 2 + force_y ** 2)
            if force_magnitude > MOVEMENT_SPEED:
                force_x /= force_magnitude
                force_y /= force_magnitude
                force_x *= MOVEMENT_SPEED
                force_y *= MOVEMENT_SPEED

            new_pos_x = agent_pos[0] + force_x
            new_pos_y = agent_pos[1] + force_y

            # Check for collision with obstacles and adjust the new position accordingly
            for obstacle in obstacles:
                dx = new_pos_x - obstacle.x
                dy = new_pos_y - obstacle.y
                distance = math.sqrt(dx ** 2 + dy ** 2)
                if distance <= AGENT_RADIUS + obstacle.radius:
                    angle = math.atan2(dy, dx)
                    new_pos_x = obstacle.x + (AGENT_RADIUS + obstacle.radius) * math.cos(angle)
                    new_pos_y = obstacle.y + (AGENT_RADIUS + obstacle.radius) * math.sin(angle)
                    break

            return (new_pos_x, new_pos_y)

        path = [self.agent.start]
        current_pos = self.agent.start

        while True:
            # Move towards the next position based on the total force
            next_pos = move_towards(current_pos, goal, self.obstacles)
            path.append(next_pos)

            # Check if the agent has reached the goal position
            if math.sqrt((next_pos[0] - goal[0]) ** 2 + (next_pos[1] - goal[1]) ** 2) <= MOVEMENT_SPEED:
                break

            current_pos = next_pos

        return path

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
        # Find path using A* search algorithm
        path = algorithm.apf_search(goal)
        agent.path = path.copy()
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
