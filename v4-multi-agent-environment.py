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
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y
        self.path = []

    def move(self):
        if self.path:
            next_pos = self.path[0]
            dx = next_pos[0][0] - self.x
            dy = next_pos[0][1] - self.y
            distance = (dx ** 2 + dy ** 2) ** 0.5
            if distance <= MOVEMENT_SPEED:
                self.x = next_pos[0][0]
                self.y = next_pos[0][1]
                self.path.pop(0)
            else:
                direction_x = int(dx / distance * MOVEMENT_SPEED)
                direction_y = int(dy / distance * MOVEMENT_SPEED)
                self.x += direction_x
                self.y += direction_y

    def draw(self, screen):
        pygame.draw.circle(screen, GREEN, (self.x, self.y), AGENT_RADIUS)

class Swarm:
    def __init__(self, agents):
        self.agents = agents

class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

    def draw(self, screen):
        pygame.draw.circle(screen, BLACK, (self.x, self.y), self.radius)

class Algorithm:
    def __init__(self, swarm, obstacles):
        self.swarm = swarm
        self.obstacles = obstacles

    def a_star_search(self, start, goal):
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
            neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]  # 4-connected grid
            valid_neighbors = []
            for neighbor in neighbors:
                if is_valid(neighbor):
                    valid_neighbors.append(neighbor)
            return valid_neighbors

        # A* search algorithm
        frontier = [(0, start)]  # Priority queue of nodes to explore
        came_from = {}  # Dictionary to store the parent of each node
        cost_so_far = {start: 0}  # Dictionary to store the cost to reach each node

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
        paths = []
        for agent in self.swarm.agents:
            path = []
            current = goal
            while current != start:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            paths.append(path)

        return paths


def run_scenario_1():
    # Initialize Pygame
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Path Planning Simulation")
    clock = pygame.time.Clock()

    # Create swarm of agents
    agents = [
        Agent(1, 100, 100),
        Agent(2, 100, 100),
        Agent(3, 100, 100)
    ]

    obstacles = [
        Obstacle(300, 200, OBSTACLE_RADIUS),
        Obstacle(400, 200, OBSTACLE_RADIUS),
        Obstacle(500, 200, OBSTACLE_RADIUS),
        Obstacle(600, 200, OBSTACLE_RADIUS),
        Obstacle(300, 400, OBSTACLE_RADIUS),
        Obstacle(400, 400, OBSTACLE_RADIUS),
        Obstacle(500, 400, OBSTACLE_RADIUS),
        Obstacle(600, 400, OBSTACLE_RADIUS),
    ]  # Initialize obstacles at specific positions

    # Create swarm instance
    swarm = Swarm(agents)

    # Create an instance of the Algorithm class
    algorithm = Algorithm(swarm, obstacles)

    # Set start and goal positions for all agents
    start_position = (100, 100)
    goal_position = (700, 500)

    # Find path for each agent using A* search algorithm
    for agent in swarm.agents:
        path = algorithm.a_star_search(start_position, goal_position)
        agent.path = path.copy()

    # Choose one agent to display
    agent_to_display = swarm.agents[0]

    # Game loop
    running = True
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Update the agent's position
        agent_to_display.move()

        # Clear the screen
        screen.fill(WHITE)

        # Draw the agent and obstacles
        agent_to_display.draw(screen)
        for obstacle in obstacles:
            obstacle.draw(screen)

        # Draw the start and goal positions
        pygame.draw.circle(screen, BLUE, start_position, 5)
        pygame.draw.circle(screen, BLUE, goal_position, 5)

        # Draw the path
        if path:
            points = [(int(round(pos[0])), int(round(pos[1]))) for pos in path]
            pygame.draw.lines(screen, BLUE, False, points)

        # Update the display
        pygame.display.flip()
        clock.tick(60)

    # Quit the simulation
    pygame.quit()

def run_scenario_2():
    # Initialize Pygame
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Path Planning Simulation")
    clock = pygame.time.Clock()

    # Create agent and obstacles
    agent = Agent(100, 100)  # Initialize agent at the start point
    obstacles = [
        Obstacle(200, 300, OBSTACLE_RADIUS),
        Obstacle(400, 300, OBSTACLE_RADIUS),
        Obstacle(600, 300, OBSTACLE_RADIUS),
        Obstacle(400, 100, OBSTACLE_RADIUS),
        Obstacle(400, 200, OBSTACLE_RADIUS),
        Obstacle(400, 400, OBSTACLE_RADIUS),
        Obstacle(400, 500, OBSTACLE_RADIUS),
        Obstacle(300, 200, 40),
        Obstacle(300, 400, 60),
        Obstacle(500, 200, 60),
        Obstacle(500, 400, 40),
        Obstacle(600, 200, 30),
        Obstacle(600, 400, 30),
    ]  # Initialize obstacles at specific positions

    # Create algorithm instance
    algorithm = Algorithm(agent, obstacles)

    # Set start and goal positions
    start = (100, 100)
    goal = (700, 500)

    # Find path using A* search algorithm
    path = algorithm.a_star_search(start, goal)
    agent.path = path.copy()

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
        pygame.draw.circle(screen, BLUE, start, 5)
        pygame.draw.circle(screen, BLUE, goal, 5)

        # Draw the path
        if path:
            pygame.draw.lines(screen, BLUE, False, path)

        # Update the display
        pygame.display.flip()
        clock.tick(60)

    # Quit the simulation
    pygame.quit()

def run_scenario_3():
    # Initialize Pygame
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Path Planning Simulation")
    clock = pygame.time.Clock()

    # Create agent and obstacles
    agent = Agent(100, 100)  # Initialize agent at the start point
    obstacles = [
        Obstacle(200, 100, OBSTACLE_RADIUS),
        #Obstacle(400, 100, OBSTACLE_RADIUS),
        Obstacle(600, 100, OBSTACLE_RADIUS),
        #Obstacle(200, 300, OBSTACLE_RADIUS),
        Obstacle(400, 300, OBSTACLE_RADIUS),
        Obstacle(600, 300, OBSTACLE_RADIUS),
        Obstacle(200, 500, OBSTACLE_RADIUS),
        Obstacle(400, 500, OBSTACLE_RADIUS),
        Obstacle(600, 500, OBSTACLE_RADIUS),
        Obstacle(300, 200, OBSTACLE_RADIUS),
        Obstacle(500, 200, OBSTACLE_RADIUS),
        Obstacle(300, 400, OBSTACLE_RADIUS),
        #Obstacle(500, 400, OBSTACLE_RADIUS),
        Obstacle(400, 400, OBSTACLE_RADIUS),
        Obstacle(400, 200, OBSTACLE_RADIUS),
        Obstacle(400, 100, OBSTACLE_RADIUS),
        Obstacle(100, 300, 60),
        Obstacle(525, 375, 100),
        Obstacle(700, 300, 10),
        Obstacle(400, 50, 70),
        Obstacle(400, 550, 80),
        Obstacle(250, 350, 60),
        #Obstacle(550, 350, 60),
        Obstacle(300, 100, 40),
        Obstacle(500, 100, 60),
        Obstacle(300, 500, 20),
        Obstacle(500, 500, 40),
    ]  # Initialize obstacles at specific positions

    # Create algorithm instance
    algorithm = Algorithm(agent, obstacles)

    # Set start and goal positions
    start = (100, 100)
    goal = (700, 500)

    # Find path using A* search algorithm
    path = algorithm.a_star_search(start, goal)
    agent.path = path.copy()

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
        pygame.draw.circle(screen, BLUE, start, 5)
        pygame.draw.circle(screen, BLUE, goal, 5)

        # Draw the path
        if path:
            pygame.draw.lines(screen, BLUE, False, path)

        # Update the display
        pygame.display.flip()
        clock.tick(60)

    # Quit the simulation
    pygame.quit()


# Run the scenarios
run_scenario_1()
#run_scenario_2()
#run_scenario_3()
