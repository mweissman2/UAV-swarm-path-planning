import random
import pygame
import heapq
import math
from maddpg_code import *
import statistics
import numpy
import imageio

# Constants
WIDTH = 800  # Width of the simulation window
HEIGHT = 608  # Height of the simulation window
AGENT_RADIUS = 10  # Radius of the agent
OBSTACLE_RADIUS = 30  # Radius of the obstacles
MOVEMENT_SPEED = 3  # Movement speed of the agent

# For APF
SEARCH_RADIUS = 50

# For MADDPG


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

            # if distance <= MOVEMENT_SPEED:

            # else:
            #     direction_x = int(dx / distance * MOVEMENT_SPEED)
            #     direction_y = int(dy / distance * MOVEMENT_SPEED)
            #     self.x += direction_x
            #     self.y += direction_y

    def draw(self, screen):
        pygame.draw.circle(screen, GREEN, (self.x, self.y), AGENT_RADIUS)


class Wolf(Agent):
    def __init__(self, agent_id, x, y):
        super().__init__(agent_id, x, y)
        self.fitness = 0.0
        self.search_radius = SEARCH_RADIUS
        self.is_alpha = False
        self.temppath = []

    def make_alpha(self):
        self.is_alpha = True

    def make_omega(self):
        self.is_alpha = False

    def is_visible(self, x, y):
        # checks if new positions are within the wolf's search space, i.e., visible to the wolf
        dist_to_desired_pos = math.sqrt(((x - self.x) ** 2 + (y - self.y) ** 2))
        return dist_to_desired_pos <= self.search_radius

    def update_fitness(self, goal):
        dx = self.x - goal[0]
        dy = self.y - goal[1]
        distance_to_goal = math.sqrt((dx ** 2 + dy ** 2))
        self.fitness = distance_to_goal

    def update_position(self, alpha_position, goal):
        # find new position based on alpha/omega designation
        if self.is_alpha == True:
            # calculate distance and direction to goal
            dx = goal[0] - self.x
            dy = goal[1] - self.y
            magnitude = math.sqrt((dx ** 2 + dy ** 2))

            # normalize direction vector to goal
            if magnitude > 0:
                dx /= magnitude
                dy /= magnitude

            # calculate new position based on direction towards goal
            new_x = self.x + dx * MOVEMENT_SPEED
            new_y = self.y + dy * MOVEMENT_SPEED

        else:
            # implement position update logic based on alpha position
            strength = random.uniform(0.01, 0.2)  # randomized strength "pull" towards alpha wolf

            # calculate distance and direction to alpha wolf
            dx_alpha = alpha_position[0] - self.x
            dy_alpha = alpha_position[1] - self.y
            distance_alpha = math.sqrt((dx_alpha ** 2 + dy_alpha ** 2))

            # update position to move towards alpha, dependent on strength variable
            if distance_alpha > 0:
                dx = goal[0] - self.x
                dy = goal[1] - self.y
                magnitude = math.sqrt((dx ** 2 + dy ** 2))

                # normalize direction vector to goal
                if magnitude > 0:
                    dx /= magnitude
                    dy /= magnitude

                new_x = self.x + dx * MOVEMENT_SPEED
                new_y = self.y + dy * MOVEMENT_SPEED
                # direction_x = int(dx_alpha / distance_alpha * strength * MOVEMENT_SPEED)
                # direction_y = int(dy_alpha / distance_alpha * strength * MOVEMENT_SPEED)
                # new_x = self.x + direction_x
                # new_y = self.y + direction_y
                print("hello")
            else:
                dx = goal[0] - self.x
                dy = goal[1] - self.y
                magnitude = math.sqrt((dx ** 2 + dy ** 2))

                if magnitude > 0:
                    dx /= magnitude
                    dy /= magnitude

                new_x = self.x + dx * MOVEMENT_SPEED
                new_y = self.y + dy * MOVEMENT_SPEED

        # checks and limits new position within search space
        # if self.is_visible(new_x, new_y):
        self.x = new_x
        self.y = new_y
        self.path.append((self.x, self.y))
        self.temppath.append((self.x, self.y,))

        # else:
        #    self.x = max(search_space[0], min(new_x, search_space[1]))
        #    self.y = max(search_space[2], min(new_y, search_space[3]))


class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

    def draw(self, screen):
        pygame.draw.circle(screen, BLACK, (self.x, self.y), self.radius)


class Boundary:
    def __init__(self, x_i, y_i, x_f, y_f):
        self.start_x = x_i
        self.start_y = y_i
        self.final_x = x_f
        self.final_y = y_f


# ******* new addition below   *******************

# def create_mad_agents(min_x, max_x, min_y, max_y, num_agents, goal, obstacles):
#     # list to contain all agents
#     agent_objects = []
#     #
#
#     for agent_id in range(1, num_agents + 1):
#         x = int(random.uniform(min_x, max_x))
#         y = int(random.uniform(min_y, max_y))
#         e_th = .56 * random.uniform(2, max_y)
#         cool = 0.05 * random.uniform(min_y, max_y)
#         temp = 1500
#         count = 50
#         mad_agent = MADDPG_agent((x, y), goal, count, temp, cool, e_th, obstacles, agent_id)
#         agent_objects.append(mad_agent)
#     return agent_objects


def create_mad_agents_from_agents(agents_in, goal, obstacles):
    # list to contain all agents
    agent_objects = []
    #
    for agent in agents_in:
        # edit parameter initialization at some point
        # might be randomized later
        e_th = .70
        temp = 1000
        mad_agent = MADDPG_agent((agent.x, agent.y), goal, temp, e_th, obstacles, agent.agent_id)
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
        # agent = Agent(agent_id, right_x - 2*agent_id, right_y)
        agent = Agent(agent_id, right_x, right_y - 20 * agent_id)
        agent_objects.append(agent)
    return agent_objects


def create_wolf_population(right_x, right_y, num_wolves):
    wolf_objects = []
    for agent_id in range(1, num_wolves + 1):
        wolf = Wolf(agent_id, right_x, right_y - 20 * agent_id)
        wolf_objects.append(wolf)
    return wolf_objects


class Algorithm:
    def __init__(self, list_of_agents, obstacles):
        self.list_of_agents = list_of_agents
        self.obstacles = obstacles

    def a_star_search(self, goal):
        # Heuristic function (Euclidean distance)
        def heuristic(node, goal_in):
            x, y = node
            goal_x, goal_y = goal_in
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
            agent.get_id()

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
        # Compute the attractive force between the agent and the goal
        def attractive_force(agent_pos, goal_pos):
            k_att = 50.0  # Attractive force gain
            dx = (goal_pos[0] - agent_pos[0])
            dy = goal_pos[1] - agent_pos[1]
            angle = math.atan2(dy, dx)  # The direction doesn't matter if the UAVs are represented as circles
            return k_att * dx, k_att * dy

        # Compute the repulsive force between the agent and an obstacle
        def repulsive_force(agent_pos, obstacle_pos, obstacle_radius):
            k_rep = 100.0  # Repulsive force gain
            buffer = 0
            p0 = AGENT_RADIUS + obstacle_radius + buffer  # Influence radius of F_rep
            obst_dist_x = agent_pos[0] - obstacle_pos[0]
            obst_dist_y = agent_pos[1] - obstacle_pos[1]
            dist = math.sqrt(obst_dist_x ** 2 + obst_dist_y ** 2)  # Dist btwn UAV and obstacle
            if dist <= SEARCH_RADIUS + obstacle_radius:  # checks if obstacle is in search radius
                if dist <= p0:
                    x_rep = k_rep * ((1 / obst_dist_x - (1 / p0)) * (1 / obst_dist_x) ** 2)
                    y_rep = k_rep * ((1 / obst_dist_y - (1 / p0)) * (1 / obst_dist_y) ** 2)
                    return x_rep, y_rep
                else:
                    return (0.0, 0.0)
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
                # Checking boundary of simulation
                if new_pos_x < 0 or new_pos_x >= WIDTH or new_pos_y < 0 or new_pos_y >= HEIGHT:
                    angle = math.atan2(dy, dx)
                    new_pos_x = obstacle.x + (AGENT_RADIUS + obstacle.radius) * math.cos(angle)
                    new_pos_y = obstacle.y + (AGENT_RADIUS + obstacle.radius) * math.sin(angle)
                    break

            return (new_pos_x, new_pos_y)

        paths = []

        for agent in self.list_of_agents:

            temp_path = [agent.start]
            current_pos = agent.start

            while True:
                # Move towards the next position based on the total force
                next_pos = move_towards(current_pos, goal, self.obstacles)
                temp_path.append(next_pos)

                # Check if the agent has reached the goal position
                if math.sqrt((next_pos[0] - goal[0]) ** 2 + (next_pos[1] - goal[1]) ** 2) <= MOVEMENT_SPEED:
                    break

                current_pos = next_pos

            paths.append(temp_path)
        for agent, temp_path in zip(self.list_of_agents, paths):
            agent.path = temp_path[1:]

        return paths

    def mad_search(self):
        q = []
        y = .54  # discount value, also currently
        paths = []
        disp_paths = []
        new_gradient = []
        past_gradient = []
        compare_list = []
        episode = 0
        agents_pos = []
        iteration = 10000
        for agent in self.list_of_agents:
            new_gradient.append(0)
            past_gradient.append(0)

        for episode in range(0, iteration):
            # while not agent.goal_test():

            for agent in self.list_of_agents:

                if agent.goal != agent.position:
                    path, length, reward = agent.action()

                # if agent reaches goal, stop that agent, revise q value to push agents to be more aggresive
                else:
                    path = []
                    length = 0
                    reward = 0

                # update rewards and gradients
                agent.reward_mem.append(reward + y * agent.next_reward())  # this is q-value stored in agent
                if episode > 1:

                    if agent.goal == agent.position:
                        past_gradient[agent.agent_id - 1] = -100
                        new_gradient[agent.agent_id - 1] = -1000
                    else:
                        past_gradient[agent.agent_id - 1] = (agent.reward_mem[episode - 2] - agent.reward_mem[
                            episode - 1]) / 2
                        new_gradient[agent.agent_id - 1] = (agent.reward_mem[episode - 1] - agent.reward_mem[
                            episode]) / 2

            if episode > 1:
                before_mean = statistics.mean(past_gradient)
                new_mean = statistics.mean(new_gradient)
                compare = new_mean - before_mean

                p_change = 0.85
                compare_list.append(compare)
                # maybe multiply q-gradient by the parameters?
                if compare < 0:
                    for mad_agent in self.list_of_agents:
                        if mad_agent.e_th - 0.05 * p_change <= 0:  # decrease threshold --> more risky
                            e_update = 0.01
                        else:
                            e_update = mad_agent.e_th - 0.05 * p_change
                        e_update = mad_agent.e_th - 0.05 * p_change  # increase temp -> more risky
                        if mad_agent.temp + 500 * p_change > 3000:
                            temp_update = 3000
                        else:
                            temp_update = mad_agent.temp + 500 * p_change
                        mad_agent.update_critic(e_update, temp_update)

                else:
                    for mad_agent in self.list_of_agents:
                        if mad_agent.e_th + 0.05 * p_change >= 1:  # increase threshold --> less risky
                            e_update = 0.99
                        else:
                            e_update = mad_agent.e_th + 0.05 * p_change
                        if mad_agent.temp - 500 * p_change <= 0:  # decrease temp -> less risky
                            temp_update = 100
                        else:
                            temp_update = mad_agent.temp - 500 * p_change
                        mad_agent.update_critic(e_update, temp_update)

        for mad_agent in self.list_of_agents:
            paths.append(mad_agent.long_mem)
            disp_paths.append(mad_agent.disp_path)

        print(compare_list)
        msg = f'Sim completed at episode {episode}'  # only for while statement
        print(msg)
        return disp_paths

    def simplified_gwo_search(self, goal, max_iterations):
        def heuristic(node, goal):
            x, y = node
            goal_x, goal_y = goal
            return ((x - goal_x) ** 2 + (y - goal_y) ** 2) ** 0.5

        def is_valid(node):
            x, y = node
            if x < 0 or x >= WIDTH or y < 0 or y >= HEIGHT:
                return False
            for obstacle in self.obstacles:
                if ((x - obstacle.x) ** 2 + (y - obstacle.y) ** 2) ** 0.5 <= AGENT_RADIUS + obstacle.radius:
                    return False
            return True

        def get_neighbors(node):
            x, y = node
            neighbors = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]  # 4-connected grid
            valid_neighbors = []
            for neighbor in neighbors:
                if is_valid(neighbor):
                    valid_neighbors.append(neighbor)
            return valid_neighbors

        def update_hierarchy(wolfFitnessDict):
            # finds minimum fitness value in the dictionary, assumes global minimum
            alpha_wolf_id = min(wolfFitnessDict, key=wolfFitnessDict.get)
            alpha_position = self.list_of_agents[0].x, self.list_of_agents[0].y
            for wolf in self.list_of_agents:
                if wolf.agent_id == alpha_wolf_id:
                    wolf.make_alpha()
                    alpha_position = (wolf.x, wolf.y)
                else:
                    wolf.make_omega()
            return alpha_position

        paths = []
        temppath = []

        # MAIN LOOP OF GWO ALGORITHM
        wolfFitnessDict = {}

        # use first wolf as preliminary alpha
        for wolf in self.list_of_agents:
            wolf.path.append(wolf.start)
            wolf.temppath.append(wolf.start)
            wolf.update_position((self.list_of_agents[0].x, self.list_of_agents[0].y), goal)
            wolf.update_fitness(goal)
            wolfFitnessDict[wolf.agent_id] = wolf.fitness  # save new fitness values
        alpha_position = update_hierarchy(wolfFitnessDict)

        # cycle thru episodes to find iterative alphas
        for e in range(max_iterations):
            for wolf in self.list_of_agents:
                wolf.update_position(alpha_position, goal)
                wolf.update_fitness(goal)
                wolfFitnessDict[wolf.agent_id] = wolf.fitness  # save new fitness values
            alpha_position = update_hierarchy(wolfFitnessDict)

        # reconstruct path
        for wolf in self.list_of_agents:
            paths.append(wolf.path)
            temppath.append(wolf.temppath)
            paths.reverse()
            temppath.reverse()
        return temppath


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
    paths = []

    # Find paths for each agent depending on search method
    # Add the way your algorithm is accessed here
    if algorithm_type == "A Star":
        # Create an instance of the Algorithm class
        algorithm = Algorithm(agents, obstacles)
        paths = algorithm.a_star_search(goal_position)
        print(paths)
    elif algorithm_type == "APF":
        algorithm = Algorithm(agents, obstacles)
        paths = algorithm.apf_search(goal_position)
    elif algorithm_type == "GWO":
        algorithm = Algorithm(agents, obstacles)
        paths = algorithm.simplified_gwo_search(goal_position, max_iterations=1000)
    elif algorithm_type == "MAD":
        mad_agents = create_mad_agents_from_agents(agents, goal_position, obstacles)
        mad_algorithm = Algorithm(mad_agents, obstacles)
        paths = mad_algorithm.mad_search()
        # print(paths)
    else:
        print("invalid algorithm")

    frames = []

    # Game loop
    running = True
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Update the agent's position
        if algorithm_type == "MAD":
            for agent in mad_agents:
                agent.move()
        else:
            for agent in agents:
                agent.move()

        # Clear the screen
        screen.fill(WHITE)

        # Draw the search radius
        if algorithm_type == "APF":
            for agent in agents:
                pygame.draw.circle(screen, RED, (agent.x, agent.y), SEARCH_RADIUS)

        # Draw the agent
        # Draw the start and goal positions
        if algorithm_type == "MAD":
            multiplier = 1.75
            threshold = 50
            goal_r0 = AGENT_RADIUS * 4  # assumes agent radius is same as goal collision radius

            # goal_r1 = goal_r0 * multiplier  # 80
            # goal_r2 = goal_r1 * multiplier  # 160
            # goal_r3 = goal_r2 * multiplier  # 320
            # goal_r4 = goal_r3 * multiplier  # 640
            # goal_r5 = goal_r4 * multiplier
            #

            goal_r1 = goal_r0 + threshold
            goal_r2 = goal_r1 + threshold
            goal_r3 = goal_r2 + threshold
            goal_r4 = goal_r3 + threshold
            goal_r5 = goal_r4 + threshold

            pygame.draw.circle(screen, (165, 42, 42), goal_position, goal_r5)
            pygame.draw.circle(screen, (255, 255, 0), goal_position, goal_r4)
            pygame.draw.circle(screen, (230, 230, 250), goal_position, goal_r3)
            pygame.draw.circle(screen, (255, 192, 203), goal_position, goal_r2)
            pygame.draw.circle(screen, GREEN, goal_position, goal_r1)
            pygame.draw.circle(screen, RED, goal_position, goal_r0)

            for agent in mad_agents:
                agent.draw(screen)
                pygame.draw.circle(screen, BLUE, agent.start, 5)
                pygame.draw.circle(screen, BLUE, goal_position, 5)

        else:
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

        # Capture the screen as an image
        frame = pygame.surfarray.array3d(screen)
        # Flip the frame vertically
        frame = numpy.flipud(numpy.rot90(frame, k=1))
        frames.append(frame)

        clock.tick(60)

    # Quit the simulation
    pygame.quit()

    # Ignore the imageio warning
    imageio.mimsave('simulation.mp4', frames, fps=60)
