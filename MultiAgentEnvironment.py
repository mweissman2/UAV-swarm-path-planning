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
        self.temp_path = []

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
                #direction_x = int(dx_alpha / distance_alpha * strength * MOVEMENT_SPEED)
                #direction_y = int(dy_alpha / distance_alpha * strength * MOVEMENT_SPEED)
                #new_x = self.x + direction_x
                #new_y = self.y + direction_y
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
        #if self.is_visible(new_x, new_y):
        self.x = new_x
        self.y = new_y
        self.path.append((self.x, self.y))
        self.temppath.append((self.x, self.y,))

        #else:
        #    self.x = max(search_space[0], min(new_x, search_space[1]))
        #    self.y = max(search_space[2], min(new_y, search_space[3]))


class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

    def draw(self, screen):
        pygame.draw.circle(screen, BLACK, (self.x, self.y), self.radius)
        pygame.draw.l


# ******* new addition below   *******************

def create_mad_agents(min_x, max_x, min_y, max_y, num_agents, goal, obstacles):
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
        mad_agent = MADDPG_agent((x, y), goal, count, temp, cool, e_th, obstacles, agent_id)
        agent_objects.append(mad_agent)
    return agent_objects


def create_mad_agents_from_agents(agents_in, goal, obstacles):
    # list to contain all agents
    agent_objects = []
    #
    for agent in agents_in:
        # edit parameter initialization at some point
        # might be randomized later
        e_th = .1
        temp = 1500
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
        self.apf_list_of_agents = list_of_agents.copy()
        self.obstacles = obstacles
        self.force_record = []
        self.angle_record = [0.0]

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
        def attractive_force(agent_pos):
            k_att = 5.0  # Attractive force gain
            k_att2 = 3.0  # Velocity gain
            dx = (goal[0] - agent_pos[0])
            dy = goal[1] - agent_pos[1]

            return k_att * dx + k_att2 * MOVEMENT_SPEED, k_att * dy + k_att2 * MOVEMENT_SPEED

        # Compute the repulsive force between the agent and an obstacle
        def repulsive_force(agent_pos, obstacle_pos, obstacle_radius):
            # k_rep = 10000000  # Repulsive force gain
            # k_rep = 500
            k_rep = 200
            buffer = SEARCH_RADIUS * (1/2)  # was 1/5
            p0 = obstacle_radius + buffer  # Influence radius of F_rep
            obst_dist_x = (agent_pos[0] - obstacle_pos[0])
            obst_dist_y = (agent_pos[1] - obstacle_pos[1])
            # dist = (obst_dist_x ** 2 + obst_dist_y ** 2)**0.5  # Dist btwn UAV and obstacle
            dist = (obst_dist_x ** 2 + obst_dist_y ** 2) ** 0.5 - obstacle_radius  # Dist btwn UAV and obstacle
            if dist <= p0:
                # x_rep = k_rep * ((1 / obst_dist_x - (1 / p0)) * (1 / obst_dist_x) ** 2)
                # y_rep = k_rep * ((1 / obst_dist_y - (1 / p0)) * (1 / obst_dist_y) ** 2)
                x_rep = k_rep**3 * ((1 / dist - 1 / p0) * (obst_dist_x / dist) * (1 / dist) ** 2)
                y_rep = k_rep**3 * ((1 / dist - 1 / p0) * (obst_dist_y / dist) * (1 / dist) ** 2)

                return x_rep, y_rep
                # return k_rep * (1/obst_dist_x), k_rep * (1/obst_dist_y)
            else:
                return (0.0, 0.0)

        def inter_agent_force(agent, agent_pos):
            dx_agent_pos = (goal[0] - agent_pos[0])
            dy_agent_pos = (goal[1] - agent_pos[1])
            dist_agent_pos = (dx_agent_pos ** 2 + dy_agent_pos ** 2) ** 0.5

            dx_agent = (goal[0] - agent.x)
            dy_agent = (goal[1] - agent.y)
            dist_agent = (dx_agent ** 2 + dy_agent ** 2) ** 0.5

            threshold = 50

            if (agent.x, agent.y) != agent_pos and (dist_agent_pos > threshold or dist_agent > threshold):
                k_rep = 100000  # Repulsive force gain
                buffer = AGENT_RADIUS + 1
                p0 = AGENT_RADIUS + buffer  # Influence radius of F_rep
                inter_dist_x = agent_pos[0] - agent.x
                inter_dist_y = agent_pos[1] - agent.y
                dist = (inter_dist_x ** 2 + inter_dist_y ** 2) ** 0.5  # Dist btwn UAV and obstacle
                if dist <= p0:
                    if inter_dist_x > 0:
                        # x_rep = k_rep * ((1 / inter_dist_x - (1 / p0)) * (1 / inter_dist_x) ** 2)
                        x_rep = k_rep**3 * ((1 / dist - 1 / p0) * (inter_dist_x / dist) * (1 / dist) ** 2)
                    else:
                        x_rep = 0
                    if inter_dist_y > 0:
                        # y_rep = k_rep * ((1 / inter_dist_y - (1 / p0)) * (1 / inter_dist_y) ** 2)
                        y_rep = k_rep**3 * ((1 / dist - 1 / p0) * (inter_dist_y / dist)* (1 / dist) ** 2)
                    else:
                        y_rep = 0
                    return x_rep, y_rep
                else:
                    return 0.0, 0.0
            else:
                return 0.0, 0.0

        # Compute the total force acting on the agent at its current position
        def total_force(agent_pos, obstacles):
            force_x, force_y = attractive_force(agent_pos)
            # print("att_fx:", force_x, "att_fy:", force_y)

            for obstacle in obstacles:
                rep_force_x, rep_force_y = repulsive_force(agent_pos, (obstacle.x, obstacle.y), obstacle.radius)
                force_x += rep_force_x
                force_y += rep_force_y
            # print("tot_fx:", force_x, "tot_fy:", force_y)

            for agent in self.apf_list_of_agents:
                rep_force_x, rep_force_y = inter_agent_force(agent, agent_pos)
                force_x += rep_force_x
                force_y += rep_force_y

            return (force_x, force_y)

        # Move the agent towards the goal position based on the total force
        def move_towards(episode, agent_pos, obstacles):
            force_x, force_y = total_force(agent_pos, obstacles)
            self.force_record.append((force_x, force_y))
            angle = math.atan2(force_y, force_x)
            self.angle_record.append(angle)
            force_magnitude = math.sqrt(force_x ** 2 + force_y ** 2)
            force_x /= force_magnitude
            force_y /= force_magnitude

            f = 1.3  # jitter coeff
            del_angle_x = angle - self.angle_record[episode-1]
            del_angle_y = math.pi/2 - del_angle_x
            del_angle = (del_angle_x**2 + del_angle_y**2)**0.5
            thresh = 178
            if thresh <= math.degrees(abs(del_angle)) < 180:
                jitter_buff_x = f * math.cos(self.angle_record[episode-1] + 0.5*del_angle_x)
                jitter_buff_y = f * math.cos((math.pi/2 - self.angle_record[episode-1]) + 0.5*del_angle_y)
                print("jitter", episode)
            else:
                jitter_buff_x = 1
                jitter_buff_y = 1

            force_x *= MOVEMENT_SPEED * jitter_buff_x
            force_y *= MOVEMENT_SPEED * jitter_buff_y
            # print("angle", math.degrees(del_angle), "tot:", MOVEMENT_SPEED * f * math.cos(self.angle_record[episode-1] + 0.5*del_angle))

            new_pos_x = agent_pos[0] + force_x
            new_pos_y = agent_pos[1] + force_y

            return (new_pos_x, new_pos_y)

        paths = []

        for episode in range(0, 1000):
            all_goal = True

            for agent in self.apf_list_of_agents:

                # Check if the agent has reached the goal position (close enough)
                if math.sqrt((agent.x - goal[0]) ** 2 + (agent.y - goal[1]) ** 2) <= MOVEMENT_SPEED:
                    next_pos = goal
                    agent.path.append(goal)
                    agent.temp_path.append(goal)
                    # self.list_of_agents.remove(agent) # Need to remove the agent somehow if it reaches the goal

                else:
                    all_goal = False
                    next_pos = move_towards(episode, (agent.x, agent.y), self.obstacles)
                    agent.temp_path.append(next_pos)
                    agent.path.append(next_pos)

                agent.x, agent.y = next_pos

            if all_goal:
                print("Episode:", episode)
                break

        for agent in self.list_of_agents:
            paths.append(agent.temp_path)

        return paths

    def mad_search(self):
        q = []
        y = .54  # discount value, also currently
        paths = []
        new_gradient = []
        past_gradient = []
        for agent in self.list_of_agents:
            new_gradient.append(0)
            past_gradient.append(0)

        for episode in range(0, 2000):

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
                if compare <= 0:
                    print("updating params, compare: " + str(compare))
                    for mad_agent in self.list_of_agents:
                        # currently arbitrary
                        e_update = mad_agent.e_th * 0.99  # gets smaller -> more risky
                        temp_update = (mad_agent.temp + 10) * 0.01  # gets smaller -> more risky
                        mad_agent.update_critic(e_update, temp_update)
                else:
                    print("done good, compare: " + str(compare))
                    for mad_agent in self.list_of_agents:
                        # currently arbitrary
                        e_update = (mad_agent.e_th + 1) * 0.01  # increase necessary prob for bad moves
                        temp_update = (mad_agent.temp + 0.05) * 0.01  # cool down, decrease prob
                        mad_agent.update_critic(e_update, temp_update)

        for agent in self.list_of_agents:
            paths.append(agent.long_mem)

        return paths

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
            wolfFitnessDict[wolf.agent_id] = wolf.fitness   # save new fitness values
        alpha_position = update_hierarchy(wolfFitnessDict)

        # cycle thru episodes to find iterative alphas
        for e in range(max_iterations):
            for wolf in self.list_of_agents:
                wolf.update_position(alpha_position, goal)
                wolf.update_fitness(goal)
                wolfFitnessDict[wolf.agent_id] = wolf.fitness   # save new fitness values
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
