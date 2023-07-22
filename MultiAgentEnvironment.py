import random
import pygame
import heapq
import math

from matplotlib import pyplot as plt

from maddpg_code import *
import statistics
import numpy
import imageio
import time


# Constants
WIDTH = 800  # Width of the simulation window
HEIGHT = 600  # Height of the simulation window
AGENT_RADIUS = 10  # Radius of the agent
OBSTACLE_RADIUS = 30  # Radius of the obstacles
MOVEMENT_SPEED = 3  # Movement speed of the agent

# For APF
SEARCH_RADIUS = 50

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


class Wolf(Agent):
    def __init__(self, agent_id, x, y):
        super().__init__(agent_id, x, y)
        self.fitness = 0.0
        self.search_radius = SEARCH_RADIUS
        self.is_alpha = False
        self.is_commensal = True
        self.temppath = []

    def heuristic(self, point):
        dx = point[0] - self.x
        dy = point[1] - self.y
        return math.sqrt((dx ** 2 + dy ** 2))

    def heuristic2(self, point, goal):
        dx = point[0] - goal[0]
        dy = point[1] - goal[1]
        return math.sqrt((dx ** 2 + dy ** 2))

    def make_alpha(self):
        self.is_alpha = True

    def make_omega(self):
        self.is_alpha = False

    def make_commensal(self):
        self.is_commensal = True

    def i_already_explored(self):
        self.is_commensal = False

    def is_valid(self, node, obstacles):
        x, y = node
        if x < 0 or x >= WIDTH or y < 0 or y >= HEIGHT:
            return False
        for obstacle in obstacles:
            if ((x - obstacle.x) ** 2 + (y - obstacle.y) ** 2) ** 0.5 <= AGENT_RADIUS + obstacle.radius:
                return False
        return True

    def is_visible(self, obstacle):
        distance_from_obstacle = (((self.x - obstacle.x)**2 + (self.y - obstacle.y)**2)**0.5)-obstacle.radius
        if distance_from_obstacle < (self.search_radius + obstacle.radius):
            return True, distance_from_obstacle
        else:
            return False, distance_from_obstacle

    def obstacles_in_range(self, obstacles):
        # finds obstacles within search radius and returns visible obstacles and their distances
        list_of_threats = []
        for obstacle in obstacles:
            threat, distance_from_obstacle = self.is_visible(obstacle)
            if threat:
                list_of_threats.append(1/(distance_from_obstacle**2))
        return list_of_threats

    def explore(self, goal, obstacles):
        i = 0
        j = 1
        while self.is_commensal and i < 6:
            # randomly generate an angle
            new_angle = random.uniform(0, 2*math.pi)
            # find a point on that angle
            new_destination = (self.x + (np.cos(new_angle)*j*MOVEMENT_SPEED), self.y + (np.sin(new_angle)*j*MOVEMENT_SPEED))
            value_new_destination = self.heuristic2(new_destination, goal)   # check the heuristic value of that point
            i += 1
            if value_new_destination < self.heuristic2((self.x, self.y), goal) and self.is_valid(new_destination, obstacles):
                self.x = new_destination[0]
                self.y = new_destination[1]
                self.path.append((self.x, self.y))
                self.temppath.append((self.x, self.y,))
                self.i_already_explored()


    def update_fitness(self, goal, obstacles):
        # J_fuel = len(self.temppath)
        J_threat = 0.0
        mu = 0.2    # we liked 0.9
        k = 500
        list_of_threats = self.obstacles_in_range(obstacles)
        for obstacle in list_of_threats:
            J_threat = J_threat + obstacle*k  # larger cost the closer it gets to the obstacle
        J_fuel = self.heuristic(goal)   # distance from goal
        J_cost = mu * J_fuel + (1 - mu) * J_threat

        #if J_cost != 0:
        #    print('J_fuel' + str(J_fuel) + 'J_threat' + str(J_cost))
        self.fitness = J_cost

    def update_position(self, alpha_position, goal, obstacles):
        # find new position based on alpha/omega designation
        if math.sqrt((self.x - goal[0]) ** 2 + (self.y - goal[1]) ** 2) <= MOVEMENT_SPEED:
            self.path.append(goal)
            self.temppath.append(goal)
            self.x = goal[0]
            self.y = goal[1]

        if self.is_alpha:
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

        else:   # omega wolves
            # implement position update logic based on alpha position
            strength = random.uniform(0.5, 2)  # randomized strength "pull" towards alpha wolf

            # calculate distance and direction to alpha wolf
            dx_alpha = alpha_position[0] - self.x
            dy_alpha = alpha_position[1] - self.y
            distance_alpha = math.sqrt((dx_alpha ** 2 + dy_alpha ** 2))

            # update position to move towards alpha, dependent on strength variable
            if distance_alpha > 0:
                direction_x = int(dx_alpha / distance_alpha * strength * MOVEMENT_SPEED)
                direction_y = int(dy_alpha / distance_alpha * strength * MOVEMENT_SPEED)
                new_x = self.x + direction_x
                new_y = self.y + direction_y
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
        if self.is_valid((new_x, new_y), obstacles):
            self.x = new_x
            self.y = new_y
            self.path.append((self.x, self.y))
            self.temppath.append((self.x, self.y,))


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
        e_th = .8
        temp = 10
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
            dx = (goal[0] - agent_pos[0])
            dy = goal[1] - agent_pos[1]
            angle = math.atan2(dy, dx)  # Not required, but may be useful later
            return k_att * dx, k_att * dy

        # Compute the repulsive force between the agent and an obstacle
        def repulsive_force(agent_pos, obstacle_pos, obstacle_radius):
            k_rep = 10000000  # Repulsive force gain
            # k_rep = 500
            buffer = SEARCH_RADIUS * (1/3)  # was 1/5
            p0 = obstacle_radius + buffer  # Influence radius of F_rep
            obst_dist_x = (agent_pos[0] - obstacle_pos[0])
            obst_dist_y = (agent_pos[1] - obstacle_pos[1])
            dist = (obst_dist_x ** 2 + obst_dist_y ** 2)**0.5  # Dist btwn UAV and obstacle
            # dist = (obst_dist_x ** 2 + obst_dist_y ** 2) ** 0.5 - obstacle_radius  # Dist btwn UAV and obstacle
            if dist <= p0:
                # x_rep = k_rep * ((1 / obst_dist_x - (1 / p0)) * (1 / obst_dist_x) ** 2)
                # y_rep = k_rep * ((1 / obst_dist_y - (1 / p0)) * (1 / obst_dist_y) ** 2)
                x_rep = k_rep**3 * ((1 / dist - 1 / p0) * (obst_dist_x / dist) * (1 / dist) ** 2)
                y_rep = k_rep**3 * ((1 / dist - 1 / p0) * (obst_dist_y / dist) * (1 / dist) ** 2)

                print("repulsed")
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
                buffer = 15
                p0 = AGENT_RADIUS*2 + buffer  # Influence radius of F_rep
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
            print("att_fx:", force_x, "att_fy:", force_y)

            for obstacle in obstacles:
                rep_force_x, rep_force_y = repulsive_force(agent_pos, (obstacle.x, obstacle.y), obstacle.radius)
                force_x += rep_force_x
                force_y += rep_force_y
            print("tot_fx:", force_x, "tot_fy:", force_y)

            for agent in self.apf_list_of_agents:
                rep_force_x, rep_force_y = inter_agent_force(agent, agent_pos)
                force_x += rep_force_x
                force_y += rep_force_y

            return (force_x, force_y)

        # Move the agent towards the goal position based on the total force
        def move_towards(agent_pos, obstacles):
            force_x, force_y = total_force(agent_pos, obstacles)
            force_magnitude = math.sqrt(force_x ** 2 + force_y ** 2)
            force_x /= force_magnitude
            force_y /= force_magnitude
            force_x *= MOVEMENT_SPEED
            force_y *= MOVEMENT_SPEED

            new_pos_x = agent_pos[0] + force_x
            new_pos_y = agent_pos[1] + force_y


            # Check for collision with obstacles and adjust the new position accordingly
            # for obstacle in obstacles:
            #     dx = new_pos_x - obstacle.x
            #     dy = new_pos_y - obstacle.y
            #     distance = math.sqrt(dx ** 2 + dy ** 2)
            #     if distance <= AGENT_RADIUS + obstacle.radius:
            #         angle = math.atan2(dy, dx)
            #         new_pos_x = obstacle.x + (AGENT_RADIUS + obstacle.radius) * math.cos(angle)
            #         new_pos_y = obstacle.y + (AGENT_RADIUS + obstacle.radius) * math.sin(angle)
            #         break

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
                    next_pos = move_towards((agent.x, agent.y), self.obstacles)
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
        disp_paths = []
        new_gradient = []
        past_gradient = []
        compare_list = []
        episode = 0
        agents_pos = []
        iteration = 15000
        for agent in self.list_of_agents:
            new_gradient.append(0)
            past_gradient.append(0)

        for episode in range(0, iteration):
            # while not agent.goal_test():
            all_agents_reached_goal = True

            list_agents_triggered = []
            for agent in self.list_of_agents:
                if agent.position != agent.goal:
                    list_agents_triggered.append(agent)

            for agent in self.list_of_agents:
                if agent.position != agent.goal:
                    all_agents_reached_goal = False

                # agent.feed_pos(agents_pos)
                path, length, reward = agent.action()

                # update rewards and gradients
                agent.reward_mem.append(reward + y * agent.next_reward())  # this is q-value stored in agent
                if episode > 1:
                    past_gradient[agent.agent_id - 1] = (agent.reward_mem[episode - 2] - agent.reward_mem[
                        episode - 1]) / 2
                    new_gradient[agent.agent_id - 1] = (agent.reward_mem[episode - 1] - agent.reward_mem[episode]) / 2
                # agents_pos.append(agent.get_pos())

            if all_agents_reached_goal:
                break

            if episode > 1:

                compare = statistics.mean(new_gradient) - statistics.mean(past_gradient)
                compare_list.append(compare)

                # maybe multiply q-gradient by the parameters?
                if abs(compare) < (abs(max(compare_list))*(1/(10 + (len(list_agents_triggered)*5)))):
                    for mad_agent in self.list_of_agents:
                        e_update, temp_update = mad_agent.e_th, mad_agent.temp
                        if mad_agent.e_th > 0.1:
                            e_update = mad_agent.e_th - 0.1  # gets smaller -> more risky
                        if mad_agent.temp < 60:
                            temp_update = (mad_agent.temp + 1.2)  # gets larger -> more risky
                        mad_agent.update_critic(e_update, temp_update)
                elif compare <= 0:
                    # print("updating params, compare: " + str(compare))
                    for mad_agent in self.list_of_agents:
                        e_update, temp_update = mad_agent.e_th, mad_agent.temp
                        if mad_agent.e_th > 0.02:
                            e_update = mad_agent.e_th - 0.02  # gets smaller -> more risky
                        if mad_agent.temp < 60:
                            temp_update = (mad_agent.temp + 0.5)  # gets larger -> more risky
                        mad_agent.update_critic(e_update, temp_update)
                else:
                    # print("done good, compare: " + str(compare))
                    for mad_agent in self.list_of_agents:
                        e_update, temp_update = mad_agent.e_th, mad_agent.temp
                        if mad_agent.e_th < 0.9:
                            e_update = mad_agent.e_th + 0.1  # gets larger -> less risky
                        if mad_agent.temp > 10:
                            temp_update = mad_agent.temp - 10  # gets smaller -> less risky
                        elif mad_agent.temp > 1:
                            temp_update = mad_agent.temp*0.4
                        mad_agent.update_critic(e_update, temp_update)

            # episode += 1 # use only when implementing while statement

        for mad_agent in self.list_of_agents:
            paths.append(mad_agent.long_mem)
            disp_paths.append(mad_agent.disp_path)

        print("compare_list:")
        print(compare_list)
        print(max(compare_list))
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

        def update_hierarchy(wolfFitnessDict):
            # finds minimum fitness value in the dictionary, assumes global minimum
            alpha_wolf_id = min(wolfFitnessDict, key=wolfFitnessDict.get)
            # alpha_position = self.list_of_agents[alpha_wolf_id].x, self.list_of_agents[alpha_wolf_id].y
            for wolf in self.list_of_agents:
                if wolf.agent_id == alpha_wolf_id:
                    wolf.make_alpha()
                    alpha_position = (wolf.x, wolf.y)
                    wolf.make_commensal()
                else:
                    wolf.make_omega()
                    wolf.make_commensal()
            return alpha_position

        paths = []
        temppath = []

        # MAIN LOOP OF GWO ALGORITHM
        wolfFitnessDict = {}
        i = random.randint(0, len(self.list_of_agents)-1)
        # use first wolf as preliminary alpha
        for wolf in self.list_of_agents:
            wolf.path.append(wolf.start)
            wolf.temppath.append(wolf.start)
            wolf.update_position((self.list_of_agents[i].x, self.list_of_agents[i].y), goal, self.obstacles)
            wolf.explore(goal, self.obstacles)
            wolf.update_fitness(goal, self.obstacles)
            wolfFitnessDict[wolf.agent_id] = wolf.fitness   # save new fitness values
        alpha_position = update_hierarchy(wolfFitnessDict)

        # cycle thru episodes to find iterative alphas
        e = 0
        all_agents_at_target = False
        while e < max_iterations and not all_agents_at_target:
            e += 1
            all_agents_at_target = True
            for wolf in self.list_of_agents:
                if (wolf.x, wolf.y) != goal:
                    all_agents_at_target = False
                wolf.update_position(alpha_position, goal, self.obstacles)
                wolf.explore(goal, self.obstacles)
                wolf.update_fitness(goal, self.obstacles)
                wolfFitnessDict[wolf.agent_id] = wolf.fitness  # save new fitness values
            alpha_position = update_hierarchy(wolfFitnessDict)

        # reconstruct path
        for wolf in self.list_of_agents:
            paths.append(wolf.path)
            temppath.append(wolf.temppath)
            paths.reverse()
            temppath.reverse()
        return temppath

def path_length_diagnostics(paths, goal, obstacles):
    total_path_length = 0
    incomplete_paths = 0
    complete_paths = 0

    for path in paths:
        temp_length = 0
        path_complete = False
        prev_point = path[0]
        for point in path:
            temp_length += 1
            if point == goal:
                path_complete = True
                break
            for obstacle in obstacles:
                dx = point[0] - obstacle.x
                dy = point[1] - obstacle.y
                distance = math.sqrt(dx ** 2 + dy ** 2)
                if distance <= AGENT_RADIUS + obstacle.radius:
                    path_complete = False
                    break

            dx = point[0] - prev_point[0]
            dy = point[1] - prev_point[1]
            distance = math.sqrt(dx ** 2 + dy ** 2)
            if distance > 5:
                path_complete = False
                break
            prev_point = point

        if path_complete:
            total_path_length += temp_length
            complete_paths += 1
        else:
            incomplete_paths += 1

    if complete_paths > 0:
        average_path_length = float(total_path_length)/complete_paths
    else:
        average_path_length = 0
    completion_percentage = float(complete_paths)/(complete_paths + incomplete_paths)

    return average_path_length, completion_percentage

def run_scenario_multi_agent_diagnostics(lo_obstacles, lo_agents, goal_in, algorithm_type):
    for agents in lo_agents:
        environment_complexities = []
        i = 0
        elapsed_times = []
        average_lengths = []
        completion_percentages = []

        for obstacles in lo_obstacles:
            i += 1
            # input variables
            goal_position = goal_in
            paths = []

            # time the length of the algorithm for results
            start_time = time.time()

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
            else:
                print("invalid algorithm")

            end_time = time.time()
            elapsed_time = end_time - start_time
            average_length, completion_percentage = path_length_diagnostics(paths, goal_position, obstacles)

            # Store the data in the respective lists
            elapsed_times.append(elapsed_time)
            average_lengths.append(average_length)
            completion_percentages.append(completion_percentage)

            # Store the complexity value for the current environment
            environment_complexities.append("obstacle difficulty: " + str(i))
            print("datapoint complete")

        agents_string = str(len(agents))
        # Plot the data for the current agent
        plt.figure()
        plt.suptitle("Algorithm: " + algorithm_type + "\nAmount of agents: " + agents_string)
        plt.subplot(311)
        plt.plot(environment_complexities, elapsed_times, marker='o')
        plt.xlabel('Environment Complexity')
        plt.ylabel('Elapsed Time')

        plt.subplot(312)
        plt.plot(environment_complexities, average_lengths, marker='o')
        plt.xlabel('Environment Complexity')
        plt.ylabel('Average Length')

        plt.subplot(313)
        plt.plot(environment_complexities, completion_percentages, marker='o')
        plt.xlabel('Environment Complexity')
        plt.ylabel('Completion Percentage')

        plt.tight_layout()
        plt.savefig("Algorithm_" + algorithm_type + "agents_" + agents_string + ".png")  # Save the plot as an image file
        plt.close()  # Close the figure to release resources

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

    # time the length of the algorithm for results
    start_time = time.time()

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
        paths = algorithm.simplified_gwo_search(goal_position, max_iterations=4000)
    elif algorithm_type == "MAD":
        mad_agents = create_mad_agents_from_agents(agents, goal_position, obstacles)
        mad_algorithm = Algorithm(mad_agents, obstacles)
        paths = mad_algorithm.mad_search()
        # print(paths)
    else:
        print("invalid algorithm")

    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"The algorithm block took {elapsed_time} seconds to execute.")

    average_length, completion_percentage = path_length_diagnostics(paths, goal_position, obstacles)
    print(f"The average path length of the swarm was {average_length} points")
    print(f"The percentage of robots that made it to the goal was {completion_percentage * 100}%")

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
                if (agent.x, agent.y) == goal_position and not agent.disp_goal_reached:
                    print("agent reached goal")
                    agent.disp_goal_reached = True


        # Clear the screen
        screen.fill(WHITE)

        # Draw the search radius
        if algorithm_type == "APF":
            for agent in agents:
                pygame.draw.circle(screen, RED, (agent.x, agent.y), SEARCH_RADIUS)

        # Draw the agent
        # Draw the start and goal positions
        if algorithm_type == "MAD":
            # multiplier = 1.75
            # threshold = 50
            # goal_r0 = AGENT_RADIUS * 4  # assumes agent radius is same as goal collision radius
            #
            # # goal_r1 = goal_r0 * multiplier  # 80
            # # goal_r2 = goal_r1 * multiplier  # 160
            # # goal_r3 = goal_r2 * multiplier  # 320
            # # goal_r4 = goal_r3 * multiplier  # 640
            # # goal_r5 = goal_r4 * multiplier
            # #
            #
            # goal_r1 = goal_r0+threshold
            # goal_r2 = goal_r1+threshold
            # goal_r3 = goal_r2+threshold
            # goal_r4 = goal_r3+threshold
            # goal_r5 = goal_r4+threshold
            #
            # pygame.draw.circle(screen, (165,42,42), goal_position, goal_r5)
            # pygame.draw.circle(screen, (255, 255, 0), goal_position, goal_r4)
            # pygame.draw.circle(screen, (230,230,250), goal_position, goal_r3)
            # pygame.draw.circle(screen, (255, 192, 203), goal_position, goal_r2)
            # pygame.draw.circle(screen, GREEN, goal_position, goal_r1)
            # pygame.draw.circle(screen, RED, goal_position, goal_r0)
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
            # print(path)
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
