import random
import pygame
import heapq
import math

from matplotlib import pyplot as plt

from maddpg_code import *
from hsgwo_code import *
import statistics
import numpy
import imageio
import time
import csv
import pandas as pd



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



class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

    def draw(self, screen):
        pygame.draw.circle(screen, BLACK, (self.x, self.y), self.radius)

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

class Algorithm:
    def __init__(self, list_of_agents, obstacles):
        self.list_of_agents = list_of_agents
        self.apf_list_of_agents = list_of_agents.copy()
        self.obstacles = obstacles
        self.force_record = []
        self.angle_record = [0.0]


    # A* search ______________________________________________________________
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

    # APF search
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
                # print("jitter", episode)
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
    # MADDPG search
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

        # print("compare_list:")
        # print(compare_list)
        # print(max(compare_list))
        msg = f'Sim completed at episode {episode}'  # only for while statement
        print(msg)
        return disp_paths

    # HSGWO search
    def hsgwo_msos(self, goal, max_iterations):
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

def save_to_csv(data_dict, file_name):
    # Create an Excel writer object
    writer = pd.ExcelWriter(file_name, engine='xlsxwriter')

    # Iterate through the dictionary and save each tab to the Excel writer
    for sheet_name, sheet_data in data_dict.items():
        # Save the DataFrame to the Excel writer with the sheet_name
        sheet_data.to_excel(writer, sheet_name=sheet_name, index=False)

    # Save the Excel writer to the file
    writer.save()

    print(f"Data saved to '{file_name}' successfully!")


def path_length_diagnostics(paths, goal, obstacles):
    total_path_length = 0
    incomplete_paths = 0
    complete_paths = 0

    for path in paths:
        temp_length = 0
        path_complete = True
        prev_point = path[0]
        for point in path:
            if not path_complete:
                break
            for obstacle in obstacles:
                dx = point[0] - obstacle.x
                dy = point[1] - obstacle.y
                distance = math.sqrt(dx ** 2 + dy ** 2)
                if distance <= AGENT_RADIUS + obstacle.radius:
                    path_complete = False

            dx = point[0] - prev_point[0]
            dy = point[1] - prev_point[1]
            distance = math.sqrt(dx ** 2 + dy ** 2)
            temp_length += distance
            if distance > MOVEMENT_SPEED*10:
                path_complete = False
            prev_point = point

        if path[-1] != goal:
            path_complete = False

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
    print("completion percentage: ")
    print(completion_percentage)

    return average_path_length, completion_percentage




def run_scenario_multi_agent_diagnostics(lo_obstacles, algorithm_type):

    col_names = ['agent_list', 'num of agents', 'obstacle difficulty', 'time', 'path length', 'completion %']
    data_dict = {}
    sheet = pd.DataFrame(data_dict)

    i = 0



    for obstacles in lo_obstacles:
        i += 1
        j = 0
        elapsed_times = []
        average_lengths = []
        completion_percentages = []
        new_dict = {}

        # stupid code for defining diagnostic stuff
        agents_center_line_10 = create_agent_line(100, 300, 10)
        agents_center_line_5 = create_agent_line(100, 300, 5)
        agents_center_line_3 = create_agent_line(100, 300, 3)
        diagnostics_agents = [agents_center_line_3, agents_center_line_5, agents_center_line_10]

        wolves_center_line_10 = create_wolf_population(100, 300, 10)
        wolves_center_line_5 = create_wolf_population(100, 300, 5)
        wolves_center_line_3 = create_wolf_population(100, 300, 3)
        diagnostics_wolves = [wolves_center_line_3, wolves_center_line_5, wolves_center_line_10]

        if algorithm_type == "GWO":
            diagnostics_line = diagnostics_wolves
        else:
            diagnostics_line = diagnostics_agents

        for agents in diagnostics_line:
            j += 1
            # input variables
            goal_position = (700, int(random.uniform(150, 550)))
            paths = []

            agent_list_name = "Agent " + str(j)
            agent_len = len(agents)

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
                paths = algorithm.hsgwo_msos(goal_position, max_iterations=1000)
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
            print("datapoint complete")

            new_dict[col_names[0]] = agent_list_name
            new_dict[col_names[1]] = agent_len
            new_dict[col_names[2]] = i
            new_dict[col_names[3]] = elapsed_time
            new_dict[col_names[4]] = average_length
            new_dict[col_names[5]] = completion_percentage

            # Append the data point using append method
            new_data_point = pd.DataFrame(new_dict, index=[i + j])
            sheet = pd.concat([sheet, new_data_point], ignore_index=True)

    print(sheet.to_string())
    return sheet

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
        # print(paths)
    elif algorithm_type == "APF":
        algorithm = Algorithm(agents, obstacles)
        paths = algorithm.apf_search(goal_position)
    elif algorithm_type == "GWO":
        algorithm = Algorithm(agents, obstacles)
        paths = algorithm.hsgwo_msos(goal_position, max_iterations=4000)
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
