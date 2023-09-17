

from matplotlib import pyplot as plt
# from Agent import *  # Inherits from Agent automatically through other scripts
from maddpg_code import MADDPG_agent
from hsgwo_code import *
from improvedAPF_code import *
import statistics
import numpy
import imageio
import time
import pandas as pd





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

        # Move the agent towards the goal position based on the total force
        def move_towards(episode, agent, obstacles):
            force_x, force_y = total_force(agent, obstacles, goal, self.apf_list_of_agents)
            self.force_record.append((force_x, force_y))
            angle = math.atan2(force_y, force_x)
            self.angle_record.append(angle)
            force_magnitude = math.sqrt(force_x ** 2 + force_y ** 2)
            force_x /= force_magnitude
            force_y /= force_magnitude

            f = 1.03  # jitter coeff
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

            new_pos_x = agent.x + force_x
            new_pos_y = agent.y + force_y

            return (new_pos_x, new_pos_y)

        paths = []
        num_iterations = 1000

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
                    next_pos = move_towards(episode, agent, self.obstacles)
                    agent.temp_path.append(next_pos)
                    agent.path.append(next_pos)

                agent.x, agent.y = next_pos

            if all_goal:
                print("Episode:", episode)
                num_iterations = episode
                break

        for agent in self.list_of_agents:
            paths.append(agent.temp_path)

        return paths,num_iterations
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
        total_reward = 0
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
                total_reward += reward

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

        avg_total_reward = float(total_reward) / len(self.list_of_agents)
        # print("compare_list:")
        # print(compare_list)
        # print(max(compare_list))
        msg = f'Sim completed at episode {episode}'  # only for while statement
        print(msg)
        return disp_paths, episode, avg_total_reward

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
        cost_list = []
        while e < max_iterations and not all_agents_at_target:
            e += 1
            all_agents_at_target = True
            temp_cost = 0
            for wolf in self.list_of_agents:
                if (wolf.x, wolf.y) != goal:
                    all_agents_at_target = False
                wolf.update_position(alpha_position, goal, self.obstacles)
                wolf.explore(goal, self.obstacles)
                wolf.update_fitness(goal, self.obstacles)
                temp_cost += wolf.update_fitness(goal, self.obstacles)
                wolfFitnessDict[wolf.agent_id] = wolf.fitness  # save new fitness values
            alpha_position = update_hierarchy(wolfFitnessDict)
            cost_list.append(float(temp_cost) / len(self.list_of_agents))

        # reconstruct path
        for wolf in self.list_of_agents:
            paths.append(wolf.path)
            temppath.append(wolf.temppath)
            paths.reverse()
            temppath.reverse()
        total_cost_per_agent = sum(cost_list)
        return temppath, e, total_cost_per_agent
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

    col_names = ['agent_list', 'num of agents', 'obstacle difficulty', 'time', 'path length', 'completion %',
                 'iterations used']
    if algorithm_type == 'GWO':
        col_names.append('total cost per agent')
    if algorithm_type == 'MAD':
        col_names.append('total reward per agent')
    data_dict = {}
    sheet = pd.DataFrame(data_dict)

    i = 0

    for obstacles in lo_obstacles:
        i += 1
        j = 0

        # Defining diagnostics
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
                iterations = len(paths[0])
                print(paths)
            elif algorithm_type == "APF":
                algorithm = Algorithm(agents, obstacles)
                paths, iterations = algorithm.apf_search(goal_position)
            elif algorithm_type == "GWO":
                algorithm = Algorithm(agents, obstacles)
                paths, iterations, cost_per_agent = algorithm.hsgwo_msos(goal_position, max_iterations=1000)
            elif algorithm_type == "MAD":
                mad_agents = create_mad_agents_from_agents(agents, goal_position, obstacles)
                mad_algorithm = Algorithm(mad_agents, obstacles)
                paths,iterations, avg_total_reward = mad_algorithm.mad_search()
            else:
                print("invalid algorithm")
                iterations = 0

            end_time = time.time()
            elapsed_time = end_time - start_time
            average_length, completion_percentage = path_length_diagnostics(paths, goal_position, obstacles)

            # Storing information from simulations
            new_dict = {}
            new_dict[col_names[0]] = [agent_list_name]
            new_dict[col_names[1]] = [agent_len]
            new_dict[col_names[2]] = [i]
            new_dict[col_names[3]] = [elapsed_time]
            new_dict[col_names[4]] = [average_length]
            new_dict[col_names[5]] = [completion_percentage]
            new_dict[col_names[6]] = [iterations]
            if algorithm_type == 'GWO':
                new_dict[col_names[7]] = [cost_per_agent]
            if algorithm_type == 'MAD':
                new_dict[col_names[7]] = [avg_total_reward]
            print("datapoint complete")

            # Append the data point using append method
            new_data_point = pd.DataFrame(new_dict, index=None)
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
        iterations = len(paths[0])
        # print(paths)
    elif algorithm_type == "APF":
        algorithm = Algorithm(agents, obstacles)
        paths, iterations = algorithm.apf_search(goal_position)
    elif algorithm_type == "GWO":
        algorithm = Algorithm(agents, obstacles)
        paths, iterations, cost_per_agent = algorithm.hsgwo_msos(goal_position, max_iterations=4000)
    elif algorithm_type == "MAD":
        mad_agents = create_mad_agents_from_agents(agents, goal_position, obstacles)
        mad_algorithm = Algorithm(mad_agents, obstacles)
        paths, iterations, avg_total_reward = mad_algorithm.mad_search()
        # print(paths)
    else:
        print("invalid algorithm")
        iterations = 0

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
