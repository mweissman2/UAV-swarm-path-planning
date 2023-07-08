import SingleAgentEnvironment
import MultiAgentEnvironment
import random

# Define Constant Goals, Starts, and Obstacles
OBSTACLE_RADIUS = 30  # Radius of the obstacles
goal_1 = (700, int(random.uniform(150, 550)))
goal_2 = (700, 500)
agent_1 = SingleAgentEnvironment.Agent(100, 100)
agents_1 = [
    MultiAgentEnvironment.Agent(1, 100, 100),
    MultiAgentEnvironment.Agent(2, 100, 140),
    MultiAgentEnvironment.Agent(3, 100, 170)
]
agents_random = MultiAgentEnvironment.create_random_agents(80, 140, 100, 500, 1)
agents_rand_line = MultiAgentEnvironment.create_agent_line(100, int(random.uniform(100, 550)), 5)

list_of_algos = ["A Star", "APF", "MAD", "GWO"]

obstacles_1 = [
    SingleAgentEnvironment.Obstacle(300, 200, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 200, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(500, 200, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(600, 200, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(300, 400, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 400, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(500, 400, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(600, 400, OBSTACLE_RADIUS),
]

obstacles_2 = [
    SingleAgentEnvironment.Obstacle(200, 300, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 300, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(600, 300, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 100, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 200, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 400, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 500, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(300, 200, 40),
    SingleAgentEnvironment.Obstacle(300, 400, 60),
    SingleAgentEnvironment.Obstacle(500, 200, 60),
    SingleAgentEnvironment.Obstacle(500, 400, 40),
    SingleAgentEnvironment.Obstacle(600, 200, 30),
    SingleAgentEnvironment.Obstacle(600, 400, 30),
]

obstacles_3 = [
    SingleAgentEnvironment.Obstacle(200, 100, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(600, 100, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 300, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(600, 300, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(200, 500, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 500, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(600, 500, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(300, 200, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(500, 200, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(300, 400, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 400, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 200, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 100, OBSTACLE_RADIUS),
    # SingleAgentEnvironment.Obstacle(100, 300, 60),
    SingleAgentEnvironment.Obstacle(525, 375, 100),
    # SingleAgentEnvironment.Obstacle(700, 300, 10),
    SingleAgentEnvironment.Obstacle(400, 50, 70),
    SingleAgentEnvironment.Obstacle(400, 550, 80),
    SingleAgentEnvironment.Obstacle(250, 350, 60),
    SingleAgentEnvironment.Obstacle(300, 100, 40),
    SingleAgentEnvironment.Obstacle(500, 100, 60),
    SingleAgentEnvironment.Obstacle(300, 500, 20),
    SingleAgentEnvironment.Obstacle(500, 500, 40),
]

obstacles_4 = SingleAgentEnvironment.create_obstacle_objects(200, 600, 50, 550, 10, 50, 15)

obstacles_array = [obstacles_1, obstacles_2, obstacles_3, obstacles_4]


def main():
    algo = ""
    while algo not in list_of_algos:
        algo = input("\nWhat algorithm would you like to use?\n")
        if algo not in list_of_algos:
            print(list_of_algos)
            print("Not in list of algorithms, choose from the list above")

    obstacles_to_use = 0
    while obstacles_to_use > len(obstacles_array) or obstacles_to_use < 1:
        obstacles_to_use = int(input("\nWhich Obstacle Set would you like to use? (options: 1, 2, 3, 4)\n"))

    obstacles = obstacles_array[obstacles_to_use - 1]

    num_rand_agents = 0
    while num_rand_agents > 15 or num_rand_agents < 1:
        num_rand_agents = int(input("\nHow many random agents would you like to generate?\n"))

    if algo != "GWO":
        agents = MultiAgentEnvironment.create_agent_line(100, int(random.uniform(100, 600)), num_rand_agents)
    else:
        agents = MultiAgentEnvironment.create_wolf_population(100, int(random.uniform(100, 600)), num_rand_agents)


    MultiAgentEnvironment.run_scenario_multi_agent(obstacles, agents, goal_1, algo)
    # SingleAgentEnvironment.run_scenario_single_agent(obstacles, agent_1, goal_1, "A Star")


if __name__ == "__main__":
    main()
