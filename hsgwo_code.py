
from MultiAgentEnvironment import*


# Constants
WIDTH = 800  # Width of the simulation window
HEIGHT = 608  # Height of the simulation window
AGENT_RADIUS = 5  # Radius of the agent
OBSTACLE_RADIUS = 30  # Radius of the obstacles
MOVEMENT_SPEED = 3  # Movement speed of the agent

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