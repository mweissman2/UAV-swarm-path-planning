from Agent import *


def attractive_force(agent, goal):
    k_att = 5.0  # Attractive force gain
    k_att2 = 3.0  # Velocity gain
    dx = goal[0] - agent.x
    dy = goal[1] - agent.y

    return k_att * dx + k_att2 * MOVEMENT_SPEED, k_att * dy + k_att2 * MOVEMENT_SPEED


# Compute the repulsive force between the agent and an obstacle
def repulsive_force(agent, obstacle):
    k_rep = 150
    buffer = SEARCH_RADIUS * (1 / 2)  # was 1/5
    p0 = obstacle.radius + buffer  # Influence radius of F_rep
    obst_dist_x = (agent.x - obstacle.x)
    obst_dist_y = (agent.y - obstacle.y)
    dist = (obst_dist_x ** 2 + obst_dist_y ** 2) ** 0.5 - obstacle.radius  # Dist btwn UAV and obstacle
    if dist <= p0:
        x_rep = k_rep ** 3 * ((1 / dist - 1 / p0) * (obst_dist_x / dist) * (1 / dist) ** 2)
        y_rep = k_rep ** 3 * ((1 / dist - 1 / p0) * (obst_dist_y / dist) * (1 / dist) ** 2)

        return x_rep, y_rep
    else:
        return (0.0, 0.0)


def inter_agent_force(agent, agent_pos, goal):
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
                x_rep = k_rep ** 3 * ((1 / dist - 1 / p0) * (inter_dist_x / dist) * (1 / dist) ** 2)
            else:
                x_rep = 0
            if inter_dist_y > 0:
                y_rep = k_rep ** 3 * ((1 / dist - 1 / p0) * (inter_dist_y / dist) * (1 / dist) ** 2)
            else:
                y_rep = 0
            return x_rep, y_rep
        else:
            return 0.0, 0.0
    else:
        return 0.0, 0.0

# Compute the total force acting on the agent at its current position
def total_force(agent, obstacles, goal, agent_list):
    agent_pos = (agent.x, agent.y)
    force_x, force_y = attractive_force(agent, goal)
    # print("att_fx:", force_x, "att_fy:", force_y)

    for obstacle in obstacles:
        rep_force_x, rep_force_y = repulsive_force(agent, obstacle)
        force_x += rep_force_x
        force_y += rep_force_y
    # print("tot_fx:", force_x, "tot_fy:", force_y)

    for agent in agent_list:
        rep_force_x, rep_force_y = inter_agent_force(agent, agent_pos, goal)
        force_x += rep_force_x
        force_y += rep_force_y

    return force_x, force_y
