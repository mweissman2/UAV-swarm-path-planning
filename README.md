# Swarm Robotics Coordinated Path Planning with Obstacle Avoidance:<br> <i>A Simulated Study of State-of-the-Art Novel Optimization Methods</i>

![Simulation Project](./path/to/your/simulation/image.png)

This project focuses on swarm robotics and its application to multi-agent path planning for UAVs in unfamiliar environments with unknown static obstacles. Three state-of-the-art (SOTA) methods are evaluated: <i>Multi-Agent Deep Deterministic Policy Gradient (MADDPG), Hybrid Simplified Grey Wolf Optimization with Modified Symbiotic Organism Search (HSGWO-MSOS),</i> and <i>Improved Artificial Potential Field (APF)</i>. The A* Search algorithm was used as a baseline algorithm. The project aims to optimize path planning in a 2D simulated environment, considering execution time, complexity, optimality, convergence, completeness, and scalability. The project successfully implemented all three methods and developed an effective simulation environment for testing. The 2D simulation environment features three obstacle complexities, as well as a randomized obstacle scenario. Three swarm sizes (3-, 5-, and 10-agent swarms) were studied, and swarms with up to 15 agents can be simulated. Future work will focus on tuning algorithm parameters, exploring variations of the SOTA methods, and enhancing the realism of the environment, considering inter-agent collisions, moving obstacles, and 3D pathfinding.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Usage](#usage)
- [Results](#results)
- [Acknowledgments](#acknowledgments)


## Introduction
For the purposes of this project, the considered application is a swarm navigating to a known destination through an unfamiliar environment with unknown static obstacles. A real-world example could be a swarm of UAV traversing through an area with fixed enemy defenses to collect data/conduct surveillance at predetermined waypoints. The agents all start and end at relatively the same points, and move at the same time. The agents know how far their Euclidean distance is to their goal based on sensor data, such as GPS. However, they have not been provided nor have been developed to map the environment; they utilize LiDAR, cameras, or other sensors to detect obstacles such as trees and other agents within a limited search radius. Unlike some swarm applications (such as drone light shows), synchronized or coordinated trajectory planning is not a performance metric – agents independently search for their own best path to the goal, using real-time information from their swarm to guide or improve their search. 

This problem is deterministic because the environment and obstacles are predictable, without any random elements, and it can be assumed that the UAVs make decisions based on consistent sensor data. Many traditional and SOTA methods for swarm path planning, including those studied in this project, introduce stochastic elements to improve their optimization search. Although the algorithms may have some randomness, it doesn't alter the overall deterministic nature of the problem. While the agents' choices may vary across different episodes, the fundamental characteristic of the problem remains deterministic, as it has been defined in the limited scope of this project. Thus, this specific swarm path planning domain is a partially observable, deterministic, episodic, dynamic, continuous problem with multiple agents. 

## Features

- Development of 2D simulation environment:
  - Supports swarm sizes up to 15 agents
  - Features four environment variations with increasing obstacle complexities
  - Animates swarms + generated paths
- Implementation of optimziation methods:
  - A* Search (baseline)
  - Multi-Agent Deep Deterministic Policy Gradient (MADDPG)
  - Hybrid Simplified Grey Wolf Optimization with Modified Symbiotic Organism Search (HSGWO-MSOS)
  - Improved Artificial Potential Field (APF)
- Development of batch-scripting for method performance evaluation


## Usage

### Prerequisites

The project was developed in Python. The following dependencies are required:
- pygame, used to visualize the environment and animate the final trajectory,  
- heapq, used to implement priority queues in methods of the Algorithm class,
- numpy, used for more advanced numerical and matrix calculations,
- imageio, used for saving each simulation run as an mp4 video, and
- pandas, used for data collection and formatting.

### Running the Simulation

```bash
Use **RunSim.py**.

List of algorithms codes: A Star, MAD, GWO, APF

Environment scenarios: 1 (least complex), 2 (moderately complex), 3 (most complex), 4 (randomized)

Max number of agents per swarm: 15

``` 

## Results


## Acknowledgements
<div class="centered">
  
![Project Team](https://i.ibb.co/G53Zxpm/Screenshot-2023-07-29-at-10-19-10-PM.png)
<table>
  <tr>
    <td align="center">Wyatt Harris <a href="https://github.com/wyatt522"><img src="https://i.ibb.co/9t86L7m/github-mark.png" alt="Github Icon" width="20" height="20" /></a> <a href="mailto:wwharris@wpi.edu"><img src="https://cdn-icons-png.flaticon.com/512/3178/3178158.png" alt="Email Icon" width="20" height="20" /></a> </td>
    <td align="center">Sean Tseng <a href="https://github.com/CuriousBeluga"><img src="https://i.ibb.co/9t86L7m/github-mark.png" alt="Github Icon" width="20" height="20" /></a> <a href="mailto:sltseng@wpi.edu"><img src="https://cdn-icons-png.flaticon.com/512/3178/3178158.png" alt="Email Icon" width="20" height="20" /></a></td>
    <td align="center">Tabatha Viso <a href="https://github.com/tabathaviso"><img src="https://i.ibb.co/9t86L7m/github-mark.png" alt="Github Icon" width="20" height="20" /></a> <a href="mailto:tabatha.viso@gmail.com"><img src="https://cdn-icons-png.flaticon.com/512/3178/3178158.png" alt="Email Icon" width="20" height="20" /></a></td>
    <td align="center">Max Weissman <a href="https://github.com/mweissman2"><img src="https://i.ibb.co/9t86L7m/github-mark.png" alt="Github Icon" width="20" height="20" /></a> <a href="mailto:mweissman@wpi.edu"><img src="https://cdn-icons-png.flaticon.com/512/3178/3178158.png" alt="Email Icon" width="20" height="20" /></a></td>
  </tr>
</table>
</div>
This project was completed for the CS534 Artifical Intelligence graduate course taught by Prof. Dr. Ben Ngan at Worcester Polytechnic Institute. 

### Contributing

### Licensing

