# RBE 550 Wildfire Simulation

![Valet Flow Chart](docs/imgs/RBE550_Valet_Flow_Chart.png)
![A* Pathfinding](videos/astar.gif)
![PRM Pathfinding](PRM-2.gif)

Welcome to the RBE550 Wildfire assignment repository. This simulation project involves an automated Mercedes fire truck with Ackermann steering in a high-speed environment. In this simulation, time is accelerated, where one second represents 5 seconds or more of real-time. The objective is to combat an arsonist who periodically ignites fires in the environment, leading to the rapid spread of flames to nearby trees.

## Running the Simulation

You can run this simulation with either a local planner alone or a combination of a local planner and a global planner.

### Global Planner (PRM)

The global planner utilizes a Probabilistic Road Map (PRM) generated at the start of the program. It generates thousands of random positions for the firetruck with random orientations and checks for collisions. Collision-free positions are marked as potential nodes in the PRM. These nodes are organized in a KD-tree using `sklearn`. When the global planner needs to navigate from the current location to a fire, it finds the closest node to the firetruck and the closest node to the fire. The global planner then employs A* search, using Euclidean distance as the heuristic, to navigate between these nodes.

### Local Planner (A*)

The local planner performs path planning between desired nodes in the PRM or between the start and goal locations, depending on whether the global planner is used. The local planner is also based on A* search, but it explores the continuous space by utilizing kinematic primitives derived from the Ackermann-style chassis of the robot. This ensures that all planned movements are technically feasible within the defined constraints of the platform.

## Report

For a detailed explanation of the assignment and the algorithms used, please refer to the [report.pdf](RBE595_Wildfire.pdf).

## Usage

To run the simulation, use the following command:

```bash
python run.py
