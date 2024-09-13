# FaRe-CPP: Fast Revisit Coverage Path Planning for Autonomous Mobile Patrol Robots Using Long-Range Vision Information

## Overview
**FaRe-CPP** is an algorithm designed for efficient revisit coverage path planning for autonomous mobile patrol robots using long-range vision information. This repository provides the tools for generating optimized patrol paths and executing them in simulation environments such as **AWS RoboMaker** and the **Dynamic Logistics Warehouse**.

## Getting Started

### 1. Download Simulation Environment Occupancy grid Maps

Before running the FaRe-CPP algorithm, download and set up one of the following simulation environments:
- [AWS RoboMaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world)
- [Dynamic Logistics Warehouse](https://github.com/belal-ibrahim/dynamic_logistics_warehouse)

### 2. Clone the FaRe-CPP Repository

Clone this repository using the following command:

```bash
git clone https://github.com/Srinikstudent/FaRe_CPP.git
```
After cloning, update the file paths in config.py to point to where your environment map files (.pgm and .yaml) are stored. Adjust the parameters to match your robot's sensor capabilities.

3. Install the required dependencies:

    ```bash
    pip install -r requirements.txt
    ```

4. Execute the surveillance script:

    ```bash
    python Cpp/Surveillance.py
    ```

This will process the environment Occupancy grid map and save the waypoints in the output directory. These waypoints will be used for navigation in the simulation.

## Online Navigation(Patrolling)
To execute patrols in the simulation, please make sure ROS and Gazebo are installed and follow these steps.
## Steps for Online Navigation
1. ros-noetic and Gazebo simulation installed and working
2. Follow the procedure  on how to set up the simulation environment from here - [AWS RoboMaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world)  or - [Dynamic Logistics Warehouse](https://github.com/belal-ibrahim/dynamic_logistics_warehouse) or you can use environment of your choice.
3. launch world 
   ```bash
   roslaunch aws_robomaker_small_house_world view_small_house.launch
   ```
   or
   ```bash
   roslaunch dynamic_logistics_warehouse logistics_warehouse.launch
   ```
4. Launch TurtleBot 3 in the world: In a second terminal, make sure you have set the TurtleBot 3 model and source the environment again
   ```bash
   export TURTLEBOT3_MODEL=burger  
   source /opt/ros/noetic/setup.bash
   ```
5. Then, spawn TurtleBot 3 in the world:
   ```bash
   roslaunch turtlebot3_gazebo turtlebot3_world.launch
   ```
6. Run the Patrol Simulation: before executing this step, ensure you first run the FaRe algorithm on the same environment map and see if the optimized waypoints are saved.
   ```bash
   Python3 Cpp/PatrolSim.py
   ```
   This will execute the patrol path in the simulation, using the waypoints generated by the FaRe-CPP algorithm.
   
   
   
