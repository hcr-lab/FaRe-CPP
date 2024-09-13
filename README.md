# FaRe-CPP: Fast Revisit Coverage path Planning for Autonomous Mobile Patrol Robots Using Long-range Vision Information.



## Steps on how to use the FaRe cpp algorithm

1. download environment maps 
- [AWS RoboMaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world)
- [Dynamic Logistics Warehouse](https://github.com/belal-ibrahim/dynamic_logistics_warehouse)

2. clone the project using the command 'git clone https://github.com/Srinikstudent/FaRe_CPP.git'. change paths in config.py to the location where you saved pgm and yamp files, and change parameters based on sensor capabilities. 

3. Install the required dependencies:

    ```bash
    pip install -r requirements.txt
    ```

4. Execute the surveillance script:

    ```bash
    python Cpp/Surveillance.py
    ```

all the results will be saved in the output directory

## For Online Navigation(Patrolling)

##Prerequisites
1. ros-noetic and Gazebo simulation installed and working
2. Follow the procedure  on how to set up the  environment from here - [AWS RoboMaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world)  or - [Dynamic Logistics Warehouse](https://github.com/belal-ibrahim/dynamic_logistics_warehouse) or you can use environment of your choice.
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
   source /opt/ros/foxy/setup.bash
   ```
5. Then, spawn TurtleBot 3 in the world:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```
6. before execute this step make sure you run the FaRe algorithm first on the same environment map with all the optimized waypoints saved.
   ```bash
   Python3 PatrolSim.py
   ```
   
   
   
   
