# FaRe-CPP: Fast Revisit Coverage path Planning for Autonomous Mobile Patrol Robots Using Long-range Vision Information.

Steps on how to use the algorithm

  

## Download house or warehouse world maps

- [AWS RoboMaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world)
- [Dynamic Logistics Warehouse](https://github.com/belal-ibrahim/dynamic_logistics_warehouse)

clone the project using command 'git clone https://github.com/Srinikstudent/FaRe_CPP.git'. change paths in config.py to the location where you saved pgm and yamp files, and change parameters based on sensor capabilities. 

Then execute the following commands

## Installation and Execution

1. Install the required dependencies:

    ```bash
    pip install -r requirements.txt
    ```

2. Execute the surveillance script:

    ```bash
    python Cpp/Surveillance.py
    ```

all the results will be saved in the output directory

## for online navigation 

1. follow the procedure  on how to setup simulation environment from here - [AWS RoboMaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world) - [Dynamic Logistics Warehouse](https://github.com/belal-ibrahim/dynamic_logistics_warehouse)
