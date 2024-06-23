# Maze Solver for GoPiGo3

Welcome to the Maze Solver for GoPiGo3 repository! This project was developed by our team composed of Girvana Hurreesing, Chavishka Jotee, Yassir Hoossan Buksh and Paavalen Lingachetti as part of the ROBO2 module. It features a maze-solving script designed to be used with the GoPiGo3 robot platform.

## Table of Contents

- [Project Overview](#project-overview)
- [Algorithm Explanation](#algorithm-explanation)
- [Installation](#installation)
  - [Prerequisites](#prerequisites)
  - [Setting Up the Catkin Workspace](#setting-up-the-catkin-workspace)
  - [Installing Dependencies](#installing-dependencies)
  - [Building the Package](#building-the-package)
- [Usage](#usage)
- [Note](#note)

## Project Overview

The Maze Solver for GoPiGo3 project consists of a ROS package that enables the GoPiGo3 robot to navigate and solve mazes autonomously. The robot uses three ultrasound sensors to detect and follow paths within the maze, making decisions at intersections to reach the goal.

## Algorithm Explanation

The maze-solving algorithm uses a combination of wall-following and obstacle-avoidance techniques. Here's a brief overview of how the algorithm works:

1. **Initialization**: 
   - The robot initializes ROS nodes and subscribers for the distance sensors.
   - It sets up publishers for robot movement commands.

2. **Sensor Data Handling**:
   - The robot reads data from three distance sensors: front, left, and right.

3. **Movement Commands**:
   - The robot publishes movement commands based on sensor readings.

4. **Wall Following**:
   - The robot follows the left wall by maintaining a desired distance using a proportional control approach. It adjusts its speed and direction to stay at the desired distance from the wall.

5. **Obstacle Detection and Avoidance**:
   - If the robot detects an obstacle in front, it stops and decides whether to turn left or right based on the distances from the left and right sensors.
   - In case of a dead end (obstacles on all sides), the robot backtracks and re-evaluates the situation.

6. **Dead-End Handling**:
   - If the robot encounters a dead end, it moves backward for a short duration and attempts to turn to find a new path.

7. **Backtracking**:
   - The robot can backtrack a limited number of times if it repeatedly encounters dead ends.

The robot continuously performs these actions in a loop, adjusting its path until it successfully navigates through the maze.

## Installation

### Prerequisites

Before you begin, ensure you have the following software installed on your system (raspberry pi 4 running ubuntu 20.04):

- [ROS (Robot Operating System)](http://wiki.ros.org/ROS/Installation)
- [Python 3](https://www.python.org/downloads/)

### Setting Up the Catkin Workspace

1. Open a terminal window.
2. Create a catkin workspace if you don't already have one:

   ```sh
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/
   catkin_make
   ```

3. Navigate to the `src` directory of your catkin workspace:

   ```sh
   cd ~/catkin_ws/src
   ```

4. Clone the repository into the `src` directory:

   ```sh
   git clone https://github.com/Paavalen/ROBO2-Gopigo3-Maze-solver.git
   ```

### Installing Dependencies

Navigate to the root of your catkin workspace:

```sh
cd ~/catkin_ws
```

Install any necessary dependencies using `rosdep`:

```sh
rosdep install --from-paths src --ignore-src -r -y
```

### Building the Package

Build the catkin workspace to compile the package:

```sh
catkin build
```

Source the setup file to overlay the workspace on your environment:

```sh
source devel/setup.bash
```

## Usage

### Methode 1: Launch file

To run the maze solver script on your GoPiGo3, follow these steps:

1. Ensure your GoPiGo3 robot is powered on and connected.
2. Open a terminal and navigate to your catkin workspace:

   ```sh
   cd ~/catkin_ws
   ```

3. Place the robot at the start of the maze, open a new terminal, source your workspace, and launch the maze solver:

   ```sh
   cd ~/catkin_ws
   source devel/setup.bash
   roslaunch driver_robot mazeDriver.launch
   ```

The robot should now start solving the maze autonomously.

### Methode 2: Launch all scripts manually

open Four other terminals and run these commands on each.

1. Run the ROS master:

   ```sh
   roscore
   ```

2. Run the gopigo driver

   ```sh
   cd ~/catkin_ws/src/driver_robot/src
   rosrun driver_robot gopigo3_driver.py
   ```

3. Run Distant Sensor 

   ```sh
   cd ~/catkin_ws/src/driver_robot/src
   rosrun driver_robot distance-sensor.py
   ```

4. Launch the script
   
   ```sh
   cd ~/catkin_ws/src/driver_robot/src
   rosrun driver_robot robot_driverV3.py
   ```

## Note

There are other unused scripts in the package that we used for testing.
