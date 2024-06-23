# Maze Solver for GoPiGo3

Welcome to the Maze Solver for GoPiGo3 repository! This project was developed by our team composed of Girvana Hurreesing, Chavishka Jotee, Yassir Hoossan Buksh and Paavalen Lingachetti as part of the ROBO2 module. It features a maze-solving script designed to be used with the GoPiGo3 robot platform.

## Table of Contents

- [Project Overview](#project-overview)
- [Installation](#installation)
  - [Prerequisites](#prerequisites)
  - [Setting Up the Catkin Workspace](#setting-up-the-catkin-workspace)
  - [Installing Dependencies](#installing-dependencies)
  - [Building the Package](#building-the-package)
- [Usage](#usage)
- [Note](#note)


## Project Overview

The Maze Solver for GoPiGo3 project consists of a ROS package that enables the GoPiGo3 robot to navigate and solve mazes autonomously. The robot uses three ultrasound sensors to detect and follow paths within the maze, making decisions at intersections to reach the goal.

## Installation

### Prerequisites

Before you begin, ensure you have the following software installed on your system:

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

To run the maze solver script on your GoPiGo3, follow these steps:

1. Ensure your GoPiGo3 robot is powered on and connected.
2. Open a terminal and navigate to your catkin workspace:

   ```sh
   cd ~/catkin_ws
   ```

3. Run the ROS master:

   ```sh
   roscore
   ```

4. Place the robot at the start of the maze, open a new terminal, source your workspace, and launch the maze solver:

   ```sh
   cd ~/catkin_ws
   source devel/setup.bash
   roslaunch driver_robot mazeDriver.launch
   ```

The robot should now start solving the maze autonomously.

If the launch file donnot works, open three other terminals and run these commands on each.

Terminal 1

```sh
cd ~/catkin_ws/src/driver_robot/src
python3 gopigo3_driver.py
```

Terminal 2

```sh
cd ~/catkin_ws/src/driver_robot/src
python3 distance-sensor.py
```

Terminal 3

```sh
cd ~/catkin_ws/src/driver_robot/src
python3 robot_driverV3.py
```

## Note

There are other unused script in the package that we used for testing.
