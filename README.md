# Robot planning and its application Student Interface for Coordinated evaquation


This package is used by us to complete the assignment of the course. We choose the Project 1: Coordinated evacuation.


## The Scenario

The project is about designing a coordinated evacuation of a site cluttered by several obstacles. Three robots have to react to an emergency reaching a gate in minimum time.

The requirements are the following:
1. The robots move at a constant speed (so they can execute Dubins Manoeuvres).
2. The robots have to move without touching the border of the map and the obstacles.
3. The robots must not collide with each other.


The work is developed upon the following assumptions:
1. The robots are initially deployed at a random position.
2. A ROS component will provide the exact position of the obstacles and of the robots in real-time.


Steps to complete the task:
1. Create a roadmap that enables the robot to move in minimum time.
2. The roadmap will have to be decorated with information regarding the nodes and the links that cannot be occupied at the same time by two robots, lest a collision occurs.
3. Set up an automatic strategy to decide the motion of the robots so that they evacuate the area without creating collisions.
UCS-based algorithm for finding the strategy for the given initial node towards the escape gate.


Simplification:
Hypothesis of a synchronous behaviour: i.e., the system evolves in a sequence of states and each state is characterised by having the robot located in one of the nodes of the roadmap. In this way, every step of the plan is ended when all robots have reached a
location and no robot is allowed to move to the next location before a step completes.


## Setup

For running the code Ubuntu 16.04 is required. Moreover, before running the code make sure to follow the following steps:
1. Have installed the OpenCV library following the instructions reported in the [install_opencv.md](install_opencv.md) file.
2. Have installed ROS and Gazebo following the instructions reported in the [install_ros_and_gazebo.md](install_ros_and_gazebo.md) file.
3. Have completed the setup of the simulator and of the project following the instructions reported in the [simulator_setup.md](simulator_setup.md) file.


## Execute the code

Open terminator and split it in 4 terminals. In the terminals use these commands:
1. In the first one run the simulator and wait until the log stops in order to have the simulator ready

```bash
$ AR_simulator_gui n:=3
```

or

```bash
$ AR_simulator n:=3
```

depending whether to have also Gazebo as GUI or not.

2. In the second one launch the CV pipeline to detect all the elements

```bash
$ AR_pipeline n:=3
```

3. Optionally in the third launch RViz, for having a schematic visualization

```bash
$ AR_rviz
```

4. In the last one, run the planning part for moving of one node in the graph. This should be launched multiple times until all the robots reach the exit gate

```bash
$ AR_run
```

## Change the map

It is also possible to modify the environment by using the following command:

```bash
$ gedit $AR_map_file
```

It is also possible to modify the robots initial configurations by using the following command:

```bash
$ gedit $AR_sim_file
```

