# RSDPTeam10
AERO62520 Robotic Systems Design Project Team 10

## Introduction
This repository contains ROS2 code used in the RSDP Leo Rover project. This code is organised into ROS2 packages and nodes which execute separate logic. Most code currently lives in branches as teammates work on individual task solving. This will be merged into `main` as packages are completed and integrated.

## Contributing
This repo follows standard software development best practice. All commits should be made on branches, and merged into `main` via pull requests. For more details on using Git, see the internal team [Guide to Git](https://livemanchesterac-my.sharepoint.com/:w:/g/personal/alexander_inch_postgrad_manchester_ac_uk/IQBNWKkrWXlWRrZvdg0988FWATn3JgwjHPRF73FxCxYHUuE?e=fSUf5N). The intention is that the `main` branch remains clean; this is where the code run on the main Rover will be contained, so it is paramount that it stays relatively clean.

## Repo Structure
A diagram indicating the planned software architecture is shown below. Each blue node below corresponds to a separate ROS2 package which will run during the robot's mission.

![repo structure](imgs/rsdp_software_blocks.png)

A quick explainer of the individual packages is given below. Contributors should update this list as they merge relevant pull requests introducting new packages:

- `rover_interface` Defines the messages and actions used by other packages.
- `rover_controller` Defines a finite state machine which tracks mission progress and executes the main plan.
- `rover_decription` Defines a modularised URDF of the team robot to be used in both sim and for real transform tree.
- `rsdp_perception` Implements the computer vision sensing logic, using a DepthCamera. To run this node you need to follow some more installation steps - take a look at the vision README at `ros2_ws/src/rsdp_perception/README.md`

## Running the Simulator
To run the sim, you need to export a shell variable so ROS knows where to find the models. You can also add the export to your `.bashrc` or `.zshrc` so you don't need to re-run it. I'm sure there's a better way to do this automatically, but I don't know how to do it. We also install the leo package which provides the Leo Rover description
```bash
> sudo apt install ros-jazzy-leo

> export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:<path_to_the_repo>/RSDPTeam10/ros2_ws/src/rover_sim_gazebo/models
> export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gz-sim-8/plugin
```

You can then run the simulator using 
```bash
> ros2 launch rover_controller launch_sim.py
```

The sim exposes stub modules which do basic actions - like a vision stub which generates noisy observations by directly reading the Gazebo positions. You can disable specific stubs as follows.

```bash
> ros2 launch rover_controller launch_sim.py run_vision_stub:=false run_navigation_stub:=false run_smooth_observations:=false run_rover_controller:=false
```

Check the launch file for the full list of launch arguments.

## Launch the SLAM Node
The SLAM Node consumes a laser scan to produce a map on the `/map` topic. You can run the node itself by running 
```bash
ros2 launch rover_slam launch_slam.py
```

To also launch `rviz` and the real-world lidar (check permissions below) along it, specify the launch arguments
```bash
ros2 launch rover_slam slam_launch.py launch_rviz:=true launch_rplidar:=true
```

To run the RPLidar A2M12 (ie. the real lidar) you need to set the permissions so the computer can communicate with it using this command.
```bash
sudo chmod 777 /dev/ttyUSB0
## Required ROS2 Package Installation
There are a number of packages, not included in the repository which our custom packages rely on. These are listed as follows:

`mycobot_ros2` - Provided by ElephantRobotics and providing functionality and description for our Arm (and apparently and rather unhelpfully, every single other manipulator they sell). Installation steps taken from [Elephant Robotics](https://github.com/elephantrobotics/mycobot_ros2). You may have to separately install `python3-tk` and `python3-numpy` if running on Ubuntu 24.04.
```bash
> cd <path_to_the_repo>/RSDPTeam10/ros2_ws/src/

> pip install pymycobot --user  # --break-system-packages is likely to be required
> git clone -b humble --depth 1 https://github.com/elephantrobotics/mycobot_ros2.git  # There is no Jazzy branch. ROS is a pain
> cd <path_to_the_repo>/RSDPTeam10/ros2_ws
> vcs import src < src/warehouse_ros_mongo.repos
> sudo apt-get update && rosdep install --from-paths src --ignore-src -r -y --skip-keys="python-tk python-numpy"
> sudo apt-get python3-tk python3-numpy
```

## Running the vision node
The vision node requires various installs to run. Check out the full instructions in the README at `ros2_ws/src/rsdp_perception/README.md`.

After all the required installs, you can launch the node + Realsense camera with 
```bash
ros2 launch rsdp_perception vision.launch.py
```

## Running the Navigation node
To run the navigation need you need the following installs:

```bash
sudo apt update
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-simple-commander ros-jazzy-turtlebot3-msgs ros-jazzy-tf2-geometry-msgs
```

The repo directory for the navigation node should look like...
```bash
navigation_2/
├── config/
│   └── neo_robot.yaml
│   └── behavior_trees
│       └── navigate_through_poses_w_replanning_and_recovery.xml
│       └── navigate_to_pose_w_replanning_and_recovery.xml
├── launch/
│   └── navigation.launch.py
├── navigation_2/
│   └── navigation_service_node.py
├── package.xml
└── setup.py
```

For more instructions on running it check the readme file in `ros2_ws/src/navigation_2`.
