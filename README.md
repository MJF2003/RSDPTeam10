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

## Launch the robot for autonomous exploration and mapping
First, synchronize the robot’s internal clock with the computer (or NUC) clock:
```bash
>ssh pi@10.0.0.1
```
password:raspberry
```bash
sudo date -u -s "2026-02-20 12:15:00" #Change to the current time.
```
## Use on simulation:
Terminal 1: Start the simulation
```bash
cd ~/RSDPTeam10-main/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/RSDPTeam10-main/ros2_ws/install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/RSDPTeam10-main/ros2_ws/src/rover_description:~/RSDPTeam10-main/ros2_ws/src/leo_gz_worlds
ros2 launch rover_controller launch_sim.py run_navigation_stub:=false
```
Terminal 2: Start SLAM
```bash
cd /home/student02/SLAM_2D_Learn/SLAM_2D_Learn
source /opt/ros/jazzy/setup.bash
source ~/RSDPTeam10-main/ros2_ws/install/setup.bash
source /home/student02/SLAM_2D_Learn/SLAM_2D_Learn/install/setup.bash
ros2 launch leo_explore slam_only_sim.launch.py
```
Terminal 3: Start Nav2
```bash
cd /home/student02/SLAM_2D_Learn/SLAM_2D_Learn
source /opt/ros/jazzy/setup.bash
source ~/RSDPTeam10-main/ros2_ws/install/setup.bash
source /home/student02/SLAM_2D_Learn/SLAM_2D_Learn/install/setup.bash
ros2 launch leo_explore nav2_minimal_sim.launch.py
```
Terminal 4: Start the explore action server
```bash
cd /home/student02/SLAM_2D_Learn/SLAM_2D_Learn
source /opt/ros/jazzy/setup.bash
source ~/RSDPTeam10-main/ros2_ws/install/setup.bash
source /home/student02/SLAM_2D_Learn/SLAM_2D_Learn/install/setup.bash
ros2 run leo_explore explore_action_server --ros-args -p use_sim_time:=true
```
Terminal 5: Send a 120-second exploration goal
```bash
source /opt/ros/jazzy/setup.bash
source ~/RSDPTeam10-main/ros2_ws/install/setup.bash
source /home/student02/SLAM_2D_Learn/SLAM_2D_Learn/install/setup.bash
ros2 action send_goal /explore leo_explore_interfaces/action/Explore "{max_runtime_sec: 120.0, run_until_cancelled: false}" 
```
Alternatively: Keep exploring until manually cancelled
```bash
ros2 action send_goal /explore leo_explore_interfaces/action/Explore "{max_runtime_sec: 0.0, run_until_cancelled: true}"
```
