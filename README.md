# RSDPTeam10
AERO62520 Robotic Systems Design Project Team 10

## Introduction
This repository contains ROS2 code used in the RSDP Leo Rover project. This code is organised into ROS2 packages and nodes, with a structure defined in TKTKT.

## Contributing
This repo follows standard software development best practice. All commits should be made on branches, and merged into `main` via pull requests. For more details on using Git, see the internal team Guide to Git. The intention is that the `main` branch remains clean; this is where the code run on the main Rover will be contained, so it is paramount that it stays relatively clean.

## Repo Structure
A diagram indicating the planned software architecture is shown below. Each blue node below corresponds to a separate ROS2 package which will run during the robot's mission.

![repo structure](imgs/rsdp_software_blocks.png)

A quick explainer of the individual packages is given below. Contributors should update this list as they merge relevant pull requests introducting new packages:

- `rover_interface` Defines the messages and actions used by other packages.
- `rover_controller` Defines a finite state machine which tracks mission progress and executes the main plan.
