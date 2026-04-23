# Navigation Node
This package is a robot navigation package developed based on the ROS 2 Nav2 (Navigation 2 Stack) framework. It provides a comprehensive solution for leo-rover, ranging from path planning to motion control, and integrates a custom Action Server interface that allows external programs to control the robot's movement via coordinate points.

# Installation of Navigation Node
## 1. Packages
```bash
sudo apt update
sudo apt install ros-jazzy-navigation2
sudo apt install ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-nav2-simple-commander
sudo apt install ros-jazzy-turtlebot3-msgs
sudo apt install ros-jazzy-tf2-geometry-msgs
```

## 2. Model Weights
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

- `navigation.launch.py`: Launch all navigation components (controllers, planners, etc.) with a single click and bind them to the lifecycle manager.
- `neo_robot.yaml`: Standardize the definitions of the robot's physical radius, maximum speed, obstacle avoidance distance, and behavior tree paths.
- `navigation_service_node.py`: Listen for external topics and pass coordinate commands to the Nav2 engine.
- `navigate_through_poses_w_replanning_and_recovery.xml` & `navigate_to_pose_w_replanning_and_recovery.xml`: Behavior tree logic definition.


## 3. Edit path in neo_robot.yaml
Change absolute path of the behavior tree:
1. Change `default_nav_to_pose_bt_xml` to `default_nav_to_pose_bt_xml: "<path_to_the_repo>/install/navigation_2/share/navigation_2/config/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"`
2. Change `default_nav_through_poses_bt_xml: ` to `default_nav_through_poses_bt_xml: "<path_to_the_repo>/install/navigation_2/share/navigation_2/config/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml"`

## 4. Integration
- Map topic: /map
- Odometer topic: /wheel_odom
- Speed command listener topic: /cmd_vel
  
- Global coordinate system name: map
- Odometer coordinate system name: odom
- Robot coordinate system: base_link

# Running the Node
```bash
# Terminal1
ros2 launch rover_controller launch_sim.py run_navigation_stub:=false
```
```bash
# Terminal2
ros2 launch navigation_2 navigation.launch.py use_sim_time:=true
```

`launch_sim.py` already starts the simulated SLAM stack by default through
`run_slam_node:=true`, so do not launch `rover_slam/slam_launch.py` separately
unless you disable it in `launch_sim.py`.

Next, you can launch RViz and send the target point
```bash
# Terminal3
ros2 run rviz2 rviz2
```
```bash
# Terminal4
ros2 action send_goal /navigate_to_pos rover_interface/action/NavigateToPos "{target_pos: {header: {frame_id: 'map'}, point: {x: 3.0, y: 0.0, z: 0.0}}}"
```
