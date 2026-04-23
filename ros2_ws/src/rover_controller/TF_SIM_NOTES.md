# Sim TF Notes

## Expected frame ownership

- `map -> odom`: SLAM/localization
- `odom -> base_footprint`: simulation odometry
- `base_footprint/base_link -> robot descendants`: `robot_state_publisher`

This keeps a single owner for each section of the TF tree and avoids duplicate transform publishers.

For the current Gazebo + Cartographer setup, Cartographer should publish the pose of `odom` in `map`, not the pose of `base_link` in `map`. If Cartographer is configured with `published_frame = "base_link"` while Gazebo already publishes `odom -> base_footprint`, the tree effectively gives `base_link` a second parent and the fixed `base_footprint -> base_link` offset can collapse toward zero.

The tracking frame should also stay on the ground-contact frame for this 2D setup. Using `tracking_frame = "base_link"` with an elevated chassis frame can shift the `map -> odom` transform down by the chassis height. Using `tracking_frame = "base_footprint"` keeps the SLAM pose aligned with the floor plane.

## Why the arm and wheels disappeared in RViz

If fixed links like the lidar, depth camera, and antenna resolve, but `rocker_*`, `wheel_*`, `arm_joint*`, and `arm_gripper*` do not, the URDF is usually not the primary problem.

Those missing links sit behind non-fixed joints, so `robot_state_publisher` only publishes them when it receives matching entries on `/joint_states`.

## Sim fallback for the arm

Until the arm stack exists, `launch/launch_sim.py` starts `sim_arm_joint_state_publisher` by default. It publishes a parked all-zero arm pose onto `joint_states`, which is enough for `robot_state_publisher` to populate the arm TF subtree in RViz.

This is intentionally a joint-state fallback, not a static TF publisher. The arm remains an articulated chain in URDF, so the same TF path stays valid when real arm motion arrives later.

To disable the fallback when a real arm state source is running:

```bash
ros2 launch rover_controller launch_sim.py run_arm_joint_state_fallback:=false
```

## MoveIt2 handoff

For MoveIt2, the arm must stay joint-driven:

- Keep the arm joints revolute in URDF.
- Keep `robot_state_publisher` as the source of arm link TF.
- Replace the fallback joint-state publisher with the real arm/controller/MoveIt-driven joint states when manipulation is ready.

Only one source should publish a given arm joint state at a time. The future manipulation stack should own these joints:

- `arm_joint2_to_joint1`
- `arm_joint3_to_joint2`
- `arm_joint4_to_joint3`
- `arm_joint5_to_joint4`
- `arm_joint6_to_joint5`
- `arm_joint6output_to_joint6`
- `arm_gripper_controller`

The mimic gripper joints should continue to be derived from the URDF model rather than having separate TF publishers.

## Useful checks

```bash
ros2 topic echo /joint_states
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map arm_gripper_base
ros2 run tf2_ros tf2_echo base_link wheel_FL_link
```

When running the SLAM stack in Gazebo, launch it with sim time enabled so Cartographer, RViz, and the rest of the TF chain all use `/clock` consistently:

```bash
ros2 launch rover_slam slam_launch.py launch_rviz:=true
```

If the same launch file is used with real hardware later, override it with `use_sim_time:=false`.
