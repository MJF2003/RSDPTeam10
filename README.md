# RSDPTeam10
AERO62520 Robotic Systems Design Project Team 10

## Yolov5 itself
(THIS IS NOT FULL OF THE CODE, WEIGHT FILES ARE IN NUC)

Test using Realsense:
(in venv)

cd /home/team10/yolov5

QT_QPA_PLATFORM=xcb python infer_live_realsense_final.py \
  --weights train/blocks_bins_platform4/weights/best.pt \
  --block_clf_dir /home/team10/yolov5/bins_blocks_dataset/block_attrs/clf_out \
  --bin_clf_dir   /home/team10/yolov5/bins_blocks_dataset/bin_color/clf_out \
  --conf 0.25 \
  --show_depth

## ROS node for Yolov5 (in ROS_node file)
### Main pose topics

/cv/bin_poses
Type: geometry_msgs/PoseArray
(each pose = smoothed XYZ, identity orientation)

/cv/block_poses
Type: geometry_msgs/PoseArray

/cv/platform_poses
Type: geometry_msgs/PoseArray

### Extra topics

/cv/bin_opening_poses
Type: geometry_msgs/PoseArray

/cv/tracks_json
Type: std_msgs/String

### RUN
#### Terminal A:
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=10
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_sync:=true

#### Terminal B:
source /home/laptop30/Dev/tutorial_venv/bin/activate
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=10

python ~/ros2_ws/src/rsdp_perception/rsdp_perception/perception_stable_attrs_node.py \
  --ros-args \
  -p weights:=/home/laptop30/yolov5/runs/train/blocks_bins_platform4/weights/best.pt \
  -p imgsz:=416 \
  -p conf:=0.25 \
  -p block_attrs_dir:=/home/laptop30/yolov5/bins_blocks_dataset/block_attrs/clf_out \
  -p bin_color_dir:=/home/laptop30/yolov5/bins_blocks_dataset/bin_color/clf_out \
  -p min_attr_conf:=0.20 \
  -p vote_window:=10 \
  -p min_votes_to_output:=1 \
  -p color_topic:=/camera/camera/color/image_raw \
  -p depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  -p info_topic:=/camera/camera/color/camera_info
 
#### Terminal C:
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=10
python3 ~/ros2_ws/src/rsdp_perception/rsdp_perception/tracks_monitor_attrs_node.py
