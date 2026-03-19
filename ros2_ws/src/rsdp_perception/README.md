# Installation of Vision Node
To run the YOLO model, you need a number of packages installed. The best way to do this is by installing the relevant packages from the ultralytics repository - this will ensure the versions are compatible. NOTE: as with other ROS2 crap, this will install into the global system environment. You may also need to install `scikit-learn` for the classifiers.

```bash
pip3 install --break-system-packages -r https://raw.githubusercontent.com/ultralytics/yolov5/master/requirements.txt
```

You also need to install the realsense ROS2 package to interface with the camera
```bash
sudo apt install 'ros-jazzy-realsense2-*'
```

Alternatively, you can try installing the packages by hand. I recommend avoiding this if possible.
## package installs 
- pandas
- ultralytics
- joblib
- torch
- torchvision
- tqdm
- seaborn
- numpy < 2.0.0
- opencv

# Running the Node
To run the node directly, run
```bash
ros2 run rsdp_perception vision_node
```

To launch the RealSense camera and `vision_node` together, run
```bash
ros2 launch rsdp_perception vision.launch.py
```

You can override a few useful `vision_node` parameters from launch without editing code. For example:
```bash
ros2 launch rsdp_perception vision.launch.py conf:=0.35 vote_window:=6 min_votes_to_output:=2
```

The launch file also exposes the subscribed topics and RealSense sync/aligned-depth settings:
```bash
ros2 launch rsdp_perception vision.launch.py \
  align_depth.enable:=true \
  enable_sync:=true \
  color_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  info_topic:=/camera/camera/color/camera_info
```
