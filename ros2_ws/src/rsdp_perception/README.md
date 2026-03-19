# Installation of Vision Node
To run the YOLO model, you need a number of packages installed. The best way to do this is by installing the relevant packages from the ultralytics repository - this will ensure the versions are compatible. NOTE: as with other ROS2 crap, this will install into the global system environment. You may also need to install `scikit-learn` for the classifiers.

```bash
pip3 install --break-system-packages -r https://raw.githubusercontent.com/ultralytics/yolov5/master/requirements.txt
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
