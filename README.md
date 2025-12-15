# RSDPTeam10
AERO62520 Robotic Systems Design Project Team 10

How it works:
block_localizer_node
Block Locator: This core coordinate transformation programme receives pixel position and depth data detected by the camera. It converts pixel coordinates into three-dimensional coordinates within the camera coordinate system using the camera's intrinsic parameters. Subsequently, it employs TF coordinate transformation to convert these into the robot base coordinate system, ultimately outputting the block's actual position relative to the robot.

static_tf_publisher
Static TF Publisher: This is a coordinate system relationship definition programme that records the precise position and orientation of the camera mounted on the robot body. It provides a fixed reference baseline for all programmes requiring coordinate transformation. Currently, it is assumed to be positioned 0.1 metres forward, 0.2 metres high, and facing directly forward.

test_block_publisher
Test Data Publisher: This is a hypothetical data generation programme supplying the data required by the Yolo model. It simulates the camera's detection function, generating positional data for two virtual cubes every three seconds (one positioned 1 metre from the centre of the frame, the other 1.5 metres to the left of the frame). This facilitates system debugging and functional verification when the actual camera is unavailable.


Terminal 1: Launch RealSense camera
ros2 launch realsense2_camera rs_launch.py

Terminal 2: Launch static TF publisher
python3 src/leo_navigation/scripts/static_tf_publisher.py

Terminal 3: Launch coordinate transformation node
python3 src/leo_navigation/scripts/block_localizer_node.py

Terminal 4: Launch test publisher
python3 src/leo_navigation/scripts/test_block_publisher.py

Monitor the position of the converted block:
ros2 topic echo /navigable_blocks
