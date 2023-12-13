# Dependencies
- wheeltec rosbot plus: https://www.roboworks.net/store/p/rosbot-pro-4wdis
- The manufacture provided SD card includes the cartographer package and other packages for controlling the robot
- This specific build of pytorch: https://github.com/Qengineering/PyTorch-Jetson-Nano
- Ultralytics was used for person detection using the yolo models: https://github.com/ultralytics/ultralytics
- Joy_Linux ros package for uisng a xbox 360 wireless controller: https://index.ros.org/p/joy_linux/

# Steps for our results
- build using `colcon build --symlink-install`
- run `source ./install/setup.bash`
- run `ros2 launch final_project all_launch.py` and in a seperate terminal run `ros2 bag record /camera/color/camera_info /camera/color/image_raw /camera/depth/camera_info /camera/depth/image_raw /camera/depth/points /camera/extrinsic/depth_to_color /cmd_vel /constraint_list /joint_states /map /mobile_base/sensors/imu_data /odom_combined /prob_topic /robot_description /robotpose /robotvel /tf /tf_static /trajectory_node_list /lslidar_driver_node/transition_event /lslidar_order /scan /scan_matched_points2`
- Because of some time constraints the person detection code just outputs the person probability on the topic '/prob_topic' to add in the arrows you will need to run the command `ros2 run final_project viz_people` then in a seperate terminal `ros2 bag play <rosbag_name>` and then in another terminal `ros2 bag record /camera/color/camera_info /camera/color/image_raw /camera/depth/camera_info /camera/depth/image_raw /camera/depth/points /camera/extrinsic/depth_to_color /cmd_vel /constraint_list /joint_states /map /mobile_base/sensors/imu_data /odom_combined /prob_topic /robot_description /robotpose /robotvel /tf /tf_static /trajectory_node_list /lslidar_driver_node/transition_event /lslidar_order /scan /scan_matched_points2 /person_display_topic`
- This will give a ros bag like our video

# Running the Bag file
- To run the ros bag file download the bag from here: https://usu.box.com/s/9ukvlxlshevzdykpf4s9d8bsue3pphyi
- Open rviz and use the config file named `project_default.rviz`
- run the command `ros2 bag play rosbag2`
- You should see the robot map and plot directions on where people are detected
- It taks a while to get started because of the waiting for the models to load in

# Launch Files
- `all_launch.py` this runs both person detection and mapping at the same time
- `gamepad_launch.py` this runs the robot in teleop mode with no mapping
- `mapping_launch.py` this runs the robot with teleop and mapping
- `navigaion_launch` this is the start of the navigation launch file that never worked
- `person_detection_launch.py` this runs the yolo models on the robot using the camera

# Node Files
- `person_detection.py` loads and runs the yolo model using the robots camera
- `viz_people.py` takes in the predictions from the yolo model and outputs the direction the people are in from the robot
- `wireless_controller.py` teleops the robot
