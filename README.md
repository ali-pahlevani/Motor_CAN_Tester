# Autonomous Forklift

**Web Dashboard**
- Install ***ROSBridge server***:
sudo apt install ros-humble-rosbridge-server

- Launch the ROSBridge server (default):
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

- Start a HTTP server (in the directory where the webpage's files exist): python3 -m http.server 7000

- In a webpage, enter http://172.19.224.192:7000/ (local - got the ip in WSL2 using hostname -I)
+ Remember 'webpage_address' and 'rosbridge_address' lines for probable future use

- Use ***roslibjs*** to connect to our rosbridge_server (in the HTML file)

- Use ***nippleJS*** to add a Joystick (for controlling the robot)

- Finally, we add an image tag; so that we are able to stream the camera video online

-----------

**Launching Sensors**
- ***PF***: ros2 launch pf_driver r2000.launch.py
=> In Rviz (and also, in the launch file):
    + Reliability: Best Effort
    + Durability:  Volatile

- ***Sick***: ros2 launch sick_safetyscanners2 sick_safetyscanners2_launch.py (This is Classis node, but we also can use Lifecycle node)
=> In Rviz (and also, in the launch file):
    + Reliability: Reliable  |  Best Effort (both)
    + Durability:  Volatile

- ***Vzense***: ros2 run ScepterROS scepter_camera
=> For now (should change):
    + Frame name:      Scepter_frame (plus lots of other frames for other stuff)
    + RGB image topic:   /Scepter/color/image_raw
    + Depth point-cloud: /Scepter/depth/points

=> In Rviz (and also, in the launch file):
    + Reliability: Reliable  |  Best Effort (both)
    + Durability:  Volatile

-----------

**Sensors' Static IPs**
- 169.254.228.118  =>  Vzense (Depth Camera)
- 169.254.228.117  =>  Sick   (Safety LiDAR)
- 169.254.228.116  =>  PF     (Navigation LiDAR)

-----------

**Creating master.dcf file**
- dcfgen -r src/autofork/autofork_control/config/robot_control/bus.yml (Edited: dcfgen --no-strict -r bus.yml)

    - Make sure eds files are correctly put in the place
    - Finally, put the master.dcf file in the correct location
    - After having canopen (in raspberry), uncomment the respective lines in autofork_control's CMakeLists.txt file

----------------------------------------------------------------------------

**Run Pallet Detection Node**
ros2 run autofork_pallet detection_node --ros-args -p model_path:=src/autofork/autofork_pallet/models/best_yolov8x_detection.pt -p image_topic:=/camera/image_raw

**Run Pallet Segmentation Node**
ros2 run autofork_pallet segmentation_node --ros-args -p model_path:=src/autofork/autofork_pallet/models/best_yolov8x_segmentation.pt -p image_topic:=/camera/image_raw

- Needs a **Numpy** version **below 2.0 (NumPy 1.x)**

----------------------------------------------------------------------------

**Teleoperate Twist Keyboard (with Remapping)**
- ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cmd_vel_key

**Run Localization (using AMCL in nav2)**
- ros2 launch autofork_launch localization_launch.py map:=mapFile.yaml (save, not serialize) use_sim_time:=true

**Run Navigation (Nav2)**
- ros2 launch autofork_launch navigation_launch.py use_sim_time:=true
=> When after localization, we want to start Nav2, we should also give it an extra flag: "map_subscribe_transient_local:=true"

**Twist Mux**
- ros2 run twist_mux twist_mux --ros-args --params-file:=file.yaml -r cmd_vel_out:=input_cmd_vel

**Run Localization (using slam_toolbox and a pre-built map)**
- ros2 launch autofork_launch online_async_localizer_launch.py slam_params_file:=src/autofork/autofork_navigation/config/localization_config/localizer_params_online_async.yaml use_sim_time:=true

----------------------------------------------------------------------------

**Run SLAM (using slam_toolbox)**
- ros2 launch autofork_launch online_async_mapper_launch.py slam_params_file:=src/autofork/autofork_navigation/config/slam_config/mapper_params_online_async.yaml use_sim_time:=true

**How to save maps?**
- Using slam_toolbox services (save_map & serialize_map)
- Using Rviz plugin: "SlamToolboxPlugin"

**Loading a saved map**
- ros2 run nav2_map_server map_server --ros-args -p yaml_file_name:="file address".yaml -p use_sim_time:=true
- Activating the map_server: ros2 run nav2_util lifecycle_bringup map_server

**Adaptive Monte Carlo Localization (AMCL):**
- ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true
- Activating the amcl: ros2 run nav2_util lifecycle_bringup amcl
- Set the "initial pose" (either by clicking or publishing to "/initialpose" topic)
=> For forklift, set the "use_sim_time" to "false"

----------------------------------------------------------------------------

**Open CAN (RPi):**
- sudo ip link set can0 up type can bitrate 1000000
- sudo ifconfig can0 txqueuelen 65536
- sudo ifconfig can0 up

-----------

**ros2_canopen Dependencies:**
- cd <path/to/workspace>
- rosdep install --from-paths src/ros2_canopen --ignore-src -r -y

-> Webpage: https://ros-industrial.github.io/ros2_canopen/manual/humble/quickstart/installation.html

- for cloning: git clone -b humble https://github.com/ros-industrial/ros2_canopen.git

-----------

**Sick_Safetyscanners ROS2 Driver Dependencies:**
- sudo apt-get install ros-humble-sick-safetyscanners2-interfaces
- sudo apt-get install ros-humble-sick-safetyscanners-base

-> Webpage: https://github.com/SICKAG/sick_safetyscanners2

-----------

**ROS2 driver for R2000 and R2300 laser scanners Dependencies:**
- export ROS_DISTRO=humble
- cd <path/to/workspace>
- rosdep update --include-eol-distros
- rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y

-> Webpage: https://github.com/PepperlFuchs/pf_lidar_ros2_driver

-----------

**Vzense Camera Dependencies:**
- sudo apt install python3
- sudo apt install python3-colcon-common-extensions
- sudo apt install libpcl-dev

-> Webpage: https://wiki.vzense.com/#/en/ScepterSDK/3rd-party-plugin/ros2?!d=%20422-install-the-ros-package