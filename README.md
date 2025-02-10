# ROS2 Autonomous Navigation

This repository contains the necessary files to launch and navigate a robot autonomously in a simulated Gazebo environment.

## Installation
Clone this repository and set up your ROS2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/DJIITBH/ROS2_Assignment.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Execution Steps

### 1. Launch Gazebo and Spawn the World
Run the following command to launch Gazebo with the defined world:
```bash
ros2 launch bbot_description gazebo.launch.py
```

### 2. Launch RViz and Nav2 Bringup for Navigation
Start RViz and bring up the Navigation2 stack using:
```bash
ros2 launch bbot_description gazebo.launch.py
```

### 3. Run the Navigation File to Give a Pose Goal (C++)
Execute the following command to run the navigation script and send pose goals:
```bash
ros2 run bbot_description pose_navigator
```

## Notes
- Ensure that Gazebo is installed and properly set up.
- The world file should be correctly defined for autonomous navigation.
- Make sure to source the ROS2 workspace before running any commands:
  ```bash
  source ~/ros2_ws/install/setup.bash
  ```

## License
This project is licensed under the MIT License.

