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
ros2 launch robot_nav2 nav2_bringup.launch.py
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

## 🎥 Demo Video
Click the image below to watch the demonstration:

[![Watch the Demo](https://youtu.be/T_X8_3UtNWY)

## Approach
- First I mapped a custom created world using teleoperation of mobile robot
- I used Diff-Drive plugin present in gazebo to control motors
- After Mapping using SLAM toolbox, I tuned the local and global costmap as well as controller server parameters for dynamic obstacle avoidance
- Wrote code for nav2 goal in C++

## Challenges Faced
- As it was written not to use TurtleBot so I made a custom mobile robot using CAD
- Moreover, for dynamic obstacles that are not mapped, I need to tune the parameters wisely and it took a lot of time to tune costmaps and tolerance values so that it can plan path optimally
- C++ code for nav2 goal further posed issues as it was my first time to use nav2 in c++.

# ROS2 Autonomous Navigation with Voice Command

This repository provides a ROS2 package for autonomous navigation using Gazebo simulation and voice command processing.

## 📌 Installation & Dependencies

Before running the package, install the necessary dependencies:

```bash
pip install sounddevice soundfile librosa numpy wavio
sudo apt-get install portaudio19-dev
```

## 🚀 Execution Steps

### 1️⃣ Launch Gazebo and Spawn the World
```bash
ros2 launch bbot_description gazebo.launch.py
```

### 2️⃣ Launch RViz and Nav2 for Navigation
```bash
ros2 launch bbot_description nav2.launch.py
```

### 3️⃣ Run the Navigation Node (C++ Pose Goal)
```bash
ros2 run bbot_description pose_navigator
```

### 4️⃣ Convert `.m4a` File to `.wav` Format for Processing
```bash
ffmpeg -i backward.m4a -acodec pcm_s16le -ar 44100 -ac 2 output_back.wav
```

### 5️⃣ Run the Voice Command Nodes

- **Start the Client Node (to send voice recording requests)**
  ```bash
  ros2 run bbot_description voice_client.py
  ```

- **Start the Service Node (to process voice data and return text response)**
  ```bash
  ros2 run bbot_description voice_service.py
  ```

## 🎥 Demo Video
Click the image below to watch the demonstration:

[![Watch the Demo](https://youtu.be/HVS1NEqfuc8)

---
### 📌 Notes
- Ensure all dependencies are installed before running the commands.
- The robot will be spawned in Gazebo with navigation capabilities.
- Voice commands can be processed using the service node to assist navigation.


