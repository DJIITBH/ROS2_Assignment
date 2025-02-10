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
Click below to watch the demonstration:

📹 [Watch the demonstration video](https://youtu.be/T_X8_3UtNWY)

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

### Launch Gazebo and Spawn the World
```bash
ros2 launch bbot_description gazebo.launch.py
```

### Convert `.m4a` File to `.wav` if you have priorly recorded audio files
```bash
ffmpeg -i backward.m4a -acodec pcm_s16le -ar 44100 -ac 2 output_back.wav
```

### Run the Voice Command Nodes

- **Start the Client Node (to send voice recording requests)**
  ```bash
  ros2 run bbot_description voice_client.py
  ```

- **Start the Service Node (to process voice data and return text response)**
  ```bash
  ros2 run bbot_description voice_service.py
  ```

## 🎥 Demo Video
Click below to watch the demonstration:

📹 [Watch the demonstration video](https://youtu.be/HVS1NEqfuc8)

---
### 📌 Notes
- Ensure all dependencies are installed before running the commands.
- The robot will be spawned in Gazebo with navigation capabilities.
- Voice commands can be processed using the service node to assist navigation.

## Approach
- I used a speech to text conversion model to process speech commands
- Then I used service client architecture to send request for audio commands
- After receiving response of audio command, I published them on /cmd_vel topic to make robot move
- Made a custom service to send and receive response of text data
  
## Challenges
- Integration of Whisper model with service client architecture was difficult

# ROS2 Object Detection using YOLO

Before running the package, install the necessary dependencies:

```bash
pip install ultralytics cv2
```
## Execution Steps

### 1. Launch Gazebo and Spawn the World
Run the following command to launch Gazebo with the defined world:
```bash
ros2 launch bbot_description gazebo.launch.py
```

### 3. Run the Object detection File 
Execute the following command to run the detection script and send coordinates and class of object:
```bash
ros2 run bbot_description obj_publisher.py 
```
Execute the following command to receive coordinates and class of object:
```bash
ros2 run bbot_description image_subscriber.py
```
## 🎥 Demo Video
Click below to watch the demonstration:

📹 [Watch the demonstration video](https://youtu.be/c3vxh45bRcM)

---
### 📌 Notes
- Ensure all dependencies are installed before running the commands.
- Ensure you have YOLOv11 installed


