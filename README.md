# Xmobot
Xmobot is a flexible, modular mobile robot platform for SLAM, path planning, and navigation research benchmarking across diverse drive types, including differential, Ackermann, Mecanum (omnidirectional drives), and four-wheel independent steering with driving (4WISD).

---

## Features
- Support for ROS1 Noetic
- Modular robot design (X-Shape 4-wheel configuration)
- Parameterized robot size and drive mode
- Equipped with Laser Lidar and Front Camera
- Gazebo simulation ready
- Easy to extend to other mobile bases

---

## System Requirements

- **Ubuntu 20.04 LTS**
- **ROS1 Noetic** installed
- Recommended tools:
  - RViz
  - Gazebo 9 (comes with ROS Noetic)
  - `robot_state_publisher`, `joint_state_publisher`, `teleop_twist_keyboard`
    ```bash
    sudo apt install ros-noetic-joint-state-publisher ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-teleop_twist_keyboard
    ```
If you don't have ROS1 Noetic installed yet, follow [this guide](http://wiki.ros.org/noetic/Installation/Ubuntu).

---

## Quick Start

### 1. Clone the repository

```bash
cd ~/catkin_ws/src
git clone https://github.com/YeatsWang/xmobot.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
If you don't have a catkin_ws workspace yet:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

### 2. Launch the Xmobot model in RViz
```bash
roslaunch xmobot display.launch
```
This will bring up RViz with the Xmobot model, showing its base, wheels, lidar and camera.

### 3. Launch the Xmobot simulation in Gazebo (optional)
```bash
roslaunch xmobot gazebo.launch
```
You will see Xmobot spawned in a simple flat world.
You can use keyboard teleoperation to control it:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel
```

### 4. Drive Modes
You can easily switch between different drive modes (planned feature):
- Differential Drive (default)
- Ackermann Steering (optional, configurable)
- Mecanum (omnidirectional drives)
- 4WISD (four-wheel independent steering with driving)
To change drive type, modify parameters inside:
```bash
urdf/xmobot_drive.xacro
```

---

## Project Structure
```plaintext
xmobot/
├── README.md             # 项目首页描述
├── urdf/
│   ├── xmobot_base.xacro      # 基础机器人主体
│   ├── xmobot_wheel.xacro     # 轮子模块
│   ├── xmobot_sensors.xacro   # 激光雷达+相机模块
│   ├── xmobot_drive.xacro     # 驱动方式参数（差速、阿克曼、全向）
│   ├── xmobot.urdf.xacro      # 主入口，组合调用以上模块
├── config/
│   ├── control.yaml        # 控制器参数（diff_drive_controller、ackermann_steering_controller等）
│   ├── robot_description.yaml # robot_state_publisher参数
├── launch/
│   ├── display.launch.py   # RViz展示
│   ├── gazebo.launch.py    # Gazebo仿真
├── meshes/                 # 模型网格文件
│   ├── base.dae
│   ├── wheel.dae
│   ├── lidar.stl
│   ├── camera.stl
├── scripts/
│   ├── model_generator.py  # 未来支持自动生成不同底盘的脚本
├── worlds/
│   ├── simple_world.world  # Gazebo仿真环境
├── LICENSE
```

---

## Contribution
We welcome contributions!
If you want to:
- Add new drive types
- Improve mechanical design and simulation fidelity
- Add new sensor modules
- Extend to ROS2 support
Please feel free to submit pull requests or open issues!

---

## License
This project is licensed under the MIT License.
