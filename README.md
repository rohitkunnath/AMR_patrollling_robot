# 🦾 AMR Patrolling Robot (TurtleBot3 + ROS 2 Humble)

This project demonstrates an **Autonomous Mobile Robot (AMR)** performing **patrolling behavior** using **TurtleBot3**, **ROS 2 Humble**, **Gazebo**, and **RViz**.  
The robot navigates autonomously in a simulated environment using the **Nav2** navigation stack and a custom autonomy node.

---

## 🧩 Prerequisites

Before running this project, ensure the following are installed and working correctly:

- **ROS 2 Humble Hawksbill**
- **Gazebo**
- **RViz2**
- **TurtleBot3 Packages**


⚙️ Build Instructions

  Clone this repository into your ROS 2 workspace (for example ~/amr_ws/src):
  ```bash
mkdir -p ~/AMR_patrolling_robot/src
cd ~/AMR_patrolling_robot/src
git clone https://github.com/rohitkunnath/AMR_patrollling_robot.git
```
Build the workspace:
```bash
cd ~/AMR_patrolling_robot
colcon build
```
Source the workspace:
```bash
 source install/setup.bash
```
🚀 How to Run the Simulation
🖥️ Terminal 1 – Launch Gazebo World
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
ros2 launch tb3_sim turtlebot3_world.launch.py
export TURTLEBOT3_MODEL=waffle_pi
```
🧭 Terminal 2 – Launch Navigation Stack (Nav2)
```bash
source ./install/setup.bash
ros2 launch tb3_sim nav2.launch.py
```
🤖 Terminal 3 – Launch Autonomous Patrolling Node
```bash
source ./install/setup.bash
ros2 launch tb3_autonomy autonomy.launch.py
```
🧠 Project Overview

The AMR Patrolling Robot:

    Uses TurtleBot3 Waffle Pi as the simulation base.

    Navigates autonomously using Nav2 and SLAM-based localization.

    Executes patrolling routes autonomously in a simulated Gazebo world.

    Provides visualization in RViz2 for real-time monitoring.

📂 Repository Structure
```bash
AMR_patrolling_robot/
├── build/
├── install/
├── log/
├── src/
│   ├── .vscode/
│   ├── tb3_autonomy/
│   │   ├── bt_xml/
│   │   ├── include/
│   │   ├── launch/
│   │   ├── src/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── tb3_sim/
│       ├── config/
│       ├── launch/
│       ├── maps/
│       ├── resource/
│       ├── tb3_sim/
│       ├── test/
│       ├── package.xml
│       ├── setup.cfg
│       └── setup.py

```
🧾 Notes

Always ensure you source both your ROS 2 setup and workspace setup before running any command:
```bash
source /opt/ros/humble/setup.bash
source ~/amr_ws/install/setup.bash
```
