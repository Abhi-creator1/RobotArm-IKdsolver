# Inverse Kinematics solver for 4DOF Robot Arm ROS2 Project

## Project Overview
This project demonstrates the control and simulation of a 4 Degree-of-Freedom (DOF) robotic arm using ROS2 (Robot Operating System 2). The system consists of a robot model defined using URDF (Unified Robot Description Format) and features inverse kinematics (IK) for controlling the arm's motion. The arm can be interacted with using interactive markers in RViz to move the arm's end effector to a target position in 3D space.

## Features
- **4-DOF Robotic Arm**: A robotic arm with four joints (base, shoulder, elbow, wrist) designed for precise control in simulation.
- **ROS2 Integration**: Uses ROS2 for communication, allowing for seamless interaction with other ROS-based systems.
- **Interactive Markers**: Provides a 3D interactive marker in RViz for users to move and rotate the arm's end effector.
- **Inverse Kinematics**: Solves the inverse kinematics to determine the joint angles based on the target position of the end effector.
- **URDF Model**: The robot's model is created using URDF with custom mesh files for each link of the robot.

## Requirements
- ROS2 (humble)
- Python 3.x
- RViz (for visualization)
- Robot model in URDF format with meshes
- ROS2 packages for `interactive_markers` and `geometry_msgs`

## Installation
1. **Clone the repository**:
    ```bash
    git clone https://github.com/Abhi-creator1/RobotArm-IKdsolver.git
    ```
    rename downloaded folder as `my_robot_arm`

2. **Install dependencies**:
    Make sure you have ROS2 installed and sourced. You will also need the necessary dependencies like `interactive_markers`, `geometry_msgs`, etc.

3. **Build the workspace**:
    ```bash
    colcon build --symlink-install
    ```

4. **Source the workspace**:
    ```bash
    source install/setup.bash
    ```

5. **Launch project**:
    Start the interactive marker publisher and other necessary nodes:
    ```bash
    ros2 launch my_robot_arm display.launch.py
    ```

## Usage
1. **Launch RViz** and load the robot model. You should see the 4DOF robotic arm along with interactive markers.
2. **Move the interactive marker** to the desired position to control the robot's TCP.
3. The inverse kinematics will be calculated, and the robot's joints will move accordingly.


## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
