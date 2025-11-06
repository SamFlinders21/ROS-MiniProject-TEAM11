# ROS Mini-Project Setup Guide

This guide explains how to set up your computer to run this ROS 2 project.

## 1. Prerequisites (Do This Once)

You must have the core ROS 2 system installed. We are using **ROS 2 Humble** on **Ubuntu 22.04**.

1.  **Install WSL:** On your Windows machine, [install WSL 2 and Ubuntu 22.04](https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-11-with-gui-support#1-overview)
2.  **Launch Ubuntu:** Open your Ubuntu 22.04 terminal.
3.  **Install ROS 2 Humble:** Run the following commands in your Ubuntu terminal. (You can find the [full install guide here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), but these are the key steps).

    * **Set up sources:**
        ```bash
        sudo apt update && sudo apt install curl
        sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        ```

    * **Install ROS 2 (Desktop):**
        ```bash
        sudo apt update
        sudo apt install ros-humble-desktop
        ```

    * **Install Build Tools:**
        ```bash
        sudo apt install ros-dev-tools
        ```

    * **Source ROS 2 (for this terminal):**
        ```bash
        source /opt/ros/humble/setup.bash
        ```

## 2. Project Setup (Do This Once)

Now that you have ROS 2, you can download and build our project.

1.  **Open a New Terminal:** Open a fresh Ubuntu 22.04 terminal.
2.  **Clone the Repository:** This downloads the project code from GitHub. We'll clone it into a folder named `ros2mp`.
    ```bash
    cd ~ 
    git clone [https://github.com/SamFlinders21/ROS-MiniProject-TEAM11.git](https://github.com/SamFlinders21/ROS-MiniProject-TEAM11.git) ros2mp
    ```
3.  **Install Dependencies:** This is a crucial step. This command reads our package's dependencies (like `sensor_msgs`, `visualization_msgs`) and installs them.
    * Navigate into your new workspace:
        ```bash
        cd ~/ros2mp
        ```
    * Initialize `rosdep` (you only need to do this *once ever* per computer):
        ```bash
        sudo rosdep init
        rosdep update
        ```
    * Run the installer. This command looks in your `src` folder, finds all package dependencies, and `apt install`s them.
        ```bash
        rosdep install --from-paths src -y --ignore-src
        ```
4.  **Build the Workspace:** Now you can `colcon build` our custom packages.
    ```bash
    # (You should still be in the ~/ros2mp folder)
    colcon build
    ```
5. **Optional: Adjust Shell Startup Script for Ease of Use**
   To avoid constantly having to run:
   ```bash
        source /opt/ros/humble/setup.bash
   ```
   Edit your startup script by typing:
   ```bash
        nano ~/.bashrc
   ```
   Once in this file go ahead and use your arrow keys to navigate to the bottom where you are going to add: source /opt/ros/humble/setup.bash

## 3. How to Run The Project

Every time you want to run the code, you need to open 2-3 terminals and run the nodes.

**In *each* terminal, you must first source your workspace:**
```bash
source ~/ros2mp/install/setup.bash
ros2 run mp2 fk_node
```
## Terminal 2 (The Visualizer):
```bash
rviz2
```
In RViz, set the "Fixed Frame" to world and click "Add" (bottom left), go to the "By topic" tab, and add the /arm_marker topic



## Terminal 3 (The Joint Publisher):

```bash
source ~/ros2mp/install/setup.bash
ros2 run mp2 joint_publisher
```
