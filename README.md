# Open Manipulator Pro Robot

Repository to group URDF, launch files, rviz configuration, meshes, controller configurations for the OpenManipulatorPro

## Table of Contents

- [Open Manipulator Pro Robot](#open-manipulator-pro-robot)
  - [Table of Contents](#table-of-contents)
  - [Installation Premises](#installation-premises)
  - [Installing](#installing)
    - [Non ROS Dependencies](#non-ros-dependencies)
    - [Installing from source](#installing-from-source)
  - [Building](#building)
  - [Static code analysis](#static-code-analysis)
  - [Usage](#usage)
    - [View Robot Only](#view-robot-only)
  - [Config files](#config-files)
  - [Nodes](#nodes)
  - [Contributing](#contributing)

## Installation Premises

1. This repository has been tested on [ROS2 Humble];

2. These instructions assume that you have already installed ROS2 Humble Hawksbill on your machine. If not, please follow the [recommended ubuntu installation tutorial];

3. Before installing the package, you will need to have an ament workspace set up. If you don't have one, follow the instructions in the [Creating a workspace tutorial]. Once you have created the workspace, clone this repository in the source folder of your workspace.

## Installing

### Non ROS Dependencies

- [vcstool](https://github.com/dirk-thomas/vcstool): Refer to its repository for instructions on how to install.

### Installing from source

> **ATTENTION:** These commands assume that you have created a workspace called "ros_ws" in your home folder. If you used a different directory or name, please adjust the commands accordingly.

After installing ROS2 and creating the workspace, clone this repository in your workspace:

```
cd ~/ros_ws/src
git clone https://github.com/brschettini/open_manipulator_pro_robot.git
```

Install the binary dependencies by running the following command in the root of your workspace:

```
cd ~/ros_ws
rosdep init
rosdep update
sudo apt update
rosdep install --from-paths src/open_manipulator_pro_robot --ignore-src -r -y --rosdistro humble
```

If all dependencies are already installed, you should see the message "All required rosdeps installed successfully."

<!-- Now, install the omnidirectional controller packager running:

```
cd ~/ros_ws/src
vcs import < open_manipulator_pro_robot/open_manipulator_pro_robot.foxy.repos
``` -->

## Building

Run the following command to build the package:

```
cd ~/ros_ws
colcon build --symlink-install --event-handlers console_direct+
```

> Run `colcon build --help` to understand the arguments passed!

After building the package, open a new terminal and navigate to your workspace. Then, source the overlay by running the following command:

```
source ~/ros_ws/install/setup.bash
```

> See [Source the overlay] to learn about underlay and overlay concepts.

## Static code analysis

Run the static code analysis with

> TODO

## Usage

### View Robot Only

To only view the robot on rviz2, run the following command:

```
ros2 launch open_manipulator_pro_description view_robot.launch.py enable_joint_state_publisher_gui:=true
```

> **NOTE:** The argument `enable_joint_state_publisher_gui` is needed to provide the manipulator's tf tree. When using the ros2_control, it is not required because the `joint_state_broadcaster` provides the tree.

### Launch Simulation

```
ros2 launch open_manipulator_pro_gazebo simulation.launch.py
```

## Config files

> TODO
<!-- - **[omnidirectional_controller.yaml](axebot_control/config/omnidirectional_controller.yaml):** Parameters of the omnidirectional controller. -->

## Nodes

> TODO

## Contributing

To contribute to this package, you can either [open an issue](https://github.com/brschettini/open_manipulator_pro_robot/issues) describing the desired subject or develop the feature yourself and [submit a pull request](https://github.com/brschettini/open_manipulator_pro_robot/pulls) to the main branch.

If you choose to develop the feature yourself, please adhere to the [ROS 2 Code style and language] guidelines to improve code readability and maintainability.

[Creating a workspace tutorial]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#creating-a-workspace
[recommended ubuntu installation tutorial]: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
[Source the overlay]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-the-overlay
[ROS 2 Code style and language]: https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html#code-style-and-language-versions
[ROS2 Humble]: https://docs.ros.org/en/humble/index.html
<!-- [Classic Gazebo 11]: https://classic.gazebosim.org/ -->