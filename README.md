# ROS 2 Example Package - Foxy

This package contains an example of a ROS 2 C++ or Python Code.

- ROS Foxy
- [Official ROS 2 Tutorials](https://docs.ros.org/en/foxy/Tutorials.html)

## Requirements

- Ubuntu 20.04+
- Python 3.8.10
- ROS2 Foxy
- Anaconda / Miniconda

## Package Description

- ROS Topics (Publisher and Subscriber)
- ROS Services (Client and Server)
- ROS Actions
- Custom Messages, Services, Actions
- ROS Parameters, YAML files
- CMakeLists.txt and package.xml
- Launch files
- Useful Functions (Signal Handler, Class Programming)
- MoveIt (C++)

## Installation

### Prerequisites

- Install ROS2 Foxy: [Ubuntu Guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

        sudo apt install ros-foxy-desktop python3-argcomplete

- Install `miniconda`: [Official Guide](https://docs.conda.io/en/main/miniconda.html)

- Create a `conda` environment with `python=3.8.10`:

        conda create -n example_env python=3.8.10
        conda activate example_env

- Install Python Requirements:

        pip install -r ../path/to/this/repo/requirements.txt

## Launch Instructions

- Activate the `conda` environment:

        conda activate example_env

- Remember to source ROS2 and export the Domain ID (if not in `~/.bashrc`):

        source /opt/ros/foxy/setup.bash
        . ~/colcon_ws/install/setup.bash
        export ROS_DOMAIN_ID=10

### Launch PFL Controller

- Launch C++ Node `example_cpp.launch.py`:

        ros2 launch example_package example_cpp.launch.py

- Launch Python Node `example_python.launch.py`:

        ros2 launch example_package example_python.launch.py

- Launch Global Parameter Server Nodes `global_parameter.launch.py`:

        ros2 launch example_package global_parameter.launch.py

## Maintainers

- Davide Ferrari
