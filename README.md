# ROS2 Publisher-Subscriber System

## Project Overview

This ROS2 project includes two nodes: a publisher and a subscriber. The publisher node generates a circular x and y coordinates with added random noise and publishes the noisy position data. The subscriber node subscribes to this topic, filters out the noise, and republishes the filtered x and y coordinates data.


The project is developed using C++ and utilizes ROS2’s rclcpp library for node creation, `geometry_msgs` for pose data, and `std_msgs` for message communication.


## Directory Structure

```
ros2_ws/
└── src/
    └── cpp_pubsub/
        ├── src/
        │   ├── publisher_lambda_function.cpp
        │   └── subscriber_lambda_function.cpp
        ├── include/
        │   └── cpp_pubsub/
        ├── CMakeLists.txt
        ├── LICENSE
        └── package.xml

```

- `publisher_lambda_function.cpp`: Contains the publisher node that generates and publishes noisy circular trajectory data.
- `subscriber_lambda_function.cpp`: Contains the subscriber node that filters the noisy data and republishes the filtered trajectory.
- `CMakeLists.txt`: CMake configuration file for building the package.
- `package.xml`: ROS2 package configuration file.
- `README.md`: build steps and about workspace

## Installation and Setup

### Prerequisites

- **ROS2**: Ensure you have ROS2 installed on your system.
- **Colcon**: Use `colcon` as the build tool.

- ⚠️ You can follow ROS2 Documentation for intalling ROS2 on Ubuntu Jazzy https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

### Building the Package

⚠️⚠️ Before start create a folder which name is "<your_project_name>" on Desktop and right click than click on "Open in Terminal".

1. Clone the repository into your `ros2_ws` workspace:
   ```bash
   mkdir -p ./ros2_ws/src
   cd ./ros2_ws/src
   git clone https://github.com/anilerman/ROS2_Pub_and_Sub_System.git cpp_pubsub
   ```
2. Source the ROS2:
   ```bash
   source /opt/ros/<distro>/setup.bash 
   ```
    Change <dıstro> with your setup. In my work it was 'jazzy'

3. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select cpp_pubsub
   ```
   ```bash
   colcon build --symlink-install
   ```

### Additional Requirements

To fully utilize the functionalities of this project, it is recommended to have the following ROS2 tools installed:

- **rqt**: For data visualization and analysis.
  ```bash
  sudo apt install ros-<distro>-rqt
  ```

- **rviz2**: For 3D data visualization.
  ```bash
  sudo apt install ros-<distro>-rviz2
  ```

> Note: Replace `<distro>` with your ROS2 distribution name (e.g., `humble`, `galactic`, `jazzy`).

⚠️ ⚠️ ⚠️ **Important:** Ensure that ROS2 is correctly installed and configured on your system. Verify that the installation includes all necessary dependencies and that you are using a compatible ROS2 distribution for this project. Incorrect setup may lead to runtime errors or unexpected behavior. ⚠️ ⚠️ ⚠️ 


### Running the Nodes
   
1. In a terminal, run the publisher node:
   ```bash
   source /opt/ros/<distro>/setup.bash
   ```
   ```bash
   source ./install/setup.bash
   ```
   ```bash
   ros2 run cpp_pubsub talker
   ```
   `

2. In another terminal, run the subscriber node:
   
   ```bash
   source /opt/ros/<distro>/setup.bash
   ```
   ```bash
   source ./install/setup.bash
   ```
   ```bash
   ros2 run cpp_pubsub listener
   ```

### Visualization with RQT

To visualize the data from the publisher and subscriber, you can use the `RQT` plotting tool:

1. Open `RQT`:
   ```bash
   rqt
   ```

2. Add the `trajectory` (noisy data) and `filtered_trajectory` (filtered data) topics in the `RQT` plot to see real-time data visualization.

3. These are the topic can be shown on RQT:
   -/filtered_trajectory/pose/position/x
   -/filtered_trajectory/pose/position/y
   -/trajectory/pose/position/x
   -/trajectory/pose/position/y

## Approach and Implementation

1. **Publisher Node**: Generates a circular trajectory and adds random noise to the `x` and `y` coordinates. This noisy data is published on the `trajectory` topic.

2. **Subscriber Node**: Subscribes to the `trajectory` topic, applies a low-pass filter to remove noise, and republishes the filtered data on the `filtered_trajectory` topic.


