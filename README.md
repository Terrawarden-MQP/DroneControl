# TerraWardenDrone

A PX4-ROS2 communication node for autonomous drone operations, part of the TerraWarden Drone Cleanup MQP at WPI.

[![Demo Video](https://img.youtube.com/vi/JgRSZfa9o8A/0.jpg)](https://www.youtube.com/watch?v=JgRSZfa9o8A)

## Repository Contents

- **wpi_drone**: Core ROS2 node for PX4 communication
- **PX4_custom_build**: Custom PX4 v1.14 firmware and parameter files
  - MQP_PX6X.params: Parameters for Holybro Pixhawk 6X with indoor/outdoor flight setup, ELRS radio, optical flow and lidar sensors

## Dependencies

- ROS2 Humble
- [px4_msgs](https://github.com/PX4/px4_msgs)
- [px4_ros_com](https://github.com/PX4/px4_ros_com)

## Setup Instructions

1. Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
2. Install PX4 firmware on your flight computer
3. Install the [uXRCE-DDS bridge](https://docs.px4.io/main/en/middleware/uxrce_dds.html)
4. Connect with ethernet and configure static IP addresses on both sides
5. Configure automatic startup of the service on both flight controller and companion computer

The communication uses [uORB messages](https://docs.px4.io/main/en/msg_docs/index.html) in a similar publisher/subscriber model as ROS2.

## Running the Code

```bash
colcon build
source ~/Desktop/ros_ws/install/setup.bash
ros2 launch wpi_drone vehicle_position.launch.py
```

## Additional Resources

- [PX4 Multicopter Frame Documentation](https://docs.px4.io/main/en/frames_multicopter/)
