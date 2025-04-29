# TerraWardenDrone

A PX4-ROS2 communication node for autonomous drone operations, part of the TerraWarden Drone Cleanup MQP at WPI.

[![Demo Video](https://img.youtube.com/vi/JgRSZfa9o8A/0.jpg)](https://www.youtube.com/watch?v=JgRSZfa9o8A)

## Repository Contents

- **wpi_drone**: Core ROS2 node for PX4 communication
- **PX4_custom_build**: Custom PX4 v1.14 firmware and parameter files
  - MQP_PX6X.params: Parameters for Holybro Pixhawk 6X with indoor/outdoor flight setup, ELRS radio, optical flow and lidar sensors

## Dependencies

- [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [px4_msgs](https://github.com/PX4/px4_msgs)
- [px4_ros_com](https://github.com/PX4/px4_ros_com)
- [Terrawarden-MQP/CustomMessages](https://github.com/Terrawarden-MQP/CustomMessages) for custom message interfaces

## Setup Instructions

1. Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
2. Install [PX4 firmware](https://github.com/PX4/PX4-Autopilot) on your flight computer
3. Install the [uXRCE-DDS bridge](https://docs.px4.io/main/en/middleware/uxrce_dds.html)
4. Connect with ethernet and configure static IP addresses on both sides
5. Configure automatic startup of the bridge services on both flight controller and companion computer

The communication uses [uORB messages](https://docs.px4.io/main/en/msg_docs/index.html) in a similar publisher/subscriber model as ROS2.

## Running the Code

```bash
colcon build
source ~/Desktop/ros_ws/install/setup.bash
ros2 launch joisie_vision live_detect.launch.py
```

## Additional Resources

- [PX4 Multicopter Frame Documentation](https://docs.px4.io/main/en/frames_multicopter/)

# ROS2 Parameters and Message Types Documentation

This document provides an overview of all ROS2 parameters and message types used in the TerraWardenDrone system.

## ROS2 Parameters

The `VehicleGlobalPositionListener` node declares the following parameters:

| Parameter                 | Type    | Default             | Description                                                                              |
| ------------------------- | ------- | ------------------- | ---------------------------------------------------------------------------------------- |
| `debug_printout`          | Boolean | `True`              | Controls whether to print copious amounts of flight computer internal status information |
| `debug_printout_period_s` | Float   | `2.0`               | Sets the interval (in seconds) for debug information printouts                           |
| `drone_pose_topic`        | String  | `'drone/waypoint'`  | Topic name for drone waypoint commands                                                   |
| `drone_telemetry_topic`   | String  | `'drone/telemetry'` | Topic name for drone telemetry data                                                      |

## ROS2 Message Types

### Custom Message Types

Custom message types for TerraWardenDrone are defined in the [Terrawarden-MQP/CustomMessages](https://github.com/Terrawarden-MQP/CustomMessages) repository.

#### Published Messages

1. `CustomMessages/msg/DroneTelemetry`
   - **Published on:** `drone/telemetry` (configurable via parameter)
   - **Content:**
     ```
     geometry_msgs/PoseStamped pos      # Current position with orientation
     geometry_msgs/PoseStamped vel      # Current velocity
     sensor_msgs/NavSatFix gps          # GPS data
     bool is_flying                     # Flight status
     bool is_offboard                   # Offboard control status
     bool has_rc_link                   # RC link status
     float32 altitude_above_ground      # Height above ground
     float32 heading_degrees            # Current heading in degrees
     float32 battery_percentage         # Battery level
     string error                       # Error messages
     ```

#### Subscribed Messages

1. `CustomMessages/msg/DroneWaypoint`
   - **Subscribed on:** `drone/waypoint` (configurable via parameter)
   - **Content:**
     ```
     geometry_msgs/Point ned_pos        # Position in NED coordinates
     float32 heading_degrees            # Target heading in degrees
     float32 max_ang_vel_deg_s          # Maximum angular velocity
     float32 max_lin_vel_m_s            # Maximum linear velocity
     float32 max_z_vel_m_s              # Maximum vertical velocity
     float32 max_lin_accel_m_s2         # Maximum linear acceleration
     ```

### PX4 ROS2 Messages

The node uses PX4's ROS2 messages for communication with the flight controller.

#### Published to PX4

1. `px4_msgs/msg/OffboardControlMode`
   - **Published on:** `/fmu/in/offboard_control_mode`
   - **Purpose:** Sends heartbeat signals for offboard control
   - **Content:** Position, velocity, acceleration, attitude, and body rate control flags

2. `px4_msgs/msg/VehicleCommand`
   - **Published on:** `/fmu/in/vehicle_command`
   - **Purpose:** Sends vehicle commands (arm, disarm, land, etc.)

3. `px4_msgs/msg/TrajectorySetpoint`
   - **Published on:** `/fmu/in/trajectory_setpoint`
   - **Purpose:** Sets position/waypoint targets
   - **Content:** 3D position, yaw, velocity, acceleration

#### Subscribed from PX4

1. `px4_msgs/msg/VehicleStatus`
   - **Subscribed on:** `/fmu/out/vehicle_status`
   - **Purpose:** Monitors vehicle status, arm state, nav state, etc.

2. `px4_msgs/msg/VehicleLocalPosition`
   - **Subscribed on:** `/fmu/out/vehicle_local_position`
   - **Purpose:** Receives local position data in NED frame

3. `px4_msgs/msg/VehicleOdometry`
   - **Subscribed on:** `/fmu/out/vehicle_odometry`
   - **Purpose:** Receives vehicle odometry data (position, orientation, velocity)

4. `px4_msgs/msg/BatteryStatus`
   - **Subscribed on:** `/fmu/out/battery_status`
   - **Purpose:** Monitors battery voltage, current, and remaining capacity

5. `px4_msgs/msg/SensorGps`
   - **Subscribed on:** `/fmu/out/sensor_gps`
   - **Purpose:** Receives GPS data

6. `px4_msgs/msg/InputRc`
   - **Subscribed on:** `/fmu/out/input_rc`
   - **Purpose:** Monitors radio control link status and inputs

### Standard ROS2 Message Types Used

- `geometry_msgs/PoseStamped`
- `geometry_msgs/Point`
- `sensor_msgs/NavSatFix`

## Communication Flow

The TerraWardenDrone system uses a bridge architecture between ROS2 and PX4:

1. External systems communicate with the drone using the custom `terrawarden_interfaces` messages
2. The `VehicleGlobalPositionListener` node translates these messages to appropriate PX4 messages
3. PX4 messages are sent to the flight controller via the uXRCE-DDS bridge
4. Telemetry and status information flows back through the same path in reverse
