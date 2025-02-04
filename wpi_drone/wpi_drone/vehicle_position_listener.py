#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleGlobalPosition, OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus


class VehicleGlobalPositionListener(Node):
    def __init__(self):
        super().__init__('vehicle_global_position_listener')
        
        # Configure QoS profile for PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,  # maybe TRANSIENT_LOCAL
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Add debug logging
        self.get_logger().info("Starting subscriber with QoS profile:")
        self.get_logger().info(f"Reliability: {qos_profile.reliability}")
        self.get_logger().info(f"Durability: {qos_profile.durability}")
        self.get_logger().info(f"History: {qos_profile.history}")
        self.get_logger().info(f"Depth: {qos_profile.depth}")

        # Add initialization state tracking
        self.vehicle_status = None
        self.initialization_counter = 0
        self.arm_state = False
        self.vehicle_local_position = VehicleLocalPosition()

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscription
        self.subscription = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.listener_callback,
            qos_profile
        )
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        
        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
    
    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status
        
        
    def timer_callback(self) -> None:
        """Callback function for the timer."""
        # Publish the heartbeat
        self.publish_offboard_control_heartbeat_signal()

        # Wait for vehicle status to be initialized
        if self.vehicle_status is None:
            return

        # Initialize the drone (runs once)
        if self.initialization_counter < 10:
            self.initialization_counter += 1
    
            return

        # Handle arming sequence
        if not self.arm_state and self.initialization_counter >= 10:
            # self.engage_offboard_mode()
            # self.arm()
            # self.arm_state = True
            # self.get_logger().info("Vehicle armed and ready")
            pass

    def listener_callback(self, msg):
        self.get_logger().info('Received message!')  # Add debug logging
        # Clear terminal
        print('\n' * 30)
        
        # Print received data
        print('RECEIVED VEHICLE GLOBAL POSITION')
        print('===============================')
        print(f'timestamp: {msg.timestamp}')
        print(f'latitude: {msg.lat}')
        print(f'longitude: {msg.lon}')
        print(f'altitude: {msg.alt}')
        print(f'altitude ellipsoid: {msg.alt_ellipsoid}')
        print(f'delta alt: {msg.delta_alt}')
        print(f'velocity north: {msg.vx}')
        print(f'velocity east: {msg.vy}')
        print(f'velocity down: {msg.vz}')
        print(f'heading: {msg.heading}')
        
    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')
        
    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")


    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
        
    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0)
        self.get_logger().info("Switching to offboard mode")


def main(args=None) -> None:
    print("Starting vehicle_global_position listener node...")
    rclpy.init(args=args)
    vehicle_global_position_listener = VehicleGlobalPositionListener()
    rclpy.spin(vehicle_global_position_listener)
    
    vehicle_global_position_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)